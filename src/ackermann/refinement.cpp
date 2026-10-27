#include "hybrid_astar/ackermann/refinement.hpp"
#include "hybrid_astar/ackermann/clearance.hpp"
#include "hybrid_astar/ackermann/curved_shortcuts.hpp"
#include "hybrid_astar/differential_drive/model.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <array>
#include <utility>

namespace hybrid_astar::ackermann {
namespace {
constexpr double pi=std::numbers::pi_v<double>;
constexpr double endpointTolerance=1e-5;
bool finite(const Pose2D& p) {
  return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.yaw);
}
double yawError(double a,double b) { return std::abs(std::remainder(a-b,2*pi)); }
bool close(const Pose2D& a,const Pose2D& b) {
  return std::hypot(a.x-b.x,a.y-b.y)<=endpointTolerance &&
         yawError(a.yaw,b.yaw)<=endpointTolerance;
}
bool cancelled(const PlanRequest& req) {
  return req.cancelled && req.cancelled->load(std::memory_order_relaxed);
}
enum class Check { Valid, Invalid, Incomplete, Cancelled };
struct Evaluated {
  std::vector<MotionSection> sections;
  double weighted=0,clearance=0;
  Pose2D last;
  Gear gear=Gear::None;
};
// Stationary rotations retain the previous translational gear for cost
// switches, exactly like differential-drive planner primitive transitions.
double sectionBaseCost(const PlanRequest& req,const MotionSection& sec,Gear prior) {
  if(sec.kind==MotionKind::RotateInPlace) {
    if(req.options.model!=PlanOptions::Model::DifferentialDrive ||
       sec.gear!=Gear::None || sec.lengthMeters!=0 ||
       !std::isfinite(sec.turnRadians) || std::abs(sec.turnRadians)<=1e-12)
      return std::numeric_limits<double>::quiet_NaN();
    return req.options.rotationCostMetersPerRadian*std::abs(sec.turnRadians);
  }
  if(sec.kind!=MotionKind::Translate || sec.lengthMeters<=0 ||
     !std::isfinite(sec.lengthMeters) ||
     (sec.gear!=Gear::Forward&&sec.gear!=Gear::Reverse))
    return std::numeric_limits<double>::quiet_NaN();
  return edgeCost(sec.lengthMeters,sec.gear,prior,req.options.search);
}
// Differential drive has no Ackermann minimum turning radius. Every candidate
// constant-curvature twist can be time-scaled down to meet its wheel-rate
// bound; the strictly positive rate is not a speed-profile certification.
bool differentialMotionFeasible(const PlanRequest& req,const MotionSection& sec,
                                const differential_drive::Model& model) {
  const auto& cfg=req.options.diffPrimitives;
  const auto& p=model.parameters();
  if(sec.kind==MotionKind::RotateInPlace) {
    const double limit=2*p.wheelRadiusMeters*p.maxAbsWheelAngularRate/p.trackWidthMeters;
    const double rate=std::min(cfg.rotationSpeed,.99*limit);
    return std::isfinite(rate)&&rate>0&&
           model.valid({0,std::copysign(rate,sec.turnRadians)});
  }
  const double denom=1+0.5*p.trackWidthMeters*std::abs(sec.curvature);
  const double cap=p.wheelRadiusMeters*p.maxAbsWheelAngularRate/denom;
  const double speed=std::min(cfg.translationSpeed,.99*cap);
  if(!std::isfinite(speed)||speed<=0) return false;
  const double v=sec.gear==Gear::Forward?speed:-speed;
  return model.valid({v,v*sec.curvature});
}
// Verify EVERY section, including the original, with the independent strict
// checker. No EDT, no search cache. Recompute objective from actual arcs.
Check validate(const PlanRequest& req,const std::vector<MotionSection>& proposed,
               const ExactCollisionChecker& checker, const ClearanceScorer* scorer,
               std::size_t& checks,std::size_t maxChecks,Evaluated& out) {
  const auto& opt=req.options;
  const bool differential=opt.model==PlanOptions::Model::DifferentialDrive;
  std::unique_ptr<differential_drive::Model> diffModel;
  if(differential) diffModel=std::make_unique<differential_drive::Model>(opt.diffParameters);
  out=Evaluated{};
  out.sections.reserve(proposed.size());
  out.last=req.start;
  if (cancelled(req)) return Check::Cancelled;
  if (checks>=maxChecks) return Check::Incomplete;
  ++checks;
  if (checker.check(req.start)!=PoseCollision::Clear) return Check::Invalid;
  for (const auto& part:proposed) {
    if (cancelled(req)) return Check::Cancelled;
    if (!finite(part.start)||!finite(part.end)||!std::isfinite(part.curvature)||
        !std::isfinite(part.lengthMeters)||!std::isfinite(part.turnRadians)||
        !close(part.start,out.last)) return Check::Invalid;
    const bool spin=part.kind==MotionKind::RotateInPlace;
    if(spin) {
      if(!differential||part.gear!=Gear::None||part.lengthMeters!=0||
         part.curvature!=0||std::abs(part.turnRadians)<=1e-12)
        return Check::Invalid;
    } else if(part.kind!=MotionKind::Translate || part.lengthMeters<=0 ||
              (part.gear!=Gear::Forward&&part.gear!=Gear::Reverse)||
              std::abs(part.turnRadians)>1e-12 ||
              (!differential &&
               std::abs(part.curvature)>1.0/opt.minimumTurningRadiusMeters+1e-10))
      return Check::Invalid;
    const auto sec=spin?makeRotationSection(out.last,part.turnRadians)
                       :makeSection(out.last,part.gear,part.curvature,part.lengthMeters);
    if (!close(sec.end,part.end) ||
        (differential&&!differentialMotionFeasible(req,sec,*diffModel)))
      return Check::Invalid;
    const double radius=checker.maxBodyRadius();
    const double countD=spin?radius*std::abs(sec.turnRadians)/opt.finalMaxBodyPointMotionMeters
      :std::max(sec.lengthMeters/opt.finalSampleSpacingMeters,
               sec.lengthMeters*(1+radius*std::abs(sec.curvature))/
                   opt.finalMaxBodyPointMotionMeters);
    if (!std::isfinite(countD)||countD>static_cast<double>(opt.maxFinalSamplesPerSection-1))
      return Check::Incomplete;
    const auto n=std::max<std::size_t>(1,static_cast<std::size_t>(std::ceil(countD)));
    // Fail before the loop if the entire requested check cannot be afforded.
    if (n+1>maxChecks-checks) return Check::Incomplete;
    for (std::size_t i=0;i<=n;++i) {
      if (cancelled(req)) return Check::Cancelled;
      const auto extent=sec.progressExtent();
      const auto pose=sec.evaluate(i==n ? extent : extent*double(i)/double(n));
      ++checks;
      if (checker.check(pose)!=PoseCollision::Clear) return Check::Invalid;
    }
    const double base=sectionBaseCost(req,sec,out.gear);
    double extra=0;
    if (scorer && !spin) {
      std::size_t unusedSamples=0;
      try {extra=scorer->sectionCost(sec,opt.searchSampleSpacingMeters,
                opt.searchMaxBodyPointMotionMeters,opt.maxSamplesPerSection,unusedSamples);}
      catch(const std::invalid_argument&) {return Check::Incomplete;}
    }
    if (!std::isfinite(base+extra)||!std::isfinite(out.weighted+base+extra))
      return Check::Invalid;
    out.weighted+=base+extra;
    out.clearance+=extra;
    out.last=sec.end;
    if(!spin)out.gear=sec.gear;
    out.sections.push_back(sec);
  }
  if (std::hypot(out.last.x-req.goal.x,out.last.y-req.goal.y)>opt.goalXYToleranceMeters+1e-9||
      yawError(out.last.yaw,req.goal.yaw)>opt.goalYawToleranceRadians+1e-9)
    return Check::Invalid;
  return Check::Valid;
}
std::vector<MotionSection> straightShortcut(const Pose2D& start,const Pose2D& end) {
  if (yawError(start.yaw,end.yaw)>endpointTolerance) return {};
  const double dx=end.x-start.x,dy=end.y-start.y;
  const double c=std::cos(start.yaw),s=std::sin(start.yaw);
  const double longitudinal=c*dx+s*dy;
  const double lateral=-s*dx+c*dy;
  if (std::abs(lateral)>endpointTolerance||std::abs(longitudinal)<1e-8) return {};
  const auto gear=longitudinal>0?Gear::Forward:Gear::Reverse;
  return {makeSection(start,gear,0,std::abs(longitudinal))};
}
PlanResult makeVerified(const PlanRequest& req, const PlanResult& source,
                        const Evaluated& evaluated) {
  PlanResult result=source;
  result.status=PlanStatus::Success;
  result.sections=evaluated.sections;
  result.reached=evaluated.last;
  result.finalGear=evaluated.gear;
  result.goalXYErrorMeters=std::hypot(evaluated.last.x-req.goal.x,evaluated.last.y-req.goal.y);
  result.goalYawErrorRadians=yawError(evaluated.last.yaw,req.goal.yaw);
  result.weightedCostMeters=evaluated.weighted;
  result.clearanceCostMeters=evaluated.clearance;
  result.geometricLengthMeters=0;
  result.reverseLengthMeters=0;
  result.gearChangeSectionIndices.clear();
  Gear prior=Gear::None;
  for(std::size_t i=0;i<result.sections.size();++i) {
    const auto& part=result.sections[i];
    result.geometricLengthMeters+=part.lengthMeters;
    if(part.gear==Gear::Reverse) result.reverseLengthMeters+=part.lengthMeters;
    if(part.kind!=MotionKind::RotateInPlace) {
      if(prior!=Gear::None&&part.gear!=prior) result.gearChangeSectionIndices.push_back(i);
      prior=part.gear;
    }
  }
  result.mapRevision=req.map->revision;
  result.mapFrameId=req.map->frameId;
  result.mapSourceId=req.map->sourceId;
  result.effectiveOccupancy=req.occupancy;
  result.collisionEvidence=CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled;
  result.modelGeometryValid=true;
  result.trajectoryActuatorFeasible=false;
  return result;
}
} // namespace

RefinementResult refinePath(const PlanRequest& req,const PlanResult& original,
                            RefinementConnector connector,RefinementOptions options) {
  RefinementResult result;
  if (!req.map||!finite(req.start)||!finite(req.goal)||
      original.status!=PlanStatus::Success||!original.modelGeometryValid||
      original.collisionEvidence!=CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled||
      original.mapRevision!=req.map->revision||original.mapFrameId!=req.map->frameId||
      original.mapSourceId!=req.map->sourceId||
      original.effectiveOccupancy.maxTraversableOccupancy!=req.occupancy.maxTraversableOccupancy||
      original.effectiveOccupancy.unknown!=req.occupancy.unknown||
      options.maxAttempts==0||options.maxAccepted==0||options.maxReplacementSections==0||
      options.maxTotalPoseChecks==0||!std::isfinite(options.minimumCostReductionMeters)||
      options.minimumCostReductionMeters<=0||
      !std::isfinite(options.differentialCurveRadiusMeters)||
      options.differentialCurveRadiusMeters<=0||
      (req.options.model!=PlanOptions::Model::Ackermann &&
       req.options.model!=PlanOptions::Model::DifferentialDrive)||
      (req.options.model==PlanOptions::Model::DifferentialDrive &&
       (req.analyticConnector||req.analyticDistance||req.options.clearanceWeight>0))||
      !std::isfinite(req.options.minimumTurningRadiusMeters)||req.options.minimumTurningRadiusMeters<=0||
      !std::isfinite(req.options.goalXYToleranceMeters)||req.options.goalXYToleranceMeters<0||
      !std::isfinite(req.options.goalYawToleranceRadians)||req.options.goalYawToleranceRadians<0||
      !std::isfinite(req.options.finalSampleSpacingMeters)||req.options.finalSampleSpacingMeters<=0||
      !std::isfinite(req.options.finalMaxBodyPointMotionMeters)||req.options.finalMaxBodyPointMotionMeters<=0||
      req.options.maxFinalSamplesPerSection<2||
      !std::isfinite(req.options.searchSampleSpacingMeters)||req.options.searchSampleSpacingMeters<=0||
      !std::isfinite(req.options.searchMaxBodyPointMotionMeters)||req.options.searchMaxBodyPointMotionMeters<=0||
      req.options.maxSamplesPerSection<2||!std::isfinite(req.options.clearanceWeight)||
      req.options.clearanceWeight<0||!std::isfinite(req.options.clearanceInfluenceRadiusMeters)||
      req.options.clearanceInfluenceRadiusMeters<=0) return result;
  try {
    validate(req.options.search);
    ExactCollisionChecker checker(*req.map,req.occupancy,req.footprint);
    std::unique_ptr<ClearanceScorer> clearance;
    if(req.options.clearanceWeight>0)
      clearance=std::make_unique<ClearanceScorer>(*req.map,req.occupancy,req.footprint,
                   req.options.clearanceWeight,req.options.clearanceInfluenceRadiusMeters);
    Evaluated baseline;
    const auto initial=validate(req,original.sections,checker,clearance.get(),
                                result.exactPoseChecks,options.maxTotalPoseChecks,baseline);
    if(initial!=Check::Valid) {
      result.status=initial==Check::Cancelled?RefinementStatus::Cancelled:
                    RefinementStatus::OriginalNotVerified;
      return result; // No unverified path leaks to consumers.
    }
    result.path=makeVerified(req,original,baseline);
    if(cancelled(req)) {result.status=RefinementStatus::Cancelled;return result;}
    bool limit=false;
    for (std::size_t pass=0;pass<options.maxAccepted;++pass) {
      bool changed=false;
      const auto n=result.path.sections.size();
      for (std::size_t i=0;i+1<n&&!changed;++i) {
        // Try widest eligible window first, deterministic with finite count.
        for (std::size_t j=n;j>=i+2;--j) {
          if(result.attempted>=options.maxAttempts ||
             result.exactPoseChecks>=options.maxTotalPoseChecks) {limit=true;break;}
          if(cancelled(req)) {result.status=RefinementStatus::Cancelled;return result;}
          ++result.attempted;
          const auto& before=result.path.sections;
          const Pose2D from=before[i].start,to=before[j-1].end;
          std::vector<std::vector<MotionSection>> proposals;
          try {
            if (connector) proposals.push_back(connector(from,to));
            else if (options.useCurvatureBoundedShortcuts)
              proposals=generateCurvatureBoundedShortcuts(
                  from,to,req.options.minimumTurningRadiusMeters);
            else proposals.push_back(straightShortcut(from,to));
          } catch(const std::exception&) {++result.rejected;continue;}
          bool accepted=false;
          // Every geometric proposal gets the SAME independently exact,
          // complete-path validation. Work is bounded by total pose checks;
          // the native CSC provider returns at most eight alternatives.
          for(const auto& replacement:proposals) {
            if(cancelled(req)) {result.status=RefinementStatus::Cancelled;return result;}
            if(replacement.empty()||replacement.size()>options.maxReplacementSections) {
              ++result.rejected;continue;
            }
            std::vector<MotionSection> candidate;
            candidate.reserve(before.size()-(j-i)+replacement.size());
            candidate.insert(candidate.end(),before.begin(),before.begin()+i);
            candidate.insert(candidate.end(),replacement.begin(),replacement.end());
            candidate.insert(candidate.end(),before.begin()+j,before.end());
            // Clearance penalties are nonnegative. Hence the weighted base
            // cost of the whole path is a SAFE lower bound on its true cost.
            // Skip costly exact checking when no strict improvement is possible.
            Gear priorGear=Gear::None;
            double baseLowerBound=0.0;
            bool competitive=true;
            for (const auto& section:candidate) {
              const double edge=sectionBaseCost(req,section,priorGear);
              if(!std::isfinite(edge)||edge<=0){competitive=false;break;}
              baseLowerBound+=edge;
              if(section.kind!=MotionKind::RotateInPlace)priorGear=section.gear;
              if(!std::isfinite(baseLowerBound)||
                 baseLowerBound>=result.path.weightedCostMeters-
                    options.minimumCostReductionMeters) {competitive=false;break;}
            }
            if(!competitive) {
              ++result.rejected; ++result.prunedByCostLowerBound; continue;
            }
            Evaluated evaluated;
            const auto check=validate(req,candidate,checker,clearance.get(),
                                     result.exactPoseChecks,options.maxTotalPoseChecks,evaluated);
            if(check==Check::Cancelled) {result.status=RefinementStatus::Cancelled;return result;}
            if(check==Check::Incomplete) {limit=true;break;}
            if(check!=Check::Valid || evaluated.weighted>=
               result.path.weightedCostMeters-options.minimumCostReductionMeters) {
              ++result.rejected;continue;
            }
            result.path=makeVerified(req,result.path,evaluated);
            result.path.solutionQuality=SolutionQuality::FeasibleAfterRefinement;
            ++result.accepted;
            changed=true;accepted=true;
            break;
          }
          if(limit||accepted) break;
        }
        if(limit) break;
      }
      if(limit||!changed) break;
    }
    result.status=limit?RefinementStatus::WorkBudgetExceeded:
                   (result.accepted?RefinementStatus::Improved:RefinementStatus::Unchanged);
  } catch (const std::exception&) {
    // If original was validated before an exceptional proposal, it is safe
    // to return. Otherwise never advertise a path as verified.
    result.status=result.path.status==PlanStatus::Success?
                   (result.accepted?RefinementStatus::Improved:RefinementStatus::Unchanged):
                   RefinementStatus::OriginalNotVerified;
  }
  return result;
}
} // namespace hybrid_astar::ackermann
