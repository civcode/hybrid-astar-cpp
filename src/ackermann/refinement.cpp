#include "hybrid_astar/ackermann/refinement.hpp"
#include "hybrid_astar/ackermann/clearance.hpp"
#include "hybrid_astar/ackermann/curved_shortcuts.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <stdexcept>
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
// Verify EVERY section, including the original, with the independent strict
// checker. No EDT, no search cache. Recompute objective from actual arcs.
Check validate(const PlanRequest& req,const std::vector<MotionSection>& proposed,
               const ExactCollisionChecker& checker, const ClearanceScorer* scorer,
               std::size_t& checks,std::size_t maxChecks,Evaluated& out) {
  const auto& opt=req.options;
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
        !std::isfinite(part.lengthMeters)||part.lengthMeters<=0||
        std::abs(part.curvature)>1.0/opt.minimumTurningRadiusMeters+1e-10||
        (part.gear!=Gear::Forward&&part.gear!=Gear::Reverse)||
        !close(part.start,out.last)) return Check::Invalid;
    const auto sec=makeSection(out.last,part.gear,part.curvature,part.lengthMeters);
    if (!close(sec.end,part.end)) return Check::Invalid;
    const double bodyFactor=1.0+checker.maxBodyRadius()*std::abs(sec.curvature);
    const double countD=std::max(sec.lengthMeters/opt.finalSampleSpacingMeters,
                   sec.lengthMeters*bodyFactor/opt.finalMaxBodyPointMotionMeters);
    if (!std::isfinite(countD)||countD>static_cast<double>(opt.maxFinalSamplesPerSection-1))
      return Check::Incomplete;
    const auto n=std::max<std::size_t>(1,static_cast<std::size_t>(std::ceil(countD)));
    // Fail before the loop if the entire requested check cannot be afforded.
    if (n+1>maxChecks-checks) return Check::Incomplete;
    for (std::size_t i=0;i<=n;++i) {
      if (cancelled(req)) return Check::Cancelled;
      const auto pose=sec.evaluate(sec.lengthMeters*double(i)/double(n));
      ++checks;
      if (checker.check(pose)!=PoseCollision::Clear) return Check::Invalid;
    }
    const double base=edgeCost(sec.lengthMeters,sec.gear,out.gear,opt.search);
    double extra=0;
    if (scorer) {
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
    out.gear=sec.gear;
    out.sections.push_back(sec);
  }
  if (std::hypot(out.last.x-req.goal.x,out.last.y-req.goal.y)>opt.goalXYToleranceMeters||
      yawError(out.last.yaw,req.goal.yaw)>opt.goalYawToleranceRadians)
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
    if(prior!=Gear::None&&part.gear!=prior) result.gearChangeSectionIndices.push_back(i);
    prior=part.gear;
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
              if(section.gear==Gear::None||!std::isfinite(section.lengthMeters)||
                 section.lengthMeters<=0) {competitive=false;break;}
              const double edge=edgeCost(section.lengthMeters,section.gear,
                                         priorGear,req.options.search);
              baseLowerBound+=edge;
              priorGear=section.gear;
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
