#include "hybrid_astar/ackermann/refinement.hpp"
#include "hybrid_astar/ackermann/clearance.hpp"
#include "hybrid_astar/ackermann/curved_shortcuts.hpp"
#include "hybrid_astar/differential_drive/model.hpp"
#include "hybrid_astar/mecanum/model.hpp"
#include "hybrid_astar/motion/primitive.hpp"
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
  if(req.options.model==PlanOptions::Model::Mecanum) {
    if(sec.gear!=Gear::None||
       (sec.kind!=MotionKind::RotateInPlace &&
        sec.kind!=MotionKind::HolonomicTwist) ||
       !std::isfinite(sec.lengthMeters)||sec.lengthMeters<0 ||
       !std::isfinite(sec.turnRadians))
      return std::numeric_limits<double>::quiet_NaN();
    return sec.lengthMeters+
        req.options.rotationCostMetersPerRadian*std::abs(sec.turnRadians)+
        (sec.kind==MotionKind::RotateInPlace?
         req.options.mecanumStationarySpinPenaltyMeters:0.0);
  }
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
// Inverse SE(2) exponential: exactly connects two planar states in a single
// simultaneous translation+rotation without introducing a new stop. The
// ideal mecanum model can time-scale finite twists to respect wheel limits.
std::vector<MotionSection> holonomicShortcut(const PlanRequest& req,
                                              const Pose2D& start,
                                              const Pose2D& end) {
  if(!finite(start)||!finite(end))return {};
  const double dx=end.x-start.x,dy=end.y-start.y;
  const double c=std::cos(start.yaw),s=std::sin(start.yaw);
  const double x=c*dx+s*dy,y=-s*dx+c*dy;
  const double angle=motion::wrapAngle(end.yaw-start.yaw);
  if(std::hypot(x,y)<1e-8)return {};
  const double a=std::abs(angle)<1e-7?1-angle*angle/6:std::sin(angle)/angle;
  const double b=std::abs(angle)<1e-7?.5*angle-angle*angle*angle/24:
                     (1-std::cos(angle))/angle;
  const double denom=a*a+b*b;
  if(!std::isfinite(denom)||denom<1e-10)return {};
  const double forward=(a*x+b*y)/denom,lateral=(a*y-b*x)/denom;
  if(!std::isfinite(forward)||!std::isfinite(lateral)||
     std::hypot(forward,lateral)<=1e-8)return {};
  const auto section=makeHolonomicSection(start,forward,lateral,angle);
  if(!close(section.end,end))return {};
  return {section};
}
bool mecanumMotionFeasible(const PlanRequest& req,const MotionSection& sec,
                          const mecanum::Model& model) {
  const auto& p=model.parameters();
  if(sec.kind==MotionKind::RotateInPlace) {
    if(!std::isfinite(sec.turnRadians)||sec.turnRadians==0)return false;
    const double omega=std::min(req.options.mecanumPrimitives.rotationSpeed,
        .99*p.wheelRadiusMeters*p.maxAbsWheelAngularRate/
              (p.halfWheelbaseMeters+p.halfTrackMeters));
    return std::isfinite(omega)&&omega>0 &&
           model.valid({0,0,std::copysign(omega,sec.turnRadians)});
  }
  if(sec.kind!=MotionKind::HolonomicTwist || sec.lengthMeters<=0)return false;
  const double cap=p.wheelRadiusMeters*p.maxAbsWheelAngularRate;
  const double seconds=std::max({
      sec.lengthMeters/req.options.mecanumPrimitives.translationSpeed,
      std::abs(sec.turnRadians)/req.options.mecanumPrimitives.rotationSpeed,
      1.01*(std::abs(sec.forwardMeters)+std::abs(sec.lateralMeters)+
            (p.halfWheelbaseMeters+p.halfTrackMeters)*std::abs(sec.turnRadians))/cap,
      1e-5});
  return std::isfinite(seconds) && model.valid({
      sec.forwardMeters/seconds,sec.lateralMeters/seconds,
      sec.turnRadians/seconds});
}
// Verify EVERY section, including the original, with the independent strict
// checker. No EDT, no search cache. Recompute objective from actual arcs.
Check validate(const PlanRequest& req,const std::vector<MotionSection>& proposed,
               const ExactCollisionChecker& checker, const ClearanceScorer* scorer,
               std::size_t& checks,std::size_t maxChecks,Evaluated& out) {
  const auto& opt=req.options;
  const bool differential=opt.model==PlanOptions::Model::DifferentialDrive;
  const bool mecanumMode=opt.model==PlanOptions::Model::Mecanum;
  std::unique_ptr<differential_drive::Model> diffModel;
  if(differential) diffModel=std::make_unique<differential_drive::Model>(opt.diffParameters);
  std::unique_ptr<mecanum::Model> mecanumModel;
  if(mecanumMode)mecanumModel=std::make_unique<mecanum::Model>(opt.mecanumParameters);
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
    const bool holo=part.kind==MotionKind::HolonomicTwist;
    if(spin) {
      if((!differential&&!mecanumMode)||part.gear!=Gear::None||
         part.lengthMeters!=0||part.curvature!=0||
         std::abs(part.turnRadians)<=1e-12)return Check::Invalid;
    } else if(holo) {
      if(!mecanumMode||part.gear!=Gear::None ||
         !std::isfinite(part.forwardMeters)||
         !std::isfinite(part.lateralMeters)||
         part.lengthMeters<=0||std::abs(part.curvature)>1e-12||
         std::abs(part.lengthMeters-
             std::hypot(part.forwardMeters,part.lateralMeters))>1e-6)
        return Check::Invalid;
    } else if(part.kind!=MotionKind::Translate || part.lengthMeters<=0 ||
              (part.gear!=Gear::Forward&&part.gear!=Gear::Reverse)||
              std::abs(part.turnRadians)>1e-12 ||
              (!differential &&
               std::abs(part.curvature)>1.0/opt.minimumTurningRadiusMeters+1e-10))
      return Check::Invalid;
    if(mecanumMode&&!spin&&!holo)return Check::Invalid;
    const auto sec=spin?makeRotationSection(out.last,part.turnRadians)
                   :holo?makeHolonomicSection(out.last,part.forwardMeters,
                                               part.lateralMeters,part.turnRadians)
                   :makeSection(out.last,part.gear,part.curvature,part.lengthMeters);
    if (!close(sec.end,part.end) ||
        (differential&&!differentialMotionFeasible(req,sec,*diffModel)) ||
        (mecanumMode&&!mecanumMotionFeasible(req,sec,*mecanumModel)))
      return Check::Invalid;
    const double radius=checker.maxBodyRadius();
    const double countD=spin?radius*std::abs(sec.turnRadians)/opt.finalMaxBodyPointMotionMeters
      :holo?std::max(sec.lengthMeters/opt.finalSampleSpacingMeters,
             (sec.lengthMeters+radius*std::abs(sec.turnRadians))/
              opt.finalMaxBodyPointMotionMeters)
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
// Differential refinement must never CREATE an in-place rotation.
// Existing rotations may be removed by straight/CSC replacements; unavoidable
// rotations in the independently verified original remain untouched.
std::vector<std::vector<MotionSection>> differentialShortcuts(
    const Pose2D& from,const Pose2D& to,const RefinementOptions& opt) {
  std::vector<std::vector<MotionSection>> candidates;
  if(opt.useCurvatureBoundedShortcuts)
    candidates=generateCurvatureBoundedShortcuts(
        from,to,opt.differentialCurveRadiusMeters);
  auto straight=straightShortcut(from,to);
  if(!straight.empty())candidates.push_back(std::move(straight));
  return candidates;
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
    if(part.kind==MotionKind::Translate) {
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
       req.options.model!=PlanOptions::Model::DifferentialDrive &&
       req.options.model!=PlanOptions::Model::Mecanum)||
      (req.options.model!=PlanOptions::Model::Ackermann &&
       (req.analyticConnector||req.analyticDistance||req.options.clearanceWeight>0))||
      (req.options.model==PlanOptions::Model::Mecanum &&
       (!std::isfinite(req.options.mecanumStationarySpinPenaltyMeters)||
        req.options.mecanumStationarySpinPenaltyMeters<0))||
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
    ExactCollisionChecker checker(*req.map,req.occupancy,req.footprint,
                                  req.options.footprintInflationMeters);
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
      const auto remaining=options.maxAttempts-result.attempted;
      // A finite window list is only a selection buffer, not evidence that
      // every eligible shortcut was examined. Preserve the historical
      // WorkBudgetExceeded result if unexamined windows remain.
      bool windowsTruncated=false;
      std::vector<std::pair<std::size_t,std::size_t>> windows;
      windows.reserve(remaining);
      const auto add=[&](std::size_t i,std::size_t j) {
        if(j>n||j<=i+1||windows.size()>=remaining)return;
        const auto window=std::pair{i,j};
        if(std::find(windows.begin(),windows.end(),window)==windows.end())
          windows.push_back(window);
      };
      if(req.options.model==PlanOptions::Model::DifferentialDrive||
         req.options.model==PlanOptions::Model::Mecanum) {
        // Spread the bounded attempt budget along long, oscillating routes.
        // Otherwise widest-first only tries windows starting at section zero.
        const std::array<std::size_t,7> widths{
          n,std::min(n,std::size_t{48}),std::min(n,std::size_t{24}),
          std::min(n,std::size_t{12}),std::min(n,std::size_t{6}),
          std::min(n,std::size_t{3}),std::min(n,std::size_t{2})};
        const std::size_t perScale=std::max<std::size_t>(1,remaining/widths.size());
        for(const auto width:widths) {
          if(width<2||windows.size()>=remaining)continue;
          const auto maxStart=n-width;
          const auto stride=std::max<std::size_t>(1,
              (maxStart+perScale-1)/perScale);
          for(std::size_t i=0;i<=maxStart&&windows.size()<remaining;i+=stride)
            add(i,i+width);
          add(maxStart,n);
        }
        // The differential schedule intentionally samples different scales.
        // With >=3 sections, reaching its cap leaves eligible windows.
        if(n>=3 && windows.size()>=remaining)windowsTruncated=true;
      } else {
        // Preserve Ackermann's historical widest-window-first traversal,
        // including its work-limit result when later windows are skipped.
        for(std::size_t i=0;i+1<n&&!windowsTruncated;++i)
          for(std::size_t j=n;j>=i+2;--j) {
            if(windows.size()>=remaining) {windowsTruncated=true;break;}
            add(i,j);
          }
      }
      for(const auto [i,j]:windows) {
          if(result.attempted>=options.maxAttempts ||
             result.exactPoseChecks>=options.maxTotalPoseChecks) {limit=true;break;}
          if(cancelled(req)) {result.status=RefinementStatus::Cancelled;return result;}
          ++result.attempted;
          const auto& before=result.path.sections;
          // Do not splice a new rolling shortcut immediately after/before
          // an existing stationary turn. That creates an apparent sharp
          // junction (a stop remains at the shortcut boundary) even though
          // the candidate is geometrically shorter. Include the spin inside
          // the replaced window instead, so it can be eliminated safely.
          if(req.options.model==PlanOptions::Model::DifferentialDrive &&
             ((i>0 && before[i-1].kind==MotionKind::RotateInPlace)||
              (j<before.size() && before[j].kind==MotionKind::RotateInPlace))) {
            ++result.rejected;
            continue;
          }
          const Pose2D from=before[i].start,to=before[j-1].end;
          std::vector<std::vector<MotionSection>> proposals;
          try {
            if (connector) proposals.push_back(connector(from,to));
            else if(req.options.model==PlanOptions::Model::DifferentialDrive)
              proposals=differentialShortcuts(from,to,options);
            else if(req.options.model==PlanOptions::Model::Mecanum)
              proposals.push_back(holonomicShortcut(req,from,to));
            else if (options.useCurvatureBoundedShortcuts)
              proposals=generateCurvatureBoundedShortcuts(
                  from,to,req.options.minimumTurningRadiusMeters);
            else proposals.push_back(straightShortcut(from,to));
          } catch(const std::exception&) {++result.rejected;continue;}
          // Every geometric proposal gets the SAME independently exact,
          // complete-path validation. Work is bounded by total pose checks;
          // the native CSC provider returns at most eight alternatives.
          for(const auto& replacement:proposals) {
            if(cancelled(req)) {result.status=RefinementStatus::Cancelled;return result;}
            if(replacement.empty()||replacement.size()>options.maxReplacementSections) {
              ++result.rejected;continue;
            }
            if(req.options.model==PlanOptions::Model::DifferentialDrive) {
              // Preserve rolling continuity at both splice endpoints: a
              // shortcut must not add a new stop/reversal or a sharper
              // curvature jump than the original seam already contained.
              // Existing mandatory cusps/spins stay available in the raw
              // validated path when no rolling alternative can remove them.
              const auto& first=replacement.front();
              const auto& last=replacement.back();
              const auto spliceAcceptable=[&](const MotionSection& adjacent,
                                              const MotionSection& oldEdge,
                                              const MotionSection& newEdge) {
                if(adjacent.kind!=MotionKind::Translate ||
                   oldEdge.kind!=MotionKind::Translate ||
                   newEdge.kind!=MotionKind::Translate)
                  return true;
                const bool originalSwitch=adjacent.gear!=oldEdge.gear;
                const bool newSwitch=adjacent.gear!=newEdge.gear;
                if(newSwitch&&!originalSwitch)return false;
                return std::abs(adjacent.curvature-newEdge.curvature)<=
                       std::abs(adjacent.curvature-oldEdge.curvature)+1e-8;
              };
              if((i>0&&!spliceAcceptable(before[i-1],before[i],first))||
                 (j<before.size()&&
                  !spliceAcceptable(before[j],before[j-1],last))) {
                ++result.rejected;continue;
              }
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
            changed=true;
            break;
          }
        if(limit||changed) break;
      }
      if(!changed && windowsTruncated &&
         result.attempted>=options.maxAttempts)
        limit=true;
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
