#include "apps/demo/demo.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace hybrid_astar::demo {
namespace {
namespace art=articulated_steering;
using ackermann::MapSnapshot;

bool finitePose(Pose2D p) {
  return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.yaw)&&
    p.x>=-1000&&p.x<=1000&&p.y>=-1000&&p.y<=1000&&
    std::abs(p.yaw)<=10000;
}
bool validBody(ackermann::Footprint b) {
  return std::isfinite(b.rearX)&&std::isfinite(b.frontX)&&
    std::isfinite(b.halfWidth)&&b.rearX<b.frontX&&b.halfWidth>0.0&&
    b.rearX>=-20.0&&b.frontX<=20.0&&b.halfWidth<=10.0;
}
bool validLength(double d) {return std::isfinite(d)&&d>0.0&&d<=20.0;}
void quoted(std::ostream& os,std::string_view s) {
  os<<'"';
  for(unsigned char c:s) {
    switch(c) {
      case '"':os<<"\\\"";break;
      case '\\':os<<"\\\\";break;
      case '\n':os<<"\\n";break;
      case '\r':os<<"\\r";break;
      case '\t':os<<"\\t";break;
      default:if(c<32)os<<'?';else os<<char(c);
    }
  }
  os<<'"';
}
void rearPoseJson(std::ostream& os,art::RearState r) {
  os<<'['<<r.xr<<','<<r.yr<<','<<r.thetaRear<<']';
}
void rearStateJson(std::ostream& os,art::RearState r) {
  os<<'['<<r.xr<<','<<r.yr<<','<<r.thetaRear<<','<<r.gamma<<']';
}
void footprintJson(std::ostream& os,ackermann::Footprint b) {
  os<<'['<<b.rearX<<','<<b.frontX<<','<<b.halfWidth<<']';
}
const char* heuristicModeName(art::HeuristicMode mode) {
  switch(mode) {
    case art::HeuristicMode::ObstacleGrid:return "grid";
    case art::HeuristicMode::AnalyticOnly:return "analytic";
    case art::HeuristicMode::Combined:return "combined";
    case art::HeuristicMode::CombinedHeadingAware:return "heading";
  }
  throw std::invalid_argument("Invalid articulated heuristic mode");
}
const char* statusName(art::Status s) {
  switch(s) {
    case art::Status::Success:return "Success";
    case art::Status::InvalidRequest:return "InvalidRequest";
    case art::Status::InvalidMap:return "InvalidMap";
    case art::Status::RejectedUnknownMap:return "RejectedUnknownMap";
    case art::Status::StartInCollision:return "StartInCollision";
    case art::Status::GoalInCollision:return "GoalInCollision";
    case art::Status::NoPath:return "NoPath";
    case art::Status::BudgetExceeded:return "BudgetExceeded";
    case art::Status::Cancelled:return "Cancelled";
    case art::Status::ValidationIncomplete:return "ValidationIncomplete";
    case art::Status::StrictValidationFailed:return "StrictValidationFailed";
  }
  return "InvalidRequest";
}
void pathJson(std::ostream& os,const art::Result& r) {
  const bool success=r.status==art::Status::Success&&
                     r.independentlySampledBodyValidated;
  double totalLength=0.0,reverseLength=0.0;
  for(const auto& motion:r.motions) {
    const double distance=std::abs(motion.control.frontSpeedMetersPerSecond)*
                          motion.durationSeconds;
    totalLength+=distance;
    if(motion.control.frontSpeedMetersPerSecond<0.0)reverseLength+=distance;
  }
  os<<"{\"status\":";quoted(os,statusName(r.status));
  os<<",\"validated\":"<<(success?"true":"false")
    <<",\"collisionEvidence\":";
  quoted(os,success?"TwoCarriageIndependentAdaptiveSampled":"None");
  os<<",\"actuatorFeasible\":false"
    <<",\"cost\":"<<r.costMeters
    <<",\"length\":"<<totalLength
    <<",\"reverseLength\":"<<reverseLength
    <<",\"expanded\":"<<r.diagnostics.expanded
    <<",\"generated\":"<<r.diagnostics.generated
    <<",\"prunedByKey\":"<<r.diagnostics.pruned
    <<",\"strictGoalCandidates\":"<<r.diagnostics.strictGoalCandidates
    <<",\"strictRejected\":"<<r.diagnostics.strictRejected
    <<",\"goalConnectorAttempts\":"<<r.diagnostics.goalConnectorAttempts
    <<",\"goalConnectorCandidates\":"<<r.diagnostics.goalConnectorCandidates
    <<",\"goalConnectorValidated\":"<<r.diagnostics.goalConnectorValidated
    <<",\"goalConnectorRejectedCollision\":"<<r.diagnostics.goalConnectorRejectedCollision
    <<",\"goalConnectorRejectedTerminal\":"<<r.diagnostics.goalConnectorRejectedTerminal
    <<",\"goalConnectorRejectedStrict\":"<<r.diagnostics.goalConnectorRejectedStrict
    <<",\"goalConnectorEvaluations\":"<<r.diagnostics.goalConnectorEvaluations
    <<",\"goalConnectorBodyChecks\":"<<r.diagnostics.goalConnectorBodyChecks
    <<",\"goalConnectorGlobalUsed\":"<<r.diagnostics.goalConnectorGlobalUsed
    <<",\"goalConnectorNearUsed\":"<<r.diagnostics.goalConnectorNearUsed
    <<",\"goalConnectorDeferred\":"<<r.diagnostics.goalConnectorDeferred
    <<",\"goalConnectorCoupledCandidates\":"<<r.diagnostics.goalConnectorCoupledCandidates
    <<",\"goalConnectorClearancePenalized\":"<<r.diagnostics.goalConnectorClearancePenalized
    <<",\"goalConnectorFirstExpansion\":"<<r.diagnostics.goalConnectorFirstExpansion
    <<",\"goalConnectorLastExpansion\":"<<r.diagnostics.goalConnectorLastExpansion
    <<",\"goalConnectorClosestXYMeters\":"<<r.diagnostics.goalConnectorClosestXYMeters
    <<",\"anytimeFirstSolutionExpansion\":"<<r.diagnostics.anytimeFirstSolutionExpansion
    <<",\"anytimeExtraExpansions\":"<<r.diagnostics.anytimeExtraExpansions
    <<",\"anytimeExtraConnectorAttempts\":"<<r.diagnostics.anytimeExtraConnectorAttempts
    <<",\"anytimeValidatedCandidates\":"<<r.diagnostics.anytimeValidatedCandidates
    <<",\"anytimeImprovements\":"<<r.diagnostics.anytimeImprovements
    <<",\"anytimeCostBoundPrunes\":"<<r.diagnostics.anytimeCostBoundPrunes
    <<",\"anytimeFirstCostMeters\":"<<r.diagnostics.anytimeFirstCostMeters
    <<",\"anytimeBestCostMeters\":"<<r.diagnostics.anytimeBestCostMeters
    <<",\"searchBodyChecks\":"<<r.diagnostics.searchBodyChecks
    <<",\"precomputedAnalyticDriveTemplates\":"<<r.diagnostics.precomputedAnalyticDriveTemplates
    <<",\"precomputedStationaryRk4Templates\":"<<r.diagnostics.precomputedStationaryRk4Templates
    <<",\"precomputedCoupledRk4Templates\":"<<r.diagnostics.precomputedCoupledRk4Templates
    <<",\"appliedCoupledTemplates\":"<<r.diagnostics.appliedCoupledTemplates
    <<",\"searchSweptAabbAccepted\":"<<r.diagnostics.searchSweptAabbAccepted
    <<",\"searchSweptAabbFallback\":"<<r.diagnostics.searchSweptAabbFallback
    <<",\"precomputedSampleCount\":"<<r.diagnostics.precomputedSampleCount
    <<",\"appliedPrecomputedTemplates\":"<<r.diagnostics.appliedPrecomputedTemplates
    <<",\"gridSettlements\":"<<r.diagnostics.gridSettlements
    <<",\"gridFallbacks\":"<<r.diagnostics.gridFallbacks
    <<",\"gridWorkBudgetFallbacks\":"<<r.diagnostics.gridWorkBudgetFallbacks
    <<",\"finalChecks\":"<<r.diagnostics.finalBodyChecks
    <<",\"goalXYErrorMeters\":"<<r.goalXYErrorMeters
    <<",\"goalYawErrorRadians\":"<<r.goalYawErrorRadians
    <<",\"goalGammaErrorRadians\":"<<r.goalGammaErrorRadians
    <<",\"sections\":[";
  bool first=true;
  for(const auto& motion:r.motions) {
    if(!first)os<<',';
    first=false;
    const double v=motion.control.frontSpeedMetersPerSecond;
    const double u=motion.control.gammaRateRadiansPerSecond;
    const char* gear=v<0.0?"reverse":v>0.0?"forward":"articulation";
    os<<"{\"gear\":";quoted(os,gear);
    os<<",\"kind\":";
    quoted(os,v==0.0?"articulation":u!=0.0?"coupled":"translation");
    os<<",\"length\":"<<std::abs(v)*motion.durationSeconds
      <<",\"durationSeconds\":"<<motion.durationSeconds
      <<",\"frontSpeed\":"<<v
      <<",\"gammaRate\":"<<u
      <<",\"turnRadians\":"<<motion.end.thetaRear-motion.start.thetaRear
      <<",\"curvature\":0"
      <<",\"points\":[";
    // JSON visualization points are deliberately decimated; collision
    // evidence comes from the complete independently validated samples.
    std::vector<std::size_t> indices;
    const auto& samples=motion.samples;
    if(!samples.empty()) {
      const std::size_t step=std::max<std::size_t>(1,(samples.size()-1+199)/200);
      for(std::size_t i=0;i+1<samples.size();i+=step)indices.push_back(i);
      indices.push_back(samples.size()-1);
    }
    for(std::size_t i=0;i<indices.size();++i) {
      if(i)os<<',';
      rearPoseJson(os,samples[indices[i]].rear);
    }
    os<<"],\"articulatedPoints\":[";
    for(std::size_t i=0;i<indices.size();++i) {
      if(i)os<<',';
      rearStateJson(os,samples[indices[i]].rear);
    }
    os<<"]}";
  }
  os<<"]}";
}
} // namespace

ArticulatedOutcome runArticulated(const Options& opt) {
  if(opt.model!="articulated")
    throw std::invalid_argument("Expected articulated planning model");
  if(opt.ompl||opt.refine||opt.curvedRefinement||opt.steeringSmoothness||
     opt.rsDistanceHeuristic||opt.improve)
    throw std::invalid_argument(
      "Articulated model does not support rigid-body RS, smoothing or improvement options");
  ArticulatedOutcome out;
  Pose2D start,goal;
  if(opt.scenario=="custom" || opt.scenario=="nav2_warehouse_full" ||
     opt.scenario=="nav2_warehouse_344128") {
    const auto map=opt.scenario=="custom"?opt.customMap:
      opt.scenario=="nav2_warehouse_full"?opt.nav2WarehouseFullMap:
      opt.nav2Warehouse344128Map;
    if(!map)throw std::invalid_argument("Selected map is unavailable");
    out.map=map;
    const auto toWorld=[&](double x,double y) {
      const double c=std::cos(map->origin.yaw),sn=std::sin(map->origin.yaw);
      return Pose2D{map->origin.x+c*x-sn*y,
                    map->origin.y+sn*x+c*y,map->origin.yaw};
    };
    start=toWorld(map->width*map->resolutionMeters*.2,
                  map->height*map->resolutionMeters*.5);
    goal=toWorld(map->width*map->resolutionMeters*.8,
                 map->height*map->resolutionMeters*.5);
  } else {
    const auto base=makeRequest(opt.scenario);
    out.map=base.map;
    start=base.start;goal=base.goal;
  }
  out.map=applyCellEdits(out.map,opt.edits,opt.editGeneration);
  if(opt.start)start=*opt.start;
  if(opt.goal)goal=*opt.goal;
  if(!finitePose(start)||!finitePose(goal))
    throw std::invalid_argument("Articulated rear base_link pose is invalid");
  constexpr double rad=std::numbers::pi_v<double>/180.0;
  if(!std::isfinite(opt.startGammaDegrees)||!std::isfinite(opt.goalGammaDegrees)||
     std::abs(opt.startGammaDegrees)>180||std::abs(opt.goalGammaDegrees)>180)
    throw std::invalid_argument("Start/goal articulation angles must be within +/-180 degrees");
  if(!validLength(opt.LfMeters)||!validLength(opt.LrMeters)||
     !validBody(opt.articulatedFront)||!validBody(opt.articulatedRear))
    throw std::invalid_argument("Articulated link lengths or carriage footprint values are invalid");
  if(!std::isfinite(opt.footprintInflationMeters)||
     opt.footprintInflationMeters<0.0||opt.footprintInflationMeters>10.0)
    throw std::invalid_argument("Inflation must be between 0 and 10 m");
  // Positive lower bound over the *entire* admitted joint range. The
  // planner's conservative collision sampling uses this bound; setting it
  // arbitrarily close to zero would unnecessarily exhaust every primitive.
  constexpr double jointLimit=1.2;
  const double minimumD=opt.LrMeters+opt.LfMeters*std::cos(jointLimit);
  out.parameters={
    .LfMeters=opt.LfMeters,.LrMeters=opt.LrMeters,
    .minGammaRadians=-jointLimit,.maxGammaRadians=jointLimit,
    .maxAbsFrontSpeedMetersPerSecond=1.0,
    .maxAbsGammaRateRadiansPerSecond=0.8,
    .minDenominatorMeters=0.95*minimumD
  };
  // These are example-only joint limits; production values must be
  // calibrated to the actual robot before any real-robot deployment.
  const art::Model model(out.parameters);
  out.start={start.x,start.y,start.yaw,opt.startGammaDegrees*rad};
  out.goal={goal.x,goal.y,goal.yaw,opt.goalGammaDegrees*rad};
  (void)model.fromRear(out.start);
  (void)model.fromRear(out.goal);
  out.bodies.front=opt.articulatedFront;
  out.bodies.rear=opt.articulatedRear;
  out.bodies.inflationMeters=opt.footprintInflationEnabled?
                             opt.footprintInflationMeters:0.0;
  // Honor the SAME 1M expansion ceiling advertised by the Planning Lab UI.
  // The prior 50k / 200k special-case caps were temporary and have been
  // removed now that articulated successor templates are precomputed.
  // Four generated labels per permitted expansion is an independent
  // search/memory budget; high budgets can still use substantial RAM.
  constexpr std::size_t kMaxUiExpansions=1'000'000;
  out.settings.maxExpansions=std::min(
      opt.maxExpansions.value_or(kMaxUiExpansions),kMaxUiExpansions);
  out.settings.maxGenerated=out.settings.maxExpansions*4;
  if(!std::isfinite(opt.articulatedHeuristicWeight)||
     opt.articulatedHeuristicWeight<=0.0||
     opt.articulatedHeuristicWeight>3.0)
    throw std::invalid_argument("Articulated heuristic weight must be in (0, 3]");
  (void)heuristicModeName(opt.articulatedHeuristicMode); // reject invalid enum values
  out.settings.heuristicMode=opt.articulatedHeuristicMode;
  out.settings.heuristicWeight=opt.articulatedHeuristicWeight;
  if(!std::isfinite(opt.articulatedHeadingGuidanceWeight)||
     opt.articulatedHeadingGuidanceWeight<0.0||
     opt.articulatedHeadingGuidanceWeight>3.0)
    throw std::invalid_argument("Articulated heading guidance weight must be in [0, 3]");
  out.settings.goalHeadingGuidanceWeight=opt.articulatedHeadingGuidanceWeight;
  out.settings.enableCoupledMotion=opt.articulatedEnableCoupledMotion;
  out.settings.useSweptAabbPrefilter=opt.articulatedUseSweptAabbPrefilter;
  out.settings.enableGoalConnector=opt.articulatedEnableGoalConnector;
  if(opt.articulatedEnableAnytimeGoalRefinement&&
     !opt.articulatedEnableGoalConnector)
    throw std::invalid_argument("Articulated anytime refinement requires native goal connector");
  if(opt.articulatedAnytimeExtraExpansions<1||
     opt.articulatedAnytimeExtraExpansions>100000)
    throw std::invalid_argument("Articulated anytime extra expansions must be in [1, 100000]");
  out.settings.enableAnytimeGoalRefinement=
      opt.articulatedEnableAnytimeGoalRefinement;
  out.settings.anytimeExtraExpansions=opt.articulatedAnytimeExtraExpansions;
  art::Request request;
  request.map=out.map;
  request.parameters=out.parameters;
  request.bodies=out.bodies;
  request.start=out.start;request.goal=out.goal;
  request.options=out.settings;
  const auto t0=std::chrono::steady_clock::now();
  out.result=art::plan(request);
  out.elapsedMilliseconds=
      std::chrono::duration<double,std::milli>(
        std::chrono::steady_clock::now()-t0).count();
  return out;
}
std::string articulatedToJson(const ArticulatedOutcome& out) {
  if(!out.map)throw std::invalid_argument("Missing articulated map snapshot");
  const auto& m=*out.map;
  std::ostringstream os;
  os<<std::setprecision(12);
  os<<"{\"model\":\"articulated\",\"width\":"<<m.width
    <<",\"height\":"<<m.height<<",\"resolution\":"<<m.resolutionMeters
    <<",\"revision\":"<<m.revision<<",\"frameId\":";
  quoted(os,m.frameId);
  os<<",\"sourceId\":";
  quoted(os,m.sourceId);
  os<<",\"origin\":["<<m.origin.x<<','<<m.origin.y<<','<<m.origin.yaw<<']'
    <<",\"footprintInflationMeters\":"<<out.bodies.inflationMeters
    <<",\"articulatedGeometry\":{\"Lf\":"<<out.parameters.LfMeters
    <<",\"Lr\":"<<out.parameters.LrMeters<<",\"front\":";
  footprintJson(os,out.bodies.front);
  os<<",\"rear\":";
  footprintJson(os,out.bodies.rear);
  os<<",\"minGamma\":"<<out.parameters.minGammaRadians
    <<",\"maxGamma\":"<<out.parameters.maxGammaRadians<<"}"
    <<",\"start\":";
  rearPoseJson(os,out.start);
  os<<",\"goal\":";
  rearPoseJson(os,out.goal);
  os<<",\"startArticulated\":";
  rearStateJson(os,out.start);
  os<<",\"goalArticulated\":";
  rearStateJson(os,out.goal);
  os<<",\"milliseconds\":"<<out.elapsedMilliseconds
    <<",\"limits\":{\"maxExpansions\":"<<out.settings.maxExpansions
    <<",\"maxGeneratedNodes\":"<<out.settings.maxGenerated
    <<",\"maxFinalPoseChecks\":"<<out.settings.maxTotalFinalBodyChecks
    <<"},\"articulatedSearch\":{\"heuristicMode\":";
  quoted(os,heuristicModeName(out.settings.heuristicMode));
  os<<",\"heuristicWeight\":"<<out.settings.heuristicWeight
    <<",\"headingGuidanceWeight\":"<<out.settings.goalHeadingGuidanceWeight
    <<",\"enableCoupledMotion\":"<<(out.settings.enableCoupledMotion?"true":"false")
    <<",\"useSweptAabbPrefilter\":"<<(out.settings.useSweptAabbPrefilter?"true":"false")
    <<",\"enableGoalConnector\":"<<(out.settings.enableGoalConnector?"true":"false")
    <<",\"enableAnytimeGoalRefinement\":"<<(out.settings.enableAnytimeGoalRefinement?"true":"false")
    <<",\"anytimeExtraExpansions\":"<<out.settings.anytimeExtraExpansions
    <<"},\"mapRuns\":[";
  bool first=true;
  for(std::size_t y=0;y<m.height;++y) {
    for(std::size_t x=0;x<m.width;) {
      if(m.cells[y*m.width+x]==0){++x;continue;}
      const auto cell=m.cells[y*m.width+x];
      const auto begin=x;
      while(x<m.width&&m.cells[y*m.width+x]==cell)++x;
      if(!first)os<<',';
      first=false;
      os<<'['<<begin<<','<<y<<','<<(x-begin)<<','<<int(cell)<<']';
    }
  }
  os<<"],\"original\":";
  pathJson(os,out.result);
  os<<",\"refinement\":null}";
  return os.str();
}
} // namespace hybrid_astar::demo
