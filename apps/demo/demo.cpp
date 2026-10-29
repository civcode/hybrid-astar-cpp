#include "apps/demo/demo.hpp"
#ifdef HYBRID_ASTAR_DEMO_WITH_OMPL
#include "hybrid_astar/steering/reeds_shepp_evaluator.hpp"
#endif
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace hybrid_astar::demo {
namespace {
using namespace ackermann;
void rectangle(MapSnapshot& m,int x0,int y0,int x1,int y1,int value=100) {
  for(int y=std::max(0,y0);y<std::min(int(m.height),y1);++y)
    for(int x=std::max(0,x0);x<std::min(int(m.width),x1);++x)
      m.cells[std::size_t(y)*m.width+std::size_t(x)]=std::int8_t(value);
}
bool finite(Pose2D p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.yaw);}
std::string quoted(std::string_view s) {
  std::string v="\"";
  for(unsigned char c:s) {
    switch(c){case '"':v+="\\\"";break;case '\\':v+="\\\\";break;
      case '\n':v+="\\n";break;case '\r':v+="\\r";break;case '\t':v+="\\t";break;
      default: if(c<32){v+='?';}else v+=char(c);}
  }
  return v+'"';
}
const char* statusName(PlanStatus s) {
  switch(s){case PlanStatus::Success:return "Success";case PlanStatus::NoPath:return "NoPath";
  case PlanStatus::InvalidRequest:return "InvalidRequest";case PlanStatus::InvalidMap:return "InvalidMap";
  case PlanStatus::RejectedUnknownMap:return "RejectedUnknownMap";
  case PlanStatus::StartInCollision:return "StartInCollision";
  case PlanStatus::BudgetExceeded:return "BudgetExceeded";case PlanStatus::Cancelled:return "Cancelled";
  case PlanStatus::ValidationIncomplete:return "ValidationIncomplete";
  case PlanStatus::StrictValidationFailed:return "StrictValidationFailed";
  default:return "InternalError";}
}
const char* refinementName(RefinementStatus s) {
  switch(s) {case RefinementStatus::Improved:return "Improved";
  case RefinementStatus::Unchanged:return "Unchanged";
  case RefinementStatus::OriginalNotVerified:return "OriginalNotVerified";
  case RefinementStatus::InvalidInput:return "InvalidInput";
  case RefinementStatus::WorkBudgetExceeded:return "WorkBudgetExceeded";
  default:return "Cancelled";}
}
const char* qualityName(SolutionQuality q) {
  switch(q){
    case SolutionQuality::FeasibleFirst:return "FeasibleFirst";
    case SolutionQuality::FeasibleAfterImprovement:return "FeasibleAfterImprovement";
    case SolutionQuality::FeasibleAfterRefinement:return "FeasibleAfterRefinement";
  }
  return "Unknown";
}
const char* terminationName(TerminationReason reason) {
  switch(reason) {
    case TerminationReason::None:return "None";
    case TerminationReason::FirstValidated:return "FirstValidated";
    case TerminationReason::ImprovementBudgetExhausted:return "ImprovementBudgetExhausted";
    case TerminationReason::SearchExhausted:return "SearchExhausted";
    case TerminationReason::WorkBudgetExceeded:return "WorkBudgetExceeded";
    case TerminationReason::CancelledAfterIncumbent:return "CancelledAfterIncumbent";
    case TerminationReason::ValidationBudgetExhausted:return "ValidationBudgetExhausted";
  }
  return "Unknown";
}
void poseJson(std::ostream& os,Pose2D p) {
  os<<"["<<p.x<<","<<p.y<<","<<p.yaw<<"]";
}
void pathJson(std::ostream& os,const PlanResult& path) {
  os<<"{\"status\":"<<quoted(statusName(path.status))
    <<",\"mapRevision\":"<<path.mapRevision
    <<",\"mapFrameId\":"<<quoted(path.mapFrameId)
    <<",\"mapSourceId\":"<<quoted(path.mapSourceId)
    <<",\"solutionQuality\":"<<quoted(qualityName(path.solutionQuality))
    <<",\"terminationReason\":"<<quoted(terminationName(path.terminationReason))
    <<",\"collisionEvidence\":"<<quoted(path.collisionEvidence==CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled?"FullFootprintBodyMotionAdaptiveSampled":"None")
    <<",\"actuatorFeasible\":"<<(path.trajectoryActuatorFeasible?"true":"false")
    <<",\"goalXYErrorMeters\":"<<path.goalXYErrorMeters
    <<",\"goalYawErrorRadians\":"<<path.goalYawErrorRadians
    <<",\"cost\":"<<path.weightedCostMeters
    <<",\"length\":"<<path.geometricLengthMeters
    <<",\"reverseLength\":"<<path.reverseLengthMeters
    <<",\"expanded\":"<<path.diagnostics.expanded
    <<",\"generated\":"<<path.diagnostics.generated
    <<",\"prunedByKey\":"<<path.diagnostics.prunedByKey
    <<",\"gridSettlements\":"<<path.diagnostics.gridSettlements
    <<",\"gridFallbacks\":"<<path.diagnostics.gridFallbacks
    <<",\"terminalAttempts\":"<<path.diagnostics.terminalConnectionAttempts
    <<",\"terminalValidated\":"<<path.diagnostics.terminalValidated
    <<",\"terminalRejected\":"<<path.diagnostics.terminalRejected
    <<",\"strictGoalCandidates\":"<<path.diagnostics.strictGoalCandidates
    <<",\"strictRejectedCollision\":"<<path.diagnostics.strictRejectedCollision
    <<",\"strictRejectedContinuity\":"<<path.diagnostics.strictRejectedContinuity
    <<",\"strictRejectedGoalMismatch\":"<<path.diagnostics.strictRejectedGoalMismatch
    <<",\"discPrefilterCountUsed\":"<<path.diagnostics.discPrefilterCountUsed
    <<",\"emptyRegionPrefixSumUsed\":"<<(path.diagnostics.emptyRegionPrefixSumUsed?"true":"false")
    <<",\"spinShortcutsAttempted\":"<<path.diagnostics.mecanumSpinShortcutsAttempted
    <<",\"spinShortcutsAccepted\":"<<path.diagnostics.mecanumSpinShortcutsAccepted
    <<",\"finalChecks\":"<<path.diagnostics.finalPoseChecks
    <<",\"heuristicCertifiedAdmissible\":"<<(path.heuristicCertifiedAdmissible?"true":"false")
    <<",\"validated\":"<<(path.status==PlanStatus::Success?"true":"false")
    <<",\"sections\":[";
  for(std::size_t i=0;i<path.sections.size();++i) {
    const auto& section=path.sections[i];
    if(i) os<<',';
    const bool spin=section.kind==MotionKind::RotateInPlace;
    const bool holonomic=section.kind==MotionKind::HolonomicTwist;
    const double extent=section.progressExtent();
    // Rotation is parameterized by radians; drawing must retain heading
    // samples even though all reference positions coincide.
    // Combined holonomic heading motion needs angular display samples too.
    const std::size_t steps=std::max<std::size_t>(1,static_cast<std::size_t>(
        std::ceil(std::max(extent/(spin ? .07 : .13),
                      holonomic?std::abs(section.turnRadians)/.07:0.0))));
    // Visualization samples are separate from strict collision evidence.
    os<<"{\"gear\":"<<quoted(spin?"rotate":holonomic?"holonomic":section.gear==Gear::Reverse?"reverse":"forward")
      <<",\"kind\":"<<quoted(spin?"rotation":holonomic?"holonomic":"translation")
      <<",\"forwardMeters\":"<<section.forwardMeters
      <<",\"lateralMeters\":"<<section.lateralMeters
      <<",\"turnRadians\":"<<section.turnRadians
      <<",\"curvature\":"<<section.curvature<<",\"length\":"<<section.lengthMeters<<",\"points\":[";
    for(std::size_t j=0;j<=steps;++j) {
      if(j)os<<',';
      const auto p=section.evaluate(j==steps?extent:extent*double(j)/double(steps));
      poseJson(os,p);
    }
    os<<"]}";
  }
  os<<"]}";
}
} // namespace

ackermann::PlanRequest makeRequest(std::string_view name) {
  if(name!="open"&&name!="reverse"&&name!="parking"&&name!="warehouse"&&name!="blocked")
    throw std::invalid_argument("Unknown scenario (open, reverse, parking, warehouse, blocked)");
  auto map=std::make_shared<MapSnapshot>();
  map->width=200;map->height=160;map->resolutionMeters=.25;
  map->origin={0,0,0};map->frameId="demo/map";map->sourceId="built-in";
  map->revision=1;map->cells.assign(map->width*map->height,0);
  PlanRequest r;r.map=map;r.start={10,15,0};r.goal={17,15,0};
  if(name=="reverse")r.goal={6,15,0};
  if(name=="parking")r.goal={8,16,0};
  if(name=="warehouse") {
    rectangle(*map,48,12,53,48);rectangle(*map,48,92,53,145);
    rectangle(*map,94,0,99,46);rectangle(*map,94,100,99,160);
    rectangle(*map,141,12,146,55);rectangle(*map,141,104,146,145);
    r.start={8,20,0};r.goal={43,20,0};
    // Waypoint-free planning must find its own way around warehouse obstacles.
  }
  if(name=="blocked") {
    rectangle(*map,94,0,99,160);
    r.start={10,20,0};r.goal={35,20,0};
  }
  return r;
}

Outcome run(const Options& opt) {
  Outcome out;
  if(opt.scenario=="custom" || opt.scenario=="nav2_warehouse_full" ||
     opt.scenario=="nav2_warehouse_344128") {
    const auto selected=opt.scenario=="custom"?opt.customMap:
        opt.scenario=="nav2_warehouse_full"?opt.nav2WarehouseFullMap:
        opt.nav2Warehouse344128Map;
    if(!selected)throw std::invalid_argument("Selected PGM map is unavailable; run tools/prepare_nav2_warehouse.py");
    const auto& m=*selected;
    out.request.map=selected;
    const auto toWorld=[&](double x,double y) {
      const double c=std::cos(m.origin.yaw),sn=std::sin(m.origin.yaw);
      return Pose2D{m.origin.x+c*x-sn*y,m.origin.y+sn*x+c*y,m.origin.yaw};
    };
    out.request.start=toWorld(m.width*m.resolutionMeters*.2,m.height*m.resolutionMeters*.5);
    out.request.goal=toWorld(m.width*m.resolutionMeters*.8,m.height*m.resolutionMeters*.5);
  } else out.request=makeRequest(opt.scenario);
  // The map is never modified in-place. All search, strict validation and
  // optional refinement use the same request-local edited revision.
  out.request.map=applyCellEdits(out.request.map,opt.edits,opt.editGeneration);
  if(opt.start)out.request.start=*opt.start;
  if(opt.goal)out.request.goal=*opt.goal;
  for(auto p:{out.request.start,out.request.goal})
    if(!finite(p)||p.x < -1000||p.x > 1000||p.y < -1000||p.y > 1000||
       std::abs(p.yaw)>10000)throw std::invalid_argument("Pose is not finite or within demo bounds");
  if(opt.model=="differential" || opt.model=="diff") {
    if(opt.ompl)
      throw std::invalid_argument("Reeds-Shepp analytic steering is unavailable for differential drive");
    out.request.options.model=PlanOptions::Model::DifferentialDrive;
    out.request.footprint={-.35,.35,.25}; // compact robot body, reference center
    out.request.options.rotationCostMetersPerRadian=.4;
  } else if(opt.model=="mecanum" || opt.model=="holonomic") {
    if(opt.ompl || opt.curvedRefinement)
      throw std::invalid_argument("Reeds-Shepp and curved CSC are unavailable for mecanum");
    out.request.options.model=PlanOptions::Model::Mecanum;
    out.request.footprint={-.35,.35,.25};
    out.request.options.rotationCostMetersPerRadian=.4;
  } else if(opt.model!="ackermann")
    throw std::invalid_argument("Unknown model (ackermann, differential or mecanum)");
  if(opt.rsDistanceHeuristic && (!opt.ompl || !opt.performanceHeuristic || opt.model!="ackermann"))
    throw std::invalid_argument("RS distance guidance requires Ackermann, the RS connector and obstacle-grid heuristic");
  if(!std::isfinite(opt.steeringWeightMetersSquared) ||
     opt.steeringWeightMetersSquared<0 || opt.steeringWeightMetersSquared>100 ||
     (opt.steeringSmoothness && opt.steeringWeightMetersSquared==0))
    throw std::invalid_argument("Enabled steering smoothness weight must be greater than 0 and at most 100 m^2");
  if(opt.steeringSmoothness && (opt.model!="ackermann" || !opt.refine))
    throw std::invalid_argument("Steering smoothness requires Ackermann and strict post-path refinement");
  if(opt.maxAcceptedReplacements) {
    const auto count=*opt.maxAcceptedReplacements;
    if(count!=8 && count!=12 && count!=16)
      throw std::invalid_argument("Accepted replacements must be 8, 12 or 16");
    if(!opt.refine)
      throw std::invalid_argument("Accepted-replacement override requires strict post-path refinement");
    // Record the selected cap even if search finds no valid path to refine.
    out.refinementSettings.maxAccepted=count;
  }
  // Every model can use the selected acceptance cap. Its search-attempt and
  // exact collision-check limits remain model-specific and unchanged.
  if(out.request.options.model==PlanOptions::Model::Mecanum) {
    out.refinementSettings.maxAttempts=220;
    out.refinementSettings.maxTotalPoseChecks=240000;
  }
  out.appliedSteeringWeightMetersSquared=
      opt.steeringSmoothness?opt.steeringWeightMetersSquared:0.0;
  if(!std::isfinite(opt.footprintInflationMeters)||
     opt.footprintInflationMeters<0||opt.footprintInflationMeters>10)
    throw std::invalid_argument("Footprint inflation must be between 0 and 10 m");
  out.request.options.footprintInflationMeters=
      opt.footprintInflationEnabled?opt.footprintInflationMeters:0.0;
  out.rsDistanceHeuristicEnabled=opt.rsDistanceHeuristic;
  out.request.options.heuristicMode=opt.performanceHeuristic
      ?HeuristicMode::PerformanceReedsSheppAndObstacleGrid
      :HeuristicMode::CertifiedRegionEuclidean;
  if(opt.improve) {
    out.request.options.solutionPolicy=SolutionPolicy::ImproveIncumbent;
    out.request.options.maxExtraExpansionsAfterFirst=700;
  }
  // Explicit work budget. The generated-node ceiling scales with it so the
  // frontier limit never silently becomes the effective budget.
  if(opt.maxExpansions) {
    out.request.options.maxExpansions=*opt.maxExpansions;
    out.request.options.maxGeneratedNodes=
      std::max<std::size_t>(out.request.options.maxGeneratedNodes,*opt.maxExpansions*4);
  }
  const auto t0=std::chrono::steady_clock::now();
#ifdef HYBRID_ASTAR_DEMO_WITH_OMPL
  if(opt.ompl) {
    if(opt.refine) {
      ackermann::RefinementOptions settings=out.refinementSettings; // Ackermann: 160 / 8|12|16 / 200k
      settings.useCurvatureBoundedShortcuts=opt.curvedRefinement;
      settings.curvatureVariationWeightMetersSquared=
          out.appliedSteeringWeightMetersSquared;
      out.refinementSettings=settings;
      const auto combined=steering::planAndRefineWithReedsShepp(out.request,settings,opt.rsDistanceHeuristic);
      out.original=combined.original;
      out.refinement=combined.refinement;
    } else out.original=steering::planWithReedsShepp(out.request,opt.rsDistanceHeuristic);
  } else
#endif
  {
    if(opt.ompl)throw std::invalid_argument("OMPL adapter is not enabled in this demo build");
    out.original=ackermann::plan(out.request);
    if(opt.refine&&out.original.status==PlanStatus::Success) {
      RefinementOptions settings=out.refinementSettings;
      if(out.request.options.model==PlanOptions::Model::DifferentialDrive) {
        settings.differentialCurveRadiusMeters=.75;
        settings.differentialRadiusVariants=4; // opt-in Planning Lab multi-radius
      }
      // Accepted cap: 8, 12 or 16 for every model. Ackermann and differential
      // retain 160 / 200k; mecanum retains 220 / 240k (attempts / checks).
      settings.useCurvatureBoundedShortcuts=opt.curvedRefinement;
      settings.curvatureVariationWeightMetersSquared=
          out.appliedSteeringWeightMetersSquared;
      out.refinementSettings=settings;
      out.refinement=refinePath(out.request,out.original,{},settings);
    }
  }
  out.elapsedMilliseconds=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t0).count();
  return out;
}
std::string toJson(const Outcome& out) {
  const auto& m=*out.request.map;
  std::ostringstream os;os<<std::setprecision(12);
  os<<"{\"model\":"<<quoted(out.request.options.model==PlanOptions::Model::DifferentialDrive?"differential":
                     out.request.options.model==PlanOptions::Model::Mecanum?"mecanum":"ackermann")
    <<",\"footprint\":["<<out.request.footprint.rearX<<','<<out.request.footprint.frontX
    <<','<<out.request.footprint.halfWidth<<"],\"footprintInflationMeters\":"
    <<out.request.options.footprintInflationMeters<<",\"width\":"<<m.width<<",\"height\":"<<m.height<<",\"resolution\":"<<m.resolutionMeters
    <<",\"frameId\":"<<quoted(m.frameId)<<",\"sourceId\":"<<quoted(m.sourceId)<<",\"revision\":"<<m.revision
    <<",\"origin\":";poseJson(os,m.origin);
  os<<",\"rsDistanceHeuristic\":"<<(out.rsDistanceHeuristicEnabled?"true":"false")
    <<",\"refinementOptions\":{\"maxAttempts\":"<<out.refinementSettings.maxAttempts
    <<",\"maxAccepted\":"<<out.refinementSettings.maxAccepted
    <<",\"maxTotalPoseChecks\":"<<out.refinementSettings.maxTotalPoseChecks
    <<",\"steeringSmoothness\":"<<(out.appliedSteeringWeightMetersSquared>0?"true":"false")
    <<",\"steeringWeightMetersSquared\":"<<out.appliedSteeringWeightMetersSquared<<"}";
  os<<",\"start\":";poseJson(os,out.request.start);
  os<<",\"goal\":";poseJson(os,out.request.goal);
  os<<",\"milliseconds\":"<<out.elapsedMilliseconds
    <<",\"limits\":{\"maxExpansions\":"<<out.request.options.maxExpansions
    <<",\"maxGeneratedNodes\":"<<out.request.options.maxGeneratedNodes
    <<",\"maxFinalPoseChecks\":"<<out.request.options.maxTotalFinalPoseChecks<<"}";
  os<<",\"mapRuns\":[";
  bool first=true;
  for(std::size_t y=0;y<m.height;++y) {
    for(std::size_t x=0;x<m.width;) {
      if(m.cells[y*m.width+x]==0){++x;continue;}
      const auto cell=m.cells[y*m.width+x];const auto begin=x;
      while(x<m.width&&m.cells[y*m.width+x]==cell)++x;
      if(!first)os<<',';first=false;
      os<<'['<<begin<<','<<y<<','<<(x-begin)<<','<<int(cell)<<']';
    }
  }
  os<<"],\"original\":";pathJson(os,out.original);
  os<<",\"refinement\":";
  if(!out.refinement)os<<"null";
  else {
    os<<"{\"status\":"<<quoted(refinementName(out.refinement->status))
      <<",\"accepted\":"<<out.refinement->accepted<<",\"attempted\":"<<out.refinement->attempted
      <<",\"path\":";
    if(out.refinement->path.status==PlanStatus::Success)pathJson(os,out.refinement->path);
    else os<<"null";
    os<<'}';
  }
  os<<'}';return os.str();
}
std::string capabilitiesJson(const ackermann::MapSnapshot* customMap,
                             const ackermann::MapSnapshot* nav2WarehouseFull,
                             const ackermann::MapSnapshot* nav2Warehouse344128) {
#ifdef HYBRID_ASTAR_DEMO_WITH_OMPL
  const char* ompl="true";
#else
  const char* ompl="false";
#endif
  std::ostringstream os;os<<std::setprecision(15);
  os<<"{\"ompl\":"<<ompl<<",\"differential\":true,\"mecanum\":true,\"customMap\":"<<(customMap?"true":"false")
    <<",\"nav2WarehouseFull\":"<<(nav2WarehouseFull?"true":"false")
    <<",\"nav2Warehouse344128\":"<<(nav2Warehouse344128?"true":"false");
  os<<",\"nav2Maps\":{";
  bool first=true;
  const auto add=[&](const char* key,const ackermann::MapSnapshot* map) {
    if(!map)return;
    if(!first)os<<',';
    first=false;
    os<<quoted(key)<<":{\"width\":"<<map->width
      <<",\"height\":"<<map->height
      <<",\"resolution\":"<<map->resolutionMeters
      <<",\"origin\":["<<map->origin.x<<','<<map->origin.y<<','<<map->origin.yaw<<"]}";
  };
  add("nav2_warehouse_full",nav2WarehouseFull);
  add("nav2_warehouse_344128",nav2Warehouse344128);
  os<<'}';
  if(customMap) os<<",\"mapWidthMeters\":"<<customMap->width*customMap->resolutionMeters
                  <<",\"mapHeightMeters\":"<<customMap->height*customMap->resolutionMeters
                  <<",\"origin\":["<<customMap->origin.x<<','<<customMap->origin.y<<','<<customMap->origin.yaw<<']';
  os<<'}';return os.str();
}
} // namespace hybrid_astar::demo
