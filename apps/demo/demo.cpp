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
    <<",\"finalChecks\":"<<path.diagnostics.finalPoseChecks
    <<",\"validated\":"<<(path.status==PlanStatus::Success?"true":"false")
    <<",\"sections\":[";
  for(std::size_t i=0;i<path.sections.size();++i) {
    const auto& section=path.sections[i];
    if(i) os<<',';
    const std::size_t steps=std::max<std::size_t>(1,static_cast<std::size_t>(std::ceil(section.lengthMeters/.13)));
    // Visualization samples are separate from strict collision evidence.
    os<<"{\"gear\":"<<quoted(section.gear==Gear::Reverse?"reverse":"forward")
      <<",\"curvature\":"<<section.curvature<<",\"length\":"<<section.lengthMeters<<",\"points\":[";
    for(std::size_t j=0;j<=steps;++j) {
      if(j)os<<',';
      const auto p=section.evaluate(j==steps?section.lengthMeters:
                                                section.lengthMeters*double(j)/double(steps));
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
  r.options.maxExpansions=15000;r.options.maxGeneratedNodes=70000;
  if(name=="reverse")r.goal={6,15,0};
  if(name=="parking")r.goal={8,16,0};
  if(name=="warehouse") {
    rectangle(*map,48,12,53,48);rectangle(*map,48,92,53,145);
    rectangle(*map,94,0,99,46);rectangle(*map,94,100,99,160);
    rectangle(*map,141,12,146,55);rectangle(*map,141,104,146,145);
    r.start={8,20,0};r.goal={43,20,0};
    // Waypoint-free planning must find its own way around warehouse obstacles.
    r.options.maxExpansions=25000;
  }
  if(name=="blocked") {
    rectangle(*map,94,0,99,160);
    r.start={10,20,0};r.goal={35,20,0};
    r.options.maxExpansions=5000;
  }
  return r;
}

Outcome run(const Options& opt) {
  Outcome out;
  if(opt.scenario=="custom") {
    if(!opt.customMap)throw std::invalid_argument("No custom PGM map has been loaded");
    const auto& m=*opt.customMap;
    out.request.map=opt.customMap;
    const auto toWorld=[&](double x,double y) {
      const double c=std::cos(m.origin.yaw),sn=std::sin(m.origin.yaw);
      return Pose2D{m.origin.x+c*x-sn*y,m.origin.y+sn*x+c*y,m.origin.yaw};
    };
    out.request.start=toWorld(m.width*m.resolutionMeters*.2,m.height*m.resolutionMeters*.5);
    out.request.goal=toWorld(m.width*m.resolutionMeters*.8,m.height*m.resolutionMeters*.5);
    out.request.options.maxExpansions=25000;
  } else out.request=makeRequest(opt.scenario);
  if(opt.start)out.request.start=*opt.start;
  if(opt.goal)out.request.goal=*opt.goal;
  for(auto p:{out.request.start,out.request.goal})
    if(!finite(p)||p.x < -1000||p.x > 1000||p.y < -1000||p.y > 1000||
       std::abs(p.yaw)>10000)throw std::invalid_argument("Pose is not finite or within demo bounds");
  if(opt.performanceHeuristic)
    out.request.options.heuristicMode=HeuristicMode::PerformanceReedsSheppAndObstacleGrid;
  if(opt.improve) {
    out.request.options.solutionPolicy=SolutionPolicy::ImproveIncumbent;
    out.request.options.maxExtraExpansionsAfterFirst=700;
  }
  const auto t0=std::chrono::steady_clock::now();
#ifdef HYBRID_ASTAR_DEMO_WITH_OMPL
  if(opt.ompl) {
    if(opt.refine) {
      ackermann::RefinementOptions settings;
      settings.maxAttempts=35;settings.maxAccepted=3;
      const auto combined=steering::planAndRefineWithReedsShepp(out.request,settings,opt.performanceHeuristic);
      out.original=combined.original;
      out.refinement=combined.refinement;
    } else out.original=steering::planWithReedsShepp(out.request,opt.performanceHeuristic);
  } else
#endif
  {
    if(opt.ompl)throw std::invalid_argument("OMPL adapter is not enabled in this demo build");
    out.original=ackermann::plan(out.request);
    if(opt.refine&&out.original.status==PlanStatus::Success) {
      RefinementOptions settings;settings.maxAttempts=35;settings.maxAccepted=3;
      settings.useCurvatureBoundedShortcuts=opt.curvedRefinement;
      out.refinement=refinePath(out.request,out.original,{},settings);
    }
  }
  out.elapsedMilliseconds=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t0).count();
  return out;
}
std::string toJson(const Outcome& out) {
  const auto& m=*out.request.map;
  std::ostringstream os;os<<std::setprecision(12);
  os<<"{\"width\":"<<m.width<<",\"height\":"<<m.height<<",\"resolution\":"<<m.resolutionMeters
    <<",\"frameId\":"<<quoted(m.frameId)<<",\"sourceId\":"<<quoted(m.sourceId)<<",\"revision\":"<<m.revision
    <<",\"origin\":";poseJson(os,m.origin);
  os<<",\"start\":";poseJson(os,out.request.start);
  os<<",\"goal\":";poseJson(os,out.request.goal);
  os<<",\"milliseconds\":"<<out.elapsedMilliseconds<<",\"mapRuns\":[";
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
std::string capabilitiesJson(const ackermann::MapSnapshot* customMap) {
#ifdef HYBRID_ASTAR_DEMO_WITH_OMPL
  const char* ompl="true";
#else
  const char* ompl="false";
#endif
  std::ostringstream os;
  os<<"{\"ompl\":"<<ompl<<",\"customMap\":"<<(customMap?"true":"false");
  if(customMap) os<<",\"mapWidthMeters\":"<<customMap->width*customMap->resolutionMeters
                  <<",\"mapHeightMeters\":"<<customMap->height*customMap->resolutionMeters
                  <<",\"origin\":["<<customMap->origin.x<<','<<customMap->origin.y<<','<<customMap->origin.yaw<<']';
  os<<'}';return os.str();
}
} // namespace hybrid_astar::demo
