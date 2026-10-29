#pragma once
// Full-resolution Nav2 warehouse, deterministic paired margin A/B benchmark.
// Heavy scenario discovery is NEVER part of normal CTest or timed runs.
#include "hybrid_astar/ackermann/refinement.hpp"
#include "apps/demo/map_yaml.hpp"
#ifdef HYBRID_ASTAR_NAV2_PIPELINE_WITH_OMPL
#include "hybrid_astar/steering/reeds_shepp_evaluator.hpp"
#endif
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace hybrid_astar::bench::nav2_pipeline {
using namespace hybrid_astar::ackermann;
using Clock=std::chrono::steady_clock;
inline double millis(Clock::time_point a,Clock::time_point b) {
  return std::chrono::duration<double,std::milli>(b-a).count();
}
struct Config {
  std::string yamlPath="apps/demo/maps/nav2_warehouse_full.yaml";
  std::string model="differential"; // match the long successful Planning Lab case
  std::string connector="native";
  std::size_t cases=10;
  std::size_t repeats=3;
  std::size_t minimumExpansions=30000;
  std::size_t searchBudget=500000;
  std::size_t maxCandidates=2500; // bounded discovery; 0 explicitly opts into unbounded
  std::uint64_t seed=20261029;
  std::size_t acceptedLimit=16;
  bool curved=true;
};
inline std::size_t integer(const std::string& value,bool allowZero=false) {
  if(value.empty()||value[0]=='-')throw std::invalid_argument("Expected nonnegative integer");
  std::size_t consumed=0;
  const auto n=std::stoull(value,&consumed);
  if(consumed!=value.size()||(!allowZero&&n==0))
    throw std::invalid_argument("Expected positive integer");
  return static_cast<std::size_t>(n);
}
inline Config parseArgs(int argc,char** argv) {
  // Named parameters prevent positional argument errors in long benchmark runs.
  Config c;
  for(int i=1;i<argc;++i) {
    const std::string arg=argv[i];
    if(arg=="--help"||arg=="-h")
      throw std::invalid_argument("Usage: BENCH [--map YAML] [--model differential|ackermann|mecanum] "
          "[--connector native|ompl] [--cases 10] [--repeats 3] [--min-expansions 30000] "
          "[--search-budget 500000] [--max-candidates 2500 (0=unbounded)] "
          "[--seed 20261029] [--max-accepted 8|12|16] [--no-curved]");
    if(arg=="--no-curved"){c.curved=false;continue;}
    if(i+1>=argc)throw std::invalid_argument("Missing value for "+arg);
    const std::string val=argv[++i];
    if(arg=="--map")c.yamlPath=val;
    else if(arg=="--model")c.model=val;
    else if(arg=="--connector")c.connector=val;
    else if(arg=="--cases")c.cases=integer(val);
    else if(arg=="--repeats")c.repeats=integer(val);
    else if(arg=="--min-expansions")c.minimumExpansions=integer(val);
    else if(arg=="--search-budget")c.searchBudget=integer(val);
    else if(arg=="--max-candidates")c.maxCandidates=integer(val,true);
    else if(arg=="--seed")c.seed=integer(val,true);
    else if(arg=="--max-accepted")c.acceptedLimit=integer(val);
    else throw std::invalid_argument("Unknown argument: "+arg);
  }
  if((c.model!="ackermann"&&c.model!="differential"&&c.model!="mecanum")||
     (c.connector!="native"&&c.connector!="ompl")||
     (c.connector=="ompl"&&c.model!="ackermann")||
     c.cases>1000||c.repeats>100||c.minimumExpansions>1000000||
     c.searchBudget<30000||c.searchBudget>1000000||
     c.minimumExpansions>c.searchBudget||
     (c.acceptedLimit!=8&&c.acceptedLimit!=12&&c.acceptedLimit!=16)||
     (c.maxCandidates!=0&&c.maxCandidates<c.cases))
    throw std::invalid_argument("Invalid benchmark configuration");
#ifndef HYBRID_ASTAR_NAV2_PIPELINE_WITH_OMPL
  if(c.connector=="ompl")throw std::invalid_argument("OMPL connector unavailable in this build");
#endif
  return c;
}
struct RandomStream {
  std::uint64_t state;
  explicit RandomStream(std::uint64_t seed):state(seed) {}
  std::uint64_t next() {
    state+=0x9e3779b97f4a7c15ULL;
    auto z=state;
    z=(z^(z>>30))*0xbf58476d1ce4e5b9ULL;
    z=(z^(z>>27))*0x94d049bb133111ebULL;
    return z^(z>>31);
  }
  double unit(){return static_cast<double>(next()>>11)*0x1.0p-53;}
};
struct Endpoints { Pose2D start,goal; std::size_t startRegion,goalRegion; };
inline std::size_t regionOf(const Pose2D& p,const MapSnapshot& map) {
  const auto dx=p.x-map.origin.x,dy=p.y-map.origin.y;
  const double co=std::cos(map.origin.yaw),si=std::sin(map.origin.yaw);
  const double x=(co*dx+si*dy)/(map.width*map.resolutionMeters);
  const double y=(-si*dx+co*dy)/(map.height*map.resolutionMeters);
  const auto col=std::min<std::size_t>(2,static_cast<std::size_t>(std::clamp(x,0.0,1.0)*3));
  const auto row=std::min<std::size_t>(2,static_cast<std::size_t>(std::clamp(y,0.0,1.0)*3));
  return row*3+col;
}
inline std::optional<Endpoints> candidateAt(std::size_t attempt,const Config& config,
                                             const MapSnapshot& map,Footprint footprint) {
  RandomStream rng(config.seed+static_cast<std::uint64_t>(attempt)*0xd1342543de82ef95ULL);
  const double w=map.width*map.resolutionMeters,h=map.height*map.resolutionMeters;
  const double border=std::hypot(std::max(std::abs(footprint.rearX),std::abs(footprint.frontX)),
                                 footprint.halfWidth)+.15;
  if(2*border>=w||2*border>=h)return std::nullopt;
  const auto randomPose=[&]() {
    const double x=border+(w-2*border)*rng.unit();
    const double y=border+(h-2*border)*rng.unit();
    const double co=std::cos(map.origin.yaw),si=std::sin(map.origin.yaw);
    return Pose2D{map.origin.x+co*x-si*y,map.origin.y+si*x+co*y,
                  -std::numbers::pi_v<double>+2*std::numbers::pi_v<double>*rng.unit()};
  };
  // Reject trivial local routes; same seeded attempts for all models/variants.
  for(int retry=0;retry<32;++retry) {
    const auto start=randomPose(),goal=randomPose();
    if(std::hypot(start.x-goal.x,start.y-goal.y)<15.0)continue;
    const auto startRegion=regionOf(start,map),goalRegion=regionOf(goal,map);
    if(startRegion==goalRegion)continue;
    return Endpoints{start,goal,startRegion,goalRegion};
  }
  return std::nullopt;
}
inline PlanRequest baseRequest(const Config& config,std::shared_ptr<const MapSnapshot> map) {
  PlanRequest req;
  req.map=std::move(map);
  req.options.maxExpansions=config.searchBudget;
  req.options.maxGeneratedNodes=4*config.searchBudget;
  req.options.heuristicMode=HeuristicMode::PerformanceReedsSheppAndObstacleGrid;
  if(config.model=="differential") {
    req.options.model=PlanOptions::Model::DifferentialDrive;
    req.footprint={-.35,.35,.25};
    req.options.rotationCostMetersPerRadian=.4;
  } else if(config.model=="mecanum") {
    req.options.model=PlanOptions::Model::Mecanum;
    req.footprint={-.35,.35,.25};
    req.options.rotationCostMetersPerRadian=.4;
  }
  return req;
}
inline RefinementOptions refinementOptions(const Config& config) {
  RefinementOptions opt;
  opt.maxAccepted=config.acceptedLimit;
  opt.useCurvatureBoundedShortcuts=config.curved&&config.model!="mecanum";
  if(config.model=="differential") {
    opt.differentialCurveRadiusMeters=.75;
    opt.differentialRadiusVariants=4;
  } else if(config.model=="mecanum") {
    opt.maxAttempts=220;
    opt.maxTotalPoseChecks=240000;
  }
  return opt;
}
inline PlanResult planOnce(const PlanRequest& request,const Config& config) {
#ifdef HYBRID_ASTAR_NAV2_PIPELINE_WITH_OMPL
  if(config.connector=="ompl")return steering::planWithReedsShepp(request);
#endif
  return plan(request);
}
inline RefinementResult refineOnce(const PlanRequest& request,
                                   const PlanResult& raw,const Config& config) {
  auto options=refinementOptions(config);
#ifdef HYBRID_ASTAR_NAV2_PIPELINE_WITH_OMPL
  if(config.connector=="ompl") {
    steering::ReedsSheppEvaluator evaluator(request.options.minimumTurningRadiusMeters);
    const RefinementConnector connector=[&](const Pose2D& a,const Pose2D& b) {
      return evaluator.shortestPath(a,b);
    };
    return refinePath(request,raw,connector,options);
  }
#endif
  return refinePath(request,raw,{},options);
}
struct Scenario {
  std::size_t id=0,drawIndex=0;
  Endpoints endpoints;
  std::size_t discoveryExpansions=0;
};
inline bool strictSuccess(const PlanResult& p) {
  return p.status==PlanStatus::Success && p.modelGeometryValid &&
         p.collisionEvidence==CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled;
}
// Qualify ONLY on the zero-margin discovery search, never on inflated success.
// This avoids censoring safety-margin failures and preserves paired inputs.
// Both endpoint footprints must nevertheless be initially collision-free at 0.1 m.
inline std::vector<Scenario> discover(const Config& config,const PlanRequest& base) {
  ExactCollisionChecker inflatedChecker(*base.map,base.occupancy,base.footprint,.1);
  std::set<std::array<std::int64_t,6>> endpointKeys;
  std::vector<Scenario> cases;
  cases.reserve(config.cases);
  std::size_t blocked=0,unsolved=0,tooEasy=0,duplicates=0,unusable=0;
  std::cerr<<"Discovering "<<config.cases<<" distinct >= "<<config.minimumExpansions
           <<" expansion paths on full Nav2 warehouse; max_candidates="
           <<config.maxCandidates<<" (0=unbounded), model="<<config.model<<'\n';
  for(std::size_t i=0;cases.size()<config.cases &&
       (!config.maxCandidates||i<config.maxCandidates);++i) {
    if(i&&i%100==0)std::cerr<<"Discovery "<<cases.size()<<'/'<<config.cases
        <<" after "<<i<<" draws (blocked "<<blocked<<", unsolved "<<unsolved
        <<", below-threshold "<<tooEasy<<")\n";
    const auto endpoints=candidateAt(i,config,*base.map,base.footprint);
    if(!endpoints){++unusable;continue;}
    if(inflatedChecker.check(endpoints->start)!=PoseCollision::Clear ||
       inflatedChecker.check(endpoints->goal)!=PoseCollision::Clear) {
      ++blocked;continue;
    }
    const auto q=[](double x){return static_cast<std::int64_t>(std::llround(x*1000));};
    const std::array<std::int64_t,6> key{
      q(endpoints->start.x),q(endpoints->start.y),q(endpoints->start.yaw),
      q(endpoints->goal.x),q(endpoints->goal.y),q(endpoints->goal.yaw)};
    if(endpointKeys.contains(key)){++duplicates;continue;}
    auto trial=base;
    trial.start=endpoints->start;trial.goal=endpoints->goal;
    trial.options.footprintInflationMeters=0.0;
    const auto p=planOnce(trial,config);
    if(!strictSuccess(p)){++unsolved;continue;}
    if(p.diagnostics.expanded<config.minimumExpansions){++tooEasy;continue;}
    endpointKeys.insert(key);
    cases.push_back({cases.size(),i,*endpoints,p.diagnostics.expanded});
    std::cerr<<"Accepted "<<cases.size()<<'/'<<config.cases
        <<" after draw "<<i<<" with "<<p.diagnostics.expanded<<" expansions\n";
  }
  if(cases.size()!=config.cases)
    throw std::runtime_error("Found only "+std::to_string(cases.size())+"/"+
        std::to_string(config.cases)+" qualifying routes; no CSV emitted. "
        "Raise --max-candidates (0=unbounded), --search-budget, or reduce --cases. "
        "Rejected: blocked="+std::to_string(blocked)+", unsolved="+
        std::to_string(unsolved)+", below_30k="+std::to_string(tooEasy)+
        ", duplicate="+std::to_string(duplicates)+", unusable="+
        std::to_string(unusable));
  return cases;
}
inline double percentile(std::vector<double> v,double fraction) {
  if(v.empty())return 0;
  std::sort(v.begin(),v.end());
  return v[std::min(v.size()-1,
    static_cast<std::size_t>(std::ceil(v.size()*fraction))-1)];
}
struct Measurement {
  PlanResult raw;
  std::optional<RefinementResult> refinement;
  double searchP50=0,searchP95=0,refineP50=0,refineP95=0,totalP50=0,totalP95=0;
};
inline Measurement measure(const Config& c,const PlanRequest& request) {
  const auto variantName=request.options.footprintInflationMeters==0?"off":"0.10";
  auto once=[&]() {
    const auto t0=Clock::now();
    auto raw=planOnce(request,c);
    const auto t1=Clock::now();
    std::optional<RefinementResult> refinement;
    if(strictSuccess(raw)) {
      refinement=refineOnce(request,raw,c);
      if(!strictSuccess(refinement->path)||
         refinement->accepted>c.acceptedLimit ||
         refinement->attempted>refinementOptions(c).maxAttempts ||
         refinement->exactPoseChecks>refinementOptions(c).maxTotalPoseChecks)
        throw std::runtime_error(std::string("Refinement invalid/over budget for margin ")+variantName);
    }
    const auto t2=Clock::now();
    return std::tuple{std::move(raw),std::move(refinement),
                      millis(t0,t1),millis(t1,t2),millis(t0,t2)};
  };
  // One full pipeline warmup per variant, outside measured data.
  (void)once();
  std::vector<double> search,refine,total;
  Measurement m;
  for(std::size_t rep=0;rep<c.repeats;++rep) {
    auto [raw,r,searchMs,refineMs,totalMs]=once();
    if(rep>0&&(raw.status!=m.raw.status ||
       raw.diagnostics.expanded!=m.raw.diagnostics.expanded ||
       (strictSuccess(raw)&&std::abs(raw.geometricLengthMeters-
                                   m.raw.geometricLengthMeters)>1e-7) ||
       r.has_value()!=m.refinement.has_value() ||
       (r&&m.refinement &&
         (r->accepted!=m.refinement->accepted ||
          std::abs(r->path.weightedCostMeters-
                   m.refinement->path.weightedCostMeters)>1e-7))))
      throw std::runtime_error("Non-deterministic pipeline output across timed repeats");
    m.raw=std::move(raw);
    m.refinement=std::move(r);
    search.push_back(searchMs);refine.push_back(refineMs);total.push_back(totalMs);
  }
  m.searchP50=percentile(search,.5);m.searchP95=percentile(search,.95);
  m.refineP50=percentile(refine,.5);m.refineP95=percentile(refine,.95);
  m.totalP50=percentile(total,.5);m.totalP95=percentile(total,.95);
  return m;
}
inline const char* statusName(PlanStatus status) {
  switch(status) {
    case PlanStatus::Success:return "Success";
    case PlanStatus::StartInCollision:return "StartInCollision";
    case PlanStatus::BudgetExceeded:return "BudgetExceeded";
    case PlanStatus::Cancelled:return "Cancelled";
    case PlanStatus::ValidationIncomplete:return "ValidationIncomplete";
    case PlanStatus::NoPath:return "NoPath";
    case PlanStatus::StrictValidationFailed:return "StrictValidationFailed";
    default:return "Other";
  }
}
inline const char* refinerName(RefinementStatus status) {
  switch(status) {
    case RefinementStatus::Improved:return "Improved";
    case RefinementStatus::Unchanged:return "Unchanged";
    case RefinementStatus::WorkBudgetExceeded:return "WorkBudgetExceeded";
    case RefinementStatus::OriginalNotVerified:return "OriginalNotVerified";
    case RefinementStatus::Cancelled:return "Cancelled";
    default:return "Other";
  }
}
inline double nsMs(std::uint64_t v){return static_cast<double>(v)*1e-6;}
inline void header() {
  std::cout<<"case_id,draw_index,seed,model,connector,map_cells,resolution_m,"
    "start_region,goal_region,start_x_m,start_y_m,start_yaw_rad,goal_x_m,goal_y_m,goal_yaw_rad,"
    "minimum_expansions,discovery_expansions,margin_m,status,repeats,"
    "search_p50_ms,search_p95_ms,refine_p50_ms,refine_p95_ms,total_p50_ms,total_p95_ms,"
    "expanded,generated,pruned_by_key,search_pose_checks,final_pose_checks,"
    "grid_settlements,grid_fallbacks,grid_blocked_query_fallbacks,grid_budget_fallbacks,"
    "grid_queue_pops,grid_queue_pushes,open_queue_pops,open_queue_pushes,open_queue_stale_pops,"
    "label_finds,label_insertions,hash_calls,search_motion_calls,final_motion_calls,"
    "exact_collision_pose_calls,disc_exact_fallback,disc_definitely_clear,"
    "analytic_attempts,analytic_accepted,strict_goal_candidates,strict_rejected_collision,"
    "strict_rejected_continuity,raw_sections,raw_length_m,raw_cost_m,raw_reverse_m,"
    "refine_status,refine_attempted,refine_accepted,refine_rejected,refine_exact_pose_checks,"
    "refined_sections,refined_length_m,refined_cost_m,refined_reverse_m,"
    "profile_enabled,search_motion_hotpath_ms,final_motion_hotpath_ms,exact_collision_hotpath_ms,"
    "grid_queue_hotpath_ms,open_queue_hotpath_ms,label_lookup_hotpath_ms,"
    "label_write_hotpath_ms,hash_hotpath_ms\n";
}
inline void row(const Config& c,const Scenario& s,const PlanRequest& req,const Measurement& m) {
  const auto& d=m.raw.diagnostics;
  const auto refined=m.refinement&&strictSuccess(m.refinement->path);
#ifdef HYBRID_ASTAR_NAV2_PIPELINE_PROFILE_ENABLED
  constexpr int profiling=1;
#else
  constexpr int profiling=0;
#endif
  std::cout<<std::fixed<<std::setprecision(6)
    <<s.id<<','<<s.drawIndex<<','<<c.seed<<','<<c.model<<','<<c.connector<<','
    <<req.map->cells.size()<<','<<req.map->resolutionMeters<<','
    <<s.endpoints.startRegion<<','<<s.endpoints.goalRegion<<','
    <<req.start.x<<','<<req.start.y<<','<<req.start.yaw<<','
    <<req.goal.x<<','<<req.goal.y<<','<<req.goal.yaw<<','
    <<c.minimumExpansions<<','<<s.discoveryExpansions<<','
    <<req.options.footprintInflationMeters<<','<<statusName(m.raw.status)<<','
    <<c.repeats<<','<<m.searchP50<<','<<m.searchP95<<','
    <<m.refineP50<<','<<m.refineP95<<','<<m.totalP50<<','<<m.totalP95<<','
    <<d.expanded<<','<<d.generated<<','<<d.prunedByKey<<','
    <<d.searchPoseChecks<<','<<d.finalPoseChecks<<','
    <<d.gridSettlements<<','<<d.gridFallbacks<<','
    <<d.gridBlockedQueryFallbacks<<','<<d.gridWorkBudgetFallbacks<<','
    <<d.gridQueuePops<<','<<d.gridQueuePushes<<','
    <<d.openQueuePops<<','<<d.openQueuePushes<<','<<d.staleHeapPops<<','
    <<d.labelFinds<<','<<d.labelInsertions<<','<<d.hashCalls<<','
    <<d.searchMotionCalls<<','<<d.finalMotionCalls<<','
    <<d.exactCollisionPoseCalls<<','<<d.discExactFallback<<','
    <<d.discDefinitelyClear<<','
    <<d.analyticAttempts<<','<<d.analyticAccepted<<','
    <<d.strictGoalCandidates<<','<<d.strictRejectedCollision<<','
    <<d.strictRejectedContinuity<<','
    <<m.raw.sections.size()<<','<<m.raw.geometricLengthMeters<<','
    <<m.raw.weightedCostMeters<<','<<m.raw.reverseLengthMeters<<','
    <<(refined?refinerName(m.refinement->status):"NotRun")<<','
    <<(refined?m.refinement->attempted:0)<<','
    <<(refined?m.refinement->accepted:0)<<','
    <<(refined?m.refinement->rejected:0)<<','
    <<(refined?m.refinement->exactPoseChecks:0)<<','
    <<(refined?m.refinement->path.sections.size():0)<<','
    <<(refined?m.refinement->path.geometricLengthMeters:0)<<','
    <<(refined?m.refinement->path.weightedCostMeters:0)<<','
    <<(refined?m.refinement->path.reverseLengthMeters:0)<<','
    <<profiling<<','
    <<nsMs(d.searchMotionNanoseconds)<<','<<nsMs(d.finalMotionNanoseconds)<<','
    <<nsMs(d.exactCollisionNanoseconds)<<','<<nsMs(d.gridQueueNanoseconds)<<','
    <<nsMs(d.openQueueNanoseconds)<<','<<nsMs(d.labelLookupNanoseconds)<<','
    <<nsMs(d.labelWriteNanoseconds)<<','<<nsMs(d.hashNanoseconds)<<'\n';
}
} // namespace hybrid_astar::bench::nav2_pipeline
