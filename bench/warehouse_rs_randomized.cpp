// Manual-only matched-pair RS performance test: 100 randomized warehouse
// start/goal scenarios, tested in BOTH modes (200 planning runs total).
// Produces a CSV on stdout and an aggregate comparison on stderr.
// Usage: hybrid_astar_warehouse_rs_randomized [seed=20261027] [max_expansions=1000000]
// Deliberately not a CTest: a million-expansion budget on 200 calls is expensive.
#include "bench/warehouse_rs_fixture.hpp"
#include "hybrid_astar/steering/reeds_shepp_evaluator.hpp"
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
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

using namespace hybrid_astar;
using namespace hybrid_astar::ackermann;

namespace {
constexpr std::size_t scenarioCount=100;
constexpr std::size_t scenariosPerBand=25;
constexpr double pi=std::numbers::pi_v<double>;
// Straight-line start/goal separation bands, in metres. Neither configuration
// uses these distances for gating analytic expansion.
constexpr std::array<double,5> distanceEdges{6.,12.,20.,30.,60.};

struct Scenario {
  Pose2D start,goal;
  std::size_t distanceBand;
  double directDistance;
};
struct Observation {
  double milliseconds=0;
  std::size_t expanded=0;
  bool success=false;
};
struct Summary {
  std::vector<double> times,expansions;
  std::size_t successes=0;
};
const char* modeName(std::size_t mode) {
  return mode==0?"grid_plus_rs_connector":"grid_plus_rs_connector_and_distance";
}
const char* statusName(PlanStatus s) {
  switch(s) {
    case PlanStatus::Success:return "Success";
    case PlanStatus::NoPath:return "NoPath";
    case PlanStatus::InvalidRequest:return "InvalidRequest";
    case PlanStatus::InvalidMap:return "InvalidMap";
    case PlanStatus::RejectedUnknownMap:return "RejectedUnknownMap";
    case PlanStatus::StartInCollision:return "StartInCollision";
    case PlanStatus::BudgetExceeded:return "BudgetExceeded";
    case PlanStatus::Cancelled:return "Cancelled";
    case PlanStatus::ValidationIncomplete:return "ValidationIncomplete";
    case PlanStatus::StrictValidationFailed:return "StrictValidationFailed";
    case PlanStatus::InternalError:return "InternalError";
  }
  return "Unknown";
}
const char* terminationName(TerminationReason t) {
  switch(t) {
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
std::uint64_t unsignedArg(const char* chars,std::uint64_t maximum) {
  const std::string value(chars);
  if(value.empty()||value[0]=='-')throw std::invalid_argument("Invalid unsigned argument");
  std::size_t consumed=0;
  const auto parsed=std::stoull(value,&consumed);
  if(consumed!=value.size()||parsed>maximum)
    throw std::invalid_argument("Unsigned argument outside allowed range");
  return parsed;
}
double uniform01(std::mt19937_64& rng) {
  // Use exactly the upper 53 RNG bits: reproducible across standard libraries,
  // unlike implementation-dependent uniform_real_distribution transforms.
  return static_cast<double>(rng()>>11)*(1.0/9007199254740992.0);
}
Pose2D randomPose(std::mt19937_64& rng) {
  return {50.*uniform01(rng),40.*uniform01(rng),2*pi*uniform01(rng)-pi};
}
std::vector<Scenario> makeScenarios(const PlanRequest& request,std::uint64_t seed) {
  std::mt19937_64 rng(seed);
  ExactCollisionChecker checker(*request.map,request.occupancy,
                                request.footprint,request.options.footprintInflationMeters);
  std::vector<Scenario> scenarios;
  scenarios.reserve(scenarioCount);
  // 25 randomly sampled pairs per separation band, to prevent near-goal easy
  // cases from dominating the comparison. Both modes get IDENTICAL poses.
  for(std::size_t band=0;band+1<distanceEdges.size();++band) {
    std::size_t attempts=0;
    while(scenarios.size()<(band+1)*scenariosPerBand) {
      if(++attempts>2000000)
        throw std::runtime_error("Could not sample 25 collision-free pairs in a distance band");
      const auto start=randomPose(rng);
      const auto goal=randomPose(rng);
      const double d=std::hypot(start.x-goal.x,start.y-goal.y);
      if(d<distanceEdges[band]||d>=distanceEdges[band+1])continue;
      if(checker.check(start)!=PoseCollision::Clear ||
         checker.check(goal)!=PoseCollision::Clear)continue;
      scenarios.push_back({start,goal,band,d});
    }
  }
  if(scenarios.size()!=scenarioCount)
    throw std::runtime_error("Expected exactly 100 paired scenarios");
  // Shuffle distance bands to avoid a systematic easy-to-hard runtime trend.
  for(std::size_t i=scenarios.size()-1;i>0;--i) {
    const auto j=static_cast<std::size_t>(rng()% (i+1));
    std::swap(scenarios[i],scenarios[j]);
  }
  return scenarios;
}
double quantile(std::vector<double> data,double fraction) {
  if(data.empty())return std::numeric_limits<double>::quiet_NaN();
  std::sort(data.begin(),data.end());
  const auto index=std::max<std::size_t>(1,
      static_cast<std::size_t>(std::ceil(data.size()*fraction)))-1;
  return data[std::min(index,data.size()-1)];
}
Observation runMode(PlanRequest request,bool distanceHeuristic,
                    std::size_t scenarioId,std::size_t order,
                    std::uint64_t seed,double directDistance,
                    std::size_t band,std::size_t budget) {
  std::size_t pathCalls=0,distanceCalls=0;
  const auto started=std::chrono::steady_clock::now();
  // Separate evaluator per run, just as in planWithReedsShepp. Only the
  // installed distance callback differs between the two configurations.
  steering::ReedsSheppEvaluator evaluator(request.options.minimumTurningRadiusMeters);
  request.analyticConnector=[&](const Pose2D& from,const Pose2D& goal) {
    ++pathCalls;
    return evaluator.shortestPath(from,goal);
  };
  if(distanceHeuristic)
    request.analyticDistance=[&](const Pose2D& from,const Pose2D& goal) {
      ++distanceCalls;
      return evaluator.distance(from,goal);
    };
  const auto result=plan(request);
  const double elapsed=std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-started).count();
  if(result.status==PlanStatus::InvalidMap ||
     result.status==PlanStatus::InvalidRequest ||
     result.status==PlanStatus::StartInCollision ||
     result.status==PlanStatus::InternalError)
    throw std::runtime_error("Unexpected invalid planning request or internal error");
  if(result.status==PlanStatus::Success &&
     (result.collisionEvidence!=CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled ||
      !result.modelGeometryValid))
    throw std::runtime_error("Successful path lacks strict geometry validation");
  if(pathCalls!=result.diagnostics.analyticAttempts ||
     (distanceHeuristic && distanceCalls==0) ||
     (!distanceHeuristic && distanceCalls!=0))
    throw std::runtime_error("RS callback configuration or diagnostics mismatch");
  const auto& diag=result.diagnostics;
  std::cout<<scenarioId<<','<<order<<','<<seed<<','<<band<<','
           <<directDistance<<','
           <<request.start.x<<','<<request.start.y<<','<<request.start.yaw<<','
           <<request.goal.x<<','<<request.goal.y<<','<<request.goal.yaw<<','
           <<modeName(distanceHeuristic?1:0)<<','
           <<budget<<','<<statusName(result.status)<<','
           <<terminationName(result.terminationReason)<<','
           <<elapsed<<','<<diag.expanded<<','<<diag.generated<<','
           <<diag.searchPoseChecks<<','<<diag.finalPoseChecks<<','
           <<diag.gridSettlements<<','<<diag.analyticAttempts<<','
           <<diag.analyticAccepted<<','<<pathCalls<<','<<distanceCalls<<',';
  if(result.status==PlanStatus::Success)
    std::cout<<result.weightedCostMeters<<','<<result.geometricLengthMeters
             <<','<<result.reverseLengthMeters;
  else
    std::cout<<",,";
  std::cout<<'\n'<<std::flush;
  return {elapsed,diag.expanded,result.status==PlanStatus::Success};
}
}

int main(int argc,char** argv) {
  try {
    if(argc>3)throw std::invalid_argument(
      "Usage: hybrid_astar_warehouse_rs_randomized [seed=20261027] [max_expansions=1000000]");
    const auto seed=argc>1?unsignedArg(argv[1],std::numeric_limits<std::uint64_t>::max()):
                           std::uint64_t{20261027};
    const auto budget=argc>2?unsignedArg(argv[2],1'000'000):
                             std::uint64_t{1'000'000};
    if(budget<100)throw std::invalid_argument("Expansion budget must be at least 100");
    const auto fixture=bench::warehouseReedsSheppRequest();
    const auto scenarios=makeScenarios(fixture,seed);
    std::array<Summary,2> summaries{};
    std::vector<double> matchedRatios;
    std::size_t bothSuccess=0,onlyConnector=0,onlyDistance=0,neitherSuccess=0;
    std::cout<<std::fixed<<std::setprecision(6);
    std::cout<<"scenario,order,seed,distance_band,direct_distance_m,"
      "start_x,start_y,start_yaw_rad,goal_x,goal_y,goal_yaw_rad,"
      "mode,expansion_limit,status,termination,elapsed_ms,expanded,generated,"
      "search_pose_checks,final_pose_checks,grid_settlements,analytic_attempts,"
      "analytic_accepted,rs_path_calls,rs_distance_calls,"
      "path_cost_m,path_length_m,reverse_length_m\n";
    for(std::size_t i=0;i<scenarios.size();++i) {
      const auto& s=scenarios[i];
      std::array<Observation,2> observed{};
      // Alternate A/B then B/A by scenario, controlling for cache/thermal
      // effects; each run gets a fresh planner/evaluator and identical map.
      for(std::size_t j=0;j<2;++j) {
        const std::size_t mode=(i+j)%2;
        auto request=fixture;
        request.start=s.start;request.goal=s.goal;
        request.options.maxExpansions=static_cast<std::size_t>(budget);
        request.options.maxGeneratedNodes=static_cast<std::size_t>(budget*4);
        observed[mode]=runMode(request,mode==1,i+1,j+1,seed,
                               s.directDistance,s.distanceBand,budget);
        summaries[mode].times.push_back(observed[mode].milliseconds);
        summaries[mode].expansions.push_back(double(observed[mode].expanded));
        summaries[mode].successes+=observed[mode].success;
      }
      if(observed[0].success && observed[1].success) {
        ++bothSuccess;
        matchedRatios.push_back(observed[1].milliseconds/
                                std::max(1e-9,observed[0].milliseconds));
      } else if(observed[0].success)++onlyConnector;
      else if(observed[1].success)++onlyDistance;
      else ++neitherSuccess;
    }
    std::cerr<<std::fixed<<std::setprecision(3);
    std::cerr<<"seed="<<seed<<" scenarios=100 paired_runs=200 budget="<<budget<<'\n';
    for(std::size_t mode=0;mode<2;++mode)
      std::cerr<<modeName(mode)<<" success="<<summaries[mode].successes
               <<"/100 median_ms="<<quantile(summaries[mode].times,.5)
               <<" p95_ms="<<quantile(summaries[mode].times,.95)
               <<" median_expanded="<<quantile(summaries[mode].expansions,.5)<<'\n';
    std::cerr<<"both_success="<<bothSuccess
             <<" connector_only_success="<<onlyConnector
             <<" distance_only_success="<<onlyDistance
             <<" neither_success="<<neitherSuccess
             <<" median_distance_to_connector_time_ratio_on_both_success="
             <<quantile(matchedRatios,.5)<<'\n';
    return 0;
  } catch(const std::exception& error) {
    std::cerr<<"warehouse RS randomized benchmark failed: "<<error.what()<<'\n';
    return 1;
  }
}
