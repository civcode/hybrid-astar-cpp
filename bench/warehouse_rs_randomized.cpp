// Manual-only matched-pair RS benchmark. Sample randomized warehouse poses
// until 100 pairs BOTH succeed with >=10k search expansions (configurable).
// Screened-out pairs are counted but never included in the accepted CSV.
// Usage: executable [seed=20261027] [max_expansions=1000000]
//                   [min_expansions=10000] [max_candidates=20000]
// Never part of automatic CTest: potentially thousands of planning requests.
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
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace hybrid_astar;
using namespace hybrid_astar::ackermann;

namespace {
constexpr std::size_t scenarioCount=100;
constexpr double minDirectDistanceMeters=25.0; // Focus on warehouse-scale trips.
constexpr double pi=std::numbers::pi_v<double>;
// Distance bands are only diagnostic metadata; no per-band acceptance quota.
constexpr std::array<double,5> distanceEdges{0.,12.,20.,30.,60.};

struct Scenario {
  Pose2D start,goal;
  std::size_t distanceBand;
  double directDistance;
};
struct Observation {
  double milliseconds=0;
  std::size_t expanded=0;
  bool success=false;
  std::string csvRow; // Buffer output until BOTH modes qualify.
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
Scenario nextCandidate(std::mt19937_64& rng,const ExactCollisionChecker& checker,
                       std::size_t& poseDraws,std::size_t maxPoseDraws) {
  while(poseDraws<maxPoseDraws) {
    ++poseDraws;
    const auto start=randomPose(rng),goal=randomPose(rng);
    const double distance=std::hypot(start.x-goal.x,start.y-goal.y);
    if(distance<minDirectDistanceMeters)continue;
    if(checker.check(start)!=PoseCollision::Clear ||
       checker.check(goal)!=PoseCollision::Clear)continue;
    std::size_t band=0;
    while(band+1<distanceEdges.size()-1 &&
          distance>=distanceEdges[band+1])++band;
    return {start,goal,band,distance};
  }
  throw std::runtime_error("Exceeded collision-free candidate sampling limit");
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
  std::ostringstream line;
  line<<std::fixed<<std::setprecision(6);
  line<<scenarioId<<','<<order<<','<<seed<<','<<band<<','
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
    line<<result.weightedCostMeters<<','<<result.geometricLengthMeters
        <<','<<result.reverseLengthMeters;
  else
    line<<",,";
  return {elapsed,diag.expanded,result.status==PlanStatus::Success,line.str()};

}
}

int main(int argc,char** argv) {
  try {
    if(argc>5)throw std::invalid_argument(
      "Usage: hybrid_astar_warehouse_rs_randomized [seed=20261027] "
      "[max_expansions=1000000] [min_expansions=10000] [max_candidates=20000]");
    const auto seed=argc>1?unsignedArg(argv[1],std::numeric_limits<std::uint64_t>::max()):
                           std::uint64_t{20261027};
    const auto budget=argc>2?unsignedArg(argv[2],1'000'000):
                             std::uint64_t{1'000'000};
    const auto minimum=argc>3?unsignedArg(argv[3],1'000'000):
                              std::uint64_t{10'000};
    const auto maxCandidates=argc>4?unsignedArg(argv[4],100'000):
                                   std::uint64_t{20'000};
    if(budget<100||minimum<1||minimum>budget||maxCandidates<scenarioCount)
      throw std::invalid_argument("Invalid budget, minimum expansions, or candidate limit");

    const auto fixture=bench::warehouseReedsSheppRequest();
    const ExactCollisionChecker checker(*fixture.map,fixture.occupancy,
        fixture.footprint,fixture.options.footprintInflationMeters);
    std::mt19937_64 rng(seed);
    std::size_t poseDraws=0,candidates=0,qualified=0;
    std::size_t rejectedFailure=0,rejectedExpansions=0;
    std::size_t rejectedFastAOnly=0,rejectedFastBOnly=0,rejectedBothFast=0;
    std::size_t anySuccessA=0,anySuccessB=0,anyExpandedA=0,anyExpandedB=0;
    const auto maxPoseDraws=static_cast<std::size_t>(maxCandidates)*200;

    std::array<Summary,2> summaries{};
    std::vector<double> matchedRatios;
    std::size_t fasterConnector=0,fasterDistance=0;
    std::cout<<std::fixed<<std::setprecision(6);
    std::cout<<"scenario,order,seed,distance_band,direct_distance_m,"
      "start_x,start_y,start_yaw_rad,goal_x,goal_y,goal_yaw_rad,"
      "mode,expansion_limit,status,termination,elapsed_ms,expanded,generated,"
      "search_pose_checks,final_pose_checks,grid_settlements,analytic_attempts,"
      "analytic_accepted,rs_path_calls,rs_distance_calls,"
      "path_cost_m,path_length_m,reverse_length_m\\n";

    while(qualified<scenarioCount && candidates<maxCandidates) {
      // Draw on the full unchanged warehouse map. A large XY separation
      // reduces trivial single-shot connectors, but the *actual measured*
      // search-expansion requirement is the final acceptance condition.
      const auto s=nextCandidate(rng,checker,poseDraws,maxPoseDraws);
      ++candidates;
      std::array<Observation,2> observed{};
      // Alternate A/B then B/A by CANDIDATE (not accepted-case index).
      for(std::size_t order=0;order<2;++order) {
        const std::size_t mode=(candidates-1+order)%2;
        auto request=fixture;
        request.start=s.start;request.goal=s.goal;
        request.options.maxExpansions=static_cast<std::size_t>(budget);
        request.options.maxGeneratedNodes=static_cast<std::size_t>(budget*4);
        observed[mode]=runMode(request,mode==1,qualified+1,order+1,seed,
                               s.directDistance,s.distanceBand,budget);
      }
      anySuccessA+=observed[0].success;
      anySuccessB+=observed[1].success;
      anyExpandedA+=observed[0].expanded>=minimum;
      anyExpandedB+=observed[1].expanded>=minimum;

      const bool fastA=observed[0].expanded<minimum;
      const bool fastB=observed[1].expanded<minimum;
      if(!observed[0].success||!observed[1].success) {
        ++rejectedFailure;
      } else if(fastA||fastB) {
        ++rejectedExpansions;
        if(fastA && fastB)++rejectedBothFast;
        else if(fastA)++rejectedFastAOnly;
        else ++rejectedFastBOnly;
      } else {
        ++qualified;
        // ONLY qualifying paired paths are included in the primary CSV.
        for(std::size_t mode=0;mode<2;++mode) {
          std::cout<<observed[mode].csvRow<<'\\n';
          summaries[mode].times.push_back(observed[mode].milliseconds);
          summaries[mode].expansions.push_back(double(observed[mode].expanded));
          ++summaries[mode].successes;
        }
        matchedRatios.push_back(observed[1].milliseconds/
                                std::max(1e-9,observed[0].milliseconds));
        fasterConnector+=observed[0].milliseconds<observed[1].milliseconds;
        fasterDistance+=observed[1].milliseconds<observed[0].milliseconds;
      }
      if(candidates%100==0 || (qualified>0 && qualified%10==0 &&
          candidates%20==0))
        std::cerr<<"progress candidates="<<candidates<<" accepted="<<qualified
                 <<" rejected_failures="<<rejectedFailure
                 <<" rejected_easy="<<rejectedExpansions<<'\\n';
      // Keep complete qualified CSV output useful even when interrupted.
      if(qualified>0 && qualified%10==0)std::cout<<std::flush;
    }
    std::cerr<<std::fixed<<std::setprecision(3)
      <<"seed="<<seed<<" target="<<scenarioCount
      <<" accepted="<<qualified
      <<" evaluated_candidates="<<candidates<<" sampled_pose_pairs="<<poseDraws
      <<" min_expansions_both="<<minimum<<" budget="<<budget
      <<" min_direct_distance_m="<<minDirectDistanceMeters<<'\\n'
      <<"rejected_failure_any="<<rejectedFailure
      <<" rejected_under_minimum_any="<<rejectedExpansions
      <<" (both_under="<<rejectedBothFast
      <<" connector_only_under="<<rejectedFastAOnly
      <<" distance_only_under="<<rejectedFastBOnly<<")\\n"
      <<"all_candidates_connector_success="<<anySuccessA
      <<" all_candidates_distance_success="<<anySuccessB
      <<" all_candidates_connector_at_least_min="<<anyExpandedA
      <<" all_candidates_distance_at_least_min="<<anyExpandedB<<'\\n';
    for(std::size_t mode=0;mode<2;++mode)
      std::cerr<<modeName(mode)<<" qualified="<<summaries[mode].successes
               <<" median_ms="<<quantile(summaries[mode].times,.5)
               <<" p95_ms="<<quantile(summaries[mode].times,.95)
               <<" median_expanded="<<quantile(summaries[mode].expansions,.5)<<'\\n';
    std::cerr<<"qualified_connector_faster="<<fasterConnector
             <<" qualified_distance_faster="<<fasterDistance
             <<" median_distance_to_connector_time_ratio="
             <<quantile(matchedRatios,.5)<<'\\n';
    if(qualified<scenarioCount) {
      std::cerr<<"ERROR: could not find 100 qualifying pairs within max_candidates="
               <<maxCandidates<<"; partial results preserved in CSV.\\n";
      return 2;
    }
    return 0;
  } catch(const std::exception& error) {
    std::cerr<<"warehouse RS randomized benchmark failed: "<<error.what()<<'\\n';
    return 1;
  }
}
