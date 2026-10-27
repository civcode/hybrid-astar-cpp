// Manual-only, deterministic Ackermann Reeds-Shepp A/B/C benchmark.
// Usage: hybrid_astar_warehouse_rs_ab [repetitions=1] [expansion_budget=1000000]
// A: obstacle-grid heuristic only.
// B: obstacle-grid heuristic + adaptive analytic RS connector.
// C: the same connector + RS distance heuristic (Planning Lab's current mode).
// Uses one exact warehouse snapshot and pose pair for all modes; no refinement
// or profiling clocks are enabled so search/analytic differences remain visible.
#include "bench/warehouse_rs_fixture.hpp"
#include "hybrid_astar/steering/reeds_shepp_evaluator.hpp"
#include <array>
#include <chrono>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>

using namespace hybrid_astar;
using namespace hybrid_astar::ackermann;

namespace {
enum class Mode { GridOnly, AnalyticConnector, ConnectorAndDistance };
struct Config {Mode mode; const char* name;};
constexpr std::array<Config,3> modes{{
  {Mode::GridOnly,"grid_only"},
  {Mode::AnalyticConnector,"grid_plus_rs_connector"},
  {Mode::ConnectorAndDistance,"grid_plus_rs_connector_and_distance"}
}};
const char* statusName(PlanStatus status) {
  switch(status) {
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
std::size_t positiveInteger(const char* text,std::size_t max) {
  const std::string s(text);
  std::size_t parsed=0;
  const auto value=std::stoull(s,&parsed);
  if(parsed!=s.size()||s.empty()||s[0]=='-'||value==0||value>max)
    throw std::invalid_argument("Invalid positive integer argument");
  return static_cast<std::size_t>(value);
}
}

int main(int argc,char** argv) {
  try {
    if(argc>3) throw std::invalid_argument(
        "Usage: hybrid_astar_warehouse_rs_ab [repetitions=1] [expansion_budget=1000000]");
    const auto repetitions=argc>1?positiveInteger(argv[1],20):std::size_t{1};
    const auto budget=argc>2?positiveInteger(argv[2],2'000'000):std::size_t{1'000'000};
    // Construct the occupancy map once, before timing. Each run receives an
    // identical independent request; the scheduler/evaluator are request-local.
    const auto fixture=bench::warehouseReedsSheppRequest();
    std::cout<<"mode,run,expansion_limit,status,termination,elapsed_ms,expanded,generated,"
      "search_pose_checks,final_pose_checks,grid_settlements,analytic_attempts,"
      "analytic_accepted,rs_path_calls,rs_distance_calls,cost_m,length_m,reverse_m\n";
    for(std::size_t repeat=0;repeat<repetitions;++repeat) {
      // Rotate evaluation order between repetitions to reduce warm-cache bias.
      for(std::size_t offset=0;offset<modes.size();++offset) {
        const auto config=modes[(repeat+offset)%modes.size()];
        auto request=fixture;
        request.options.maxExpansions=budget;
        request.options.maxGeneratedNodes=budget*4;
        std::size_t pathCalls=0,distanceCalls=0;
        // Reproduces the two production planWithReedsShepp(..., false/true)
        // callback configurations, with counters for the expensive functions.
        std::unique_ptr<steering::ReedsSheppEvaluator> evaluator;
        const auto started=std::chrono::steady_clock::now();
        if(config.mode!=Mode::GridOnly) {
          evaluator=std::make_unique<steering::ReedsSheppEvaluator>(
              request.options.minimumTurningRadiusMeters);
          request.analyticConnector=[&](const Pose2D& from,const Pose2D& to) {
            ++pathCalls;
            return evaluator->shortestPath(from,to);
          };
          if(config.mode==Mode::ConnectorAndDistance)
            request.analyticDistance=[&](const Pose2D& from,const Pose2D& to) {
              ++distanceCalls;
              return evaluator->distance(from,to);
            };
        }
        const auto result=plan(request);
        const auto elapsed=std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-started).count();
        const auto& d=result.diagnostics;
        if(pathCalls!=d.analyticAttempts)
          throw std::runtime_error("Analytic callback count differs from planner diagnostics");
        std::cout<<config.name<<','<<repeat+1<<','<<budget<<','
          <<statusName(result.status)<<','<<terminationName(result.terminationReason)<<','
          <<std::fixed<<std::setprecision(3)<<elapsed<<','
          <<d.expanded<<','<<d.generated<<','<<d.searchPoseChecks<<','
          <<d.finalPoseChecks<<','<<d.gridSettlements<<','
          <<d.analyticAttempts<<','<<d.analyticAccepted<<','
          <<pathCalls<<','<<distanceCalls<<',';
        if(result.status==PlanStatus::Success)
          std::cout<<result.weightedCostMeters<<','<<result.geometricLengthMeters
                   <<','<<result.reverseLengthMeters;
        else
          std::cout<<",,";
        std::cout<<'\n'<<std::flush;
      }
    }
    return 0;
  } catch(const std::exception& e) {
    std::cerr<<"warehouse RS A/B/C benchmark: "<<e.what()<<'\n';
    return 1;
  }
}
