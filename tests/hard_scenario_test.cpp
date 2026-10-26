#include <catch2/catch_test_macros.hpp>
#include "bench/hard_scenarios.hpp"
#include "hybrid_astar/ackermann/refinement.hpp"
#ifdef HYBRID_ASTAR_HARD_CASES_WITH_OMPL
#include "hybrid_astar/steering/reeds_shepp_evaluator.hpp"
#endif
#include <algorithm>
#include <cmath>
#include <string>
using namespace hybrid_astar;
using namespace hybrid_astar::ackermann;

namespace {
PlanResult execute(PlanRequest q,bool performance,bool ompl) {
  if(performance)q.options.heuristicMode=HeuristicMode::PerformanceReedsSheppAndObstacleGrid;
#ifdef HYBRID_ASTAR_HARD_CASES_WITH_OMPL
  if(ompl)return steering::planWithReedsShepp(std::move(q),performance);
#else
  (void)ompl;
#endif
  return plan(q);
}
void independentlyVerified(const PlanRequest& q,const PlanResult& path) {
  REQUIRE(path.status==PlanStatus::Success);
  REQUIRE(path.collisionEvidence==CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
  REQUIRE(path.modelGeometryValid);
  REQUIRE_FALSE(path.trajectoryActuatorFeasible);
  REQUIRE(path.mapRevision==q.map->revision);
  REQUIRE(path.mapSourceId==q.map->sourceId);
  REQUIRE(path.mapFrameId==q.map->frameId);
  REQUIRE(path.goalXYErrorMeters<=q.options.goalXYToleranceMeters+1e-8);
  REQUIRE(path.goalYawErrorRadians<=q.options.goalYawToleranceRadians+1e-8);
  RefinementOptions options;
  options.maxAttempts=1; // refiner requires a positive attempt allowance
  options.maxAccepted=1;
  options.minimumCostReductionMeters=1e9; // never accept a replacement
  options.maxTotalPoseChecks=250000;
  const auto result=refinePath(q,path,{},options);
  INFO("Independent strict path revalidation status "<<static_cast<int>(result.status));
  REQUIRE(result.path.status==PlanStatus::Success);
  REQUIRE((result.status==RefinementStatus::Unchanged ||
           result.status==RefinementStatus::WorkBudgetExceeded));
  REQUIRE(result.path.collisionEvidence==CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
}
void comparable(const PlanResult& a,const PlanResult& b) {
  REQUIRE(a.status==b.status);
  REQUIRE(a.diagnostics.expanded==b.diagnostics.expanded);
  REQUIRE(a.diagnostics.generated==b.diagnostics.generated);
  REQUIRE(a.diagnostics.gridSettlements==b.diagnostics.gridSettlements);
  if(a.status==PlanStatus::Success) {
    REQUIRE(a.sections.size()==b.sections.size());
    REQUIRE(std::abs(a.weightedCostMeters-b.weightedCostMeters)<1e-8);
    REQUIRE(std::abs(a.reached.x-b.reached.x)<1e-8);
    REQUIRE(std::abs(a.reached.y-b.reached.y)<1e-8);
  }
}
}
TEST_CASE("Hard scenarios have reproducible finite validated planner results",
          "[hard-scenarios][w3]") {
  for(const auto& item:bench::hardCases()) {
    DYNAMIC_SECTION(item.name) {
      auto request=item.request;
      // Identical budgets and map, only ordering changes.
      for(bool performance:{false,true}) {
        INFO(performance?"noncertified performance grid":"certified Euclidean");
        auto first=execute(request,performance,false);
        auto second=execute(request,performance,false);
        comparable(first,second);
        REQUIRE(first.diagnostics.expanded<=request.options.maxExpansions);
        REQUIRE(first.diagnostics.generated<=request.options.maxGeneratedNodes);
        REQUIRE(first.diagnostics.finalPoseChecks<=request.options.maxTotalFinalPoseChecks);
        if(performance) {
          REQUIRE_FALSE(first.heuristicCertifiedAdmissible);
          REQUIRE(first.diagnostics.gridSettlements<=request.options.gridMaxTotalSettlements);
          REQUIRE(first.diagnostics.gridSeedCellsExamined<=16);
          REQUIRE(first.diagnostics.gridSeedCellsAccepted<=first.diagnostics.gridSeedCellsExamined);
        } else REQUIRE(first.heuristicCertifiedAdmissible);
        if(first.status==PlanStatus::Success) {
          REQUIRE_FALSE(item.impossibleByConstruction);
          independentlyVerified(request,first);
          RefinementOptions tuning;
          tuning.maxAttempts=8;tuning.maxAccepted=1;
          tuning.useCurvatureBoundedShortcuts=true;
          const auto refined=refinePath(request,first,{},tuning);
          REQUIRE(refined.path.status==PlanStatus::Success);
          REQUIRE(refined.path.weightedCostMeters<=first.weightedCostMeters+1e-8);
          REQUIRE(refined.path.mapRevision==request.map->revision);
        } else {
          REQUIRE(first.collisionEvidence==CollisionEvidence::None);
          REQUIRE(first.sections.empty());
          REQUIRE(first.status!=PlanStatus::InternalError);
        }
        if(item.baselineExpectedSuccess&&!performance)
          REQUIRE(first.status==PlanStatus::Success);
      }
    }
  }
}
TEST_CASE("W6 cumulative grid budget preserves bounded finite fallback",
          "[hard-scenarios][w6]") {
  auto q=bench::hardCases().front().request;
  q.options.heuristicMode=HeuristicMode::PerformanceReedsSheppAndObstacleGrid;
  q.options.gridMaxSettlementsPerQuery=1;
  q.options.gridMaxTotalSettlements=2;
  const auto result=plan(q);
  REQUIRE(result.diagnostics.gridSettlements<=2);
  REQUIRE_FALSE(result.heuristicCertifiedAdmissible);
  REQUIRE(result.status!=PlanStatus::InternalError);
  if(result.status==PlanStatus::Success)independentlyVerified(q,result);
  // A zero global heuristic budget is a valid finite fallback configuration.
  q.options.gridMaxTotalSettlements=0;
  const auto zero=plan(q);
  REQUIRE(zero.diagnostics.gridSettlements==0);
  REQUIRE(zero.status!=PlanStatus::InternalError);
  if(zero.status==PlanStatus::Success)independentlyVerified(q,zero);
}
#ifdef HYBRID_ASTAR_HARD_CASES_WITH_OMPL
TEST_CASE("Real OMPL paths remain strictly verified on hard maps",
          "[hard-scenarios][w5]") {
  const auto cases=bench::hardCases();
  for(std::size_t index:{std::size_t(0),std::size_t(1),std::size_t(3),std::size_t(6)}) {
    const auto& item=cases[index];
    DYNAMIC_SECTION(item.name) {
      for(bool perf:{false,true}) {
        auto path=execute(item.request,perf,true);
        REQUIRE(path.status!=PlanStatus::InternalError);
        if(path.status==PlanStatus::Success) {
          REQUIRE_FALSE(item.impossibleByConstruction);
          independentlyVerified(item.request,path);
        } else REQUIRE(path.collisionEvidence==CollisionEvidence::None);
      }
    }
  }
}
#endif
