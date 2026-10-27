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
  q.options.heuristicMode=performance
      ?HeuristicMode::PerformanceReedsSheppAndObstacleGrid
      :HeuristicMode::CertifiedRegionEuclidean;
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
        INFO((performance?"noncertified performance grid":"certified Euclidean"));
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
TEST_CASE("Goal-tolerance region seeds avoid full-map grid scans",
          "[hard-scenarios][w6]") {
  auto request=bench::hardCases().front().request;
  request.options.heuristicMode=HeuristicMode::PerformanceReedsSheppAndObstacleGrid;
  // Goal center is occupied, but cells on the tolerance disk are free.
  // No graph-only NoPath or falsely certified heuristic is permitted.
  auto map=std::make_shared<MapSnapshot>(*request.map);
  const auto x=static_cast<std::size_t>(request.goal.x/map->resolutionMeters);
  const auto y=static_cast<std::size_t>(request.goal.y/map->resolutionMeters);
  map->cells[y*map->width+x]=100;
  request.map=map;
  request.options.goalXYToleranceMeters=.50;
  const auto result=plan(request);
  REQUIRE(result.status!=PlanStatus::InternalError);
  REQUIRE_FALSE(result.heuristicCertifiedAdmissible);
  REQUIRE(result.diagnostics.gridSeedCellsExamined<50);
  REQUIRE(result.diagnostics.gridSeedCellsAccepted>0);
  REQUIRE(result.diagnostics.gridSettlements<=request.options.gridMaxTotalSettlements);
  // Changes in relaxed XY graph do not certify (un)reachability for the
  // orientation- and whole-footprint-constrained search.
  if(result.status==PlanStatus::Success)independentlyVerified(request,result);
}
TEST_CASE("Hard-map goals are footprint-feasible before search",
          "[hard-scenarios][fixtures]") {
  for(const auto& item:bench::hardCases()) {
    DYNAMIC_SECTION(item.name) {
      ExactCollisionChecker checker(*item.request.map,item.request.occupancy,item.request.footprint);
      REQUIRE(checker.check(item.request.start)==PoseCollision::Clear);
      REQUIRE(checker.check(item.request.goal)==PoseCollision::Clear);
    }
  }
  const auto cases=bench::hardCases();
  const auto& escape=cases[4];
  REQUIRE(escape.name=="dead_end");
  const auto path=plan(escape.request);
  REQUIRE(path.status==PlanStatus::Success);
  REQUIRE(path.reverseLengthMeters>5.0);
  independentlyVerified(escape.request,path);
}
TEST_CASE("W6 weighted grid enables detour within the original expansion budget",
          "[hard-scenarios][w6]") {
  const auto cases=bench::hardCases();
  auto request=cases[5].request;
  REQUIRE(cases[5].name=="obstacle_detour");
  request.options.heuristicMode=HeuristicMode::PerformanceReedsSheppAndObstacleGrid;
  request.options.gridMaxTotalSettlements=20000;
  request.options.performanceHeuristicWeight=1.2;
  auto fast=plan(request);
  REQUIRE(fast.status==PlanStatus::Success);
  REQUIRE(fast.diagnostics.expanded<=request.options.maxExpansions);
  REQUIRE_FALSE(fast.heuristicCertifiedAdmissible);
  independentlyVerified(request,fast);
  auto repeat=plan(request);
  comparable(fast,repeat);

  // A larger expansion budget with unit weight shows the tradeoff: the
  // weighted first-feasible mode can return a more costly but faster plan.
  request.options.performanceHeuristicWeight=1.;
  request.options.maxExpansions=30000;
  const auto unweighted=plan(request);
  REQUIRE(unweighted.status==PlanStatus::Success);
  independentlyVerified(request,unweighted);
  REQUIRE(unweighted.diagnostics.expanded>fast.diagnostics.expanded);

  // A weighted setting must never silently change certified mode.
  request.options.heuristicMode=HeuristicMode::CertifiedRegionEuclidean;
  request.options.performanceHeuristicWeight=1.5;
  request.options.maxExpansions=1500;
  const auto certified=plan(request);
  request.options.performanceHeuristicWeight=1.;
  const auto baseline=plan(request);
  comparable(certified,baseline);
  REQUIRE(certified.heuristicCertifiedAdmissible);

  // Validate bounded configuration and maintain explicit failure status.
  request.options.performanceHeuristicWeight=.99;
  REQUIRE(plan(request).status==PlanStatus::InvalidRequest);
  request.options.performanceHeuristicWeight=3.01;
  REQUIRE(plan(request).status==PlanStatus::InvalidRequest);
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
        auto repeat=execute(item.request,perf,true);
        comparable(path,repeat);
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
