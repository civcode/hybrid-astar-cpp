// Fast contract tests for the heavy benchmark's deterministic scenario selector.
#include <catch2/catch_test_macros.hpp>
#include "tests/ackermann_refinement_benchmark_support.hpp"
#include <array>
#include <cmath>
#include <set>
#include <vector>

using namespace hybrid_astar;
using namespace hybrid_astar::ackermann;
using namespace hybrid_astar::bench::ackermann_refinement;

TEST_CASE("A/B benchmark requires 100 validated 10k-expansion routes",
          "[benchmark][scenario-discovery]") {
  char executable[]="benchmark";
  char* argv[]={executable};
  const auto config=parseArgs(1,argv);
  REQUIRE(config.maxCases==100);
  REQUIRE(config.minimumExpansions==10000);
  REQUIRE(config.searchBudget>=config.minimumExpansions);
  REQUIRE(config.maxCandidateSearches==0); // continue until target by default
  REQUIRE(config.seed==20261028);
  REQUIRE(config.steeringWeightMetersSquared==1.0);
  REQUIRE(minCoveragePerRegion(config)==5);

  CoverageCounts startCoverage{},goalCoverage{};
  REQUIRE_FALSE(coverageSatisfied(startCoverage,goalCoverage,5));
  startCoverage.fill(5);
  goalCoverage.fill(5);
  REQUIRE(coverageSatisfied(startCoverage,goalCoverage,5));
  --goalCoverage[8];
  REQUIRE_FALSE(coverageSatisfied(startCoverage,goalCoverage,5));
}

TEST_CASE("Seeded random poses explore the complete unchanged warehouse map",
          "[benchmark][scenario-discovery]") {
  const auto config=Config{};
  const auto request=hybrid_astar::bench::mecanumWarehouseCase();
  const auto& map=*request.map;
  CoverageCounts acceptedStarts{},acceptedGoals{};
  CoverageCounts sampledStarts{},sampledGoals{};
  std::set<std::array<std::int64_t,6>> uniquePairs;
  std::size_t sampled=0;
  bool startYawNegative=false,startYawPositive=false;
  bool goalYawNegative=false,goalYawPositive=false;
  const auto quantized=[](double x) {
    return static_cast<std::int64_t>(std::llround(x*1000.0));
  };
  // No planning here: a fast, deterministic test of the sampling mechanism.
  for(std::size_t i=0;i<1500;++i) {
    const auto maybe=candidateAt(i,config,acceptedStarts,acceptedGoals,map);
    REQUIRE(maybe.has_value());
    const auto a=*maybe;
    REQUIRE(a.startRegion<coverageRegions);
    REQUIRE(a.goalRegion<coverageRegions);
    REQUIRE(a.startRegion!=a.goalRegion);
    REQUIRE(regionOf(a.start,map)==a.startRegion);
    REQUIRE(regionOf(a.goal,map)==a.goalRegion);
    REQUIRE(std::hypot(a.start.x-a.goal.x,a.start.y-a.goal.y)>=18.0);
    REQUIRE(std::isfinite(a.start.yaw));
    REQUIRE(std::isfinite(a.goal.yaw));
    REQUIRE(a.start.x>4.4);
    REQUIRE(a.goal.y>4.4);
    const auto repeat=candidateAt(i,config,acceptedStarts,acceptedGoals,map);
    REQUIRE(repeat.has_value());
    REQUIRE(repeat->start.x==a.start.x);
    REQUIRE(repeat->start.y==a.start.y);
    REQUIRE(repeat->start.yaw==a.start.yaw);
    REQUIRE(repeat->goal.x==a.goal.x);
    REQUIRE(repeat->goal.y==a.goal.y);
    REQUIRE(repeat->goal.yaw==a.goal.yaw);
    startYawNegative|=a.start.yaw<0;
    startYawPositive|=a.start.yaw>0;
    goalYawNegative|=a.goal.yaw<0;
    goalYawPositive|=a.goal.yaw>0;
    ++sampledStarts[a.startRegion];
    ++sampledGoals[a.goalRegion];
    ++sampled;
    uniquePairs.insert(std::array<std::int64_t,6>{
        quantized(a.start.x),quantized(a.start.y),quantized(a.start.yaw),
        quantized(a.goal.x),quantized(a.goal.y),quantized(a.goal.yaw)});
  }
  REQUIRE(sampled==1500);
  REQUIRE(uniquePairs.size()==sampled);
  REQUIRE(startYawNegative);
  REQUIRE(startYawPositive);
  REQUIRE(goalYawNegative);
  REQUIRE(goalYawPositive);
  for(std::size_t i=0;i<coverageRegions;++i) {
    REQUIRE(sampledStarts[i]>0);
    REQUIRE(sampledGoals[i]>0);
  }
}

TEST_CASE("Coverage sampling favors underrepresented sectors",
          "[benchmark][scenario-discovery]") {
  RandomStream random(20261028);
  CoverageCounts coverage{};
  coverage.fill(7);
  coverage[4]=0;
  std::size_t centerChosen=0;
  for(std::size_t i=0;i<1000;++i)
    centerChosen+=chooseRegion(random,coverage)==4;
  REQUIRE(centerChosen>700);
}

TEST_CASE("Distinct geometry produces distinct quantized full-path signatures",
          "[benchmark][scenario-discovery]") {
  PlanResult a,b,c;
  const Pose2D start{20,20,0};
  a.sections={makeSection(start,Gear::Forward,0,4)};
  b.sections={makeSection(start,Gear::Forward,0,5)};
  c.sections=a.sections;
  REQUIRE(pathSignature(a)!=pathSignature(b));
  REQUIRE(pathSignature(a)==pathSignature(c));
  REQUIRE(signatureHash(pathSignature(a))==signatureHash(pathSignature(c)));
}
