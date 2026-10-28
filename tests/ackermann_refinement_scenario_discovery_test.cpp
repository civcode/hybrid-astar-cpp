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

TEST_CASE("Refinement A/B benchmark demands 100 distinct paths with 10k expansions",
          "[benchmark][scenario-discovery]") {
  char executable[]="benchmark";
  char* argv[]={executable};
  const auto config=parseArgs(1,argv);
  REQUIRE(config.maxCases==100);
  REQUIRE(config.minimumExpansions==10000);
  REQUIRE(config.searchBudget>=config.minimumExpansions);
  REQUIRE(config.maxCandidateSearches>=config.maxCases);
  REQUIRE(config.steeringWeightMetersSquared==1.0);

  const std::vector<Scenario> none;
  const auto initial=candidateAt(0,none);
  REQUIRE(std::abs(initial.start.x-4.77)<1e-10);
  REQUIRE(std::abs(initial.goal.y-33.53)<1e-10);
  std::set<std::array<std::int64_t,6>> uniquePairs;
  const auto quantized=[](double x) {
    return static_cast<std::int64_t>(std::llround(x*1000.0));
  };
  for(std::size_t i=0;i<256;++i) {
    const auto candidate=candidateAt(i,none);
    REQUIRE(candidate.start.x>0);
    REQUIRE(candidate.goal.x>0);
    const auto repeat=candidateAt(i,none);
    REQUIRE(candidate.start.x==repeat.start.x);
    REQUIRE(candidate.goal.y==repeat.goal.y);
    uniquePairs.insert({quantized(candidate.start.x),quantized(candidate.start.y),
        quantized(candidate.start.yaw),quantized(candidate.goal.x),
        quantized(candidate.goal.y),quantized(candidate.goal.yaw)});
  }
  REQUIRE(uniquePairs.size()==256);
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
