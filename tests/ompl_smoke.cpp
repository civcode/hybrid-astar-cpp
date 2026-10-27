#include <catch2/catch_test_macros.hpp>
#include "hybrid_astar/steering/reeds_shepp.hpp"
#include "hybrid_astar/steering/reeds_shepp_evaluator.hpp"
#include "bench/warehouse_rs_fixture.hpp"
#include <array>
#include <cmath>
#include <memory>
#include <numbers>
#include <stdexcept>
using namespace hybrid_astar;
using namespace hybrid_astar::ackermann;
using namespace hybrid_astar::steering;
static void check(bool yes,const char* reason) { INFO(reason); REQUIRE(yes); }
static int runLegacyAssertions() {
  ReedsSheppEvaluator evaluator(4.0);
  const Pose2D start{20,20,0};
  check(evaluator.shortestPath(start,start).empty(),"zero RS path");
  const std::array<Pose2D,9> goals{{
    {21,20,0},{19,20,0},{23,22,.5},{17,18,-.7},
    {20,21,std::numbers::pi},{20,20,std::numbers::pi},
    {21,21,-std::numbers::pi/2},{25,20,1.0},{16,23,2.2}
  }};
  bool observedCusp=false;
  for(const auto& goal:goals) {
    const auto path=evaluator.shortestPath(start,goal);
    check(!path.empty(),"nonzero RS chain");
    double total=0;auto pose=start;Gear previous=Gear::None;
    for(const auto& section:path) {
      check(section.gear!=Gear::None&&section.lengthMeters>0,"signed section gear and positive arc progress");
      check(std::hypot(pose.x-section.start.x,pose.y-section.start.y)<1e-8,"section continuity");
      check(std::abs(section.curvature)<=.250000001,"curvature bounded by radius");
      if(previous!=Gear::None&&previous!=section.gear) observedCusp=true;
      previous=section.gear;
      total+=section.lengthMeters;
      pose=section.evaluate(section.lengthMeters);
    }
    check(std::hypot(pose.x-goal.x,pose.y-goal.y)<1e-5,"RS exact terminal XY");
    check(std::abs(std::remainder(pose.yaw-goal.yaw,2*std::numbers::pi))<1e-5,"RS exact terminal yaw");
    check(std::abs(total-evaluator.distance(start,goal))<1e-5,"RS length matches OMPL metric");
  }
  check(observedCusp,"test targets must exercise actual Reeds-Shepp gear reversal");
  check(std::abs(reedsSheppDistance(start,{19,20,0},4)-1)<1e-5,"convenience reverse distance");
  auto map=std::make_shared<MapSnapshot>();
  map->width=map->height=200;map->resolutionMeters=.2;
  map->cells.assign(40000,0);
  PlanRequest request;request.map=map;request.start=start;request.goal={19,20,0};
  const auto basic=planWithReedsShepp(request);
  check(basic.status==PlanStatus::Success&&basic.diagnostics.analyticAttempts==1&&
        basic.diagnostics.analyticAccepted==1,"OMPL automatically connected to complete planner");
  check(!basic.heuristicCertifiedAdmissible,"OMPL connector follows grid heuristic default");
  PlanRequest certifiedRequest=request;
  certifiedRequest.options.heuristicMode=HeuristicMode::CertifiedRegionEuclidean;
  const auto certified=planWithReedsShepp(certifiedRequest);
  check(certified.status==PlanStatus::Success&&certified.heuristicCertifiedAdmissible,
        "OMPL connector preserves explicitly selected certified Euclidean mode");
  check(basic.finalGear==Gear::Reverse&&basic.reverseLengthMeters>.9,"RS reverse path metadata");
  const auto performance=planWithReedsShepp(request,true);
  check(performance.status==PlanStatus::Success&&!performance.heuristicCertifiedAdmissible,
        "explicit noncertified RS performance ordering");
  request.goal=start;
  const auto identical=planWithReedsShepp(request);
  check(identical.status==PlanStatus::Success&&identical.sections.empty(),"zero-path goal completion");
  return 0;
}

TEST_CASE("ompl_smoke contracts", "[ompl_smoke]") {
  const int exitCode=runLegacyAssertions();
  INFO("Legacy assertion exit code: " << exitCode);
  REQUIRE(exitCode == 0);
}

TEST_CASE("Warehouse A/B/C configurations keep the same grid and use RS only when enabled",
          "[ompl][warehouse][rs-ab]") {
  // Quick CTest smoke on the EXACT edited warehouse snapshot. The full
  // 1M-expansion pose pair is an opt-in benchmark, not a routine unit test.
  const auto fixture=bench::warehouseReedsSheppRequest();
  REQUIRE(fixture.map->width==200);
  REQUIRE(fixture.map->height==160);
  REQUIRE(fixture.map->resolutionMeters==.25);
  REQUIRE(fixture.map->revision==187);
  REQUIRE(fixture.map->cells[119*200+100]==100);
  REQUIRE(fixture.options.footprintInflationMeters==.5);
  REQUIRE(fixture.options.analytic.attemptRatio==3.5);
  auto request=fixture;
  // Near the same warehouse start pose, this common easy goal is safely
  // traversable in all three modes; timing the hard case belongs in bench/.
  request.goal={6.68,6.68,0};
  request.options.maxExpansions=250;
  request.options.maxGeneratedNodes=1000;
  const auto a=plan(request);
  const auto b=planWithReedsShepp(request,false);
  const auto c=planWithReedsShepp(request,true);
  INFO("grid="<<int(a.status)<<" connector="<<int(b.status)
       <<" connector+distance="<<int(c.status));
  for(const auto* result:{&a,&b,&c}) {
    REQUIRE(result->status==PlanStatus::Success);
    REQUIRE(result->collisionEvidence==
            CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
    REQUIRE(result->diagnostics.expanded<=250);
    REQUIRE(result->geometricLengthMeters>0);
  }
  REQUIRE(a.diagnostics.analyticAttempts==0);
  REQUIRE(a.diagnostics.analyticAccepted==0);
  REQUIRE(b.diagnostics.analyticAttempts>=1);
  REQUIRE(b.diagnostics.analyticAccepted>=1);
  REQUIRE(c.diagnostics.analyticAttempts>=1);
  REQUIRE(c.diagnostics.analyticAccepted>=1);
  // All three use the same noncertified obstacle-grid heuristic configuration.
  REQUIRE_FALSE(a.heuristicCertifiedAdmissible);
  REQUIRE_FALSE(b.heuristicCertifiedAdmissible);
  REQUIRE_FALSE(c.heuristicCertifiedAdmissible);
}
