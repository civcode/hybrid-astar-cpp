#include <catch2/catch_test_macros.hpp>
#include "hybrid_astar/ackermann/planner.hpp"
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>

using namespace hybrid_astar;
using namespace hybrid_astar::ackermann;

TEST_CASE("Constant-curvature Ackermann primitives retain exact geometry",
          "[motion][ackermann]") {
  const Pose2D origin{10,10,0};
  SECTION("Forward straight propagates at exact arc length") {
    const auto s=makeSection(origin,Gear::Forward,0,1.5);
    REQUIRE(s.lengthMeters==1.5);
    REQUIRE(std::abs(s.end.x-11.5)<1e-12);
    REQUIRE(std::abs(s.evaluate(.75).x-10.75)<1e-12);
  }
  SECTION("Reverse curvature uses signed gear without reversing steering") {
    const auto s=makeSection(origin,Gear::Reverse,.25,.8);
    REQUIRE(s.gear==Gear::Reverse);
    REQUIRE(s.curvature==.25);
    REQUIRE(std::abs(s.end.yaw+.2)<1e-12);
    REQUIRE(std::hypot(s.evaluate(.8).x-s.end.x,s.evaluate(.8).y-s.end.y)<1e-12);
  }
  SECTION("Invalid segment input fails explicitly") {
    REQUIRE_THROWS_AS(makeSection(origin,Gear::None,0,1),std::invalid_argument);
    REQUIRE_THROWS_AS(makeSection(origin,Gear::Forward,0,-1),std::invalid_argument);
    REQUIRE_THROWS_AS(makeSection(origin,Gear::Forward,
                                  std::numeric_limits<double>::infinity(),1),std::invalid_argument);
  }
}

TEST_CASE("Planning exposes strictly sampled geometric evidence",
          "[planner][validation]") {
  auto map=std::make_shared<MapSnapshot>();
  map->width=200;map->height=200;map->resolutionMeters=.2;
  map->cells.assign(40000,0);map->frameId="catch2-demo";map->revision=314;
  PlanRequest request;
  request.map=map;request.start={20,20,0};request.goal={21,20,0};
  SECTION("Nominal open-field route succeeds with provenance") {
    const auto result=plan(request);
    REQUIRE(result.status==PlanStatus::Success);
    REQUIRE(result.mapRevision==314);
    REQUIRE(result.collisionEvidence==CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
    REQUIRE_FALSE(result.trajectoryActuatorFeasible);
    REQUIRE(result.goalXYErrorMeters<=request.options.goalXYToleranceMeters);
  }
  SECTION("Exhausted strict verification cannot claim success") {
    request.options.maxTotalFinalPoseChecks=1;
    const auto result=plan(request);
    REQUIRE(result.status==PlanStatus::ValidationIncomplete);
    REQUIRE(result.collisionEvidence==CollisionEvidence::None);
    REQUIRE(result.diagnostics.finalPoseChecks<=1);
  }
}
