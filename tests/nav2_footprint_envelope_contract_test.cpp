// Fast correctness contracts for the manually run full-map geometry experiment.
// Does NOT load Nav2 or run long planning cases.
#include "tests/nav2_footprint_envelope_benchmark_support.hpp"
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <memory>

using namespace hybrid_astar::ackermann;
using namespace hybrid_astar::bench::footprint_envelope;

TEST_CASE("Two geometries share outer 70x50 cm extents but not corner shape",
          "[benchmark][envelope][geometry]") {
  pipeline::Config c;
  auto map=std::make_shared<MapSnapshot>();
  map->width=300;map->height=300;map->resolutionMeters=.01;
  map->cells.assign(map->width*map->height,0);
  map->origin={0,0,0};
  const auto original=pipeline::baseRequest(c,map);
  const auto sharp=configure(original,Variant::LargeRectangle);
  const auto round=configure(original,Variant::SmallInflated);
  REQUIRE(length(sharp)==.70);
  REQUIRE(width(sharp)==.50);
  REQUIRE(length(round)==.50);
  REQUIRE(width(round)==.30);
  REQUIRE(sharp.options.footprintInflationMeters==0);
  REQUIRE(round.options.footprintInflationMeters==.1);
  REQUIRE(std::abs(length(round)+2*round.options.footprintInflationMeters-
                   length(sharp))<1e-12);
  REQUIRE(std::abs(width(round)+2*round.options.footprintInflationMeters-
                   width(sharp))<1e-12);
  REQUIRE(sharp.options.gridMaxTotalSettlements==
          round.options.gridMaxTotalSettlements);
  REQUIRE(sharp.options.model==round.options.model);
  REQUIRE(sharp.options.useInflatedFootprintBroadPhase);
  REQUIRE(round.options.useInflatedFootprintBroadPhase);
  REQUIRE_FALSE(sharp.options.collectCollisionWork);
  REQUIRE_FALSE(round.options.collectCollisionWork);

  // Occupied square INSIDE the sharp larger rectangle's upper-right
  // corner, but OUTSIDE the small rectangle + circular margin (distance
  // sqrt(.09^2+.09^2) > .10): results should legitimately differ.
  map->cells[174*map->width+184]=100;
  const Pose2D pose{1.5,1.5,0};
  const ExactCollisionChecker larger(*map,sharp.occupancy,sharp.footprint,0);
  const ExactCollisionChecker inflated(*map,round.occupancy,round.footprint,.1);
  REQUIRE(larger.check(pose)==PoseCollision::Colliding);
  REQUIRE(inflated.check(pose)==PoseCollision::Clear);

  // Both geometries collide when a blocked cell is 9 cm ahead of the
  // small physical rectangle's front face (inside the 10 cm margin).
  map->cells[174*map->width+184]=0;
  map->cells[150*map->width+184]=100;
  REQUIRE(larger.check(pose)==PoseCollision::Colliding);
  REQUIRE(inflated.check(pose)==PoseCollision::Colliding);
}

TEST_CASE("Fixed collision pose bank is deterministic and independent of geometry",
          "[benchmark][envelope][micro]") {
  pipeline::Config c;
  auto map=std::make_shared<MapSnapshot>();
  map->width=1006;map->height=1674;map->resolutionMeters=.03;
  map->origin={-15.1,-25.,.61};
  map->cells.assign(map->width*map->height,0);
  const auto original=pipeline::baseRequest(c,map);
  const pipeline::Scenario scenario{0,93,{},30000};
  const auto poses=fixedPoseBank(c,scenario,*map,512);
  const auto again=fixedPoseBank(c,scenario,*map,512);
  REQUIRE(poses.size()==512);
  for(std::size_t i=0;i<poses.size();++i) {
    REQUIRE(poses[i].x==again[i].x);
    REQUIRE(poses[i].y==again[i].y);
    REQUIRE(poses[i].yaw==again[i].yaw);
  }
  auto sharp=configure(original,Variant::LargeRectangle);
  auto round=configure(original,Variant::SmallInflated);
  const auto first=benchmarkCollision(sharp,poses,2);
  const auto second=benchmarkCollision(round,poses,2);
  REQUIRE(first.poses==second.poses);
  REQUIRE(first.collisions<=first.poses);
  REQUIRE(second.collisions<=second.poses);
  REQUIRE(first.counters.distanceTests==0);
  REQUIRE(first.counters.inflatedAxisRejections==0);
  REQUIRE(first.counters.inflatedAabbRejections==0);
  REQUIRE(second.counters.distanceTests<=second.counters.satTests);
  REQUIRE(first.medianUsPerPose>=0);
  REQUIRE(second.medianUsPerPose>=0);
}
