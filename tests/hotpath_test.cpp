#include <catch2/catch_test_macros.hpp>
#include "bench/hard_scenarios.hpp"
#include "hybrid_astar/ackermann/planner.hpp"
#include <cmath>
#include <memory>
using namespace hybrid_astar;
using namespace hybrid_astar::ackermann;

TEST_CASE("Label reservation preserves exact search outcomes on difficult maps",
          "[w6][hotpath]") {
  for(const auto& fixture:bench::hardCases()) {
    DYNAMIC_SECTION(fixture.name) {
      auto q=fixture.request;
      q.options.heuristicMode=HeuristicMode::PerformanceReedsSheppAndObstacleGrid;
      q.options.performanceHeuristicWeight=1.2;
      q.options.gridMaxTotalSettlements=20000;
      const auto baseline=plan(q);
      for(std::size_t reserved:{0ULL,4096ULL,16384ULL}) {
        q.options.searchLabelReserve=reserved;
        const auto result=plan(q);
        INFO("reserve="<<reserved);
        REQUIRE(result.status==baseline.status);
        REQUIRE(result.diagnostics.expanded==baseline.diagnostics.expanded);
        REQUIRE(result.diagnostics.generated==baseline.diagnostics.generated);
        REQUIRE(result.mapRevision==baseline.mapRevision);
        REQUIRE(result.sections.size()==baseline.sections.size());
        REQUIRE(std::abs(result.weightedCostMeters-baseline.weightedCostMeters)<1e-9);
        REQUIRE(result.collisionEvidence==baseline.collisionEvidence);
        REQUIRE(result.diagnostics.labelFinds>=result.diagnostics.expanded);
        REQUIRE(result.diagnostics.labelFindHits<=result.diagnostics.labelFinds);
        REQUIRE(result.diagnostics.labelInsertions+result.diagnostics.labelUpdates==
                result.diagnostics.generated);
        REQUIRE(result.diagnostics.labelPeakSize==result.diagnostics.labelInsertions);
        REQUIRE(result.diagnostics.exactCollisionPoseCalls>0);
        REQUIRE(result.diagnostics.openQueuePushes==result.diagnostics.generated);
        REQUIRE(result.diagnostics.openQueuePops>=result.diagnostics.expanded);
        if(result.status==PlanStatus::Success)
          REQUIRE(result.collisionEvidence==CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
      }
    }
  }
}

TEST_CASE("Optional reserve rejects impossible planner allocation request",
          "[w6][hotpath]") {
  auto q=bench::hardCases().front().request;
  q.options.maxGeneratedNodes=200;
  q.options.searchLabelReserve=201;
  REQUIRE(plan(q).status==PlanStatus::InvalidRequest);
  q.options.searchLabelReserve=0;
  REQUIRE(plan(q).status!=PlanStatus::InvalidRequest);
}

TEST_CASE("Rotated map checker preserves exact footprint decisions",
          "[w6][collision]") {
  auto map=std::make_shared<MapSnapshot>();
  map->width=120;map->height=100;map->resolutionMeters=.2;
  map->cells.assign(map->width*map->height,0);
  // Solid raster obstacle whose contact is checked in local and world frames.
  for(std::size_t y=40;y<55;++y)
    for(std::size_t x=40;x<55;++x)map->cells[y*map->width+x]=100;
  ExactCollisionChecker local(*map,{},Footprint{});
  auto rotated=std::make_shared<MapSnapshot>(*map);
  rotated->origin={5.0,-3.0,1.2};
  ExactCollisionChecker world(*rotated,{},Footprint{});
  const double c=std::cos(rotated->origin.yaw),s=std::sin(rotated->origin.yaw);
  for(int y=5;y<90;y+=3)for(int x=5;x<110;x+=3) {
    const Pose2D localPose{double(x)*.2+.037,double(y)*.2+.023,double((x+y)%57)*.1};
    const Pose2D worldPose{5.0+c*localPose.x-s*localPose.y,
                            -3.0+s*localPose.x+c*localPose.y,
                            localPose.yaw+rotated->origin.yaw};
    INFO("x="<<x<<" y="<<y);
    REQUIRE(world.check(worldPose)==local.check(localPose));
  }
}
