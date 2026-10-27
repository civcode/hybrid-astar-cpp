#include <catch2/catch_test_macros.hpp>
#include "bench/hard_scenarios.hpp"
#include "bench/warehouse_scenarios.hpp"
#include "hybrid_astar/ackermann/planner.hpp"
#include "hybrid_astar/ackermann/cover_prefilter.hpp"
#include <cmath>
#include <memory>
using namespace hybrid_astar;
using namespace hybrid_astar::ackermann;

TEST_CASE("Ackermann warehouse profiling cases match the reported poses and margins",
          "[w6][benchmark-fixture]") {
  const auto cases=bench::ackermannWarehouseCases(200000);
  REQUIRE(cases.size()==5);
  for(const auto& item:cases) {
    const auto& q=item.second;
    REQUIRE(q.options.model==PlanOptions::Model::Ackermann);
    REQUIRE(q.options.maxExpansions==200000);
    REQUIRE(q.options.maxGeneratedNodes==800000);
    REQUIRE(q.map->width==200);
    REQUIRE(q.map->height==160);
    REQUIRE(q.map->resolutionMeters==.25);
    REQUIRE(q.footprint.frontX>3.);
    REQUIRE(q.footprint.halfWidth>.5);
    REQUIRE(q.map->cells[20*q.map->width+50]==100);
  }
  REQUIRE(cases[0].first=="ack_across_first_margin_050");
  REQUIRE(cases[0].second.start.x==4.77);
  REQUIRE(cases[0].second.start.y==32.84);
  REQUIRE(cases[0].second.goal.x==44.62);
  REQUIRE(cases[0].second.options.footprintInflationMeters==.5);
  REQUIRE(cases[1].second.options.solutionPolicy==SolutionPolicy::ImproveIncumbent);
  REQUIRE(cases[2].second.start.x==3.25);
  REQUIRE(cases[2].second.goal.y==38.17);
  REQUIRE(cases[2].second.options.footprintInflationMeters==0.);
  REQUIRE(cases[3].second.options.footprintInflationMeters==.1);
  REQUIRE(cases[4].second.options.footprintInflationMeters==.5);
}

TEST_CASE("Obstacle grid is the default and certified Euclidean is explicit",
          "[heuristic][default][w6]") {
  const PlanOptions defaults;
  REQUIRE(defaults.heuristicMode==HeuristicMode::PerformanceReedsSheppAndObstacleGrid);
  REQUIRE(defaults.performanceHeuristicWeight==1.0);
  auto request=bench::hardCases().front().request;
  const auto grid=plan(request);
  REQUIRE(grid.status==PlanStatus::Success);
  REQUIRE_FALSE(grid.heuristicCertifiedAdmissible);
  REQUIRE(grid.diagnostics.gridSettlements>0);
  request.options.heuristicMode=HeuristicMode::CertifiedRegionEuclidean;
  const auto certified=plan(request);
  REQUIRE(certified.status==PlanStatus::Success);
  REQUIRE(certified.heuristicCertifiedAdmissible);
  REQUIRE(certified.diagnostics.gridSettlements==0);
  REQUIRE(grid.collisionEvidence==
          CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
  REQUIRE(certified.collisionEvidence==
          CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
}

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

TEST_CASE("Conservative discs never claim collision-free for an occupied rotated-map pose",
          "[w6][prefilter][collision]") {
  auto map=std::make_shared<MapSnapshot>();
  map->width=120;map->height=100;map->resolutionMeters=.2;
  map->cells.assign(map->width*map->height,0);
  for(std::size_t y=28;y<72;++y)
    for(std::size_t x=49;x<53;++x)map->cells[y*map->width+x]=100;
  const Footprint footprints[]={{-1.0,3.6,.9},{-.35,.35,.25}};
  auto rotated=std::make_shared<MapSnapshot>(*map);
  rotated->origin={5.,-3.,1.2};
  const double cs=std::cos(rotated->origin.yaw),sn=std::sin(rotated->origin.yaw);
  for(const auto& footprint:footprints) {
    const ExactCollisionChecker direct(*map,{},footprint);
    const ExactCollisionChecker world(*rotated,{},footprint);
    const ConservativeDiscPrefilter localCover(*map,{},footprint,4);
    const ConservativeDiscPrefilter worldCover(*rotated,{},footprint,4);
    std::size_t definitelyFree=0,uncertain=0;
    for(int iy=4;iy<96;iy+=4)for(int ix=4;ix<116;ix+=4)
      for(int angle=0;angle<8;++angle) {
        const Pose2D local{ix*.2+.037,iy*.2+.023,angle*.39269908169872414};
        const Pose2D remote{5.+cs*local.x-sn*local.y,
                            -3.+sn*local.x+cs*local.y,
                            local.yaw+rotated->origin.yaw};
        const auto directStatus=direct.check(local);
        REQUIRE(world.check(remote)==directStatus);
        if(localCover.definitelyClear(local)) {
          REQUIRE(directStatus==PoseCollision::Clear);
          ++definitelyFree;
        } else ++uncertain;
        if(worldCover.definitelyClear(remote))
          REQUIRE(world.check(remote)==PoseCollision::Clear);
      }
    REQUIRE(definitelyFree>0);
    REQUIRE(uncertain>0);
  }
}

TEST_CASE("Four-disc safety remains conservative at obstacle contact and in narrow gaps",
          "[w6][prefilter][collision]") {
  auto map=std::make_shared<MapSnapshot>();
  map->width=100;map->height=80;map->resolutionMeters=.1;
  map->cells.assign(map->width*map->height,0);
  // Narrow (0.4m) gap cannot admit even the compact 0.5m-wide chassis.
  for(std::size_t y=0;y<map->height;++y)
    if(y<38||y>=42)map->cells[y*map->width+50]=100;
  map->cells[50*map->width+60]=100; // isolated contact obstacle
  const Footprint body{-.35,.35,.25};
  const ExactCollisionChecker checker(*map,{},body);
  const ConservativeDiscPrefilter cover(*map,{},body,4);
  bool collisionSeen=false,clearSeen=false;
  // Fractional-cell positions probe contact/sliver cases and map boundaries.
  for(int ix=6;ix<94;ix+=2)for(int iy=5;iy<75;iy+=2)
    for(double yaw:{0.,.25,.7853981633974483,1.5707963267948966}) {
      const Pose2D pose{ix*.1+.003,iy*.1+.097,yaw};
      const bool prefiltered=cover.definitelyClear(pose);
      const auto status=checker.check(pose);
      if(status==PoseCollision::Colliding)collisionSeen=true;
      if(status==PoseCollision::Clear)clearSeen=true;
      INFO("x="<<pose.x<<", y="<<pose.y<<", yaw="<<yaw);
      if(prefiltered) REQUIRE(status==PoseCollision::Clear);
    }
  REQUIRE(collisionSeen);
  REQUIRE(clearSeen);
}

TEST_CASE("Four-disc prefilter preserves mecanum and narrow-gap planner results",
          "[w6][prefilter][w12]") {
  auto map=std::make_shared<MapSnapshot>();
  map->width=100;map->height=80;map->resolutionMeters=.1;
  map->cells.assign(map->width*map->height,0);
  for(std::size_t y=0;y<map->height;++y)
    if(y<38||y>=42)map->cells[y*map->width+50]=100;
  const auto compare=[&](PlanRequest q) {
    q.options.model=PlanOptions::Model::Mecanum;
    q.footprint={-.35,.35,.25};
    q.options.maxExpansions=450;
    q.options.maxGeneratedNodes=1800;
    q.options.discPrefilterCount=0;
    const auto baseline=plan(q);
    q.options.discPrefilterCount=4;
    const auto accelerated=plan(q);
    INFO("baseline="<<int(baseline.status)<<", cover="<<int(accelerated.status));
    REQUIRE(accelerated.status==baseline.status);
    REQUIRE(accelerated.diagnostics.expanded==baseline.diagnostics.expanded);
    REQUIRE(accelerated.diagnostics.generated==baseline.diagnostics.generated);
    REQUIRE(accelerated.sections.size()==baseline.sections.size());
    REQUIRE(std::abs(accelerated.weightedCostMeters-baseline.weightedCostMeters)<1e-9);
    REQUIRE(accelerated.collisionEvidence==baseline.collisionEvidence);
    REQUIRE(accelerated.diagnostics.discDefinitelyClear>0);
    if(baseline.status==PlanStatus::Success)
      REQUIRE(accelerated.collisionEvidence==
              CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
  };
  PlanRequest request;
  request.map=map;
  request.start={2.,4.,0.};request.goal={3.,4.5,0.};
  compare(request);  // Nearby feasible mecanum translation
  request.goal={8.,4.,0.};
  compare(request);  // Full-width wall with a physically impassable slit
}

TEST_CASE("Collision cell-work counters are opt-in and do not change plans",
          "[w6][collision][profile]") {
  auto request=bench::hardCases().front().request;
  request.options.maxExpansions=1000;
  request.options.maxGeneratedNodes=4000;
  request.options.collectCollisionWork=false;
  const auto plain=plan(request);
  REQUIRE_FALSE(plain.diagnostics.collectCollisionWork);
  REQUIRE(plain.diagnostics.collisionWork.candidateCells==0);
  REQUIRE(plain.diagnostics.collisionWork.satTests==0);
  request.options.collectCollisionWork=true;
  const auto measured=plan(request);
  REQUIRE(measured.diagnostics.collectCollisionWork);
  REQUIRE(measured.status==plain.status);
  REQUIRE(measured.diagnostics.expanded==plain.diagnostics.expanded);
  REQUIRE(measured.diagnostics.generated==plain.diagnostics.generated);
  REQUIRE(measured.sections.size()==plain.sections.size());
  REQUIRE(measured.diagnostics.collisionWork.candidateCells>0);
  REQUIRE(measured.diagnostics.collisionWork.occupiedCells<=
          measured.diagnostics.collisionWork.candidateCells);
  REQUIRE(measured.diagnostics.collisionWork.satTests==
          measured.diagnostics.collisionWork.occupiedCells);
  REQUIRE(measured.diagnostics.collisionWork.distanceTests==0); // no margin
  REQUIRE(measured.diagnostics.exactCollisionPoseCalls>0);
}

TEST_CASE("Four covering discs stay conservative for inflated Ackermann rectangles",
          "[w6][collision][prefilter][inflation]") {
  auto map=std::make_shared<MapSnapshot>();
  map->width=map->height=100;
  map->resolutionMeters=.25;
  map->cells.assign(10000,0);
  for(std::size_t y=30;y<70;++y)
    for(std::size_t x=48;x<52;++x)
      map->cells[y*map->width+x]=100;
  const Footprint body{-1,3.6,.9};
  for(double inflation:{0.,.5}) {
    const ExactCollisionChecker exact(*map,{},body,inflation);
    const ConservativeDiscPrefilter cover(*map,{},body,4,inflation);
    std::size_t bypass=0,fallback=0;
    for(int yi=15;yi<88;yi+=6)
      for(int xi=15;xi<88;xi+=6)
        for(double yaw:{0.,.45,1.2}) {
          const Pose2D pose{xi*.25+.013,yi*.25+.017,yaw};
          if(cover.definitelyClear(pose)) {
            ++bypass;
            REQUIRE(exact.check(pose)==PoseCollision::Clear);
          } else ++fallback;
        }
    INFO("inflation="<<inflation);
    REQUIRE(bypass>0);
    REQUIRE(fallback>0);
  }
}
