// Fast and deterministic contract checks; does not load full map or plan.
#include "tests/nav2_warehouse_pipeline_benchmark_support.hpp"
#include <catch2/catch_test_macros.hpp>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <set>
#include <string>

using namespace hybrid_astar;
using namespace hybrid_astar::ackermann;
using namespace hybrid_astar::bench::nav2_pipeline;

TEST_CASE("Nav2 warehouse benchmark defaults to full map, 30k minimum and paired margins",
          "[benchmark][nav2][pipeline]") {
  char cmd[]="nav2_pipeline_benchmark";
  char* args[]={cmd};
  const auto c=parseArgs(1,args);
  REQUIRE(c.yamlPath=="apps/demo/maps/nav2_warehouse_full.yaml");
  REQUIRE(c.model=="differential");
  REQUIRE(c.minimumExpansions==30000);
  REQUIRE(c.searchBudget>=c.minimumExpansions);
  REQUIRE(c.cases==10);
  REQUIRE(c.repeats==3);
  REQUIRE(c.acceptedLimit==16);
  REQUIRE(c.curved);
  REQUIRE(c.collisionMode=="both");
  REQUIRE(c.gridSettlements==100000);
  REQUIRE_FALSE(c.collectCollisionWork);
  auto opts=refinementOptions(c);
  REQUIRE(opts.maxAttempts==160);
  REQUIRE(opts.maxAccepted==16);
  REQUIRE(opts.maxTotalPoseChecks==200000);
  REQUIRE(opts.differentialRadiusVariants==4);
  REQUIRE(opts.useCurvatureBoundedShortcuts);

  Config mecanum=c;mecanum.model="mecanum";
  opts=refinementOptions(mecanum);
  REQUIRE(opts.maxAttempts==220);
  REQUIRE(opts.maxTotalPoseChecks==240000);
  REQUIRE_FALSE(opts.useCurvatureBoundedShortcuts);

  Config ack=c;ack.model="ackermann";
  opts=refinementOptions(ack);
  REQUIRE(opts.maxAttempts==160);
  REQUIRE(opts.maxTotalPoseChecks==200000);
  REQUIRE(opts.useCurvatureBoundedShortcuts);
}

TEST_CASE("Random Nav2 poses are seeded, map-origin aware and region diverse",
          "[benchmark][nav2][random]") {
  MapSnapshot m;
  m.width=1006;m.height=1674;m.resolutionMeters=.03;
  m.origin={-15.1,-25.,0.0};
  Config c;
  const Footprint f{-.35,.35,.25};
  std::set<std::size_t> starts,goals;
  for(std::size_t i=0;i<100;++i) {
    const auto a=candidateAt(i,c,m,f);
    REQUIRE(a.has_value());
    const auto again=candidateAt(i,c,m,f);
    REQUIRE(again.has_value());
    REQUIRE(a->start.x==again->start.x);
    REQUIRE(a->start.y==again->start.y);
    REQUIRE(a->start.yaw==again->start.yaw);
    REQUIRE(a->goal.x==again->goal.x);
    REQUIRE(a->goal.y==again->goal.y);
    REQUIRE(a->goal.yaw==again->goal.yaw);
    REQUIRE(a->startRegion==regionOf(a->start,m));
    REQUIRE(a->goalRegion==regionOf(a->goal,m));
    REQUIRE(a->startRegion!=a->goalRegion);
    REQUIRE(std::hypot(a->start.x-a->goal.x,a->start.y-a->goal.y)>=15.0);
    REQUIRE(a->start.x>m.origin.x);
    REQUIRE(a->start.x<m.origin.x+m.width*m.resolutionMeters);
    REQUIRE(a->goal.y>m.origin.y);
    REQUIRE(a->goal.y<m.origin.y+m.height*m.resolutionMeters);
    starts.insert(a->startRegion);
    goals.insert(a->goalRegion);
  }
  REQUIRE(starts.size()>=5);
  REQUIRE(goals.size()>=5);
  // A yawed origin must affect the generated coordinates, not the local bins.
  m.origin={-15.1,-25.,.75};
  const auto rotated=candidateAt(7,c,m,f);
  REQUIRE(rotated.has_value());
  REQUIRE(rotated->startRegion==regionOf(rotated->start,m));
}

TEST_CASE("Baseline paths must succeed and meet 30k expansions before selection",
          "[benchmark][nav2][qualify]") {
  PlanResult p;
  p.status=PlanStatus::Success;
  p.modelGeometryValid=true;
  p.collisionEvidence=CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled;
  p.diagnostics.expanded=29999;
  REQUIRE_FALSE(qualifiesBaseline(p,30000));
  ++p.diagnostics.expanded;
  REQUIRE(qualifiesBaseline(p,30000));
  p.modelGeometryValid=false;
  REQUIRE_FALSE(qualifiesBaseline(p,30000));
  p.modelGeometryValid=true;
  p.status=PlanStatus::NoPath;
  REQUIRE_FALSE(qualifiesBaseline(p,30000));
  // An inflated plan is never used for acceptance filtering in discovery.
  // It is replayed on the selected baseline cases, including failures.
}

TEST_CASE("Benchmark CLI rejects invalid model, caps and connectors",
          "[benchmark][nav2][args]") {
  char cmd[]="nav2_pipeline_benchmark";
  char model[]="--model", value[]="spaceship";
  char* badModel[]={cmd,model,value};
  REQUIRE_THROWS_AS(parseArgs(3,badModel),std::invalid_argument);
  char threshold[]="--min-expansions", tooHigh[]="600000";
  char* badThreshold[]={cmd,threshold,tooHigh};
  REQUIRE_THROWS_AS(parseArgs(3,badThreshold),std::invalid_argument);
  char accepted[]="--max-accepted", invalid[]="13";
  char* badAccepted[]={cmd,accepted,invalid};
  REQUIRE_THROWS_AS(parseArgs(3,badAccepted),std::invalid_argument);
  char connector[]="--connector", ompl[]="ompl";
  char* badConnector[]={cmd,model,value,connector,ompl};
  REQUIRE_THROWS_AS(parseArgs(5,badConnector),std::invalid_argument);
  char count[]="--cases", two[]="2", cap[]="--max-candidates", one[]="1";
  char* impossible[]={cmd,count,two,cap,one};
  REQUIRE_THROWS_AS(parseArgs(5,impossible),std::invalid_argument);
  REQUIRE(percentile({1.0,2.0,3.0,4.0,5.0},.95)==5.0);
  REQUIRE(percentile({1.0,2.0,3.0,4.0,5.0},.50)==3.0);
}


TEST_CASE("Checker A/B CLI supports grid settlements and rejects bad modes",
          "[benchmark][nav2][broadphase]") {
  char cmd[]="nav2_pipeline_benchmark", mode[]="--collision-mode",
       fast[]="fastpath", grid[]="--grid-settlements", large[]="250000",
       work[]="--collect-collision-work";
  char* args[]={cmd,mode,fast,grid,large,work};
  const auto c=parseArgs(6,args);
  REQUIRE(c.collisionMode=="fastpath");
  REQUIRE(c.gridSettlements==250000);
  REQUIRE(c.collectCollisionWork);
  char wrong[]="invalid";
  char* bad[]={cmd,mode,wrong};
  REQUIRE_THROWS_AS(parseArgs(3,bad),std::invalid_argument);
  char wrongGrid[]="75000";
  char* badGrid[]={cmd,grid,wrongGrid};
  REQUIRE_THROWS_AS(parseArgs(3,badGrid),std::invalid_argument);

  Measurement before,after;
  REQUIRE(equivalentMeasurements(before,after));
  after.raw.diagnostics.expanded=1;
  REQUIRE_FALSE(equivalentMeasurements(before,after));
}

TEST_CASE("Old CSV replay reads exact poses and skips separate margin row",
          "[benchmark][nav2][replay]") {
  const auto path=std::filesystem::temp_directory_path() /
      ("nav2_replay_contract_"+
       std::to_string(Clock::now().time_since_epoch().count())+".csv");
  const auto makeFile=[&](const char* seed) {
    std::ofstream out(path);
    REQUIRE(out.good());
    out<<"case_id,draw_index,seed,model,connector,map_cells,resolution_m,"
          "start_x_m,start_y_m,start_yaw_rad,goal_x_m,goal_y_m,goal_yaw_rad,"
          "discovery_expansions,margin_m,status\n"
       <<"0,42,"<<seed<<",differential,native,1684044,0.030000,"
          "12.460167,-19.279700,0.735790,-0.005304,13.783863,-2.392485,"
          "171652,0.000000,Success\n"
       <<"0,42,"<<seed<<",differential,native,1684044,0.030000,"
          "12.460167,-19.279700,0.735790,-0.005304,13.783863,-2.392485,"
          "171652,0.100000,Success\n";
  };
  auto map=std::make_shared<MapSnapshot>();
  map->width=1006;map->height=1674;map->resolutionMeters=.03;
  map->origin={-15.1,-25.,0};
  map->cells.resize(map->width*map->height,0);
  Config c;c.cases=1;c.replayCsv=path.string();
  const auto req=baseRequest(c,map);
  makeFile("20261029");
  const auto cases=replayScenarios(c,req);
  REQUIRE(cases.size()==1);
  REQUIRE(cases[0].id==0);
  REQUIRE(cases[0].drawIndex==42);
  REQUIRE(cases[0].discoveryExpansions==171652);
  REQUIRE(cases[0].endpoints.start.x==12.460167);
  REQUIRE(cases[0].endpoints.goal.y==13.783863);
  makeFile("20261028");
  REQUIRE_THROWS_AS(replayScenarios(c,req),std::invalid_argument);
  std::filesystem::remove(path);
}
