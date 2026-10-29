// Fast and deterministic contract checks; does not load full map or plan.
#include "tests/nav2_warehouse_pipeline_benchmark_support.hpp"
#include <catch2/catch_test_macros.hpp>
#include <array>
#include <cmath>
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
