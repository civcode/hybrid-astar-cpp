#include <catch2/catch_test_macros.hpp>
#include "apps/demo/demo.hpp"
#include "apps/demo/map_io.hpp"
#include <sstream>
#include <stdexcept>
#include <string>
using namespace hybrid_astar::demo;
using namespace hybrid_astar::ackermann;

TEST_CASE("PGM occupancy import preserves world-Y orientation", "[app][pgm]") {
  SECTION("ASCII P2 supports header comments and unknown gray cells") {
    std::istringstream input("P2\n# 2-column map\n2 2\n255\n0 205\n255 0\n");
    auto map=parsePgm(input,.5);
    REQUIRE(map->width==2);
    REQUIRE(map->height==2);
    REQUIRE(map->resolutionMeters==.5);
    REQUIRE(map->cells[0]==0);     // bottom-left is final PGM row
    REQUIRE(map->cells[1]==100);
    REQUIRE(map->cells[2]==100);   // top-left is first PGM row
    REQUIRE(map->cells[3]==-1);
    REQUIRE(map->sourceId=="imported-pgm");
  }
  SECTION("Binary P5 preserves arbitrary raster bytes") {
    std::string input="P5\r\n2 2\r\n255\r\n";
    input+=std::string("\0\xff\xcd\x7f",4);
    std::istringstream stream(input);
    auto map=parsePgm(stream,.25);
    REQUIRE(map->cells[0]==-1);    // bottom-left 205 -> unknown
    REQUIRE(map->cells[1]==100);   // 127 -> blocked
    REQUIRE(map->cells[2]==100);   // top-left 0 -> blocked
    REQUIRE(map->cells[3]==0);     // top-right 255 -> free
  }
  SECTION("Supports grayscale maxval and rejects extra raster data") {
    std::istringstream input("P2\n2 1\n15\n0 15\n");
    auto map=parsePgm(input,.1);
    REQUIRE(map->cells[0]==100);
    REQUIRE(map->cells[1]==0);
    std::string extra="P5\n1 1\n255\n";
    extra+=std::string("\xff\xfe",2);
    std::istringstream bad(extra);
    REQUIRE_THROWS_AS(parsePgm(bad,.25),std::invalid_argument);
  }
}

TEST_CASE("Untrusted map dimensions and payloads are bounded", "[app][pgm]") {
  for(const auto& data:{"P3\n1 1\n255\n1", "P2\n1025 1\n255\n",
                        "P2\n4096 4096\n255\n", "P2\n4097 1\n255\n", "P2\n1 1\n256\n",
                        "P2\n1 1\n255\n", "P2\n1 1\n255\n256",
                        "P2\n1 1\n255\n0 0"}) {
    std::istringstream input(data);
    REQUIRE_THROWS_AS(parsePgm(input,.25),std::invalid_argument);
  }
  SECTION("Invalid world resolution rejected") {
    std::istringstream input("P2\n1 1\n255\n255\n");
    REQUIRE_THROWS_AS(parsePgm(input,0),std::invalid_argument);
  }
  SECTION("Custom scenario requires explicitly loaded map") {
    Options o;o.scenario="custom";
    REQUIRE_THROWS_AS(run(o),std::invalid_argument);
  }
}

TEST_CASE("Imported occupancy map uses real planner and provenance", "[app][pgm]") {
  std::ostringstream pixels;
  pixels<<"P2\n160 120\n255\n";
  for(int i=0;i<160*120;++i)pixels<<"255 ";
  std::istringstream input(pixels.str());
  Options o;o.scenario="custom";o.customMap=parsePgm(input,.25);
  const auto result=run(o);
  REQUIRE(result.original.status==PlanStatus::Success);
  REQUIRE(result.original.mapSourceId=="imported-pgm");
  REQUIRE(result.original.collisionEvidence==CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
  const auto json=toJson(result);
  REQUIRE(json.find("\"width\":160")!=std::string::npos);
  REQUIRE(json.find("\"validated\":true")!=std::string::npos);
  REQUIRE(json.find("\"sourceId\":\"imported-pgm\"")!=std::string::npos);
  REQUIRE(json.find("\"collisionEvidence\":\"FullFootprintBodyMotionAdaptiveSampled\"")!=std::string::npos);
  REQUIRE(json.find("\"actuatorFeasible\":false")!=std::string::npos);
  REQUIRE(capabilitiesJson(o.customMap.get()).find("\"customMap\":true")!=std::string::npos);
}

TEST_CASE("Large PGM raster remains bounded but accepts Nav2 warehouse sizes", "[app][pgm][nav2]") {
  for(const auto [width,height]:{
      std::pair<std::size_t,std::size_t>{608,566},
      {1700,1000}}) {
    const std::string header="P5\n"+std::to_string(width)+" "+
        std::to_string(height)+"\n255\n";
    std::string data=header+std::string(width*height,static_cast<char>(255));
    std::istringstream stream(data);
    const auto map=parsePgm(stream,0.03);
    REQUIRE(map->width==width);
    REQUIRE(map->height==height);
    REQUIRE(map->cells.size()==width*height);
    REQUIRE(map->resolutionMeters==0.03);
    REQUIRE(map->cells.front()==0);
  }
  // Reject headers that exceed max pixels before allocating/reading any data.
  std::istringstream huge("P5\n4096 2000\n255\n");
  REQUIRE_THROWS_AS(parsePgm(huge,0.03),std::invalid_argument);
}

TEST_CASE("Prepared Nav2 scenarios use pinned world origin and fixed snapshots", "[app][pgm][nav2]") {
  auto map=std::make_shared<MapSnapshot>();
  map->width=160;map->height=120;map->resolutionMeters=0.03;
  map->origin={-15.1,-25.0,0.0};
  map->frameId="demo/map";map->sourceId="ros-map-yaml";map->revision=1;
  map->cells.assign(map->width*map->height,0);
  for(const auto scenario:{"nav2_warehouse_full","nav2_warehouse_344128"}) {
    Options options;
    options.scenario=scenario;
    if(options.scenario=="nav2_warehouse_full")
      options.nav2WarehouseFullMap=map;
    else options.nav2Warehouse344128Map=map;
    const auto result=run(options);
    REQUIRE(result.original.status==PlanStatus::Success);
    REQUIRE(result.request.map.get()==map.get());
    REQUIRE(result.request.start.x==-15.1+160*0.03*0.2);
    REQUIRE(result.request.goal.x==-15.1+160*0.03*0.8);
    REQUIRE(result.request.start.y==-25+120*0.03*0.5);
  }
  Options unavailable;
  unavailable.scenario="nav2_warehouse_full";
  REQUIRE_THROWS_AS(run(unavailable),std::invalid_argument);
  const auto caps=capabilitiesJson(nullptr,map.get(),map.get());
  REQUIRE(caps.find("\"nav2WarehouseFull\":true")!=std::string::npos);
  REQUIRE(caps.find("\"nav2_warehouse_344128\":")!=std::string::npos);
}
