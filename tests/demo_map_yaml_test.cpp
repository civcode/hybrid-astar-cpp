#include <catch2/catch_test_macros.hpp>
#include "apps/demo/map_yaml.hpp"
#include "apps/demo/map_io.hpp"
#include "apps/demo/demo.hpp"
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
using namespace hybrid_astar;
using namespace hybrid_astar::demo;
using namespace hybrid_astar::ackermann;

TEST_CASE("ROS map metadata includes rotated origin and occupancy policy", "[app][yaml]") {
  SECTION("Standard fields, comments, quoted image name and nonzero yaw") {
    std::istringstream yaml("# map description\nimage: 'maps/sample.pgm' # relative\n"
                            "resolution: 0.5\norigin: [5.0, -2.0, 1.5707963267948966]\n"
                            "negate: 1\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n");
    auto meta=parseRosMapYaml(yaml);
    REQUIRE(meta.image=="maps/sample.pgm");
    REQUIRE(meta.resolutionMeters==.5);
    REQUIRE(std::abs(meta.origin.yaw-1.5707963267948966)<1e-12);
    REQUIRE(meta.origin.x==5.);
    REQUIRE(meta.origin.y==-2.);
    REQUIRE(meta.negate);
  }
  SECTION("Actual checked-in YAML and PGM produce a valid world-frame plan") {
    auto map=loadRosMapYaml(HYBRID_ASTAR_SAMPLE_YAML_PATH);
    REQUIRE(map->sourceId=="ros-map-yaml");
    REQUIRE(map->width==80);
    REQUIRE(map->height==60);
    REQUIRE(std::abs(map->origin.yaw-1.5707963267948966)<1e-12);
    Options options;options.scenario="custom";options.customMap=map;
    auto outcome=run(options);
    REQUIRE(outcome.original.status==PlanStatus::Success);
    REQUIRE(std::abs(outcome.request.start.x+10)<1e-10);
    REQUIRE(std::abs(outcome.request.start.y-6)<1e-10);
    REQUIRE(std::abs(outcome.request.goal.y-30)<1e-10);
    REQUIRE(outcome.original.collisionEvidence==CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
    auto json=toJson(outcome);
    REQUIRE(json.find("\"sourceId\":\"ros-map-yaml\"")!=std::string::npos);
    REQUIRE(capabilitiesJson(map.get()).find("\"origin\":[5,-2,")!=std::string::npos);
  }
}

TEST_CASE("ROS trinary thresholds and negate remain conservative", "[app][yaml][pgm]") {
  SECTION("Unknown range remains unknown") {
    std::istringstream pgm("P2\n4 1\n255\n0 128 205 255\n");
    const auto map=parsePgm(pgm,.25,{.65,.196,false,true});
    REQUIRE(map->cells[0]==100);
    REQUIRE(map->cells[1]==-1);
    REQUIRE(map->cells[2]==-1);
    REQUIRE(map->cells[3]==0);
  }
  SECTION("Negate inverts grayscale occupancy probabilities") {
    std::istringstream pgm("P2\n3 1\n255\n0 128 255\n");
    const auto map=parsePgm(pgm,.25,{.65,.196,true,true});
    REQUIRE(map->cells[0]==0);
    REQUIRE(map->cells[1]==-1);
    REQUIRE(map->cells[2]==100);
  }
}

TEST_CASE("Malformed or unsupported ROS YAML fails explicitly", "[app][yaml]") {
  const std::string basis="image: sample.pgm\nresolution: 0.5\norigin: [0,0,0]\n";
  for(const auto& extra:{"mode: scale\n","negate: true\n",
                          "free_thresh: 0.8\noccupied_thresh: 0.65\n",
                          "extra_key: value\n","image: other.pgm\n","origin: [1,2]\n",
                          "resolution: NaN\n","origin: [0,0,inf]\n"}) {
    std::istringstream data(basis+extra);
    REQUIRE_THROWS_AS(parseRosMapYaml(data),std::invalid_argument);
  }
  std::istringstream missing("image: sample.pgm\nresolution: 0.5\n");
  REQUIRE_THROWS_AS(parseRosMapYaml(missing),std::invalid_argument);
}

TEST_CASE("Rotated occupancy cell intersects real world-frame vehicle footprint", "[app][yaml][collision]") {
  std::string raster="P2\n12 12\n255\n";
  for(int top=0;top<12;++top)
    for(int col=0;col<12;++col)
      raster+=(top==6&&col==5?"0 ":"255 ");
  std::istringstream input(raster);
  auto map=parsePgm(input,1.);
  map->origin={5,-2,1.5707963267948966};
  ExactCollisionChecker checker(*map,{},Footprint{-.2,.2,.2});
  // First PGM row is high Y, so raster row 6 is local Y cell 5.
  const Pose2D occupied{5-5.5,-2+5.5,1.5707963267948966};
  const Pose2D clear{5-2.5,-2+2.5,1.5707963267948966};
  REQUIRE(checker.check(occupied)==PoseCollision::Colliding);
  REQUIRE(checker.check(clear)==PoseCollision::Clear);
}
