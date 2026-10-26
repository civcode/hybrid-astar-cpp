#include <catch2/catch_test_macros.hpp>
#include "apps/demo/map_edits.hpp"
#include "apps/demo/demo.hpp"
#include <memory>
#include <stdexcept>
#include <string>
using namespace hybrid_astar;
using namespace hybrid_astar::ackermann;
using namespace hybrid_astar::demo;

TEST_CASE("Sparse edits create new immutable map revisions", "[app][edit]") {
  auto original=std::make_shared<MapSnapshot>();
  original->width=200;original->height=160;original->resolutionMeters=.25;
  original->origin={0,0,0};original->frameId="demo/map";
  original->sourceId="unit-test";original->revision=17;
  original->cells.assign(200*160,0);
  const auto cell=std::size_t(60*200+40);
  SECTION("Edited cells do not change the pinned source snapshot") {
    const auto edit=parseCellEdits(std::to_string(cell)+":100");
    auto changed=applyCellEdits(original,edit,1);
    REQUIRE(changed.get()!=original.get());
    REQUIRE(changed->revision==18);
    REQUIRE(changed->cells[cell]==100);
    REQUIRE(original->cells[cell]==0);
    REQUIRE(original->revision==17);
    auto reset=applyCellEdits(original,{},2);
    REQUIRE(reset->revision==19);
    REQUIRE(reset->cells[cell]==0);
    REQUIRE(changed->revision==18);
    REQUIRE(changed->cells[cell]==100);
    REQUIRE(applyCellEdits(original,{},0)==original);
  }
  SECTION("A map edit re-runs actual footprint collision and path validation") {
    Options opt;opt.scenario="custom";opt.customMap=original;
    opt.start=Pose2D{10,15,0};opt.goal=Pose2D{6,15,0};
    auto before=run(opt);
    REQUIRE(before.original.status==PlanStatus::Success);
    REQUIRE(before.original.mapRevision==17);
    opt.edits={{cell,100}};opt.editGeneration=1;
    auto after=run(opt);
    REQUIRE(after.original.status==PlanStatus::StartInCollision);
    REQUIRE(after.original.collisionEvidence==CollisionEvidence::None);
    REQUIRE(after.request.map->revision==18);
    REQUIRE(before.request.map->revision==17);
    REQUIRE(before.original.status==PlanStatus::Success);
    opt.edits={};opt.editGeneration=2;
    auto restored=run(opt);
    REQUIRE(restored.original.status==PlanStatus::Success);
    REQUIRE(restored.original.mapRevision==19);
  }
  SECTION("Unknown edit is blocked by default") {
    Options opt;opt.scenario="custom";opt.customMap=original;
    opt.start=Pose2D{10,15,0};opt.goal=Pose2D{6,15,0};
    opt.edits={{cell,-1}};opt.editGeneration=1;
    auto result=run(opt);
    REQUIRE(result.original.status==PlanStatus::StartInCollision);
    REQUIRE(result.request.map->cells[cell]==-1);
  }
}
TEST_CASE("Edit protocol rejects malformed, duplicated and noncanonical records", "[app][edit]") {
  for(const std::string input : {"0:1","1:50","-1:100","0:100,0:0","1:0,0:100",
          "0:0,","0","100:","999999:100","2:100:0","0:0,,1:100"}) {
    REQUIRE_THROWS_AS(parseCellEdits(input),std::invalid_argument);
  }
  REQUIRE(parseCellEdits("").empty());
  const auto edits=parseCellEdits("0:0,1:-1,2:100");
  REQUIRE(edits.size()==3);
  REQUIRE(edits[1].occupancy==-1);
  auto base=std::make_shared<MapSnapshot>();base->width=2;base->height=2;
  base->cells.assign(4,0);base->revision=99;
  REQUIRE_THROWS_AS(applyCellEdits(base,{{4,100}},1),std::invalid_argument);
  REQUIRE_THROWS_AS(applyCellEdits(base,{{1,100}},0),std::invalid_argument);
  REQUIRE_THROWS_AS(applyCellEdits(base,{{1,50}},1),std::invalid_argument);
  REQUIRE_THROWS_AS(applyCellEdits(base,{{1,100},{1,0}},1),std::invalid_argument);
  REQUIRE_THROWS_AS(applyCellEdits(base,{},1000000001ULL),std::invalid_argument);
  std::string many;
  for(int i=0;i<4097;++i){if(i)many+=",";many+=std::to_string(i)+":0";}
  REQUIRE_THROWS_AS(parseCellEdits(many),std::invalid_argument);
}
