#include <catch2/catch_test_macros.hpp>
#include "hybrid_astar/ackermann/cover_prefilter.hpp"
#include <cmath>
#include <iostream>
#include <memory>
#include <random>
#include <stdexcept>
using namespace hybrid_astar;
using namespace hybrid_astar::ackermann;
static void require(bool x,const char* msg) {if(!x) throw std::runtime_error(msg);}
static void runLegacyAssertions() {
  std::mt19937_64 rng(198260);
  std::uniform_real_distribution<double> xy(-2,25),yaw(-3.15,3.15);
  std::uniform_int_distribution<int> cell(0,99);
  for(double resolution:{.1,.2,.4}) {
    for(UnknownPolicy unknown:{UnknownPolicy::Occupied,UnknownPolicy::Free}) {
      auto map=std::make_shared<MapSnapshot>();
      map->width=map->height=static_cast<std::size_t>(30/resolution);
      map->resolutionMeters=resolution;
      map->origin={-2,1,.3};map->cells.resize(map->width*map->height);
      for(auto& v:map->cells) {
        const auto roll=cell(rng);
        v=roll<8?100:roll<15?-1:0;
      }
      OccupancyPolicy policy{0,unknown};
      for(std::size_t discs:{3ul,4ul,5ul}) {
        Footprint footprint{-1.0,3.6,.9};
        ConservativeDiscPrefilter pre(*map,policy,footprint,discs);
        ExactCollisionChecker exact(*map,policy,footprint);
        for(int i=0;i<500;++i) {
          const Pose2D pose{xy(rng),xy(rng),yaw(rng)};
          require(!(pre.definitelyClear(pose) && exact.check(pose)!=PoseCollision::Clear),"false-safe disc cover");
        }
      }
    }
  }
  auto clear=std::make_shared<MapSnapshot>();
  clear->width=clear->height=200;clear->resolutionMeters=.2;
  clear->cells.assign(40000,0);
  ConservativeDiscPrefilter pre(*clear,{},{});
  require(pre.definitelyClear({20,20,0}),"empty map should bypass");
  require(!pre.definitelyClear({0.5,20,0}),"map-edge overhang never bypasses");
  PlanRequest request;
  request.map=clear;request.start={20,20,0};request.goal={22,20,0};
  request.options.discPrefilterCount=4;
  const auto a=plan(request);
  require(a.status==PlanStatus::Success,"prefilter enabled plan");
  require(a.diagnostics.discDefinitelyClear>0,"planner used proven bypass");
  require(a.diagnostics.finalPoseChecks>0,"independent final samples");
  require(a.collisionEvidence==CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled,"no sweep evidence upgrade");
  std::cout<<"PREFILTER PASS\n";
}

TEST_CASE("prefilter_test contracts", "[prefilter_test]") {
  runLegacyAssertions();
}
