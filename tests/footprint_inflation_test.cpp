#include <catch2/catch_test_macros.hpp>
#include "hybrid_astar/ackermann/planner.hpp"
#include "hybrid_astar/ackermann/refinement.hpp"
#include "hybrid_astar/ackermann/cover_prefilter.hpp"
#include "hybrid_astar/ackermann/geometry_distance.hpp"
#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <stdexcept>

using namespace hybrid_astar;
using namespace hybrid_astar::ackermann;
namespace {
constexpr double pi=std::numbers::pi_v<double>;
std::shared_ptr<MapSnapshot> emptyMap() {
  auto m=std::make_shared<MapSnapshot>();
  m->width=m->height=100;m->resolutionMeters=.1;
  m->cells.assign(10000,0);
  m->revision=17;m->frameId="margin-test";m->sourceId="metric";
  return m;
}
void block(MapSnapshot& m,std::size_t x,std::size_t y) {
  m.cells[y*m.width+x]=100;
}
Pose2D toWorld(Pose2D p,const Pose2D& origin) {
  const double c=std::cos(origin.yaw),s=std::sin(origin.yaw);
  return {origin.x+c*p.x-s*p.y,origin.y+s*p.x+c*p.y,p.yaw+origin.yaw};
}
PlanResult raw(const PlanRequest& request,const MotionSection& section) {
  PlanResult result;
  result.status=PlanStatus::Success;
  result.modelGeometryValid=true;
  result.collisionEvidence=CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled;
  result.mapRevision=request.map->revision;
  result.mapFrameId=request.map->frameId;
  result.mapSourceId=request.map->sourceId;
  result.effectiveOccupancy=request.occupancy;
  result.sections={section};
  return result;
}
}
TEST_CASE("Metric margin is exact at a wall without raster dilation",
          "[inflation][geometry]") {
  auto map=emptyMap();block(*map,50,50); // square [5,5.1] x [5,5.1]
  const Footprint rectangle{-.2,.2,.1};
  const Pose2D pose{4.74,5.05,0}; // right face at 4.94, gap 0.06m
  REQUIRE(ExactCollisionChecker(*map,{},rectangle).check(pose)==PoseCollision::Clear);
  REQUIRE(ExactCollisionChecker(*map,{},rectangle,.05).check(pose)==PoseCollision::Clear);
  REQUIRE(ExactCollisionChecker(*map,{},rectangle,.06).check(pose)==PoseCollision::Colliding);
  REQUIRE(ExactCollisionChecker(*map,{},rectangle,.10).check(pose)==PoseCollision::Colliding);
  REQUIRE_THROWS_AS((ExactCollisionChecker(*map,{},rectangle,-.01)),std::invalid_argument);
  REQUIRE_THROWS_AS((ExactCollisionChecker(*map,{},rectangle,
                           std::numeric_limits<double>::infinity())),std::invalid_argument);
}
TEST_CASE("Metric inflation has rounded corners rather than inflated AABB corners",
          "[inflation][geometry]") {
  auto map=emptyMap();block(*map,50,50);
  const Footprint squareBody{-.2,.2,.2};
  // Closest corners are (4.84,4.84) and (5.00,5.00):
  // Euclidean gap sqrt(0.16^2 + 0.16^2) = 0.22627m.
  const Pose2D corner{4.64,4.64,0};
  REQUIRE(ExactCollisionChecker(*map,{},squareBody,.20).check(corner)==PoseCollision::Clear);
  REQUIRE(ExactCollisionChecker(*map,{},squareBody,.23).check(corner)==PoseCollision::Colliding);
}
TEST_CASE("Geometric inflation respects rotated occupancy-grid coordinates",
          "[inflation][rotation]") {
  auto map=emptyMap();block(*map,50,50);
  auto turned=std::make_shared<MapSnapshot>(*map);
  turned->origin={3,-2,.73};
  const Footprint rectangle{-.2,.2,.1};
  for(const double angle:{0.,.4,1.5,-1.8}) {
    for(const double inflation:{0.,.01,.05,.06,.23}) {
      const Pose2D local{4.74,5.05,angle};
      const auto global=toWorld(local,turned->origin);
      const auto a=ExactCollisionChecker(*map,{},rectangle,inflation).check(local);
      const auto b=ExactCollisionChecker(*turned,{},rectangle,inflation).check(global);
      INFO("angle="<<angle<<", margin="<<inflation);
      REQUIRE(a==b);
    }
  }
}
TEST_CASE("Map edges enforce the same metric margin",
          "[inflation][geometry][boundary]") {
  const auto map=emptyMap();
  const Footprint body{-.2,.2,.1};
  const Pose2D nearEdge{.25,5.,0}; // left face 0.05m from edge
  REQUIRE(ExactCollisionChecker(*map,{},body,.04).check(nearEdge)==PoseCollision::Clear);
  REQUIRE(ExactCollisionChecker(*map,{},body,.05).check(nearEdge)==PoseCollision::Colliding);
}
TEST_CASE("Inflated disc cover cannot falsely bypass exact collisions",
          "[inflation][prefilter]") {
  const auto map=emptyMap();
  block(*map,50,50);
  const Footprint body{-.35,.35,.25};
  for(double inflation:{0.,.05,.1}) {
    const ExactCollisionChecker checker(*map,{},body,inflation);
    const ConservativeDiscPrefilter discs(*map,{},body,4,inflation);
    std::size_t free=0,uncertain=0;
    for(int ix=20;ix<75;ix+=3)for(int iy=20;iy<75;iy+=3)
      for(double yaw:{0.,.4,1.5}) {
        const Pose2D pose{.1*ix+.013,.1*iy+.019,yaw};
        if(discs.definitelyClear(pose)){
          ++free;
          REQUIRE(checker.check(pose)==PoseCollision::Clear);
        }else ++uncertain;
      }
    REQUIRE(free>0);
    REQUIRE(uncertain>0);
  }
}
TEST_CASE("Planning start validation and strict post-refinement enforce inflation",
          "[inflation][planner][refinement]") {
  auto map=emptyMap();
  block(*map,50,50);
  PlanRequest request;
  request.map=map;
  request.options.model=PlanOptions::Model::DifferentialDrive;
  request.footprint={-.2,.2,.1};
  request.start={4.74,5.05,0};request.goal={3.5,5.05,0};
  request.options.footprintInflationMeters=.05;
  REQUIRE(plan(request).status!=PlanStatus::StartInCollision);
  request.options.footprintInflationMeters=.10;
  REQUIRE(plan(request).status==PlanStatus::StartInCollision);
  request.options.footprintInflationMeters=-.01;
  REQUIRE(plan(request).status==PlanStatus::InvalidRequest);
  // Revalidate the same reported motion against a nearby obstacle.
  // The physical body clears it by 0.05m but a 0.06m offset does not.
  auto wall=emptyMap();block(*wall,55,53);
  request.map=wall;
  request.footprint={-.2,.2,.25};
  request.start={4,5,0};request.goal={6,5,0};
  const auto motion=makeSection(request.start,Gear::Forward,0,2.);
  request.options.footprintInflationMeters=.04;
  const auto valid=refinePath(request,raw(request,motion));
  REQUIRE(valid.path.status==PlanStatus::Success);
  REQUIRE(valid.path.collisionEvidence==
          CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
  request.options.footprintInflationMeters=.06;
  const auto invalid=refinePath(request,raw(request,motion));
  REQUIRE(invalid.status==RefinementStatus::OriginalNotVerified);
  REQUIRE(invalid.path.sections.empty());
}
