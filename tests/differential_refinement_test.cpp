#include <catch2/catch_test_macros.hpp>
#include "hybrid_astar/ackermann/refinement.hpp"
#include <cmath>
#include <memory>
#include <numbers>
#include <vector>

using namespace hybrid_astar;
using namespace hybrid_astar::ackermann;

namespace {
constexpr double pi=std::numbers::pi_v<double>;
PlanRequest openRequest() {
  auto m=std::make_shared<MapSnapshot>();
  m->width=m->height=200;
  m->resolutionMeters=.1;
  m->cells.assign(m->width*m->height,0);
  m->frameId="diff-refine/map";m->sourceId="diff-refine";m->revision=44;
  PlanRequest req;
  req.map=m;req.start={10,10,0};req.goal={12,10,0};
  req.footprint={-.35,.35,.25};
  req.options.model=PlanOptions::Model::DifferentialDrive;
  return req;
}
PlanResult raw(const PlanRequest& req,std::vector<MotionSection> sections) {
  PlanResult out;
  out.status=PlanStatus::Success;
  out.modelGeometryValid=true;
  out.collisionEvidence=CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled;
  out.mapRevision=req.map->revision;
  out.mapFrameId=req.map->frameId;
  out.mapSourceId=req.map->sourceId;
  out.effectiveOccupancy=req.occupancy;
  out.sections=std::move(sections);
  // A forged cost cannot manipulate the strict refiner.
  out.weightedCostMeters=-400;
  return out;
}
std::vector<MotionSection> squiggle(PlanRequest& req) {
  std::vector<MotionSection> sections;
  auto addSpin=[&](double angle) {
    auto section=makeRotationSection(sections.empty()?req.start:sections.back().end,angle);
    sections.push_back(section);
  };
  auto addStraight=[&](double length) {
    auto section=makeSection(sections.back().end,Gear::Forward,0,length);
    sections.push_back(section);
  };
  addSpin(.2);addStraight(1);
  addSpin(-.4);addStraight(1);addSpin(.2);
  req.goal=sections.back().end;
  return sections;
}
void checkDense(const PlanRequest& req,const PlanResult& path) {
  ExactCollisionChecker checker(*req.map,req.occupancy,req.footprint);
  Pose2D end=req.start;
  for(const auto& part:path.sections) {
    REQUIRE(std::hypot(part.start.x-end.x,part.start.y-end.y)<1e-6);
    REQUIRE(std::abs(std::remainder(part.start.yaw-end.yaw,2*pi))<1e-6);
    const auto progress=part.progressExtent();
    for(int k=0;k<=80;++k) {
      const auto sample=part.evaluate(progress*double(k)/80.);
      REQUIRE(checker.check(sample)==PoseCollision::Clear);
    }
    end=part.end;
  }
  REQUIRE(std::hypot(end.x-req.goal.x,end.y-req.goal.y)<=
          req.options.goalXYToleranceMeters+1e-8);
  REQUIRE(std::abs(std::remainder(end.yaw-req.goal.yaw,2*pi))<=
          req.options.goalYawToleranceRadians+1e-8);
}
}

TEST_CASE("Differential zigzags shorten to validated direct motions",
          "[differential][refinement][smoothing]") {
  for(const bool curved:{false,true}) {
    auto req=openRequest();
    const auto sections=squiggle(req);
    const auto original=raw(req,sections);
    RefinementOptions settings;
    settings.useCurvatureBoundedShortcuts=curved;
    settings.maxAttempts=120;
    const auto refined=refinePath(req,original,{},settings);
    INFO("curved="<<curved<<" status="<<int(refined.status)
         <<" accepted="<<refined.accepted);
    REQUIRE(refined.status==RefinementStatus::Improved);
    REQUIRE(refined.accepted>0);
    REQUIRE(refined.path.status==PlanStatus::Success);
    REQUIRE(refined.path.collisionEvidence==
            CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
    REQUIRE(refined.path.solutionQuality==SolutionQuality::FeasibleAfterRefinement);
    REQUIRE(refined.path.weightedCostMeters<2.32-1e-3);
    REQUIRE(refined.path.sections.size()<sections.size());
    REQUIRE(refined.path.geometricLengthMeters<2);
    REQUIRE(refined.path.finalGear==Gear::Forward);
    REQUIRE(refined.exactPoseChecks<=settings.maxTotalPoseChecks);
    REQUIRE_FALSE(refined.path.trajectoryActuatorFeasible);
    checkDense(req,refined.path);
  }
}

TEST_CASE("Differential refiner keeps original if stricter work cap prevents verification",
          "[differential][refinement][budget]") {
  auto req=openRequest();
  auto original=raw(req,squiggle(req));
  RefinementOptions insufficient;
  insufficient.maxTotalPoseChecks=3;
  const auto result=refinePath(req,original,{},insufficient);
  REQUIRE(result.status==RefinementStatus::OriginalNotVerified);
  REQUIRE(result.path.sections.empty());
  REQUIRE(result.exactPoseChecks<=insufficient.maxTotalPoseChecks);
  original.mapRevision++;
  REQUIRE(refinePath(req,original).status==RefinementStatus::InvalidInput);
}

TEST_CASE("Differential shortcut proposals never cut through occupied wall",
          "[differential][refinement][collision]") {
  auto req=openRequest();
  req.start={3,3,0};
  std::vector<MotionSection> sections;
  auto spin=[&](double angle) {
    sections.push_back(makeRotationSection(sections.empty()?req.start:sections.back().end,angle));
  };
  auto move=[&](double length) {
    sections.push_back(makeSection(sections.back().end,Gear::Forward,0,length));
  };
  spin(pi/2);move(5);spin(-pi/2);move(8);spin(-pi/2);move(5);spin(pi/2);
  req.goal=sections.back().end;
  auto wall=std::make_shared<MapSnapshot>(*req.map);
  for(std::size_t y=20;y<65;++y)wall->cells[y*wall->width+70]=100;
  req.map=wall;
  ExactCollisionChecker checker(*req.map,req.occupancy,req.footprint);
  REQUIRE(checker.check(req.start)==PoseCollision::Clear);
  REQUIRE(checker.check(req.goal)==PoseCollision::Clear);
  REQUIRE(checker.check({7,3,0})==PoseCollision::Colliding);
  RefinementOptions settings;
  settings.useCurvatureBoundedShortcuts=true;
  settings.maxAttempts=120;
  const auto refined=refinePath(req,raw(req,sections),{},settings);
  REQUIRE(refined.path.status==PlanStatus::Success);
  REQUIRE(refined.path.collisionEvidence==
          CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
  REQUIRE(refined.path.weightedCostMeters<=18+2*pi*.4+1e-8);
  REQUIRE(refined.exactPoseChecks<=settings.maxTotalPoseChecks);
  checkDense(req,refined.path);
}

TEST_CASE("Differential post-refiner rejects a colliding swept rotation",
          "[differential][refinement][collision][spin]") {
  auto req=openRequest();
  req.start={10,10,0};
  const auto spin=makeRotationSection(req.start,pi);
  req.goal=spin.end;
  req.footprint={-1.,1.,.10};
  auto obstacle=std::make_shared<MapSnapshot>(*req.map);
  obstacle->cells[std::size_t(10.85/.1)*obstacle->width+
                  std::size_t(10/.1)]=100;
  req.map=obstacle;
  ExactCollisionChecker checker(*req.map,req.occupancy,req.footprint);
  REQUIRE(checker.check(req.start)==PoseCollision::Clear);
  REQUIRE(checker.check(spin.end)==PoseCollision::Clear);
  const auto result=refinePath(req,raw(req,{spin}));
  REQUIRE(result.status==RefinementStatus::OriginalNotVerified);
  REQUIRE(result.path.sections.empty());
}

TEST_CASE("Mecanum remains unsupported by Ackermann/differential post-refiner",
          "[differential][refinement][model]") {
  auto req=openRequest();
  req.options.model=PlanOptions::Model::Mecanum;
  req.goal=req.start;
  const auto result=refinePath(req,raw(req,{}));
  REQUIRE(result.status==RefinementStatus::InvalidInput);
}
