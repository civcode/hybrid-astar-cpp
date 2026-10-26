#include <catch2/catch_test_macros.hpp>
#include "hybrid_astar/mecanum/model.hpp"
#include "hybrid_astar/mecanum/primitives.hpp"
#include "hybrid_astar/planning/model_policy.hpp"
#include "hybrid_astar/ackermann/planner.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <stdexcept>

using namespace hybrid_astar;
namespace me=hybrid_astar::mecanum;
namespace mo=hybrid_astar::motion;
namespace a=hybrid_astar::ackermann;
namespace po=hybrid_astar::planning;
static_assert(po::PlanarSearchModel<po::MecanumPolicy>);

static a::PlanRequest request() {
  auto map=std::make_shared<a::MapSnapshot>();
  map->width=map->height=100;
  map->resolutionMeters=.2;
  map->frameId="map";map->sourceId="w12-mecanum";map->revision=1212;
  map->cells.assign(10000,0);
  a::PlanRequest r;
  r.map=map;r.start={10,10,0};r.goal={10,12,0};
  r.footprint={-.35,.35,.25};
  r.options.model=a::PlanOptions::Model::Mecanum;
  r.options.maxExpansions=8000;
  r.options.maxGeneratedNodes=32000;
  return r;
}

TEST_CASE("Mecanum wheel mapping bounds are enforced on all four wheels","[w12][model]") {
  const me::Model model;
  auto rates=model.wheelRates({0,.5,0});
  REQUIRE(rates[0]==-5.0);
  REQUIRE(rates[1]==5.0);
  REQUIRE(rates[2]==5.0);
  REQUIRE(rates[3]==-5.0);
  REQUIRE(model.valid({.5,.5,.6}));
  REQUIRE_FALSE(model.valid({2,2,10}));
  REQUIRE_FALSE(model.valid({std::numeric_limits<double>::quiet_NaN(),0,0}));
  REQUIRE_THROWS_AS(me::Model(me::Parameters{0,.25,.2,20}),std::invalid_argument);
  REQUIRE_THROWS_AS(model.wheelRates({30,0,0}),std::invalid_argument);
}

TEST_CASE("Exact lateral and rotating planar exponentials match independent integration","[w12][kinematics]") {
  const me::Model model;
  const me::Control control{.35,.25,.6};
  const Pose2D start{4,-2,.7};
  const double duration=1.25;
  const Pose2D exact=model.propagate(start,control,duration);
  Pose2D integrated=start;
  constexpr int steps=20000;
  const double dt=duration/steps;
  for(int i=0;i<steps;++i) {
    // Independent midpoint numerical approximation using the model ODE.
    const auto d1=model.derivative(integrated,control);
    const Pose2D halfway{integrated.x+.5*dt*d1.dx,
                         integrated.y+.5*dt*d1.dy,
                         integrated.yaw+.5*dt*d1.dyaw};
    const auto mid=model.derivative(halfway,control);
    integrated.x+=dt*mid.dx;
    integrated.y+=dt*mid.dy;
    integrated.yaw+=dt*mid.dyaw;
  }
  REQUIRE(std::hypot(exact.x-integrated.x,exact.y-integrated.y)<1e-8);
  REQUIRE(std::abs(mo::wrapAngle(exact.yaw-integrated.yaw))<1e-10);
  const auto sideways=mo::constantHolonomicTwist(0,2,0,1);
  REQUIRE(std::abs(sideways.x)<1e-12);
  REQUIRE(std::abs(sideways.y-2)<1e-12);
}

TEST_CASE("14 cached mecanum templates retain endpoints and bounded sweeping samples","[w12][primitives]") {
  me::PrimitiveConfig config;
  config.sampling={.10,.10,.75,std::numbers::pi_v<double>,1000};
  const auto catalog=me::buildCatalog(me::Model{},config);
  REQUIRE(catalog.size()==14);
  bool lateral=false,combined=false,spin=false;
  for(const auto& p:catalog) {
    REQUIRE(p.durationSeconds>0);
    REQUIRE(p.local.samples.size()>1);
    REQUIRE(p.local.samples.front().lambda==0);
    REQUIRE(p.local.samples.back().lambda==1);
    const auto end=p.local.evaluate(1);
    const auto cached=p.local.samples.back().localPose;
    REQUIRE(std::hypot(end.x-cached.x,end.y-cached.y)<1e-12);
    REQUIRE(std::abs(mo::wrapAngle(end.yaw-cached.yaw))<1e-12);
    lateral|=p.local.lateralTranslationMeters!=0;
    combined|=p.local.lateralTranslationMeters!=0&&p.local.headingChangeRadians!=0;
    spin|=p.local.extentKind==mo::ExtentKind::RotationRadians;
  }
  REQUIRE(lateral);REQUIRE(combined);REQUIRE(spin);
  config.translationSpeed=40;
  REQUIRE_THROWS_AS(me::buildCatalog(me::Model{},config),std::invalid_argument);
}

TEST_CASE("Mecanum planner reaches a sideways goal without rotating its body","[w12][search]") {
  const auto r=request();
  const auto planned=a::plan(r);
  REQUIRE(planned.status==a::PlanStatus::Success);
  REQUIRE(planned.diagnostics.primitiveTemplatesBuilt==14);
  REQUIRE(planned.diagnostics.primitiveTemplatePoseChecks>0);
  REQUIRE(planned.collisionEvidence==a::CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
  REQUIRE(planned.modelGeometryValid);
  REQUIRE_FALSE(planned.trajectoryActuatorFeasible);
  REQUIRE(planned.heuristicCertifiedAdmissible);
  REQUIRE(planned.mapRevision==1212);
  REQUIRE(planned.goalXYErrorMeters<=r.options.goalXYToleranceMeters);
  REQUIRE(planned.goalYawErrorRadians<=r.options.goalYawToleranceRadians);
  REQUIRE(planned.weightedCostMeters>0);
  bool sideways=false;
  a::ExactCollisionChecker checker(*r.map,r.occupancy,r.footprint);
  for(const auto& section:planned.sections) {
    if(section.kind==a::MotionKind::HolonomicTwist &&
       std::abs(section.lateralMeters)>0)sideways=true;
    const auto extent=section.progressExtent();
    const auto exactEnd=section.evaluate(extent);
    REQUIRE(std::hypot(exactEnd.x-section.end.x,exactEnd.y-section.end.y)<1e-8);
    for(int j=0;j<=100;++j)
      REQUIRE(checker.check(section.evaluate(extent*double(j)/100))==a::PoseCollision::Clear);
  }
  REQUIRE(sideways);
  const auto repeated=a::plan(r);
  REQUIRE(repeated.status==planned.status);
  REQUIRE(repeated.diagnostics.expanded==planned.diagnostics.expanded);
  REQUIRE(std::abs(repeated.weightedCostMeters-planned.weightedCostMeters)<1e-8);
}

TEST_CASE("W11 runtime adapter reuses the identical mecanum search kernel","[w12][policy]") {
  auto r=request();
  const auto native=a::plan(r);
  const po::PlanarPolicyAdapter<po::MecanumPolicy> adapter{po::MecanumPolicy{r}};
  const auto external=a::planWithPolicy(r,adapter);
  REQUIRE(native.status==a::PlanStatus::Success);
  REQUIRE(external.status==native.status);
  REQUIRE(external.diagnostics.expanded==native.diagnostics.expanded);
  REQUIRE(external.diagnostics.generated==native.diagnostics.generated);
  REQUIRE(external.diagnostics.primitiveTemplatesBuilt==native.diagnostics.primitiveTemplatesBuilt);
  REQUIRE(external.sections.size()==native.sections.size());
  REQUIRE(external.mapRevision==native.mapRevision);
  REQUIRE(external.mapFrameId==native.mapFrameId);
  REQUIRE(external.mapSourceId==native.mapSourceId);
  REQUIRE(external.collisionEvidence==native.collisionEvidence);
  REQUIRE(std::abs(external.weightedCostMeters-native.weightedCostMeters)<1e-9);
  for(std::size_t i=0;i<native.sections.size();++i) {
    const auto& lhs=native.sections[i];
    const auto& rhs=external.sections[i];
    REQUIRE(lhs.kind==rhs.kind);
    REQUIRE(std::hypot(lhs.end.x-rhs.end.x,lhs.end.y-rhs.end.y)<1e-9);
    REQUIRE(std::abs(mo::wrapAngle(lhs.end.yaw-rhs.end.yaw))<1e-9);
  }
}

TEST_CASE("Combined holonomic sections are exact and independently sweep across obstacles","[w12][collision]") {
  const auto start=Pose2D{10,10,0};
  const auto section=a::makeHolonomicSection(start,0,2,.5);
  REQUIRE(section.gear==a::Gear::None);
  REQUIRE(section.kind==a::MotionKind::HolonomicTwist);
  REQUIRE(section.progressExtent()==2);
  REQUIRE(std::abs(section.end.y-start.y)>1.0);
  auto r=request();
  auto map=std::make_shared<a::MapSnapshot>(*r.map);
  const auto middle=section.evaluate(1);
  const auto x=std::size_t(middle.x/map->resolutionMeters);
  const auto y=std::size_t(middle.y/map->resolutionMeters);
  map->cells[y*map->width+x]=100;
  a::ExactCollisionChecker checker(*map,r.occupancy,r.footprint);
  REQUIRE(checker.check(start)==a::PoseCollision::Clear);
  bool collision=false;
  for(int i=0;i<=200;++i)
    if(checker.check(section.evaluate(2.0*i/200))==a::PoseCollision::Colliding)
      collision=true;
  REQUIRE(collision);
}

TEST_CASE("Incompatible mecanum connectors and impossible wheels fail closed","[w12][safety]") {
  auto r=request();
  r.options.mecanumParameters.maxAbsWheelAngularRate=.01;
  REQUIRE(a::plan(r).status==a::PlanStatus::InvalidRequest);
  r=request();
  r.analyticDistance=[](const Pose2D&,const Pose2D&){return 1.0;};
  REQUIRE(a::plan(r).status==a::PlanStatus::InvalidRequest);
  r=request();
  r.options.clearanceWeight=1.0;
  REQUIRE(a::plan(r).status==a::PlanStatus::InvalidRequest);
  r=request();
  r.options.mecanumPrimitives.translationMeters=0;
  REQUIRE(a::plan(r).status==a::PlanStatus::InvalidRequest);
}
