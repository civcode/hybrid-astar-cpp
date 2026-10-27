#include <catch2/catch_test_macros.hpp>
#include "hybrid_astar/ackermann/planner.hpp"
#include "hybrid_astar/motion/primitive.hpp"
#include <cmath>
#include <memory>
#include <numbers>
using namespace hybrid_astar;
using namespace hybrid_astar::ackermann;
namespace mo=hybrid_astar::motion;
constexpr double pi=std::numbers::pi_v<double>;

static PlanRequest basic() {
  auto map=std::make_shared<MapSnapshot>();
  map->width=map->height=100;map->resolutionMeters=.2;
  map->frameId="map";map->sourceId="w10-test";map->revision=42;
  map->cells.assign(100*100,0);
  PlanRequest q;q.map=map;q.start={10,10,0};q.goal={10,10,pi/2};
  q.footprint={-.35,.35,.25};
  q.options.model=PlanOptions::Model::DifferentialDrive;
  q.options.maxExpansions=30000;q.options.maxGeneratedNodes=90000;
  return q;
}
TEST_CASE("Differential drive can plan a positive-cost stationary turn",
          "[w10][search][spin]") {
  auto q=basic();
  auto r=plan(q);
  REQUIRE(r.status==PlanStatus::Success);
  REQUIRE(r.collisionEvidence==CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
  REQUIRE(r.modelGeometryValid);
  REQUIRE_FALSE(r.trajectoryActuatorFeasible);
  REQUIRE_FALSE(r.heuristicCertifiedAdmissible);
  REQUIRE(r.mapRevision==42);
  REQUIRE(r.mapSourceId=="w10-test");
  REQUIRE(r.diagnostics.primitiveTemplatesBuilt==8);
  REQUIRE(r.diagnostics.primitiveTemplatePoseChecks>0);
  REQUIRE(r.goalXYErrorMeters<=q.options.goalXYToleranceMeters);
  REQUIRE(r.goalYawErrorRadians<=q.options.goalYawToleranceRadians);
  REQUIRE(r.weightedCostMeters>0);
  bool hasSpin=false;
  for(const auto& section:r.sections) {
    if(section.kind==MotionKind::RotateInPlace) {
      hasSpin=true;
      REQUIRE(section.lengthMeters==0);
      REQUIRE(std::abs(section.turnRadians)>0);
      REQUIRE(section.progressExtent()>0);
      REQUIRE(std::abs(section.evaluate(section.progressExtent()).x-section.start.x)<1e-12);
      REQUIRE(std::abs(section.evaluate(section.progressExtent()).y-section.start.y)<1e-12);
    }
  }
  REQUIRE(hasSpin);
  // Independent dense recheck, separate from the planner's search samples
  // and strict final-validation schedule.
  ExactCollisionChecker checker(*q.map,q.occupancy,q.footprint);
  for(const auto& section:r.sections) {
    const auto extent=section.progressExtent();
    for(int j=0;j<=80;++j)
      REQUIRE(checker.check(section.evaluate(extent*double(j)/80))==PoseCollision::Clear);
  }
  auto repeated=plan(q);
  REQUIRE(repeated.status==r.status);
  REQUIRE(repeated.diagnostics.expanded==r.diagnostics.expanded);
  REQUIRE(std::abs(repeated.weightedCostMeters-r.weightedCostMeters)<1e-9);
}
TEST_CASE("Independent final validation samples a rotating footprint",
          "[w10][collision][spin]") {
  auto q=basic();
  q.footprint={-1.,1.,.10};
  // At pose (10,10,0), (10,10.85) is clear. During a 180-degree turn,
  // the body passes through that cell despite both endpoints being clear.
  auto mutableMap=std::make_shared<MapSnapshot>(*q.map);
  mutableMap->cells[std::size_t(10.85/.2)*mutableMap->width+
                    std::size_t(10.0/.2)]=100;
  q.map=mutableMap;
  ExactCollisionChecker checker(*q.map,q.occupancy,q.footprint);
  REQUIRE(checker.check(q.start)==PoseCollision::Clear);
  const auto half=makeRotationSection(q.start,pi);
  REQUIRE(checker.check(half.end)==PoseCollision::Clear);
  bool hit=false;
  for(int i=0;i<=120;++i) {
    const auto p=half.evaluate(half.progressExtent()*double(i)/120);
    if(checker.check(p)==PoseCollision::Colliding)hit=true;
  }
  REQUIRE(hit);
  // Even with coarser search sampling, no unverified Success can escape
  // the independent full-footprint final checker.
  q.goal={q.start.x,q.start.y,pi};
  q.options.searchSampleSpacingMeters=.5;
  q.options.searchMaxBodyPointMotionMeters=2.0;
  q.options.finalMaxBodyPointMotionMeters=.025;
  q.options.maxExpansions=1;
  const auto r=plan(q);
  REQUIRE(r.status!=PlanStatus::Success);
  REQUIRE(r.collisionEvidence==CollisionEvidence::None);
}
TEST_CASE("Differential drive handles translated and rotated occupancy origins",
          "[w10][frames][spin]") {
  auto q=basic();
  auto m=std::make_shared<MapSnapshot>(*q.map);
  m->origin={2,-4,.8};
  q.map=m;
  q.start=mo::compose(m->origin,{10,10,0});
  q.goal=mo::compose(m->origin,{10,10,pi/2});
  const auto r=plan(q);
  REQUIRE(r.status==PlanStatus::Success);
  REQUIRE(r.mapRevision==42);
  REQUIRE(r.goalXYErrorMeters<=q.options.goalXYToleranceMeters);
  REQUIRE(r.goalYawErrorRadians<=q.options.goalYawToleranceRadians);
  REQUIRE(r.collisionEvidence==CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
}
TEST_CASE("Differential drive rejects unsupported analytic and zero turn cost",
          "[w10][config]") {
  auto q=basic();
  q.options.rotationCostMetersPerRadian=0;
  REQUIRE(plan(q).status==PlanStatus::InvalidRequest);
  q.options.rotationCostMetersPerRadian=.4;
  q.analyticConnector=[](const Pose2D&,const Pose2D&) {
    return std::vector<MotionSection>{};
  };
  REQUIRE(plan(q).status==PlanStatus::InvalidRequest);
  q.analyticConnector={};
  q.options.clearanceWeight=1;
  REQUIRE(plan(q).status==PlanStatus::InvalidRequest);
  q.options.clearanceWeight=0;
  q.options.diffParameters.trackWidthMeters=0;
  REQUIRE(plan(q).status==PlanStatus::InvalidRequest);
}
TEST_CASE("Existing Ackermann mode still uses six templates",
          "[w10][ackermann][regression]") {
  auto q=basic();
  q.options.model=PlanOptions::Model::Ackermann;
  q.footprint={-1,3.6,.9};
  q.start={10,10,0};q.goal={12,10,0};
  auto result=plan(q);
  REQUIRE(result.status==PlanStatus::Success);
  REQUIRE(result.diagnostics.primitiveTemplatesBuilt==6);
  REQUIRE(result.collisionEvidence==CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
}
