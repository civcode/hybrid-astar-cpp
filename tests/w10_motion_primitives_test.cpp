#include <catch2/catch_test_macros.hpp>
#include "hybrid_astar/ackermann/primitives.hpp"
#include "hybrid_astar/ackermann/planner.hpp"
#include "hybrid_astar/differential_drive/model.hpp"
#include "hybrid_astar/differential_drive/primitives.hpp"
#include "hybrid_astar/motion/primitive.hpp"
#include <array>
#include <cmath>
#include <memory>
#include <numbers>
#include <stdexcept>

using namespace hybrid_astar;
namespace dd=hybrid_astar::differential_drive;
namespace mo=hybrid_astar::motion;
namespace ac=hybrid_astar::ackermann;
static constexpr double pi=std::numbers::pi_v<double>;
static double yawError(double a,double b) {return std::abs(mo::wrapAngle(a-b));}
static void closePose(Pose2D a,Pose2D b,double tol=1e-10) {
  REQUIRE(std::abs(a.x-b.x)<tol);
  REQUIRE(std::abs(a.y-b.y)<tol);
  REQUIRE(yawError(a.yaw,b.yaw)<tol);
}
static Pose2D rk4(const dd::Model& model,Pose2D state,dd::Control u,double t) {
  constexpr int steps=12000;
  const double dt=t/steps;
  auto advance=[&](Pose2D p,dd::Derivative k,double scale) {
    return Pose2D{p.x+scale*k.dx,p.y+scale*k.dyaw*0.+scale*k.dy,
                  p.yaw+scale*k.dyaw};
  };
  for(int i=0;i<steps;++i) {
    const auto k1=model.derivative(state,u);
    const auto k2=model.derivative(advance(state,k1,.5*dt),u);
    const auto k3=model.derivative(advance(state,k2,.5*dt),u);
    const auto k4=model.derivative(advance(state,k3,dt),u);
    state.x+=dt*(k1.dx+2*k2.dx+2*k3.dx+k4.dx)/6;
    state.y+=dt*(k1.dy+2*k2.dy+2*k3.dy+k4.dy)/6;
    state.yaw+=dt*(k1.dyaw+2*k2.dyaw+2*k3.dyaw+k4.dyaw)/6;
  }
  state.yaw=mo::wrapAngle(state.yaw);
  return state;
}

TEST_CASE("Differential-drive exact propagation agrees with independent RK4",
          "[w10][differential-drive][rk4]") {
  dd::Model model;
  const Pose2D start{3.7,-4.2,2.13};
  for(const auto u : {dd::Control{.5,0},dd::Control{-.5,0},
                      dd::Control{.4,.7},dd::Control{-.4,-.7},
                      dd::Control{0,.8},dd::Control{0,-.8},
                      dd::Control{.25,1.e-9}}) {
    REQUIRE(model.valid(u));
    closePose(model.propagate(start,u,1.3),rk4(model,start,u,1.3),1e-8);
    const auto [left,right]=model.wheelRates(u);
    REQUIRE(std::abs((left+right)*model.parameters().wheelRadiusMeters/2-u.linearMetersPerSecond)<1e-12);
    REQUIRE(std::abs((right-left)*model.parameters().wheelRadiusMeters/
                     model.parameters().trackWidthMeters-u.angularRadiansPerSecond)<1e-12);
  }
  closePose(model.propagate(start,{0,.8},1.0),{start.x,start.y,start.yaw+.8});
  REQUIRE_THROWS_AS(model.propagate(start,{.5,.1},-1),std::invalid_argument);
  REQUIRE_THROWS_AS(dd::Model(dd::Parameters{0,.5,10}),std::invalid_argument);
  REQUIRE_FALSE(model.valid({10,0}));
}

TEST_CASE("Local SE2 templates transform exactly and contain every endpoint",
          "[w10][differential-drive][primitives]") {
  dd::Model model;
  dd::PrimitiveConfig config;
  config.sampling.maxReferenceTravelMeters=.1;
  config.sampling.maxBodyPointMotionMeters=.05;
  config.sampling.maxBodyRadiusMeters=1.;
  auto catalog=dd::buildCatalog(model,config);
  REQUIRE(catalog.size()==8);
  const Pose2D anchor{-2.7,8.9,-2.1};
  for(const auto& primitive:catalog) {
    const auto& local=primitive.local;
    REQUIRE(local.samples.size()>=2);
    REQUIRE(local.samples.front().lambda==0);
    REQUIRE(local.samples.back().lambda==1);
    REQUIRE(local.extent>0);
    closePose(local.samples.front().localPose,{});
    closePose(primitive.evaluate(anchor,0),anchor);
    closePose(primitive.evaluate(anchor,1),model.propagate(anchor,primitive.control,
                                                         primitive.durationSeconds));
    for(std::size_t i=0;i<local.samples.size();++i) {
      const auto& sample=local.samples[i];
      if(i) REQUIRE(sample.lambda>local.samples[i-1].lambda);
      closePose(mo::compose(anchor,sample.localPose),
                primitive.evaluate(anchor,sample.lambda));
    }
  }
  for(std::size_t i=6;i<8;++i) {
    const auto& p=catalog[i];
    REQUIRE(p.local.extentKind==mo::ExtentKind::RotationRadians);
    REQUIRE(p.local.signedTranslationMeters==0);
    REQUIRE(p.local.samples.size()>2);
    for(const auto& sample:p.local.samples) {
      REQUIRE(sample.localPose.x==0);
      REQUIRE(sample.localPose.y==0);
    }
  }
  dd::PrimitiveConfig invalid=config;
  invalid.sampling.maxSamples=2;
  REQUIRE_THROWS_AS(dd::buildCatalog(model,invalid),std::length_error);
  invalid=config;
  invalid.translationSpeed=100;
  REQUIRE_THROWS_AS(dd::buildCatalog(model,invalid),std::invalid_argument);
}

TEST_CASE("Ackermann six local templates reproduce analytic constant curvature",
          "[w10][ackermann][primitives]") {
  ac::SearchConfig cfg;
  const mo::SampleLimits limits{.1,.15,3.75,std::numeric_limits<double>::max(),10000};
  const auto catalog=ac::buildPrimitiveCatalog(cfg,4.,limits);
  REQUIRE(catalog.size()==6);
  const Pose2D anchors[]={{2,7,0},{-4.2,6.3,.83},{5,-7,-2.75}};
  for(const auto& primitive:catalog) {
    REQUIRE(!primitive.local.samples.empty());
    REQUIRE(primitive.local.extentKind==mo::ExtentKind::TranslationMeters);
    REQUIRE(primitive.local.samples.front().lambda==0);
    REQUIRE(primitive.local.samples.back().lambda==1);
    for(auto anchor:anchors) {
      const auto section=ac::makeSection(anchor,primitive.gear,primitive.curvature,cfg.primitiveLengthMeters);
      closePose(primitive.local.endpoint(anchor),section.end);
      for(const auto& sample:primitive.local.samples)
        closePose(mo::compose(anchor,sample.localPose),
                  section.evaluate(sample.lambda*cfg.primitiveLengthMeters));
    }
  }
  auto impossible=limits;impossible.maxSamples=2;
  const auto limited=ac::buildPrimitiveCatalog(cfg,4.,impossible);
  for(const auto& primitive:limited) REQUIRE(primitive.local.samples.empty());
}

TEST_CASE("In-place turn samples detect collision missed by both endpoint checks",
          "[w10][collision][differential-drive]") {
  auto map=std::make_shared<ac::MapSnapshot>();
  map->width=map->height=200;map->resolutionMeters=.1;
  map->origin={-5,-5,.35};
  map->cells.assign(40000,0);
  // Put an obstacle in the rotated map in the path of the rectangle's
  // swept tip at the 90-degree intermediate angle, not at either endpoint.
  const Pose2D anchor=mo::compose(map->origin,{10,10,0});
  const auto obstacle=mo::compose(anchor,{0,.85,0});
  const double dx=obstacle.x-map->origin.x,dy=obstacle.y-map->origin.y;
  const double c=std::cos(map->origin.yaw),s=std::sin(map->origin.yaw);
  const auto ix=std::size_t(std::floor((c*dx+s*dy)/map->resolutionMeters));
  const auto iy=std::size_t(std::floor((-s*dx+c*dy)/map->resolutionMeters));
  map->cells[iy*map->width+ix]=100;
  ac::ExactCollisionChecker checker(*map,{},ac::Footprint{-1,1,.10});
  REQUIRE(checker.check(anchor)==ac::PoseCollision::Clear);
  const auto spin=mo::generateConstantTwist(0,pi,mo::ExtentKind::RotationRadians,
                                           {.1,.04,1.01,.2,1000});
  REQUIRE(checker.check(spin.evaluateWorld(anchor,1))==ac::PoseCollision::Clear);
  bool collision=false;
  for(const auto& sample:spin.samples) {
    if(checker.check(mo::compose(anchor,sample.localPose))==ac::PoseCollision::Colliding)
      collision=true;
  }
  REQUIRE(collision);
}

TEST_CASE("Ackermann search really instantiates and checks cached samples",
          "[w10][ackermann][search]") {
  auto map=std::make_shared<ac::MapSnapshot>();
  map->width=map->height=160;map->resolutionMeters=.25;
  map->cells.assign(map->width*map->height,0);
  ac::PlanRequest request;
  request.map=map;request.start={20,20,0};request.goal={21.5,20,0};
  auto result=ac::plan(request);
  REQUIRE(result.status==ac::PlanStatus::Success);
  REQUIRE(result.diagnostics.primitiveTemplatesBuilt==6);
  REQUIRE(result.diagnostics.primitiveTemplateInstantiations>0);
  REQUIRE(result.diagnostics.primitiveTemplatePoseChecks>0);
  REQUIRE(result.collisionEvidence==ac::CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
  for(const auto& section:result.sections)
    closePose(section.evaluate(section.lengthMeters),section.end);
}
