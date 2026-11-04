#include <catch2/catch_test_macros.hpp>
#include "hybrid_astar/articulated_steering/planner.hpp"
#include <atomic>
#include <cmath>
#include <memory>
#include <numbers>

namespace as=hybrid_astar::articulated_steering;
namespace ac=hybrid_astar::ackermann;

namespace {
as::Request example() {
  auto map=std::make_shared<ac::MapSnapshot>();
  map->width=map->height=220;
  map->resolutionMeters=.1;
  map->frameId="map";map->sourceId="articulated-test";map->revision=19;
  map->cells.assign(map->width*map->height,0);
  as::Request r;
  r.map=map;
  r.parameters={.5,.5,-1.2,1.2,1.0,.8,.1};
  r.bodies.front={-.3,.3,.15};
  r.bodies.rear={-.3,.3,.15};
  r.start={10,10,0,0};
  r.goal=r.start;
  r.options.maxExpansions=2000;
  r.options.maxGenerated=10000;
  return r;
}
as::RearState endpoint(const as::Request& r,as::Control c,double seconds) {
  as::Model model(r.parameters);
  return model.toRear(model.propagate(model.fromRear(r.start),c,seconds,.005));
}
bool close(double a,double b,double eps=1e-6) {return std::abs(a-b)<eps;}
void occupy(ac::MapSnapshot& m,double x,double y) {
  const auto ix=std::size_t(std::floor(x/m.resolutionMeters));
  const auto iy=std::size_t(std::floor(y/m.resolutionMeters));
  m.cells[iy*m.width+ix]=100;
}
}

TEST_CASE("Articulation and incoming drive mode are explicit search key dimensions",
          "[articulated][search][key]") {
  auto r=example();
  const auto q=r.start;
  const auto a=as::makeSearchKey(q,0,r.options,r.parameters);
  auto s=q;s.gamma=.2;
  REQUIRE_FALSE(a==as::makeSearchKey(s,0,r.options,r.parameters));
  REQUIRE_FALSE(a==as::makeSearchKey(q,1,r.options,r.parameters));
  REQUIRE_FALSE(a==as::makeSearchKey(q,-1,r.options,r.parameters));
  auto qPlus=q,qMinus=q;
  qPlus.thetaRear=std::numbers::pi_v<double>;
  qMinus.thetaRear=-std::numbers::pi_v<double>;
  REQUIRE(as::makeSearchKey(qPlus,0,r.options,r.parameters)==
          as::makeSearchKey(qMinus,0,r.options,r.parameters));
}

TEST_CASE("Articulated search succeeds on a full-body-checked straight route",
          "[articulated][search][strict]") {
  auto r=example();
  r.goal=endpoint(r,{.5,0},1.0);
  auto out=as::plan(r);
  REQUIRE(out.status==as::Status::Success);
  REQUIRE(out.independentlySampledBodyValidated);
  REQUIRE_FALSE(out.trajectoryActuatorFeasible);
  REQUIRE(out.mapRevision==19);
  REQUIRE(out.diagnostics.generated>1);
  REQUIRE(out.diagnostics.searchBodyChecks>0);
  REQUIRE(out.diagnostics.finalBodyChecks>0);
  REQUIRE(out.motions.size()>=1);
  REQUIRE(out.goalXYErrorMeters<=r.options.goalXYToleranceMeters);
  REQUIRE(out.goalYawErrorRadians<=r.options.goalYawToleranceRadians);
  REQUIRE(out.goalGammaErrorRadians<=r.options.goalGammaToleranceRadians);
  as::Model model(r.parameters);
  as::BodyCollisionChecker check(*r.map,r.occupancy,r.bodies);
  for(const auto& motion:out.motions) {
    REQUIRE(motion.samples.size()>2);
    REQUIRE(close(motion.samples.front().timeSeconds,0));
    REQUIRE(close(motion.samples.back().timeSeconds,motion.durationSeconds));
    for(const auto& sample:motion.samples) {
      REQUIRE(model.validState(sample.state));
      REQUIRE(check.clear(model,sample.state));
      const auto rear=model.toRear(sample.state);
      REQUIRE(close(rear.xr,sample.rear.xr,1e-8));
      REQUIRE(close(rear.yr,sample.rear.yr,1e-8));
    }
  }
}

TEST_CASE("Articulated planner searches zero-translation joint motion and reverses",
          "[articulated][search][joint]") {
  auto r=example();
  r.goal=endpoint(r,{0,.4},.5);
  r.options.goalYawToleranceRadians=.04;
  r.options.goalGammaToleranceRadians=.04;
  auto out=as::plan(r);
  REQUIRE(out.status==as::Status::Success);
  REQUIRE(out.independentlySampledBodyValidated);
  REQUIRE(out.goalGammaErrorRadians<=.04);
  bool joint=false;
  for(const auto& motion:out.motions) {
    if(motion.control.frontSpeedMetersPerSecond==0.0 &&
       motion.control.gammaRateRadiansPerSecond!=0.0) {
      joint=true;
      REQUIRE(close(motion.start.xf,motion.end.xf,1e-9));
      REQUIRE(close(motion.start.yf,motion.end.yf,1e-9));
      REQUIRE(std::abs(motion.start.thetaFront-motion.end.thetaFront)>.01);
      REQUIRE(std::abs(motion.start.thetaRear-motion.end.thetaRear)>.01);
      REQUIRE(motion.samples.size()>2);
    }
  }
  REQUIRE(joint);
  auto reverse=example();
  reverse.goal=endpoint(reverse,{-.5,0},1.0);
  auto result=as::plan(reverse);
  REQUIRE(result.status==as::Status::Success);
  bool usedReverse=false;
  for(const auto& motion:result.motions)
    usedReverse|=motion.control.frontSpeedMetersPerSecond<0;
  REQUIRE(usedReverse);
}

TEST_CASE("Front and rear footprints are checked independently",
          "[articulated][collision][two-body]") {
  auto r=example();
  as::Model model(r.parameters);
  const auto start=model.fromRear(r.start);
  auto map=std::make_shared<ac::MapSnapshot>(*r.map);
  // The front reference is 1m ahead of the rear when gamma=0.
  // The front rectangle spans x=[10.7,11.3]; rear spans [9.7,10.3].
  occupy(*map,11.1,10.0); // front-only cell
  as::BodyCollisionChecker frontBlocked(*map,r.occupancy,r.bodies);
  REQUIRE_FALSE(frontBlocked.clear(model,start));
  REQUIRE(ac::ExactCollisionChecker(*map,r.occupancy,r.bodies.rear).check(
    model.rearPose(start))==ac::PoseCollision::Clear);
  r.map=map;
  REQUIRE(as::plan(r).status==as::Status::StartInCollision);

  map=std::make_shared<ac::MapSnapshot>(*example().map);
  occupy(*map,9.9,10.0); // rear-only cell
  as::BodyCollisionChecker rearBlocked(*map,r.occupancy,r.bodies);
  REQUIRE_FALSE(rearBlocked.clear(model,start));
  REQUIRE(ac::ExactCollisionChecker(*map,r.occupancy,r.bodies.front).check(
    model.frontPose(start))==ac::PoseCollision::Clear);
  r.map=map;
  REQUIRE(as::plan(r).status==as::Status::StartInCollision);
}

TEST_CASE("Articulated planner enforces joint limits, cancellation and work budgets",
          "[articulated][search][bounds]") {
  auto r=example();
  r.goal=endpoint(r,{0,.4},.5);
  auto cancel=std::make_shared<std::atomic_bool>(true);
  r.cancelled=cancel;
  REQUIRE(as::plan(r).status==as::Status::Cancelled);
  r.cancelled.reset();
  r.start.gamma=2.0;
  REQUIRE(as::plan(r).status==as::Status::InvalidRequest);
  r=example();
  r.goal.xr=18.;
  r.options.maxExpansions=1;
  REQUIRE(as::plan(r).status==as::Status::BudgetExceeded);
  r=example();
  r.options.maxSamplesPerPrimitive=2;
  r.goal.xr=18.;
  r.options.maxExpansions=2;
  auto limited=as::plan(r);
  REQUIRE(limited.status==as::Status::ValidationIncomplete ||
          limited.status==as::Status::BudgetExceeded);
}

TEST_CASE("Stationary articulation detects an obstacle hit only between endpoints",
          "[articulated][collision][intermediate][joint]") {
  auto r=example();
  auto map=std::make_shared<ac::MapSnapshot>(*r.map);
  // Long slender front carriage: its outer corner swings across this cell
  // during gamma=0 -> 1rad but clears the cell at both endpoints.
  r.bodies.front={-.2,1.0,.07};
  occupy(*map,11.9,10.35);
  r.map=map;
  as::Model model(r.parameters);
  const auto start=model.fromRear(r.start);
  const as::Control u{0,.4};
  const auto samples=model.trajectory(start,u,2.5,.01);
  r.goal=samples.back().rear;
  as::BodyCollisionChecker checker(*r.map,r.occupancy,r.bodies);
  REQUIRE(checker.clear(model,samples.front().state));
  REQUIRE(checker.clear(model,samples.back().state));
  bool hitBetween=false;
  for(std::size_t i=1;i+1<samples.size();++i)
    if(!checker.clear(model,samples[i].state))hitBetween=true;
  REQUIRE(hitBetween);
  // The direct 1-radian joint-only primitive is NOT allowed to slip
  // through simply because its endpoints are clear.
  r.options.articulationPrimitiveRadians=1.0;
  r.options.maxExpansions=1;
  const auto result=as::plan(r);
  REQUIRE(result.status!=as::Status::Success);
  REQUIRE(result.diagnostics.rejectedSearchCollision>0);
}

TEST_CASE("Missing independent final validation budget cannot produce Success",
          "[articulated][strict][budget]") {
  auto r=example();
  r.goal=endpoint(r,{.5,0},1.0);
  r.options.maxFinalSamplesPerPrimitive=2;
  auto result=as::plan(r);
  REQUIRE(result.status!=as::Status::Success);
  REQUIRE_FALSE(result.independentlySampledBodyValidated);
}

TEST_CASE("Articulated planning rejects missing geometry and occupied goals",
          "[articulated][collision][validation]") {
  auto r=example();
  r.bodies.front={0,0,0};
  REQUIRE(as::plan(r).status==as::Status::InvalidRequest);
  r=example();
  r.goal=endpoint(r,{.5,0},1.0);
  auto map=std::make_shared<ac::MapSnapshot>(*r.map);
  occupy(*map,11.6,10.0); // new front body at goal, not at start
  r.map=map;
  REQUIRE(as::plan(r).status==as::Status::GoalInCollision);
  r=example();
  map=std::make_shared<ac::MapSnapshot>(*r.map);
  map->cells[0]=-1;
  r.map=map;
  r.occupancy.unknown=ac::UnknownPolicy::Reject;
  REQUIRE(as::plan(r).status==as::Status::RejectedUnknownMap);
}
