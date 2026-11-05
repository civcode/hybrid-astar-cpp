#include <catch2/catch_test_macros.hpp>
#include "hybrid_astar/articulated_steering/planner.hpp"
#include "hybrid_astar/ackermann/obstacle_grid.hpp"
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
  REQUIRE(out.diagnostics.gridSettlements>0);
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
  REQUIRE((limited.status==as::Status::ValidationIncomplete ||
           limited.status==as::Status::BudgetExceeded));
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

TEST_CASE("Articulated precomputed primitives separate fixed-gamma driving and stationary steering",
          "[articulated][search][precomputed]") {
  auto r=example();
  r.goal=endpoint(r,{.5,0},1.0);
  const auto result=as::plan(r);
  REQUIRE(result.status==as::Status::Success);
  REQUIRE(result.independentlySampledBodyValidated);
  REQUIRE(result.diagnostics.precomputedAnalyticDriveTemplates>0);
  REQUIRE(result.diagnostics.precomputedStationaryRk4Templates>0);
  REQUIRE(result.diagnostics.precomputedCoupledRk4Templates>0);
  REQUIRE(result.diagnostics.precomputedSampleCount>
          result.diagnostics.precomputedAnalyticDriveTemplates+
          result.diagnostics.precomputedStationaryRk4Templates);
  REQUIRE(result.diagnostics.appliedPrecomputedTemplates>0);
  for(const auto& motion:result.motions) {
    // Gamma evolution of every drive must match its control and duration.
    if(motion.control.frontSpeedMetersPerSecond!=0.0) {
      REQUIRE(close(motion.end.gamma-motion.start.gamma,
                    motion.control.gammaRateRadiansPerSecond*
                    motion.durationSeconds,1e-9));
    } else {
      REQUIRE(motion.control.gammaRateRadiansPerSecond!=0.0);
    }
  }
}

TEST_CASE("Precomputed stationary steering reaches exact non-grid start and goal gamma",
          "[articulated][search][precomputed][joint]") {
  auto r=example();
  r.options.enableCoupledMotion=false; // stationary-only fixture
  r.start.gamma=.13; // not a multiple of the 0.2-radian lattice step
  as::Model m(r.parameters);
  const auto q0=m.fromRear(r.start);
  r.goal=m.toRear(m.propagate(q0,{0,.4},.5,.002));
  REQUIRE(close(r.goal.gamma,.33,1e-12));
  r.options.goalGammaToleranceRadians=.01;
  r.options.goalYawToleranceRadians=.01;
  r.options.goalXYToleranceMeters=.01;
  r.options.maxExpansions=3000;
  auto out=as::plan(r);
  REQUIRE(out.status==as::Status::Success);
  REQUIRE(close(out.reached.gamma,.33,1e-8));
  REQUIRE(out.independentlySampledBodyValidated);
  REQUIRE(out.motions.size()>=2); // actual stationary 0.13->...->0.33 edges
  for(const auto& mtn:out.motions) {
    REQUIRE(mtn.control.frontSpeedMetersPerSecond==0.0);
    REQUIRE(mtn.control.gammaRateRadiansPerSecond!=0.0);
    REQUIRE(close(mtn.start.xf,mtn.end.xf,1e-8));
    REQUIRE(close(mtn.start.yf,mtn.end.yf,1e-8));
    REQUIRE(mtn.samples.size()>1);
  }
}

TEST_CASE("Precomputed analytic arcs preserve unequal-link kinematics",
          "[articulated][search][precomputed][analytic]") {
  auto r=example();
  r.parameters.LfMeters=.7;
  r.parameters.LrMeters=.45;
  r.parameters.minDenominatorMeters=.1;
  r.start.gamma=.4;
  r.options.enableCoupledMotion=false; // analytic-only fixture
  r.goal=endpoint(r,{.5,0},1.0);
  auto result=as::plan(r);
  REQUIRE(result.status==as::Status::Success);
  REQUIRE(result.independentlySampledBodyValidated);
  REQUIRE(result.diagnostics.precomputedAnalyticDriveTemplates>0);
  REQUIRE(result.goalGammaErrorRadians<1e-9);
  bool foundDrive=false;
  for(const auto& motion:result.motions) {
    if(motion.control.frontSpeedMetersPerSecond==0)continue;
    foundDrive=true;
    REQUIRE(motion.control.gammaRateRadiansPerSecond==0.0);
    REQUIRE(close(motion.start.gamma,motion.end.gamma,1e-9));
  }
  REQUIRE(foundDrive);
}


TEST_CASE("Shared reverse-Dijkstra grid guides articulated search around a wall",
          "[articulated][heuristic][grid]") {
  ac::MapSnapshot map;
  map.width=map.height=7;
  map.resolutionMeters=1.0;
  map.cells.assign(49,0);
  for(std::size_t y=0;y<6;++y)map.cells[y*7+3]=100;
  as::Diagnostics diagnostics;
  ac::LazyGrid<as::Diagnostics> grid(map,{},ac::Pose2D{5.5,1.5,0},
                                     0.2,100,100,true,
                                     ac::OpenQueueMode::BinaryLazy,diagnostics);
  // Straight-line XY distance crosses a wall; the raw-grid detour goes via y=6.
  const double directed=grid.estimate({1.5,1.5,0},4.0);
  REQUIRE(directed>4.0);
  REQUIRE(diagnostics.gridSettlements>0);
  REQUIRE(diagnostics.gridFallbacks==0);

  // Low work budgets must gracefully fall back; never declare NoPath.
  as::Diagnostics limitedDiagnostics;
  ac::LazyGrid<as::Diagnostics> limited(map,{},ac::Pose2D{5.5,1.5,0},
                                        0.2,1,1,true,
                                        ac::OpenQueueMode::BinaryLazy,
                                        limitedDiagnostics);
  REQUIRE(close(limited.estimate({1.5,1.5,0},4.0),4.0));
  REQUIRE(limitedDiagnostics.gridFallbacks>0);
}

TEST_CASE("Articulated obstacle-grid guidance can be disabled for A/B testing",
          "[articulated][heuristic][grid]") {
  auto r=example();
  r.goal=endpoint(r,{.5,0},1.0);
  r.options.useObstacleGridHeuristic=false;
  const auto result=as::plan(r);
  REQUIRE(result.status==as::Status::Success);
  REQUIRE(result.diagnostics.gridSettlements==0);
  REQUIRE(result.independentlySampledBodyValidated);
}


TEST_CASE("Coupled RK4 steering drives continuously into an exact gamma target",
          "[articulated][search][coupled]") {
  auto r=example();
  r.goal=endpoint(r,{.5,.2},1.0);
  r.options.goalXYToleranceMeters=.05;
  r.options.goalYawToleranceRadians=.04;
  r.options.goalGammaToleranceRadians=.04;
  r.options.maxExpansions=20000;
  r.options.maxGenerated=80000;
  const auto out=as::plan(r);
  REQUIRE(out.status==as::Status::Success);
  REQUIRE(out.independentlySampledBodyValidated);
  REQUIRE(out.diagnostics.precomputedCoupledRk4Templates>0);
  REQUIRE(out.diagnostics.appliedCoupledTemplates>0);
  bool coupled=false;
  for(const auto& motion:out.motions) {
    if(motion.control.frontSpeedMetersPerSecond==0.0||
       motion.control.gammaRateRadiansPerSecond==0.0)continue;
    coupled=true;
    REQUIRE(close(motion.end.gamma-motion.start.gamma,
            motion.control.gammaRateRadiansPerSecond*motion.durationSeconds,1e-9));
    REQUIRE(motion.samples.size()>2);
  }
  REQUIRE(coupled);
}

TEST_CASE("Swept AABB free-space proof is conservative and can be disabled",
          "[articulated][collision][prefilter]") {
  auto r=example();
  ac::ExactCollisionChecker free(*r.map,r.occupancy,r.bodies.front,0.0,true);
  REQUIRE(free.provenEmptyMapLocalAabb(9.0,9.0,12.0,12.0));
  REQUIRE_FALSE(free.provenEmptyMapLocalAabb(-0.01,9.0,12.0,12.0));
  auto occupiedMap=std::make_shared<ac::MapSnapshot>(*r.map);
  occupy(*occupiedMap,11.0,10.0);
  ac::ExactCollisionChecker occupied(*occupiedMap,r.occupancy,r.bodies.front,0.0,true);
  REQUIRE_FALSE(occupied.provenEmptyMapLocalAabb(9.0,9.0,12.0,12.0));
  r.goal=endpoint(r,{.5,0},1.0);
  const auto fast=as::plan(r);
  REQUIRE(fast.status==as::Status::Success);
  REQUIRE(fast.diagnostics.searchSweptAabbAccepted>0);
  r.options.useSweptAabbPrefilter=false;
  const auto precise=as::plan(r);
  REQUIRE(precise.status==as::Status::Success);
  REQUIRE(precise.diagnostics.searchSweptAabbAccepted==0);
  REQUIRE(precise.diagnostics.searchBodyChecks>=fast.diagnostics.searchBodyChecks);
  REQUIRE(precise.independentlySampledBodyValidated);
}


TEST_CASE("Swept free-space proof handles rotated map origins and inflation",
          "[articulated][collision][prefilter][rotated]") {
  auto r=example();
  auto rotated=std::make_shared<ac::MapSnapshot>(*r.map);
  rotated->origin={1.0,-0.7,.37};
  r.map=rotated;
  r.bodies.inflationMeters=.10;
  r.goal=endpoint(r,{.5,.2},1.0);
  r.options.maxExpansions=20000;
  const auto out=as::plan(r);
  REQUIRE(out.status==as::Status::Success);
  REQUIRE(out.independentlySampledBodyValidated);
  REQUIRE(out.diagnostics.searchSweptAabbAccepted>0);
  REQUIRE(out.diagnostics.finalBodyChecks>0);
}
