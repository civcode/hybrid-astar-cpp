#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "hybrid_astar/articulated_steering/planner.hpp"
#include "hybrid_astar/articulated_steering/heuristic.hpp"
#include "hybrid_astar/articulated_steering/goal_connector.hpp"
#include "hybrid_astar/ackermann/obstacle_grid.hpp"
#include <array>
#include <atomic>
#include <cmath>
#include <memory>
#include <numbers>
#include <random>

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
  ac::LazyGrid<as::Diagnostics> grid(map,{},hybrid_astar::Pose2D{5.5,1.5,0},
                                     0.2,100,100,true,
                                     ac::OpenQueueMode::BinaryLazy,diagnostics);
  // Straight-line XY distance crosses a wall; the raw-grid detour goes via y=6.
  const double directed=grid.estimate({1.5,1.5,0},4.0);
  REQUIRE(directed>4.0);
  REQUIRE(diagnostics.gridSettlements>0);
  REQUIRE(diagnostics.gridFallbacks==0);

  // Low work budgets must gracefully fall back; never declare NoPath.
  as::Diagnostics limitedDiagnostics;
  ac::LazyGrid<as::Diagnostics> limited(map,{},hybrid_astar::Pose2D{5.5,1.5,0},
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

TEST_CASE("Articulated heading offset matches the ODE over distinct chassis geometries",
          "[articulated][heuristic][analytic]") {
  const std::array<as::Parameters,3> geometries{{
      {1.0,1.0,-1.2,1.2,1.0,.8,.1},
      {.75,1.2,-1.2,1.2,1.0,.8,.1},
      {1.2,.8,-1.2,1.2,1.0,.8,.1}}};
  for(const auto& p:geometries) {
    as::AnalyticGoalHeuristic h(p,{}, {.2,.15,.1},.25);
    REQUIRE(std::abs(h.headingOffset(0.0))<1e-12);
    for(int i=-20;i<=20;++i) {
      const double gamma=.055*i;
      const double step=1e-5;
      const double slope=(h.headingOffset(gamma+step)-
                          h.headingOffset(gamma-step))/(2*step);
      const double expected=p.LfMeters*std::cos(gamma)/
          (p.LrMeters+p.LfMeters*std::cos(gamma));
      REQUIRE(std::abs(slope-expected)<1e-6);
    }
  }
}

TEST_CASE("Analytic articulated bounds never exceed feasible RK4 motion cost",
          "[articulated][heuristic][lower-bound]") {
  const std::array<as::Parameters,3> geometries{{
      {1.0,1.0,-1.2,1.2,1.0,.8,.1},
      {.75,1.2,-1.2,1.2,1.0,.8,.1},
      {1.2,.8,-1.2,1.2,1.0,.8,.1}}};
  std::mt19937 rng(20261105);
  std::uniform_real_distribution<double> random(-1.0,1.0);
  std::uniform_real_distribution<double> duration(.08,.55);
  for(const auto& p:geometries) {
    as::Model model(p);
    for(int i=0;i<150;++i) {
      const as::RearState start{10.0+random(rng),10.0+random(rng),
                                2.5*random(rng),.45*random(rng)};
      const as::Control control{
          i%5==0?0.0:.8*random(rng),.6*random(rng)};
      const double dt=duration(rng);
      const auto reached=model.toRear(
          model.propagate(model.fromRear(start),control,dt,.004));
      as::RearState nominalGoal=reached;
      // Move the nominal goal within all three accepted tolerance windows.
      nominalGoal.xr+=.1*random(rng);
      nominalGoal.yr+=.1*random(rng);
      nominalGoal.thetaRear+=.1*random(rng);
      nominalGoal.gamma+=.05*random(rng);
      const double reverseMultiplier=i%2==0?.65:1.1;
      as::AnalyticGoalHeuristic h(p,nominalGoal,{.2,.15,.1},.25,
                                  reverseMultiplier);
      const double drive=std::abs(control.frontSpeedMetersPerSecond)*dt*
          (control.frontSpeedMetersPerSecond<0.0?reverseMultiplier:1.0);
      const double joint=std::abs(control.gammaRateRadiansPerSecond)*dt*.25;
      const double feasibleCost=drive+joint+
          (control.frontSpeedMetersPerSecond==0.0 ? .15 : 0.0);
      REQUIRE(h.estimate(start)<=feasibleCost+1e-5);
      REQUIRE(h.estimate(reached)<=1e-8);
      REQUIRE(std::abs(h.estimate(start)-
          h.estimate(start,h.headingOffset(start.gamma)))<1e-10);
    }
  }
}

TEST_CASE("Articulated heuristic modes preserve strictly validated planner results",
          "[articulated][heuristic][search]") {
  for(const auto mode:{as::HeuristicMode::AnalyticOnly,
                       as::HeuristicMode::Combined}) {
    auto r=example();
    r.goal=endpoint(r,{.5,.2},1.0);
    r.options.heuristicMode=mode;
    r.options.maxExpansions=20000;
    const auto out=as::plan(r);
    REQUIRE(out.status==as::Status::Success);
    REQUIRE(out.independentlySampledBodyValidated);
    REQUIRE(out.diagnostics.finalBodyChecks>0);
    REQUIRE(out.goalXYErrorMeters<=r.options.goalXYToleranceMeters);
    REQUIRE(out.goalYawErrorRadians<=r.options.goalYawToleranceRadians);
    REQUIRE(out.goalGammaErrorRadians<=r.options.goalGammaToleranceRadians);
    if(mode==as::HeuristicMode::AnalyticOnly)
      REQUIRE(out.diagnostics.gridSettlements==0);
  }
}

TEST_CASE("Native articulated terminal generator fits reversible bounded controls",
          "[articulated][connector][model]") {
  auto r=example();
  as::Model model(r.parameters);
  r.goal={9.0,10.0,0.0,0.0}; // one metre straight behind start
  std::size_t evaluations=0;
  const auto proposals=as::proposeGoalConnections(
      model,r.start,r.goal,r.options,evaluations);
  REQUIRE(evaluations>0);
  REQUIRE(evaluations<=r.options.maxGoalConnectorEvaluationsPerAttempt);
  REQUIRE_FALSE(proposals.empty());
  bool hasReverse=false;
  for(const auto& proposal:proposals) {
    auto state=model.fromRear(r.start);
    for(const auto& part:proposal.controls) {
      REQUIRE(model.validControl(part.control));
      REQUIRE(part.durationSeconds>0.0);
      if(part.control.frontSpeedMetersPerSecond<0.0)hasReverse=true;
      state=model.propagate(state,part.control,part.durationSeconds,.005);
    }
    const auto reached=model.toRear(state);
    REQUIRE(std::hypot(reached.xr-r.goal.xr,reached.yr-r.goal.yr)<
            r.options.goalXYToleranceMeters);
    REQUIRE(std::abs(std::remainder(reached.thetaRear-r.goal.thetaRear,
                                   2*std::numbers::pi_v<double>))<
            r.options.goalYawToleranceRadians);
    REQUIRE(std::abs(reached.gamma-r.goal.gamma)<
            r.options.goalGammaToleranceRadians);
  }
  REQUIRE(hasReverse);
}

TEST_CASE("First-node native terminal connection is strictly replayed",
          "[articulated][connector][strict][reverse]") {
  auto r=example();
  r.goal={9.0,10.0,0.0,0.0};
  r.options.enableGoalConnector=true;
  r.options.maxExpansions=1;
  r.options.maxGenerated=100;
  const auto out=as::plan(r);
  REQUIRE(out.status==as::Status::Success);
  REQUIRE(out.independentlySampledBodyValidated);
  REQUIRE(out.diagnostics.expanded==1);
  REQUIRE(out.diagnostics.goalConnectorAttempts==1);
  REQUIRE(out.diagnostics.goalConnectorValidated==1);
  REQUIRE(out.diagnostics.goalConnectorCandidates>=1);
  REQUIRE(out.diagnostics.goalConnectorBodyChecks>0);
  REQUIRE(out.diagnostics.finalBodyChecks>0);
  REQUIRE(out.goalXYErrorMeters<=r.options.goalXYToleranceMeters);
  REQUIRE(out.goalYawErrorRadians<=r.options.goalYawToleranceRadians);
  REQUIRE(out.goalGammaErrorRadians<=r.options.goalGammaToleranceRadians);
  bool reversed=false;
  for(const auto& motion:out.motions) {
    if(motion.control.frontSpeedMetersPerSecond<0.0)reversed=true;
    as::Model model(r.parameters);
    as::BodyCollisionChecker body(*r.map,r.occupancy,r.bodies);
    for(const auto& sample:motion.samples)
      REQUIRE(body.clear(model,sample.state));
  }
  REQUIRE(reversed);
}

TEST_CASE("Native terminal connection never tunnels through a wall",
          "[articulated][connector][collision]") {
  auto r=example();
  // A full-height barrier separates both complete two-carriage poses.
  for(std::size_t y=0;y<r.map->height;++y)
    const_cast<ac::MapSnapshot&>(*r.map).cells[
      y*r.map->width+120]=100;
  r.goal={15,10,0,0};
  r.options.enableGoalConnector=true;
  r.options.maxExpansions=50;
  const auto out=as::plan(r);
  REQUIRE(out.status!=as::Status::Success);
  REQUIRE_FALSE(out.independentlySampledBodyValidated);
  REQUIRE(out.diagnostics.goalConnectorAttempts>0);
  REQUIRE(out.diagnostics.goalConnectorValidated==0);
}

TEST_CASE("Goal clearance guide distinguishes obstructed from free articulated poses",
          "[articulated][connector][clearance]") {
  auto r=example();
  auto& map=const_cast<ac::MapSnapshot&>(*r.map);
  occupy(map,10.0,10.0);
  occupy(map,10.1,10.0);
  as::GoalConnectorObstacleGuide guide(map,r.occupancy,r.bodies);
  REQUIRE(guide.hasObstacles());
  REQUIRE(guide.penalty(r.start,r.parameters)>
          guide.penalty({5.0,5.0,0.0,0.0},r.parameters));
  // The field only ranks shooting candidates; it does NOT certify collision
  // freedom or replace the independently sampled body-footprint checker.
}

TEST_CASE("Heading-distance guidance emphasizes nonzero yaw goal error",
          "[articulated][heuristic][heading-aware]") {
  auto r=example();
  r.goal={10,10,-std::numbers::pi_v<double>/2,0};
  as::AnalyticGoalHeuristic heuristic(r.parameters,r.goal,
      {.2,.15,.1},r.options.articulationCostMetersPerRadian,
      r.options.reverseCostMultiplier);
  REQUIRE(heuristic.headingDrivingCost(r.start,
          heuristic.headingOffset(r.start.gamma))>1.0);
  REQUIRE(heuristic.headingDrivingCost(r.goal,
          heuristic.headingOffset(r.goal.gamma))==0.0);
  r.options.heuristicMode=as::HeuristicMode::CombinedHeadingAware;
  r.options.goalHeadingGuidanceWeight=.8;
  r.options.maxExpansions=1;
  const auto result=as::plan(r);
  REQUIRE(result.status==as::Status::BudgetExceeded);
  r.options.goalHeadingGuidanceWeight=-.1;
  REQUIRE(as::plan(r).status==as::Status::InvalidRequest);
}

TEST_CASE("Adaptive connector reserves later shots for near-goal improvements",
          "[articulated][connector][schedule]") {
  auto r=example();
  r.goal={15,10,0,0};
  r.options.enableGoalConnector=true;
  r.options.goalConnectorGlobalAttempts=0;
  r.options.goalConnectorNearRadiusMeters=.5;
  r.options.maxExpansions=1;
  r.options.maxGenerated=100;
  const auto result=as::plan(r);
  REQUIRE(result.status==as::Status::BudgetExceeded);
  REQUIRE(result.diagnostics.goalConnectorAttempts==0);
  REQUIRE(result.diagnostics.goalConnectorGlobalUsed==0);
  REQUIRE(result.diagnostics.goalConnectorDeferred>=1);
  r.options.goalConnectorGlobalAttempts=
      r.options.maxGoalConnectorAttempts+1;
  REQUIRE(as::plan(r).status==as::Status::InvalidRequest);
}

TEST_CASE("Anytime articulated connector preserves a verified incumbent at the expansion limit",
          "[articulated][connector][anytime][strict]") {
  auto r=example();
  r.goal={9.0,10.0,0.0,0.0};
  r.options.enableGoalConnector=true;
  r.options.maxExpansions=1; // Force termination immediately after the first shot.
  r.options.maxGenerated=100;
  const auto first=as::plan(r);
  REQUIRE(first.status==as::Status::Success);
  REQUIRE(first.independentlySampledBodyValidated);
  r.options.enableAnytimeGoalRefinement=true;
  r.options.anytimeExtraExpansions=20;
  const auto refined=as::plan(r);
  REQUIRE(refined.status==as::Status::Success);
  REQUIRE(refined.independentlySampledBodyValidated);
  REQUIRE(refined.diagnostics.expanded==1);
  REQUIRE(refined.diagnostics.anytimeFirstSolutionExpansion==1);
  REQUIRE(refined.diagnostics.anytimeExtraExpansions==0);
  REQUIRE(refined.diagnostics.anytimeValidatedCandidates>=1);
  REQUIRE(refined.diagnostics.anytimeImprovements==0);
  REQUIRE(refined.diagnostics.anytimeFirstCostMeters==
          Catch::Approx(first.costMeters));
  REQUIRE(refined.costMeters==Catch::Approx(first.costMeters));
  REQUIRE(refined.diagnostics.anytimeBestCostMeters==
          Catch::Approx(refined.costMeters));
  REQUIRE_FALSE(refined.motions.empty());
  as::Model model(r.parameters);
  as::BodyCollisionChecker body(*r.map,r.occupancy,r.bodies);
  for(const auto& motion:refined.motions)
    for(const auto& sample:motion.samples)
      REQUIRE(body.clear(model,sample.state));
}

TEST_CASE("Anytime articulated connector improves only through independently verified trajectories",
          "[articulated][connector][anytime][cost]") {
  auto r=example();
  r.goal={9.0,10.0,0.0,0.0};
  r.options.enableGoalConnector=true;
  r.options.maxExpansions=120;
  r.options.maxGenerated=1000;
  const auto first=as::plan(r);
  REQUIRE(first.status==as::Status::Success);
  r.options.enableAnytimeGoalRefinement=true;
  r.options.anytimeExtraExpansions=32;
  r.options.anytimeExtraConnectorAttempts=4;
  const auto refined=as::plan(r);
  REQUIRE(refined.status==as::Status::Success);
  REQUIRE(refined.independentlySampledBodyValidated);
  REQUIRE(refined.costMeters<=first.costMeters+1e-9);
  REQUIRE(refined.diagnostics.anytimeFirstCostMeters==
          Catch::Approx(first.costMeters));
  REQUIRE(refined.diagnostics.anytimeBestCostMeters==
          Catch::Approx(refined.costMeters));
  REQUIRE(refined.diagnostics.anytimeExtraExpansions<=32);
  REQUIRE(refined.diagnostics.anytimeExtraConnectorAttempts<=4);
  REQUIRE(refined.diagnostics.goalConnectorValidated>=1);
  REQUIRE(refined.goalXYErrorMeters<=r.options.goalXYToleranceMeters);
  REQUIRE(refined.goalYawErrorRadians<=r.options.goalYawToleranceRadians);
  REQUIRE(refined.goalGammaErrorRadians<=r.options.goalGammaToleranceRadians);
}

TEST_CASE("Anytime articulated connector requires explicit opt-in and valid budgets",
          "[articulated][connector][anytime][options]") {
  auto r=example();
  r.options.enableAnytimeGoalRefinement=true;
  REQUIRE(as::plan(r).status==as::Status::InvalidRequest);
  r.options.enableGoalConnector=true;
  r.options.anytimeExtraExpansions=0;
  REQUIRE(as::plan(r).status==as::Status::InvalidRequest);
  r.options.anytimeExtraExpansions=100001;
  REQUIRE(as::plan(r).status==as::Status::InvalidRequest);
  r.options.anytimeExtraExpansions=20;
  r.options.anytimeExtraConnectorAttempts=1001;
  REQUIRE(as::plan(r).status==as::Status::InvalidRequest);
}
