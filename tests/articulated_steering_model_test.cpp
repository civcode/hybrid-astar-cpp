#include <catch2/catch_test_macros.hpp>
#include "hybrid_astar/articulated_steering/model.hpp"
#include "hybrid_astar/motion/rk4.hpp"
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <limits>

namespace as=hybrid_astar::articulated_steering;
using as::State;
using as::Control;
using as::Model;
static as::Parameters example(){return {.5,.5,-1.2,1.2,2.0,1.0,.1};}
static bool near(double a,double b,double tol=1e-9){return std::abs(a-b)<=tol;}

TEST_CASE("Articulated five-state derivative and rear frame map to author ODE", "[articulated][kinematics]") {
  Model m(example());
  const State q{1.5,2.0,.8,.5,.3};
  const Control u{.7,.23};
  const auto d=m.derivative(q,u);
  const double D=.5*std::cos(q.gamma)+.5;
  REQUIRE(near(d.dxf,.7*std::cos(.8)));
  REQUIRE(near(d.dyf,.7*std::sin(.8)));
  REQUIRE(near(d.dthetaFront,(.7*std::sin(.3)+.5*.23)/D));
  REQUIRE(near(d.dthetaRear,(.7*std::sin(.3)-.5*std::cos(.3)*.23)/D));
  REQUIRE(near(d.dthetaFront-d.dthetaRear,d.dgamma));
  const auto rear=m.toRear(q);
  REQUIRE(near(rear.xr,q.xf-.5*std::cos(.8)-.5*std::cos(.5)));
  REQUIRE(near(rear.yr,q.yf-.5*std::sin(.8)-.5*std::sin(.5)));
  const auto back=m.fromRear(rear);
  REQUIRE(near(back.xf,q.xf));REQUIRE(near(back.yf,q.yf));
  REQUIRE(near(back.thetaFront,q.thetaFront));
  REQUIRE(near(back.thetaRear,q.thetaRear));
  REQUIRE(near(back.gamma,q.gamma));
}
TEST_CASE("Zero-speed articulation rotates both bodies with exact nonlinear split", "[articulated][rk4]") {
  Model m(example());
  const State aligned{2,3,.4,.4,0};
  const Control u{0,.2};
  const auto d=m.derivative(aligned,u);
  REQUIRE(near(d.dxf,0)); REQUIRE(near(d.dyf,0));
  REQUIRE(near(d.dthetaFront,.1)); REQUIRE(near(d.dthetaRear,-.1));
  const auto end=m.propagate(aligned,u,1,.001);
  REQUIRE(near(end.xf,aligned.xf)); REQUIRE(near(end.yf,aligned.yf));
  REQUIRE(near(end.gamma,.2,1e-12));
  REQUIRE(end.thetaFront>aligned.thetaFront);
  REQUIRE(end.thetaRear<aligned.thetaRear);
  REQUIRE(near(end.thetaFront-end.thetaRear,.2,1e-12));
  const auto curve=m.derivative({1,2,1.0,.4,.6},u);
  REQUIRE(curve.dthetaFront>u.gammaRateRadiansPerSecond/2);
  REQUIRE(std::abs(curve.dthetaRear)<u.gammaRateRadiansPerSecond/2);
  REQUIRE(m.rearSpeed({1,2,1.0,.4,.6},u)>0);
  const auto checkpoints=m.trajectory(aligned,u,.5,.01);
  REQUIRE(checkpoints.size()==51);
  REQUIRE(near(checkpoints.front().state.gamma,0));
  REQUIRE(near(checkpoints.back().timeSeconds,.5));
  REQUIRE(near(checkpoints.back().state.gamma,.1,1e-12));
  REQUIRE(std::hypot(checkpoints.back().rear.xr-checkpoints.front().rear.xr,
                     checkpoints.back().rear.yr-checkpoints.front().rear.yr)>0.0);
}
TEST_CASE("Articulated RK4 converges on constant-heading and finite joint-rate fixtures", "[articulated][rk4]") {
  Model m(example());
  const State q{1,2,.7,.7,0};
  for(double v : {-.9,0.0,.9}) {
    const auto p=m.propagate(q,{v,0},1.25,.01);
    REQUIRE(near(p.xf,q.xf+v*std::cos(.7)*1.25,1e-12));
    REQUIRE(near(p.yf,q.yf+v*std::sin(.7)*1.25,1e-12));
    REQUIRE(near(p.thetaFront,.7));REQUIRE(near(p.thetaRear,.7));
  }
  for(const auto u : {Control{.6,.4},Control{-.6,-.3},Control{0,-.7}}){
    const State initial{.3,-.8,.5,.25,.25};
    const auto coarse=m.propagate(initial,u,.5,.02);
    const auto fine=m.propagate(initial,u,.5,.001);
    REQUIRE(near(coarse.xf,fine.xf,1e-8));
    REQUIRE(near(coarse.yf,fine.yf,1e-8));
    REQUIRE(near(coarse.thetaFront,fine.thetaFront,1e-8));
    REQUIRE(near(coarse.thetaRear,fine.thetaRear,1e-8));
    REQUIRE(near(coarse.gamma,fine.gamma,1e-12));
    const auto direct=m.toRear(fine);
    REQUIRE(near(m.fromRear(direct).xf,fine.xf,1e-9));
  }
}
TEST_CASE("Articulated model rejects invalid geometry, controls, states and budgets", "[articulated][validation]") {
  auto p=example();p.LfMeters=0;REQUIRE_THROWS_AS(Model(p),std::invalid_argument);
  p=example();p.maxGammaRadians=std::numbers::pi_v<double>;p.minGammaRadians=-.5;
  REQUIRE_THROWS_AS(Model(p),std::invalid_argument);
  p=example();p.minDenominatorMeters=2;
  REQUIRE_THROWS_AS(Model(p),std::invalid_argument);
  const Model m(example());
  REQUIRE_FALSE(m.validState({0,0,.2,.3,0}));
  REQUIRE_FALSE(m.validState({0,0,0,0,std::numeric_limits<double>::quiet_NaN()}));
  REQUIRE_FALSE(m.validControl({10,0}));
  REQUIRE_FALSE(m.validControl({0,2}));
  REQUIRE_THROWS_AS(m.propagate({}, {0,0},-1),std::invalid_argument);
  REQUIRE_THROWS_AS(m.propagate({}, {0,0},1,0.0001,100),std::length_error);
  REQUIRE_THROWS_AS(m.propagate({0,0,1.19,0,1.19},{0,1},.2,.01),std::invalid_argument);
  REQUIRE_THROWS_AS(m.trajectory({}, {0,0},1,.01,10),std::length_error);
  REQUIRE(m.trajectory({}, {0,0},0).size()==1);
}
TEST_CASE("Constant articulation follows analytic circular motion in both gears", "[articulated][rk4][oracle]") {
  Model m(example());
  constexpr double gamma=.35;
  const State initial{1.2,-.7,.85,.5,gamma};
  const double D=.5*std::cos(gamma)+.5;
  for(const double v : {.8,-.8}) {
    const double duration=.9;
    const double omega=v*std::sin(gamma)/D;
    const auto q=m.propagate(initial,{v,0},duration,.02);
    const double wantX=initial.xf+v*(std::sin(initial.thetaFront+omega*duration)-
                                      std::sin(initial.thetaFront))/omega;
    const double wantY=initial.yf-v*(std::cos(initial.thetaFront+omega*duration)-
                                      std::cos(initial.thetaFront))/omega;
    REQUIRE(near(q.xf,wantX,1e-9));
    REQUIRE(near(q.yf,wantY,1e-9));
    REQUIRE(near(q.thetaFront,initial.thetaFront+omega*duration,1e-11));
    REQUIRE(near(q.thetaRear,initial.thetaRear+omega*duration,1e-11));
    REQUIRE(near(q.gamma,gamma,1e-12));
  }
}
TEST_CASE("Rear-reference instantaneous speed has no lateral component", "[articulated][frames]") {
  Model m(example());
  const State initial{1.2,-.7,.85,.5,.35};
  const Control u{.7,.3};
  const auto before=m.toRear(initial);
  constexpr double dt=1e-5;
  const auto after=m.toRear(m.propagate(initial,u,dt,dt));
  const double dx=(after.xr-before.xr)/dt,dy=(after.yr-before.yr)/dt;
  const double vr=m.rearSpeed(initial,u);
  REQUIRE(near(dx,vr*std::cos(initial.thetaRear),1e-5));
  REQUIRE(near(dy,vr*std::sin(initial.thetaRear),1e-5));
  REQUIRE(near(-dx*std::sin(initial.thetaRear)+dy*std::cos(initial.thetaRear),0,1e-5));
}
