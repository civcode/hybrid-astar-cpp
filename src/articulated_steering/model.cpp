#include "hybrid_astar/articulated_steering/model.hpp"
#include "hybrid_astar/motion/rk4.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace hybrid_astar::articulated_steering {
namespace {
constexpr double kTwoPi=2.0*std::numbers::pi_v<double>;
constexpr double kStateAngleTolerance=1e-8;
// Only accommodates RK4 floating-point summation when stopping exactly
// at a mechanical joint limit; physical excursions beyond it still fail.
constexpr double kJointLimitRoundingTolerance=1e-12;
bool finiteState(const State& s) noexcept {
  return std::isfinite(s.xf)&&std::isfinite(s.yf)&&
         std::isfinite(s.thetaFront)&&std::isfinite(s.thetaRear)&&
         std::isfinite(s.gamma);
}
State addScaled(const State& q,const Derivative& d,double step) noexcept {
  return {q.xf+step*d.dxf,q.yf+step*d.dyf,
          q.thetaFront+step*d.dthetaFront,
          q.thetaRear+step*d.dthetaRear,q.gamma+step*d.dgamma};
}
}
Model::Model(Parameters p):params_(p) {
  if(!std::isfinite(p.LfMeters)||p.LfMeters<=0.0||
     !std::isfinite(p.LrMeters)||p.LrMeters<=0.0||
     !std::isfinite(p.minGammaRadians)||!std::isfinite(p.maxGammaRadians)||
     p.minGammaRadians>=p.maxGammaRadians||
     p.minGammaRadians < -std::numbers::pi_v<double>||
     p.maxGammaRadians > std::numbers::pi_v<double>||
     !std::isfinite(p.maxAbsFrontSpeedMetersPerSecond)||
     p.maxAbsFrontSpeedMetersPerSecond<=0.0||
     !std::isfinite(p.maxAbsGammaRateRadiansPerSecond)||
     p.maxAbsGammaRateRadiansPerSecond<=0.0||
     !std::isfinite(p.minDenominatorMeters)||p.minDenominatorMeters<=0.0)
    throw std::invalid_argument("Invalid articulated geometry or operating limits");
  // Over [-pi,pi], cos(gamma) has its minimum at the largest |gamma|.
  // Require positive D throughout the ENTIRE mechanical joint interval.
  const double worstGamma=std::max(std::abs(p.minGammaRadians),
                                   std::abs(p.maxGammaRadians));
  const double minD=p.LrMeters+p.LfMeters*std::cos(worstGamma);
  if(!std::isfinite(minD)||minD<p.minDenominatorMeters)
    throw std::invalid_argument("Articulated denominator unsafe within joint limits");
}
bool Model::validState(const State& q) const noexcept {
  if(!finiteState(q)||
     q.gamma<params_.minGammaRadians-kJointLimitRoundingTolerance||
     q.gamma>params_.maxGammaRadians+kJointLimitRoundingTolerance)return false;
  const double d=params_.LfMeters*std::cos(q.gamma)+params_.LrMeters;
  if(!std::isfinite(d)||d<params_.minDenominatorMeters)return false;
  const double headingError=std::remainder(q.thetaFront-q.thetaRear-q.gamma,kTwoPi);
  return std::isfinite(headingError)&&std::abs(headingError)<=kStateAngleTolerance;
}
bool Model::validControl(Control c) const noexcept {
  return std::isfinite(c.frontSpeedMetersPerSecond)&&
         std::isfinite(c.gammaRateRadiansPerSecond)&&
         std::abs(c.frontSpeedMetersPerSecond)<=params_.maxAbsFrontSpeedMetersPerSecond&&
         std::abs(c.gammaRateRadiansPerSecond)<=params_.maxAbsGammaRateRadiansPerSecond;
}
Derivative Model::derivative(const State& q,Control c) const {
  if(!validState(q)||!validControl(c))
    throw std::invalid_argument("Invalid articulated state or control");
  const double D=params_.LfMeters*std::cos(q.gamma)+params_.LrMeters;
  const double v=c.frontSpeedMetersPerSecond,u=c.gammaRateRadiansPerSecond;
  const double drive=v*std::sin(q.gamma)/D;
  return {v*std::cos(q.thetaFront),v*std::sin(q.thetaFront),
          drive+params_.LrMeters*u/D,
          drive-params_.LfMeters*std::cos(q.gamma)*u/D,u};
}
Pose2D Model::frontPose(const State& q) const {
  if(!validState(q))throw std::invalid_argument("Invalid articulated state");
  return {q.xf,q.yf,std::remainder(q.thetaFront,kTwoPi)};
}
Pose2D Model::rearPose(const State& q) const {
  if(!validState(q))throw std::invalid_argument("Invalid articulated state");
  const double x=q.xf-params_.LfMeters*std::cos(q.thetaFront)-
                 params_.LrMeters*std::cos(q.thetaRear);
  const double y=q.yf-params_.LfMeters*std::sin(q.thetaFront)-
                 params_.LrMeters*std::sin(q.thetaRear);
  if(!std::isfinite(x)||!std::isfinite(y))
    throw std::out_of_range("Articulated frame transformation overflow");
  return {x,y,std::remainder(q.thetaRear,kTwoPi)};
}
RearState Model::toRear(const State& q) const {
  const auto r=rearPose(q);
  return {r.x,r.y,r.yaw,q.gamma};
}
State Model::fromRear(const RearState& r) const {
  if(!std::isfinite(r.xr)||!std::isfinite(r.yr)||
     !std::isfinite(r.thetaRear)||!std::isfinite(r.gamma)||
     r.gamma<params_.minGammaRadians-kJointLimitRoundingTolerance||
     r.gamma>params_.maxGammaRadians+kJointLimitRoundingTolerance)
    throw std::invalid_argument("Invalid rear-reference state");
  const double tf=r.thetaRear+r.gamma;
  const double xf=r.xr+params_.LrMeters*std::cos(r.thetaRear)+
                  params_.LfMeters*std::cos(tf);
  const double yf=r.yr+params_.LrMeters*std::sin(r.thetaRear)+
                  params_.LfMeters*std::sin(tf);
  const State q{xf,yf,tf,r.thetaRear,r.gamma};
  if(!validState(q))throw std::invalid_argument("Invalid reconstructed articulated state");
  return q;
}
double Model::rearSpeed(const State& q,Control c) const {
  if(!validState(q)||!validControl(c))
    throw std::invalid_argument("Invalid articulated state or control");
  const double D=params_.LfMeters*std::cos(q.gamma)+params_.LrMeters;
  const double v=c.frontSpeedMetersPerSecond,u=c.gammaRateRadiansPerSecond;
  return (v*(params_.LfMeters+params_.LrMeters*std::cos(q.gamma))+
          params_.LfMeters*params_.LrMeters*std::sin(q.gamma)*u)/D;
}
std::size_t Model::integrationSteps(const State& initial,Control c,
                                    double duration,double maxStep,
                                    std::size_t maxSteps) const {
  if(!validState(initial)||!validControl(c)||!std::isfinite(duration)||
     duration<0.0||!std::isfinite(maxStep)||maxStep<=0.0||maxSteps==0)
    throw std::invalid_argument("Invalid articulated integration request");
  if(duration==0.0)return 0;
  const double steps=std::ceil(duration/maxStep);
  if(!std::isfinite(steps)||steps>static_cast<double>(maxSteps))
    throw std::length_error("Articulated RK4 step budget exceeded");
  return std::max<std::size_t>(1,static_cast<std::size_t>(steps));
}
State Model::step(const State& q,Control c,double dt) const {
  const State next=motion::rk4Step(q,dt,[&](const State& s){return derivative(s,c);},
                                   addScaled);
  if(!validState(next))
    throw std::invalid_argument("Articulated RK4 endpoint violates joint or state bounds");
  return next;
}
State Model::propagate(const State& initial,Control c,double duration,
                       double maxStep,std::size_t maxSteps) const {
  const auto steps=integrationSteps(initial,c,duration,maxStep,maxSteps);
  State s=initial;
  if(!steps)return s;
  const double dt=duration/static_cast<double>(steps);
  for(std::size_t i=0;i<steps;++i)s=step(s,c,dt);
  return s;
}
std::vector<Sample> Model::trajectory(const State& initial,Control c,
                                      double duration,double maxStep,
                                      std::size_t maxSteps) const {
  const auto steps=integrationSteps(initial,c,duration,maxStep,maxSteps);
  std::vector<Sample> out;
  out.reserve(steps+1);
  State s=initial;
  out.push_back({0.0,s,toRear(s)});
  if(!steps)return out;
  const double dt=duration/static_cast<double>(steps);
  for(std::size_t i=0;i<steps;++i) {
    s=step(s,c,dt);
    out.push_back({i+1==steps?duration:dt*static_cast<double>(i+1),s,toRear(s)});
  }
  return out;
}
} // namespace hybrid_astar::articulated_steering
