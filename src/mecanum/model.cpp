#include "hybrid_astar/mecanum/model.hpp"
#include "hybrid_astar/motion/primitive.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace hybrid_astar::mecanum {
Model::Model(Parameters p):params_(p) {
  if(!std::isfinite(p.wheelRadiusMeters)||p.wheelRadiusMeters<=0||
     !std::isfinite(p.halfWheelbaseMeters)||p.halfWheelbaseMeters<=0||
     !std::isfinite(p.halfTrackMeters)||p.halfTrackMeters<=0||
     !std::isfinite(p.maxAbsWheelAngularRate)||p.maxAbsWheelAngularRate<=0||
     !std::isfinite(p.halfWheelbaseMeters+p.halfTrackMeters))
    throw std::invalid_argument("Invalid mecanum geometry or wheel speed limit");
}
bool Model::valid(Control c) const noexcept {
  const auto& p=params_;
  if(!std::isfinite(c.forwardMetersPerSecond)||
     !std::isfinite(c.lateralMetersPerSecond)||
     !std::isfinite(c.angularRadiansPerSecond))
    return false;
  const double k=p.halfWheelbaseMeters+p.halfTrackMeters;
  const double v=c.forwardMetersPerSecond,side=c.lateralMetersPerSecond,omega=k*c.angularRadiansPerSecond;
  const std::array<double,4> rates{(v-side-omega)/p.wheelRadiusMeters,
      (v+side+omega)/p.wheelRadiusMeters,
      (v+side-omega)/p.wheelRadiusMeters,
      (v-side+omega)/p.wheelRadiusMeters};
  return std::all_of(rates.begin(),rates.end(),[&](double rate){
    return std::isfinite(rate)&&std::abs(rate)<=p.maxAbsWheelAngularRate;
  });
}
std::array<double,4> Model::wheelRates(Control c) const {
  if(!valid(c))throw std::invalid_argument("Mecanum wheel rate limit exceeded");
  const auto& p=params_;
  const double k=p.halfWheelbaseMeters+p.halfTrackMeters;
  const double v=c.forwardMetersPerSecond,side=c.lateralMetersPerSecond,omega=k*c.angularRadiansPerSecond;
  return {(v-side-omega)/p.wheelRadiusMeters,(v+side+omega)/p.wheelRadiusMeters,
          (v+side-omega)/p.wheelRadiusMeters,(v-side+omega)/p.wheelRadiusMeters};
}
Derivative Model::derivative(const Pose2D& pose,Control c) const {
  if(!valid(c)||!std::isfinite(pose.x)||!std::isfinite(pose.y)||!std::isfinite(pose.yaw))
    throw std::invalid_argument("Invalid mecanum state or control");
  const double cs=std::cos(pose.yaw),sn=std::sin(pose.yaw);
  return {cs*c.forwardMetersPerSecond-sn*c.lateralMetersPerSecond,
          sn*c.forwardMetersPerSecond+cs*c.lateralMetersPerSecond,
          c.angularRadiansPerSecond};
}
Pose2D Model::propagate(const Pose2D& pose,Control c,double seconds) const {
  if(!valid(c)||!std::isfinite(seconds)||seconds<0||
     !std::isfinite(pose.x)||!std::isfinite(pose.y)||!std::isfinite(pose.yaw))
    throw std::invalid_argument("Invalid mecanum propagation");
  return motion::compose(pose,motion::constantHolonomicTwist(
      c.forwardMetersPerSecond*seconds,c.lateralMetersPerSecond*seconds,
      c.angularRadiansPerSecond*seconds,1.0));
}
} // namespace hybrid_astar::mecanum
