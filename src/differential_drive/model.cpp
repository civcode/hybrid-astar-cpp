#include "hybrid_astar/differential_drive/model.hpp"
#include "hybrid_astar/motion/primitive.hpp"
#include <cmath>
#include <stdexcept>

namespace hybrid_astar::differential_drive {
Model::Model(Parameters parameters):params_(parameters) {
  if(!std::isfinite(params_.wheelRadiusMeters)||params_.wheelRadiusMeters<=0 ||
     !std::isfinite(params_.trackWidthMeters)||params_.trackWidthMeters<=0 ||
     !std::isfinite(params_.maxAbsWheelAngularRate)||params_.maxAbsWheelAngularRate<=0)
    throw std::invalid_argument("Invalid differential-drive wheel geometry or rate limit");
}
bool Model::valid(Control c) const noexcept {
  if(!std::isfinite(c.linearMetersPerSecond)||!std::isfinite(c.angularRadiansPerSecond))
    return false;
  const double half=.5*params_.trackWidthMeters*c.angularRadiansPerSecond;
  const double right=(c.linearMetersPerSecond+half)/params_.wheelRadiusMeters;
  const double left=(c.linearMetersPerSecond-half)/params_.wheelRadiusMeters;
  return std::isfinite(left)&&std::isfinite(right) &&
         std::abs(left)<=params_.maxAbsWheelAngularRate &&
         std::abs(right)<=params_.maxAbsWheelAngularRate;
}
std::pair<double,double> Model::wheelRates(Control c) const {
  if(!valid(c))throw std::invalid_argument("Differential-drive wheel speed limit violated");
  const double half=.5*params_.trackWidthMeters*c.angularRadiansPerSecond;
  return {(c.linearMetersPerSecond-half)/params_.wheelRadiusMeters,
          (c.linearMetersPerSecond+half)/params_.wheelRadiusMeters};
}
Derivative Model::derivative(const Pose2D& pose,Control c) const {
  if(!valid(c)||!std::isfinite(pose.x)||!std::isfinite(pose.y)||!std::isfinite(pose.yaw))
    throw std::invalid_argument("Invalid differential-drive state or control");
  return {c.linearMetersPerSecond*std::cos(pose.yaw),
          c.linearMetersPerSecond*std::sin(pose.yaw),c.angularRadiansPerSecond};
}
Pose2D Model::propagate(const Pose2D& pose,Control c,double duration) const {
  if(!valid(c)||!std::isfinite(duration)||duration<0 ||
     !std::isfinite(pose.x)||!std::isfinite(pose.y)||!std::isfinite(pose.yaw))
    throw std::invalid_argument("Invalid differential-drive propagation");
  return motion::compose(pose,motion::constantTwist(
      c.linearMetersPerSecond*duration,c.angularRadiansPerSecond*duration,1.0));
}
} // namespace hybrid_astar::differential_drive
