#pragma once
#include "hybrid_astar/core/pose2d.hpp"
#include <utility>

namespace hybrid_astar::differential_drive {
// Ideal no-slip planar model. These wheel limits are kinematic only;
// acceleration, jerk, slip and body dynamics are not claimed.
struct Parameters {
  double wheelRadiusMeters = .10;
  double trackWidthMeters = .50;
  double maxAbsWheelAngularRate = 15.0; // radians/s, each wheel
};
struct Control {
  double linearMetersPerSecond = 0.0;
  double angularRadiansPerSecond = 0.0;
};
struct Derivative {
  double dx, dy, dyaw;
};
class Model {
public:
  explicit Model(Parameters parameters = {});
  const Parameters& parameters() const noexcept {return params_;}
  bool valid(Control control) const noexcept;
  std::pair<double,double> wheelRates(Control control) const;
  Derivative derivative(const Pose2D& pose,Control control) const;
  Pose2D propagate(const Pose2D& pose,Control control,double durationSeconds) const;
private:
  Parameters params_;
};
} // namespace hybrid_astar::differential_drive
