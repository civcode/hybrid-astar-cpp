#pragma once
#include "hybrid_astar/core/pose2d.hpp"
#include <array>

namespace hybrid_astar::mecanum {
// Ideal roller-equipped chassis: four independently commanded wheel rates.
// This is kinematic feasibility only; traction, slip and acceleration
// constraints require a separate vehicle/actuator validation layer.
struct Parameters {
  double wheelRadiusMeters = 0.10;
  double halfWheelbaseMeters = 0.25;
  double halfTrackMeters = 0.20;
  double maxAbsWheelAngularRate = 20.0;
};
struct Control {
  double forwardMetersPerSecond = 0.0;
  double lateralMetersPerSecond = 0.0;
  double angularRadiansPerSecond = 0.0;
};
struct Derivative { double dx,dy,dyaw; };
class Model {
public:
  explicit Model(Parameters parameters = {});
  const Parameters& parameters() const noexcept {return params_;}
  bool valid(Control control) const noexcept;
  // Order: front-left, front-right, rear-left, rear-right.
  std::array<double,4> wheelRates(Control control) const;
  Derivative derivative(const Pose2D& pose,Control control) const;
  Pose2D propagate(const Pose2D& pose,Control control,double seconds) const;
private:
  Parameters params_;
};
} // namespace hybrid_astar::mecanum
