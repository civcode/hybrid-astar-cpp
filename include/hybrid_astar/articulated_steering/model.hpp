#pragma once
#include "hybrid_astar/core/pose2d.hpp"
#include <cstddef>
#include <vector>

namespace hybrid_astar::articulated_steering {
// The five author-supplied ODE variables. Physical front/rear axle and
// collision-body offsets must be confirmed separately; this is a kinematic
// model only, NOT a complete multibody Hybrid A* search implementation.
struct State {
  double xf=0.0, yf=0.0;
  double thetaFront=0.0, thetaRear=0.0, gamma=0.0;
};
// Equivalent four-independent-variable representation, at the rear base_link.
struct RearState {
  double xr=0.0, yr=0.0, thetaRear=0.0, gamma=0.0;
};
struct Control {
  double frontSpeedMetersPerSecond=0.0;
  double gammaRateRadiansPerSecond=0.0;
};
struct Derivative {
  double dxf=0.0, dyf=0.0;
  double dthetaFront=0.0, dthetaRear=0.0, dgamma=0.0;
};
// All quantities are required: do not silently treat the ROS simulator's
// 0.5 m geometry or arbitrary angular/speed caps as production robot values.
struct Parameters {
  double LfMeters, LrMeters;
  double minGammaRadians, maxGammaRadians;
  double maxAbsFrontSpeedMetersPerSecond;
  double maxAbsGammaRateRadiansPerSecond;
  double minDenominatorMeters;
};
struct Sample {
  double timeSeconds=0.0;
  State state;
  RearState rear;
};
class Model {
public:
  explicit Model(Parameters parameters);
  const Parameters& parameters() const noexcept {return params_;}
  bool validState(const State& state) const noexcept;
  bool validControl(Control control) const noexcept;
  Derivative derivative(const State& state, Control control) const;
  Pose2D frontPose(const State& state) const;
  Pose2D rearPose(const State& state) const;
  RearState toRear(const State& state) const;
  State fromRear(const RearState& rear) const;
  double rearSpeed(const State& state, Control control) const;
  // Classical RK4 with model-independent motion::rk4Step, inclusive of the
  // endpoint, validating every step and all RK stages. No Boost dependency.
  State propagate(const State& initial, Control control, double seconds,
                  double maxStepSeconds=0.01, std::size_t maxSteps=100000) const;
  // Full-state checkpoints for later articulated primitive generation and
  // two-body collision sampling. No path-validity certificate is implied.
  std::vector<Sample> trajectory(const State& initial, Control control,
                                 double seconds, double maxStepSeconds=0.01,
                                 std::size_t maxSteps=100000) const;
private:
  Parameters params_;
  std::size_t integrationSteps(const State&,Control,double,double,std::size_t) const;
  State step(const State&,Control,double) const;
};
} // namespace hybrid_astar::articulated_steering
