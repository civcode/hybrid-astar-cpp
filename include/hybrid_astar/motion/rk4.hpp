#pragma once
#include <cmath>
#include <stdexcept>
#include <utility>

namespace hybrid_astar::motion {
// Dependency-free classical explicit RK4 for a model-defined state and tangent.
// field(state) returns the derivative. add(state,tangent,scale) implements
// tangent addition on the model's state space; do not wrap/normalize angles in
// intermediate RK stages (a model may use unwrapped angles and joint limits).
// State validity is the MODEL'S responsibility at each derivative evaluation
// and for the returned state, including bounds crossed by the final update.
template<class State, class Field, class AddScaled>
State rk4Step(const State& state, double seconds, Field&& field,
              AddScaled&& addScaled) {
  if(!std::isfinite(seconds) || seconds<=0.0)
    throw std::invalid_argument("RK4 step size must be finite and positive");
  const auto k1=field(state);
  const auto k2=field(addScaled(state,k1,0.5*seconds));
  const auto k3=field(addScaled(state,k2,0.5*seconds));
  const auto k4=field(addScaled(state,k3,seconds));
  auto next=addScaled(state,k1,seconds/6.0);
  next=addScaled(next,k2,seconds/3.0);
  next=addScaled(next,k3,seconds/3.0);
  next=addScaled(next,k4,seconds/6.0);
  return next;
}
} // namespace hybrid_astar::motion
