#include "hybrid_astar/steering/reeds_shepp.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

namespace hybrid_astar::steering {
double reedsSheppDistance(const Pose2D& from, const Pose2D& to,
                          double minimumTurningRadiusMeters) {
  if (!std::isfinite(minimumTurningRadiusMeters) || minimumTurningRadiusMeters <= 0.0 ||
      !std::isfinite(from.x) || !std::isfinite(from.y) || !std::isfinite(from.yaw) ||
      !std::isfinite(to.x) || !std::isfinite(to.y) || !std::isfinite(to.yaw)) {
    throw std::invalid_argument("Reeds-Shepp requires finite poses and positive turn radius");
  }
  auto space = std::make_shared<ompl::base::ReedsSheppStateSpace>(minimumTurningRadiusMeters);
  ompl::base::ScopedState<> start(space), goal(space);
  start[0] = from.x; start[1] = from.y; start[2] = from.yaw;
  goal[0] = to.x; goal[1] = to.y; goal[2] = to.yaw;
  return space->distance(start.get(), goal.get());
}
} // namespace hybrid_astar::steering
