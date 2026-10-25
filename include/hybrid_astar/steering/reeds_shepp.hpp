#pragma once
#include "hybrid_astar/core/pose2d.hpp"

namespace hybrid_astar::steering {
// OMPL-backed distance metric; requires the optional steering target at link time.
// No OMPL headers or types in the public API.
double reedsSheppDistance(const Pose2D& from, const Pose2D& goal,
                          double minimumTurningRadiusMeters);
} // namespace hybrid_astar::steering
