#pragma once

#include "hybrid_astar/articulated_steering/planner.hpp"
#include <cstddef>
#include <vector>

namespace hybrid_astar::articulated_steering {

// A native, non-RS candidate composed only of permitted constant controls.
// Coordinates and terminal error are evaluated using exact kinematic formulas;
// accepted candidates must still undergo independent RK4 and both-body checks.
struct GoalControl {
  Control control;
  double durationSeconds=0.0;
};
struct GoalConnection {
  std::vector<GoalControl> controls;
  RearState estimatedEnd;
  double costMeters=0.0;
};

std::vector<GoalConnection> proposeGoalConnections(
    const Model& model, RearState from, RearState goal,
    const SearchOptions& options, std::size_t& evaluations);

} // namespace hybrid_astar::articulated_steering
