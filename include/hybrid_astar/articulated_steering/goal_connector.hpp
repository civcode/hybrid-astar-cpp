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

// Request-local approximate clearance guide for nonlinear proposal fitting.
// Built once per plan, using a cheap eight-neighbor distance field. Used only
// for optimization/ranking: NEVER as collision acceptance evidence.
class GoalConnectorObstacleGuide {
public:
  GoalConnectorObstacleGuide(const ackermann::MapSnapshot& map,
                             ackermann::OccupancyPolicy occupancy,
                             BodyGeometry geometry);
  double penalty(RearState pose,const Parameters& geometry) const noexcept;
  bool hasObstacles() const noexcept {return hasObstacles_;}
private:
  std::size_t width_=0,height_=0;
  double resolution_=0.0, originX_=0.0, originY_=0.0, cosYaw_=1.0,sinYaw_=0.0;
  double frontRadius_=0.0,rearRadius_=0.0;
  bool hasObstacles_=false;
  std::vector<double> clearance_;
  double at(double worldX,double worldY,double radius) const noexcept;
};

std::vector<GoalConnection> proposeGoalConnections(
    const Model& model, RearState from, RearState goal,
    const SearchOptions& options, std::size_t& evaluations,
    const GoalConnectorObstacleGuide* guide=nullptr);

} // namespace hybrid_astar::articulated_steering
