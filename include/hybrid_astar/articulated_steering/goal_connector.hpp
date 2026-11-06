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
// Operational metrics are distinct from the planner's existing g-cost.
// They guide candidate fitting/ranking but never weaken full-path validation
// or disguise quality penalties as physical path length.
struct GoalConnectorEffort {
  double drivingMeters=0.0;
  double reverseMeters=0.0;
  double totalArticulationRadians=0.0;
  double stationarySteeringSeconds=0.0;
  double peakAbsGammaRadians=0.0;
  std::size_t gearChanges=0;
  std::size_t articulationReversals=0;
  std::size_t driveArcs=0;
  double operationalPenaltyMeters=0.0;
  double qualityScoreMeters=0.0; // planner cost + operational penalty
};
struct GoalConnection {
  std::vector<GoalControl> controls;
  RearState estimatedEnd;
  double costMeters=0.0; // unchanged path g-cost, used for incumbent comparison
  double clearancePenalty=0.0; // approximate ranking only; no clearance proof
  bool includesCoupledMotion=false;
  GoalConnectorEffort effort;
};

// Summarize physically commanded controls (with adjacent chunks merged
// conceptually). Reversals do not count artificial one-metre chunk boundaries.
GoalConnectorEffort summarizeGoalConnectorEffort(
    const std::vector<GoalControl>& controls,RearState start,
    const SearchOptions& options);

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
