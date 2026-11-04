#pragma once

#include "hybrid_astar/articulated_steering/model.hpp"
#include "hybrid_astar/ackermann/planner.hpp"
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace hybrid_astar::articulated_steering {

// The existing rigid SE(2) Hybrid A* entry point is intentionally unchanged.
// This planner carries x_r, y_r, theta_r, gamma plus incoming gear in its
// search labels. Every sampled full state checks TWO separate body footprints.
// It is a bounded sampled-collision planner, NOT a continuous swept-volume or
// actuator-trajectory certificate. Caller must provide real robot geometry.
struct BodyGeometry {
  // Invalid by default: never silently adopt the rigid Ackermann's
  // physical dimensions for a two-carriage articulated robot.
  ackermann::Footprint front{0.0,0.0,0.0}; // front reference/heading frame
  ackermann::Footprint rear{0.0,0.0,0.0};  // rear base_link/heading frame
  double inflationMeters=0.0;
};
struct SearchOptions {
  double translationSpeedMetersPerSecond=0.5;
  double articulationRateRadiansPerSecond=0.4;
  double translationPrimitiveMeters=0.5;
  double articulationPrimitiveRadians=0.2;
  double xyBinMeters=0.15;
  double yawBinRadians=0.15;
  double gammaBinRadians=0.10;
  double goalXYToleranceMeters=0.20;
  double goalYawToleranceRadians=0.15;
  double goalGammaToleranceRadians=0.10;
  double maxRk4StepSeconds=0.025;
  double searchMaxBodyPointMotionMeters=0.08;
  double finalMaxBodyPointMotionMeters=0.025;
  double articulationCostMetersPerRadian=0.25;
  double reverseCostMultiplier=1.1;
  // Performance ordering only: not a certified lower bound and not optimal.
  double heuristicWeight=1.0;
  std::size_t maxExpansions=50000;
  std::size_t maxGenerated=200000;
  std::size_t maxSamplesPerPrimitive=300;
  std::size_t maxFinalSamplesPerPrimitive=2000;
  std::size_t maxTotalFinalBodyChecks=250000;
};
enum class Status {
  Success, InvalidRequest, InvalidMap, RejectedUnknownMap,
  StartInCollision, GoalInCollision, NoPath, BudgetExceeded,
  Cancelled, ValidationIncomplete, StrictValidationFailed
};
// Distinct keys for different articulation and incoming direction.
// Exposed to allow deterministic unit tests of the state discretization.
struct SearchKey {
  std::int64_t x=0, y=0, yaw=0, gamma=0;
  int incomingGear=0;
  bool operator==(const SearchKey&) const =default;
};
SearchKey makeSearchKey(const RearState& rear, int incomingGear,
                        const SearchOptions& options,
                        const Parameters& parameters);
struct Motion {
  Control control;
  double durationSeconds=0.0;
  State start, end;
  // Independent strict-validator checkpoints: full five-state configuration
  // and derived rear base_link pose, ordered in time and including endpoints.
  std::vector<Sample> samples;
};
struct Request {
  std::shared_ptr<const ackermann::MapSnapshot> map;
  ackermann::OccupancyPolicy occupancy;
  Parameters parameters{}; // invalid until real robot dimensions/limits supplied
  BodyGeometry bodies;
  RearState start, goal; // both include gamma; goal gamma is constrained
  SearchOptions options;
  std::shared_ptr<const std::atomic_bool> cancelled;
};
struct Diagnostics {
  std::size_t expanded=0, generated=0, stalePops=0, pruned=0;
  std::size_t searchBodyChecks=0, finalBodyChecks=0;
  std::size_t rejectedSearchCollision=0, rejectedInvalidMotion=0;
  std::size_t strictGoalCandidates=0, strictRejected=0;
};
struct Result {
  Status status=Status::InvalidRequest;
  std::vector<Motion> motions;
  RearState reached;
  double costMeters=0.0;
  double goalXYErrorMeters=0.0, goalYawErrorRadians=0.0;
  double goalGammaErrorRadians=0.0;
  bool independentlySampledBodyValidated=false;
  bool trajectoryActuatorFeasible=false; // speed profiles are NOT implemented
  Diagnostics diagnostics;
  std::uint64_t mapRevision=0;
};

class BodyCollisionChecker {
public:
  BodyCollisionChecker(const ackermann::MapSnapshot& map,
                       ackermann::OccupancyPolicy occupancy,
                       BodyGeometry geometry);
  bool clear(const Model& model,const State& state) const;
  double maxFrontBodyRadius() const noexcept;
  double maxRearBodyRadius() const noexcept;
private:
  ackermann::ExactCollisionChecker front_,rear_;
};

Result plan(const Request& request);
} // namespace hybrid_astar::articulated_steering
