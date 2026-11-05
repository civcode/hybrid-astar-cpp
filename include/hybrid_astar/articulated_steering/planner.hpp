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
struct MapLocalAabb {double minX=0.0,minY=0.0,maxX=0.0,maxY=0.0;};
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
  double articulationPrimitiveRadians=0.2; // max neighboring gamma spacing (rad)
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
  // Simultaneous driving/articulation is preintegrated per lattice edge.
  bool enableCoupledMotion=true;
  // Favor steering while moving, retaining zero-speed steering as a fallback.
  double stationaryArticulationPenaltyMeters=0.15;
  // Conservative full-primitive empty-map AABB proof; false falls back to
  // the original two-body sampled search collision checking.
  bool useSweptAabbPrefilter=true;
  // Performance ordering only: not a certified lower bound and not optimal.
  double heuristicWeight=1.0;
  // Shared Ackermann reverse-Dijkstra occupancy guidance (performance only).
  // The grid is a point-robot relaxation, not an articulated clearance oracle.
  bool useObstacleGridHeuristic=true;
  std::size_t gridMaxSettlementsPerQuery=500;
  std::size_t gridMaxTotalSettlements=100000;
  // Match the Planning Lab's user-configurable 1M expansion ceiling.
  // Allow four generated labels per expansion on average; these are
  // independent work/memory budgets, not guaranteed memory reservations.
  std::size_t maxExpansions=1'000'000;
  std::size_t maxGenerated=4'000'000;
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
  // Primitive generation is map-independent and performed once per plan.
  // Search applies these rigidly transformed templates; it does NOT
  // reintegrate each successor. Final strict checks still use RK4.
  std::size_t precomputedAnalyticDriveTemplates=0;
  std::size_t precomputedStationaryRk4Templates=0;
  std::size_t precomputedCoupledRk4Templates=0;
  std::size_t appliedCoupledTemplates=0;
  std::size_t searchSweptAabbAccepted=0;
  std::size_t searchSweptAabbFallback=0;
  std::size_t precomputedSampleCount=0;
  std::size_t appliedPrecomputedTemplates=0;
  // Names match the shared grid's generic diagnostics interface.
  std::size_t gridSettlements=0,gridFallbacks=0;
  std::size_t gridSeedCellsExamined=0,gridSeedCellsAccepted=0;
  std::size_t gridQueuePushes=0,gridQueuePops=0,gridQueueStalePops=0;
  std::size_t gridQueuePeakSize=0,gridQueuePeakStorageBytes=0;
  std::size_t gridBlockedQueryFallbacks=0,gridWorkBudgetFallbacks=0;
  std::uint64_t gridQueueNanoseconds=0;
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
  // True only when BOTH swept reference-radius envelopes have an empty
  // occupancy prefix region and lie strictly inside map boundaries.
  bool provenEmptySweptAabbs(MapLocalAabb front,MapLocalAabb rear) const noexcept;
  double maxFrontBodyRadius() const noexcept;
  double maxRearBodyRadius() const noexcept;
private:
  ackermann::ExactCollisionChecker front_,rear_;
};

Result plan(const Request& request);
} // namespace hybrid_astar::articulated_steering
