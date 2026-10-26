#pragma once

#include "hybrid_astar/ackermann/search_config.hpp"
#include "hybrid_astar/steering/analytic_expansion.hpp"
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace hybrid_astar::ackermann {

// Raw OccupancyGrid semantics: -1 unknown, 0..100 known; x-fastest rows.
struct MapSnapshot {
  std::size_t width = 0, height = 0;
  double resolutionMeters = 0.0;
  Pose2D origin; // world position and yaw of map-local (0,0)
  std::string frameId, sourceId;
  std::uint64_t revision = 0;
  std::vector<std::int8_t> cells;
};

enum class UnknownPolicy { Occupied, Free, Reject };
struct OccupancyPolicy {
  int maxTraversableOccupancy = 0; // threshold equality is traversable
  UnknownPolicy unknown = UnknownPolicy::Occupied;
};

// Filled rectangle in the rear-axle/body frame, not a boundary-only outline.
struct Footprint {
  double rearX = -1.0, frontX = 3.6, halfWidth = 0.9;
};

enum class PoseCollision { Clear, Colliding };
// Construction validates the ENTIRE immutable snapshot and policy once.
// Contact (including map boundary contact) counts as collision.
class ExactCollisionChecker {
public:
  ExactCollisionChecker(const MapSnapshot& map, OccupancyPolicy policy, Footprint footprint);
  PoseCollision check(const Pose2D& pose) const;
  double maxBodyRadius() const noexcept;
private:
  const MapSnapshot& map_;
  OccupancyPolicy policy_;
  Footprint footprint_;
};

// Progress is NONNEGATIVE reference-axle arc length; curvature is signed
// steering curvature, not direction-adjusted yaw rate. Endpoints are analytic.
struct MotionSection {
  Pose2D start, end;
  Gear gear = Gear::None;
  double curvature = 0.0;
  double lengthMeters = 0.0;
  Pose2D evaluate(double progressMeters) const;
};
MotionSection makeSection(const Pose2D& start, Gear gear,
                          double curvature, double lengthMeters);

enum class CollisionEvidence { None, FullFootprintBodyMotionAdaptiveSampled };
enum class PlanStatus {
  Success, NoPath, InvalidRequest, InvalidMap, RejectedUnknownMap,
  StartInCollision, BudgetExceeded, Cancelled, ValidationIncomplete,
  StrictValidationFailed, InternalError
};

enum class HeuristicMode { CertifiedRegionEuclidean, PerformanceReedsSheppAndObstacleGrid };

struct PlanOptions {
  SearchConfig search;
  steering::AnalyticExpansionOptions analytic; // only used with connector
  HeuristicMode heuristicMode = HeuristicMode::CertifiedRegionEuclidean;
  std::size_t gridMaxSettlementsPerQuery = 500; // finite lazy Dijkstra budget
  double minimumTurningRadiusMeters = 4.0;
  double goalXYToleranceMeters = 0.25;
  double goalYawToleranceRadians = 0.17453292519943295; // 10 degrees
  double searchSampleSpacingMeters = 0.10;
  double searchMaxBodyPointMotionMeters = 0.15;
  double finalSampleSpacingMeters = 0.05;
  double finalMaxBodyPointMotionMeters = 0.05;
  std::size_t maxSamplesPerSection = 10000;
  std::size_t maxExpansions = 100000;
  std::size_t maxGeneratedNodes = 400000;
};

struct PlanRequest {
  std::shared_ptr<const MapSnapshot> map;
  OccupancyPolicy occupancy;
  Footprint footprint;
  Pose2D start, goal;
  PlanOptions options;
  std::shared_ptr<const std::atomic_bool> cancelled; // optional request-local token
  // Optional signed analytic connector; must return complete evaluable sections.
  // The planner independently checks kinematics, cost, collision and final path.
  std::function<std::vector<MotionSection>(const Pose2D&,const Pose2D&)> analyticConnector;
  std::function<double(const Pose2D&,const Pose2D&)> analyticDistance; // optional RS guidance
};

struct PlanDiagnostics {
  std::size_t expanded = 0, generated = 0, reopened = 0;
  std::size_t staleHeapPops = 0, prunedByKey = 0;
  std::size_t searchPoseChecks = 0, finalPoseChecks = 0;
  std::size_t analyticAttempts = 0, analyticAccepted = 0;
  std::size_t gridSettlements = 0, gridFallbacks = 0;
};

struct PlanResult {
  PlanStatus status = PlanStatus::InvalidRequest;
  std::vector<MotionSection> sections; // evaluable, chronologically ordered
  Pose2D reached; // ACTUAL last state, never snapped to nominal goal
  Gear finalGear = Gear::None;
  double goalXYErrorMeters = 0.0, goalYawErrorRadians = 0.0;
  double weightedCostMeters = 0.0;
  double geometricLengthMeters = 0.0, reverseLengthMeters = 0.0;
  std::vector<std::size_t> gearChangeSectionIndices; // cusp begins at this section
  CollisionEvidence collisionEvidence = CollisionEvidence::None;
  bool modelGeometryValid = false;
  bool trajectoryActuatorFeasible = false; // external timing validator needed
  bool heuristicCertifiedAdmissible = true; // Euclidean-to-goal-region only
  std::string mapFrameId, mapSourceId;
  std::uint64_t mapRevision = 0;
  OccupancyPolicy effectiveOccupancy;
  PlanDiagnostics diagnostics;
};

// Standalone C++20 Ackermann vertical slice; no ROS or OMPL dependency.
// One active arrival per (x,y,yaw,incoming gear) key; not complete/optimal.
// First strictly validated feasible path; finite work-limit aborts != NoPath.
PlanResult plan(const PlanRequest& request);

} // namespace hybrid_astar::ackermann
