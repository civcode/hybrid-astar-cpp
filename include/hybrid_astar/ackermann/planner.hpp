#pragma once

#include "hybrid_astar/ackermann/search_config.hpp"
#include "hybrid_astar/ackermann/open_queue.hpp"
#include "hybrid_astar/steering/analytic_expansion.hpp"
#include <atomic>
#include <cstddef>
#include <cmath>
#include "hybrid_astar/differential_drive/primitives.hpp"
#include "hybrid_astar/mecanum/primitives.hpp"
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace hybrid_astar::planning {class PlanarPolicy;}

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
  ExactCollisionChecker(const MapSnapshot& map, OccupancyPolicy policy,
                        Footprint footprint, double inflationMeters = 0.0);
  PoseCollision check(const Pose2D& pose) const;
  double maxBodyRadius() const noexcept;
private:
  const MapSnapshot& map_;
  OccupancyPolicy policy_;
  Footprint footprint_;
  double inflationMeters_ = 0.0;
  double originCos_, originSin_; // map-frame rotation reused for every pose check
};

// Progress is NONNEGATIVE reference-axle arc length; curvature is signed
// steering curvature, not direction-adjusted yaw rate. Endpoints are analytic.
enum class MotionKind { Translate, RotateInPlace, HolonomicTwist };
struct MotionSection {
  Pose2D start, end;
  Gear gear = Gear::None;
  double curvature = 0.0;
  double lengthMeters = 0.0;
  // Rotation is represented in radians, never conflated with travel metres.
  MotionKind kind = MotionKind::Translate;
  double turnRadians = 0.0; // signed for spin or holonomic heading motion
  double forwardMeters = 0.0; // planar body twist
  double lateralMeters = 0.0; // sideways motion, body heading independent of tangent
  double progressExtent() const noexcept {
    return kind==MotionKind::RotateInPlace?std::abs(turnRadians):lengthMeters;
  }
  // Progress is measured in metres for translation, radians for rotation.
  Pose2D evaluate(double progress) const;
};
MotionSection makeSection(const Pose2D& start, Gear gear,
                          double curvature, double lengthMeters);
MotionSection makeRotationSection(const Pose2D& start,double signedTurnRadians);
MotionSection makeHolonomicSection(const Pose2D& start,double forwardMeters,
                                  double lateralMeters,double signedTurnRadians);

enum class CollisionEvidence { None, FullFootprintBodyMotionAdaptiveSampled };
enum class PlanStatus {
  Success, NoPath, InvalidRequest, InvalidMap, RejectedUnknownMap,
  StartInCollision, BudgetExceeded, Cancelled, ValidationIncomplete,
  StrictValidationFailed, InternalError
};

enum class HeuristicMode { CertifiedRegionEuclidean, PerformanceReedsSheppAndObstacleGrid };
enum class SolutionPolicy { FirstStrictlyValidated, ImproveIncumbent };
enum class SolutionQuality { FeasibleFirst, FeasibleAfterImprovement, FeasibleAfterRefinement };
enum class TerminationReason { None, FirstValidated, ImprovementBudgetExhausted, SearchExhausted, WorkBudgetExceeded, CancelledAfterIncumbent, ValidationBudgetExhausted };

struct PlanOptions {
  SearchConfig search;
  enum class Model { Ackermann, DifferentialDrive, Mecanum } model = Model::Ackermann;
  differential_drive::Parameters diffParameters{};
  differential_drive::PrimitiveConfig diffPrimitives{};
  mecanum::Parameters mecanumParameters{};
  mecanum::PrimitiveConfig mecanumPrimitives{};
  double rotationCostMetersPerRadian = 0.40; // must be strictly positive
  // Penalize a full stop to spin when moving turns are feasible.
  // This is a search-quality preference, not literal traveled distance.
  double mecanumStationarySpinPenaltyMeters = 0.35;
  bool mecanumMergeIntermediateSpins = true;
  steering::AnalyticExpansionOptions analytic; // only used with connector
  // Performance default: bounded lazy obstacle-aware ordering is not a
  // certified continuous-space lower bound. Certified mode is still opt-in.
  HeuristicMode heuristicMode = HeuristicMode::PerformanceReedsSheppAndObstacleGrid;
  SolutionPolicy solutionPolicy = SolutionPolicy::FirstStrictlyValidated;
  std::size_t maxExtraExpansionsAfterFirst = 2500; // optional, measured separately
  std::size_t gridMaxSettlementsPerQuery = 500;
  double performanceHeuristicWeight = 1.0; // no default inflation; optional weight [1,3]
  std::size_t gridMaxTotalSettlements = 100000; // optional performance grid; bounded by reachable cells
  bool gridPreventDiagonalCornerCutting = true; // false only for controlled A/B trials
  // Geometric Minkowski offset of the *physical* footprint by a disk.
  // 0 preserves the legacy checker; applied to search and final validation.
  double footprintInflationMeters = 0.0;
  double clearanceWeight = 0.0; // zero by default: equivalent-meter integral
  double clearanceInfluenceRadiusMeters = 0.40;
  std::size_t discPrefilterCount = 0; // 0=off, optionally benchmark 4 or 5 // finite lazy Dijkstra budget
  double minimumTurningRadiusMeters = 4.0;
  double goalXYToleranceMeters = 0.25;
  double goalYawToleranceRadians = 0.17453292519943295; // 10 degrees
  double searchSampleSpacingMeters = 0.10;
  double searchMaxBodyPointMotionMeters = 0.15;
  double finalSampleSpacingMeters = 0.05;
  double finalMaxBodyPointMotionMeters = 0.05;
  std::size_t maxSamplesPerSection = 10000; // search only
  std::size_t maxFinalSamplesPerSection = 10000; // independent strict validator
  std::size_t maxTotalFinalPoseChecks = 250000; // cumulative strict whole-path checks
  OpenQueueMode gridQueueMode = OpenQueueMode::BinaryLazy; // grid frontier A/B
  OpenQueueMode openQueueMode = OpenQueueMode::BinaryLazy; // baseline
  std::size_t searchLabelReserve = 0; // opt-in initial hash-table reserve (0 = baseline)
  std::size_t maxExpansions = 1'000'000;
  std::size_t maxGeneratedNodes = 4'000'000; // allows up to four generated nodes per expanded node
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
  std::size_t primitiveTemplatesBuilt = 0, primitiveTemplateInstantiations = 0;
  std::size_t primitiveTemplatePoseChecks = 0;
  std::size_t staleHeapPops = 0, prunedByKey = 0;
  std::size_t openQueuePushes = 0, openQueuePops = 0;
  std::size_t openQueueReplacements = 0, openQueuePeakSize = 0;
  std::size_t openQueuePeakStorageBytes = 0;
  std::size_t gridQueuePushes = 0, gridQueuePops = 0;
  std::size_t gridQueueStalePops = 0, gridQueuePeakSize = 0;
  std::size_t gridQueuePeakStorageBytes = 0;
  std::size_t labelFinds = 0, labelFindHits = 0, labelInsertions = 0;
  std::size_t labelUpdates = 0, labelRehashes = 0, labelPeakBuckets = 0;
  std::size_t labelPeakSize = 0, hashCalls = 0;
  std::size_t searchMotionCalls = 0, finalMotionCalls = 0;
  std::size_t exactCollisionPoseCalls = 0;
  // Timing counters are nonzero only with HYBRID_ASTAR_PROFILE_HOTPATHS=ON.
  // Motion timings INCLUDE nested exact-collision timings: do not sum.
  std::uint64_t labelLookupNanoseconds = 0, labelWriteNanoseconds = 0;
  std::uint64_t hashNanoseconds = 0, openQueueNanoseconds = 0;
  std::uint64_t gridQueueNanoseconds = 0, exactCollisionNanoseconds = 0;
  std::uint64_t searchMotionNanoseconds = 0, finalMotionNanoseconds = 0;
  std::size_t searchPoseChecks = 0, finalPoseChecks = 0;
  std::size_t analyticAttempts = 0, analyticAccepted = 0;
  std::size_t gridSettlements = 0, gridFallbacks = 0;
  std::size_t gridSeedCellsExamined = 0, gridSeedCellsAccepted = 0;
  std::size_t gridBlockedQueryFallbacks = 0, gridWorkBudgetFallbacks = 0;
  std::size_t validatedCandidates = 0, incumbentImprovements = 0;
  // StrictValidationFailed means candidate routes failed independent checks;
  // it does not imply geometric infeasibility.
  std::size_t strictGoalCandidates = 0, strictRejectedCollision = 0;
  std::size_t strictRejectedContinuity = 0, strictRejectedGoalMismatch = 0;
  // Optional exact-goal connector attempts and strict path outcomes.
  std::size_t terminalConnectionAttempts = 0, terminalCandidates = 0;
  std::size_t terminalValidated = 0, terminalRejected = 0;
  std::size_t mecanumSpinShortcutsAttempted = 0, mecanumSpinShortcutsAccepted = 0;
  std::size_t discDefinitelyClear = 0, discExactFallback = 0;
  std::size_t clearanceSamples = 0;
};

struct PlanResult {
  PlanStatus status = PlanStatus::InvalidRequest;
  std::vector<MotionSection> sections; // evaluable, chronologically ordered
  Pose2D reached; // ACTUAL last state, never snapped to nominal goal
  Gear finalGear = Gear::None;
  double goalXYErrorMeters = 0.0, goalYawErrorRadians = 0.0;
  double weightedCostMeters = 0.0;
  double clearanceCostMeters = 0.0;
  SolutionQuality solutionQuality = SolutionQuality::FeasibleFirst;
  TerminationReason terminationReason = TerminationReason::None;
  double geometricLengthMeters = 0.0, reverseLengthMeters = 0.0;
  std::vector<std::size_t> gearChangeSectionIndices; // cusp begins at this section
  CollisionEvidence collisionEvidence = CollisionEvidence::None;
  bool modelGeometryValid = false;
  bool trajectoryActuatorFeasible = false; // external timing validator needed
  bool heuristicCertifiedAdmissible = false; // true only for certified Euclidean mode
  std::string mapFrameId, mapSourceId;
  std::uint64_t mapRevision = 0;
  OccupancyPolicy effectiveOccupancy;
  PlanDiagnostics diagnostics;
};

// Standalone C++20 Ackermann vertical slice; no ROS or OMPL dependency.
// One active arrival per (x,y,yaw,incoming gear) key; not complete/optimal.
// First strictly validated feasible path; finite work-limit aborts != NoPath.
PlanResult plan(const PlanRequest& request);
// Public planar-model extension seam: same collision, key-label, queue,
// budgets, path reconstruction and independent strict final validator.
// The caller must keep policy and its referenced configuration alive.
// Non-planar/articulated state and multibody geometry are NOT yet supported.
PlanResult planWithPolicy(const PlanRequest& request,
                          const planning::PlanarPolicy& policy);

} // namespace hybrid_astar::ackermann
