#pragma once
#include "hybrid_astar/ackermann/planner.hpp"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace hybrid_astar::ackermann {

// Tier series v2 has four capacity steps per doubling:
// 0, 65536, 81920, 98304, 114688, 131072, ...,
// 524288, 655360, 786432, 917504, 1048576, ...
// v1 was 0, 65536, 131072, 262144, 524288, 1048576, ...
// A zero reserve is the original growing-table behavior, NOT a
// no-rehash certificate for a nonempty search. Persist BOTH version and
// tier index; historical v1 indices must be decoded with seriesVersion=1.
inline constexpr std::uint32_t LabelReserveTierSeriesVersion = 2;
std::size_t labelReserveForTier(
    std::size_t tierIndex,
    std::uint32_t seriesVersion = LabelReserveTierSeriesVersion);

// Returns the smallest nonzero tier able to hold observedPeakLabels with
// headroom, or throws if that tier exceeds the planner's maxGeneratedNodes.
// This is empirical: it never promises a bound for unobserved routes.
std::size_t labelReserveTierForPeak(std::size_t observedPeakLabels,
                                    double headroomFraction,
                                    std::size_t maxGeneratedNodes);

struct LabelReserveCalibrationOptions {
  std::size_t routeSamples = 32;
  std::uint64_t seed = 20261104;
  // Auto-select approximately 2*routeSamples map-local strata when zero.
  std::size_t strataX = 0;
  std::size_t strataY = 0;
  std::size_t poseAttemptsPerStratum = 48;
  std::size_t maxPairAttemptsPerRoute = 32;
  // A value of zero chooses 15% of the map's diagonal (in metres).
  double minStartGoalDistanceMeters = 0.0;
  // Safety margin above the maximum observed live-label count.
  double headroomFraction = 0.20;
  bool verifyNoRehash = true;
};

struct LabelReserveCalibrationTrial {
  Pose2D start, goal;
  std::size_t startStratum = 0, goalStratum = 0;
  PlanStatus status = PlanStatus::InvalidRequest;
  std::size_t expanded = 0, generated = 0;
  std::size_t peakLabels = 0, baselineRehashes = 0;
  std::size_t verifiedRehashes = 0;
};

struct LabelReserveCalibrationResult {
  // The map and planning options (model, geometry, costs, budgets, policies)
  // must remain unchanged when applying this empirical recommendation.
  std::uint64_t mapRevision = 0;
  // Stable FNV-1a of occupancy and map geometry. Map revisions from YAML
  // loaders may be zero, so revision alone cannot identify calibration data.
  std::uint64_t mapFingerprint = 0;
  std::size_t mapWidth = 0, mapHeight = 0;
  std::size_t strataX = 0, strataY = 0, validStrata = 0;
  std::size_t coveredStartStrata = 0, coveredGoalStrata = 0;
  std::size_t coveredEitherStrata = 0; // unique occupied start-or-goal strata
  std::size_t requestedRoutes = 0, evaluatedRoutes = 0, successfulRoutes = 0;
  std::size_t budgetLimitedRoutes = 0;
  std::size_t maxObservedPeakLabels = 0;
  std::uint32_t tierSeriesVersion = LabelReserveTierSeriesVersion;
  std::size_t recommendedTierIndex = 0;
  std::size_t recommendedReserveLabels = 0;
  bool sampleTargetMet = false;
  bool verificationPassed = false;
  std::vector<LabelReserveCalibrationTrial> trials;
};

// Explicit OFFLINE calibration, never invoked implicitly by plan().
// Generate seeded jittered map-local strata, accept exact collision-free
// start/goal poses, explore geographically separated pairs, then choose and
// optionally verify the smallest tier above max observed peak + headroom.
// Does NOT require routes to succeed; difficult/budget-limited searches
// contribute to the observed peak. Throws for insufficient spatially
// valid pairs, invalid input, or failed no-rehash/equivalence verification.
LabelReserveCalibrationResult calibrateLabelReserve(
    const PlanRequest& prototype,
    const LabelReserveCalibrationOptions& settings = {});

} // namespace hybrid_astar::ackermann
