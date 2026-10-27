#pragma once
#include "hybrid_astar/ackermann/planner.hpp"
#include <functional>
#include <vector>

namespace hybrid_astar::ackermann {
// A bounded, POST-search path refiner. Does not modify the core planner's
// default behavior. Only exact kinematic sections are ever substituted.
// Optional connector can supply signed Reeds-Shepp arcs; when not supplied,
// the refiner attempts straight shortcuts or opt-in fixed-gear CSC curves.
using RefinementConnector =
    std::function<std::vector<MotionSection>(const Pose2D&, const Pose2D&)>;

struct RefinementOptions {
  std::size_t maxAttempts = 64;           // candidate windows, not map queries
  std::size_t maxAccepted = 4;            // full-path replacements
  std::size_t maxReplacementSections = 32;
  bool useCurvatureBoundedShortcuts = false; // opt-in fixed-gear LSL/LSR/RSL/RSR
  double differentialCurveRadiusMeters = 0.75; // base CSC radius, scaled by .5,1,2,4; not car R_min
  std::size_t maxTotalPoseChecks = 100000; // includes original revalidation
  double minimumCostReductionMeters = 1e-6;
};

enum class RefinementStatus {
  InvalidInput, OriginalNotVerified, Unchanged, Improved,
  WorkBudgetExceeded, Cancelled
};

struct RefinementResult {
  RefinementStatus status = RefinementStatus::InvalidInput;
  // Only populated if the returned sections passed INDEPENDENT strict validation
  // with the request's pinned map/occupancy/footprint. Empty on invalid input.
  PlanResult path;
  std::size_t attempted = 0, accepted = 0, rejected = 0;
  std::size_t prunedByCostLowerBound = 0;
  std::size_t exactPoseChecks = 0;
};

// Rejects stale input and revalidates the original under this request before
// trying shortcuts. Ackermann uses straight/CSC sections; differential drive
// only proposes straight or wheel-rate-feasible CSC sections (never a new spin).
// Mecanum supports strictly validated simultaneous planar twist shortcuts.
// Failed/over-budget proposals cannot erase
// the verified original. Results remain sampled, not swept/actuator certified.
RefinementResult refinePath(const PlanRequest& request, const PlanResult& original,
                            RefinementConnector connector = {},
                            RefinementOptions options = {});

} // namespace hybrid_astar::ackermann
