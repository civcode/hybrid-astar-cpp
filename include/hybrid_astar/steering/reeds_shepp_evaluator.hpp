#pragma once

#include "hybrid_astar/ackermann/planner.hpp"
#include "hybrid_astar/ackermann/refinement.hpp"
#include <memory>
#include <optional>
#include <vector>

namespace hybrid_astar::steering {

// Optional OMPL-backed evaluator; link hybrid_astar::ompl (not core).
// Reuse one instance per request/plan; scratch states are NOT thread-safe.
// No OMPL headers or types appear in this public API.
class ReedsSheppEvaluator {
public:
  explicit ReedsSheppEvaluator(double minimumTurningRadiusMeters);
  ~ReedsSheppEvaluator();
  ReedsSheppEvaluator(const ReedsSheppEvaluator&) = delete;
  ReedsSheppEvaluator& operator=(const ReedsSheppEvaluator&) = delete;
  ReedsSheppEvaluator(ReedsSheppEvaluator&&) noexcept;
  ReedsSheppEvaluator& operator=(ReedsSheppEvaluator&&) noexcept;

  double distance(const Pose2D& from, const Pose2D& goal) const;
  // Complete signed sections with their actual exact endpoints and gears.
  // If start=goal, an empty path is permitted.
  std::vector<ackermann::MotionSection> shortestPath(
      const Pose2D& from, const Pose2D& goal) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// Explicit opt-in OMPL-equipped planner entry point.
// Builds ONE evaluator for the synchronous request, with safe callback lifetime.
// By default, keeps the request's chosen heuristic (now obstacle-grid).
// A true flag forces grid mode and additionally injects RS distance guidance.
// To retain certified Euclidean guidance, set request.options.heuristicMode
// explicitly and leave useReedsSheppDistanceHeuristic false.
// Throws invalid_argument for an invalid OMPL turning radius.
ackermann::PlanResult planWithReedsShepp(ackermann::PlanRequest request,
                                        bool useReedsSheppDistanceHeuristic=false);

// Opt-in complete planning and post-search refinement with one reusable
// OMPL evaluator and strictly scoped callbacks. Refinement is attempted only
// after planning succeeds; the original validated path always remains visible.
struct ReedsSheppRefinementResult {
  ackermann::PlanResult original;
  std::optional<ackermann::RefinementResult> refinement;
};
ReedsSheppRefinementResult planAndRefineWithReedsShepp(
    ackermann::PlanRequest request,
    ackermann::RefinementOptions refinementOptions = {},
    bool useReedsSheppDistanceHeuristic = false);

} // namespace hybrid_astar::steering
