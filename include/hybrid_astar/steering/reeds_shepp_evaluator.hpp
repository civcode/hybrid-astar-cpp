#pragma once

#include "hybrid_astar/ackermann/planner.hpp"
#include <memory>
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

} // namespace hybrid_astar::steering
