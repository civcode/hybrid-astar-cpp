#include "hybrid_astar/steering/analytic_expansion.hpp"
#include <cmath>
#include <stdexcept>

namespace hybrid_astar::steering {
AnalyticGate evaluateAnalyticGuards(const Pose2D& from, const Pose2D& to,
                                   double length, double weightedCost,
                                   const AnalyticExpansionOptions& c) {
  if (!std::isfinite(c.triggerXYMeters) || c.triggerXYMeters < 0.0 ||
      !std::isfinite(c.maxPathLengthMeters) || c.maxPathLengthMeters <= 0.0 ||
      (c.maxWeightedCostMeters &&
       (!std::isfinite(*c.maxWeightedCostMeters) || *c.maxWeightedCostMeters < 0.0))) {
    throw std::invalid_argument("Invalid analytic expansion guard configuration");
  }
  if (!std::isfinite(from.x) || !std::isfinite(from.y) ||
      !std::isfinite(to.x) || !std::isfinite(to.y) ||
      !std::isfinite(from.yaw) || !std::isfinite(to.yaw) ||
      !std::isfinite(length) || length < 0.0 ||
      !std::isfinite(weightedCost) || weightedCost < 0.0) {
    throw std::invalid_argument("Invalid analytic candidate values");
  }
  if (std::hypot(from.x - to.x, from.y - to.y) > c.triggerXYMeters)
    return AnalyticGate::OutsideTrigger;
  if (length > c.maxPathLengthMeters) return AnalyticGate::OverLengthLimit;
  if (c.maxWeightedCostMeters && weightedCost > *c.maxWeightedCostMeters)
    return AnalyticGate::OverCostLimit;
  return AnalyticGate::AcceptableForFurtherValidation;
}
} // namespace hybrid_astar::steering
