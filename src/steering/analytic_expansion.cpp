#include "hybrid_astar/steering/analytic_expansion.hpp"
#include <cmath>
#include <stdexcept>

namespace hybrid_astar::steering {
AnalyticGate evaluateAnalyticGuards(const Pose2D& from, const Pose2D& to,
                                   double weightedCost,
                                   const AnalyticExpansionOptions& c) {
  if (!std::isfinite(c.triggerXYMeters) || c.triggerXYMeters < 0.0 ||
      (c.maxWeightedCostMeters &&
       (!std::isfinite(*c.maxWeightedCostMeters) || *c.maxWeightedCostMeters < 0.0))) {
    throw std::invalid_argument("Invalid analytic expansion guard configuration");
  }
  if (!std::isfinite(from.x) || !std::isfinite(from.y) ||
      !std::isfinite(to.x) || !std::isfinite(to.y) ||
      !std::isfinite(from.yaw) || !std::isfinite(to.yaw) ||
      !std::isfinite(weightedCost) || weightedCost < 0.0) {
    throw std::invalid_argument("Invalid analytic candidate values");
  }
  if (std::hypot(from.x - to.x, from.y - to.y) > c.triggerXYMeters)
    return AnalyticGate::OutsideTrigger;
  if (c.maxWeightedCostMeters && weightedCost > *c.maxWeightedCostMeters)
    return AnalyticGate::OverCostLimit;
  return AnalyticGate::AcceptableForFurtherValidation;
}
} // namespace hybrid_astar::steering
