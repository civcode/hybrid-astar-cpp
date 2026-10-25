#pragma once

#include <optional>
#include "hybrid_astar/core/pose2d.hpp"

namespace hybrid_astar::steering {
struct AnalyticExpansionOptions {
  double triggerXYMeters = 20.0;
  std::optional<double> maxWeightedCostMeters;
};
enum class AnalyticGate {
  OutsideTrigger,
  OverCostLimit,
  AcceptableForFurtherValidation
};
// Does not establish collision freedom or endpoint/model validity.
// Throws if configuration, poses, or actual connector metrics are invalid.
AnalyticGate evaluateAnalyticGuards(const Pose2D& from, const Pose2D& to,
                                   double weightedCostMeters,
                                   const AnalyticExpansionOptions& options);
} // namespace hybrid_astar::steering
