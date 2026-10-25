#pragma once

#include <limits>
#include <optional>
#include "hybrid_astar/core/pose2d.hpp"

namespace hybrid_astar::steering {
struct AnalyticExpansionOptions {
  double triggerXYMeters = 20.0;
  // Must be set from the actual turning radius/vehicle fixture.
  double maxPathLengthMeters = std::numeric_limits<double>::quiet_NaN();
  std::optional<double> maxWeightedCostMeters;
};
enum class AnalyticGate {
  OutsideTrigger,
  OverLengthLimit,
  OverCostLimit,
  AcceptableForFurtherValidation
};
// Does not establish collision freedom or endpoint/model validity.
// Throws if configuration, poses, or actual connector metrics are invalid.
AnalyticGate evaluateAnalyticGuards(const Pose2D& from, const Pose2D& to,
                                   double actualLengthMeters,
                                   double weightedCostMeters,
                                   const AnalyticExpansionOptions& options);
} // namespace hybrid_astar::steering
