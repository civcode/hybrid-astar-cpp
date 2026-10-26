#pragma once

#include <cstdint>
#include <limits>
#include <optional>

namespace hybrid_astar::steering {

struct AnalyticExpansionOptions {
  // Nav2-style adaptive scheduling, with a first-node attempt and an
  // interval that decreases with the closest observed heuristic distance.
  // 3.5 is an initial tuning value; there is no XY proximity gate.
  double attemptRatio = 3.5;
  std::optional<double> maxWeightedCostMeters; // off by default
};

enum class AnalyticCostGate {
  OverCostLimit,
  AcceptableForFurtherValidation
};

// Only screens the optional weighted cost. Does not validate geometry,
// collisions, endpoints, or the complete reconstructed path.
AnalyticCostGate evaluateAnalyticCost(double weightedCostMeters,
                                     const AnalyticExpansionOptions& options);

// Owned by one planning request; call once for each popped, non-stale node.
// The input heuristic is expressed in meters; mapResolutionMeters converts
// it to the grid-cell units used by the adaptive expansion interval.
// A scheduled attempt is ONE complete Reeds-Shepp path, with all its sections.
class AnalyticExpansionScheduler {
public:
  explicit AnalyticExpansionScheduler(AnalyticExpansionOptions options = {});

  bool shouldAttempt(double heuristicCostMeters, double searchXYBinMeters);
  void reset() noexcept;

private:
  AnalyticExpansionOptions options_;
  double closestHeuristicBins_ = std::numeric_limits<double>::infinity();
  std::uint32_t expansionsUntilNextAttempt_ = 0;
};

} // namespace hybrid_astar::steering
