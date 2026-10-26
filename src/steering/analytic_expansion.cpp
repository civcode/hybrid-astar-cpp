#include "hybrid_astar/steering/analytic_expansion.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace hybrid_astar::steering {
namespace {

void validateOptions(const AnalyticExpansionOptions& c) {
  if (!std::isfinite(c.attemptRatio) || c.attemptRatio < 1.0 ||
      (c.maxWeightedCostMeters &&
       (!std::isfinite(*c.maxWeightedCostMeters) || *c.maxWeightedCostMeters < 0.0))) {
    throw std::invalid_argument("Invalid analytic expansion options");
  }
}

} // namespace

AnalyticCostGate evaluateAnalyticCost(double weightedCost,
                                     const AnalyticExpansionOptions& c) {
  validateOptions(c);
  if (!std::isfinite(weightedCost) || weightedCost < 0.0) {
    throw std::invalid_argument("Invalid analytic connection cost");
  }
  if (c.maxWeightedCostMeters && weightedCost > *c.maxWeightedCostMeters) {
    return AnalyticCostGate::OverCostLimit;
  }
  return AnalyticCostGate::AcceptableForFurtherValidation;
}

AnalyticExpansionScheduler::AnalyticExpansionScheduler(AnalyticExpansionOptions options)
: options_(std::move(options)) {
  validateOptions(options_);
}

void AnalyticExpansionScheduler::reset() noexcept {
  closestHeuristicBins_ = std::numeric_limits<double>::infinity();
  expansionsUntilNextAttempt_ = 0;
}

bool AnalyticExpansionScheduler::shouldAttempt(double heuristicCostMeters,
                                               double searchXYBinMeters) {
  if (!std::isfinite(heuristicCostMeters) || heuristicCostMeters < 0.0 ||
      !std::isfinite(searchXYBinMeters) || searchXYBinMeters <= 0.0) {
    throw std::invalid_argument("Invalid heuristic distance or map resolution");
  }

  // Convert meters to SEARCH BIN units; occupancy resolution is irrelevant.
  const double bins = heuristicCostMeters / searchXYBinMeters;
  if (!std::isfinite(cells)) {
    throw std::out_of_range("Heuristic search-bin distance overflow");
  }
  closestHeuristicBins_ = std::min(closestHeuristicBins_, bins);
  const double proposed = std::max(
      std::floor(closestHeuristicBins_ / options_.attemptRatio),
      std::ceil(options_.attemptRatio));

  // Cap unreasonably large intervals without undefined floating-to-int casts.
  constexpr auto maxInterval = std::numeric_limits<std::uint32_t>::max();
  const auto interval = static_cast<std::uint32_t>(
      std::min(proposed, static_cast<double>(maxInterval)));
  const auto remainingAfterAttempt = interval - 1;

  // Increasing proximity can shorten the outstanding interval immediately.
  expansionsUntilNextAttempt_ =
      std::min(expansionsUntilNextAttempt_, remainingAfterAttempt);
  if (expansionsUntilNextAttempt_ == 0) {
    expansionsUntilNextAttempt_ = remainingAfterAttempt;
    return true;
  }
  --expansionsUntilNextAttempt_;
  return false;
}

} // namespace hybrid_astar::steering
