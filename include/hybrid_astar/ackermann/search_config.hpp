#pragma once

#include <cstdint>
#include "hybrid_astar/core/pose2d.hpp"

namespace hybrid_astar::ackermann {

enum class Gear : std::uint8_t { None, Forward, Reverse };

struct SearchConfig {
  double xyBinMeters = 0.25;
  std::uint32_t yawBins = 72; // 5-degree search bins; goal tolerance remains 10 degrees
  double primitiveLengthMeters = 0.50;
  double rsTriggerXYMeters = 20.0;
  double forwardCostMultiplier = 1.0;
  double reverseCostMultiplier = 1.10;
  double gearSwitchCostMeters = 0.50;
  // Clearance edge-cost scoring is deferred.
};

struct SearchKey {
  std::int64_t xBin = 0;
  std::int64_t yBin = 0;
  std::uint32_t yawBin = 0;
  Gear incomingGear = Gear::None;
  bool operator==(const SearchKey&) const = default;
};

void validate(const SearchConfig& config);
SearchKey quantize(const Pose2D& pose, Gear incomingGear, const SearchConfig& config);
// Cost uses traveled arc length, not endpoint Euclidean distance.
double edgeCost(double arcLengthMeters, Gear outgoing, Gear incoming,
                const SearchConfig& config);

} // namespace hybrid_astar::ackermann
