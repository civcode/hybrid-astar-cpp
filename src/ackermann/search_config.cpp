#include "hybrid_astar/ackermann/search_config.hpp"

#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace hybrid_astar::ackermann {
namespace {
constexpr double tau = 2.0 * std::numbers::pi_v<double>;

std::int64_t floorBin(double value, double step) {
  const double raw = std::floor(value / step);
  // Bounds must be checked before floating-point to integer conversion.
  if (!std::isfinite(raw) ||
      static_cast<long double>(raw) < static_cast<long double>(std::numeric_limits<std::int64_t>::min()) ||
      static_cast<long double>(raw) >= static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
    throw std::out_of_range("Search coordinate outside representable bins");
  }
  return static_cast<std::int64_t>(raw);
}
} // namespace

void validate(const SearchConfig& c) {
  if (!std::isfinite(c.xyBinMeters) || c.xyBinMeters <= 0.0 || c.yawBins == 0 ||
      !std::isfinite(c.primitiveLengthMeters) || c.primitiveLengthMeters <= 0.0 ||
      !std::isfinite(c.rsTriggerXYMeters) || c.rsTriggerXYMeters < 0.0 ||
      !std::isfinite(c.forwardCostMultiplier) || c.forwardCostMultiplier < 1.0 ||
      !std::isfinite(c.reverseCostMultiplier) || c.reverseCostMultiplier < 1.0 ||
      !std::isfinite(c.gearSwitchCostMeters) || c.gearSwitchCostMeters < 0.0) {
    throw std::invalid_argument("Invalid Ackermann search config");
  }
}

SearchKey quantize(const Pose2D& pose, Gear incomingGear, const SearchConfig& c) {
  validate(c);
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw)) {
    throw std::invalid_argument("Non-finite pose");
  }
  double yaw = std::fmod(pose.yaw, tau);
  if (yaw < 0.0) yaw += tau;
  const double width = tau / static_cast<double>(c.yawBins);
  auto bin = static_cast<std::uint32_t>(std::floor(yaw / width + 0.5));
  bin %= c.yawBins;
  return {floorBin(pose.x, c.xyBinMeters), floorBin(pose.y, c.xyBinMeters), bin, incomingGear};
}

double edgeCost(double length, Gear outgoing, Gear incoming, const SearchConfig& c) {
  validate(c);
  if (!std::isfinite(length) || length < 0.0 || outgoing == Gear::None) {
    throw std::invalid_argument("Invalid directed arc length");
  }
  const double multiplier = outgoing == Gear::Reverse ? c.reverseCostMultiplier : c.forwardCostMultiplier;
  const double cost = length * multiplier +
    ((incoming != Gear::None && incoming != outgoing) ? c.gearSwitchCostMeters : 0.0);
  if (!std::isfinite(cost)) throw std::out_of_range("Edge cost overflow");
  return cost;
}
} // namespace hybrid_astar::ackermann
