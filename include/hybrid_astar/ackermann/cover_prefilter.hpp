#pragma once
#include "hybrid_astar/ackermann/planner.hpp"
#include <cstddef>
#include <vector>

namespace hybrid_astar::ackermann {
// Conservative broad-phase for the filled rectangular Ackermann body.
// Per-request, pinned to one *immutable* map + policy + geometry. A true
// result proves only this POSE is clear, never a continuously swept motion.
// False returns are UNCERTAIN, not proof of collision.
class ConservativeDiscPrefilter {
public:
  ConservativeDiscPrefilter(const MapSnapshot& map, OccupancyPolicy policy,
                           Footprint footprint, std::size_t discCount = 4,
                           double inflationMeters = 0.0);
  bool definitelyClear(const Pose2D& pose) const;
  std::size_t discCount() const noexcept { return centers_.size(); }
private:
  const MapSnapshot& map_;
  std::vector<double> centerDistanceSquaredCells_;
  std::vector<double> centers_;
  double radiusMeters_ = 0.0;
  double numericalMarginMeters_ = 1e-9;
};
} // namespace hybrid_astar::ackermann
