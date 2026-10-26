#pragma once
#include "hybrid_astar/ackermann/planner.hpp"
#include <cstddef>
namespace hybrid_astar::ackermann {
// Optional exact-oriented-footprint-to-blocked-square clearance scorer.
// A quality objective only: not a collision validator or swept certificate.
// Policy/map/footprint are pinned to the calling request.
class ClearanceScorer {
public:
  ClearanceScorer(const MapSnapshot& map,OccupancyPolicy occupancy,Footprint footprint,
                  double weight,double influenceRadiusMeters);
  double poseClearanceMeters(const Pose2D& pose) const;
  // Trapezoidal nonnegative cost over evaluable section's true arc progress.
  // Throws invalid_argument for incomplete required sample budget.
  double sectionCost(const MotionSection& section,double referenceSpacing,
                     double maxBodyMotion,std::size_t maxSamples,
                     std::size_t& samples) const;
private:
  const MapSnapshot& map_;
  OccupancyPolicy occupancy_;
  Footprint footprint_;
  double lambda_,influence_,bodyRadius_;
};
} // namespace hybrid_astar::ackermann
