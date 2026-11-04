#pragma once
#include "hybrid_astar/ackermann/planner.hpp"
#include <cstddef>
#include <vector>
namespace hybrid_astar::ackermann {
// Optional exact-oriented-footprint-to-blocked-square clearance scorer.
// A quality objective only: not a collision validator or swept certificate.
// Policy/map/footprint are pinned to the calling request.
class ClearanceScorer {
public:
  ClearanceScorer(const MapSnapshot& map,OccupancyPolicy occupancy,Footprint footprint,
                  double weight,double influenceRadiusMeters,
                  bool useDistanceTransform=false,double inflationMeters=0.0);
  double poseClearanceMeters(const Pose2D& pose,
                             PlanDiagnostics* diagnostics=nullptr) const;
  // Trapezoidal nonnegative cost over evaluable section's true arc progress.
  // Throws invalid_argument for incomplete required sample budget.
  double sectionCost(const MotionSection& section,double referenceSpacing,
                     double maxBodyMotion,std::size_t maxSamples,
                     std::size_t& samples,PlanDiagnostics* diagnostics=nullptr) const;
  std::size_t distanceTransformBytes() const noexcept {
    return blockedCenterDistanceSquared_.size()*sizeof(double);
  }
  double distanceTransformBuildMilliseconds() const noexcept {return buildMs_;}
private:
  double exactPoseClearanceMeters(const Pose2D& pose) const;
  void buildDistanceTransform();
  bool edtProvesZeroCost(const Pose2D& pose) const;
  const MapSnapshot& map_;
  OccupancyPolicy occupancy_;
  Footprint footprint_;
  double lambda_,influence_,bodyRadius_,inflationMeters_;
  bool useDistanceTransform_;
  double originCos_,originSin_; // reused for every EDT/exact pose sample
  double buildMs_=0.0;
  // Squared nearest blocked-cell center distance in grid-cell units.
  std::vector<double> blockedCenterDistanceSquared_;
};
} // namespace hybrid_astar::ackermann
