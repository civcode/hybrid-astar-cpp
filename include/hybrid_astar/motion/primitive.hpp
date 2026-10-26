#pragma once
#include "hybrid_astar/core/pose2d.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>

namespace hybrid_astar::motion {

// All template poses are relative to SE(2) identity (0,0,0). Left composition
// instantiates a cached template without numerical integration at a search node.
enum class ExtentKind { TranslationMeters, RotationRadians };
struct PrimitiveSample {
  double lambda = 0.0; // monotonic local progress in [0,1]
  Pose2D localPose;     // full configuration for planar rigid chassis
};
struct SampleLimits {
  double maxReferenceTravelMeters = .10;
  double maxBodyPointMotionMeters = .10;
  double maxBodyRadiusMeters = 1.0;
  double maxAngularStepRadians = std::numbers::pi_v<double>;
  std::size_t maxSamples = 10000;
};
inline double wrapAngle(double radians) noexcept {
  return std::remainder(radians,2.0*std::numbers::pi_v<double>);
}
inline Pose2D compose(const Pose2D& origin,const Pose2D& relative) noexcept {
  const double c=std::cos(origin.yaw),s=std::sin(origin.yaw);
  return {origin.x+c*relative.x-s*relative.y,
          origin.y+s*relative.x+c*relative.y,
          wrapAngle(origin.yaw+relative.yaw)};
}
// For a constant body-frame twist the exact SE(2) exponential is evaluated
// with a stable sinc form, also valid for v=0 or omega=0.
inline Pose2D constantTwist(double signedTranslationMeters,
                            double headingChangeRadians,double lambda) {
  if(!std::isfinite(signedTranslationMeters) || !std::isfinite(headingChangeRadians) ||
     !std::isfinite(lambda) || lambda<0.0 || lambda>1.0)
    throw std::invalid_argument("Invalid constant-twist primitive progress");
  const double half=.5*headingChangeRadians*lambda;
  const double sinc=std::abs(half)<1e-5
      ? 1.0-half*half/6.0+half*half*half*half/120.0
      : std::sin(half)/half;
  const double travel=signedTranslationMeters*lambda*sinc;
  const Pose2D pose{travel*std::cos(half),travel*std::sin(half),
                    wrapAngle(headingChangeRadians*lambda)};
  if(!std::isfinite(pose.x)||!std::isfinite(pose.y)||!std::isfinite(pose.yaw))
    throw std::out_of_range("Constant-twist state overflow");
  return pose;
}
struct LocalPrimitive {
  ExtentKind extentKind = ExtentKind::TranslationMeters;
  double extent = 0.0; // >=0, meters for translation, radians for rotation
  double signedTranslationMeters = 0.0; // reverse allowed
  double headingChangeRadians = 0.0;
  std::vector<PrimitiveSample> samples;

  Pose2D evaluate(double lambda) const {
    return constantTwist(signedTranslationMeters,headingChangeRadians,lambda);
  }
  Pose2D evaluateWorld(const Pose2D& anchor,double lambda) const {
    return compose(anchor,evaluate(lambda));
  }
  Pose2D endpoint(const Pose2D& anchor) const {
    return compose(anchor,samples.empty()?evaluate(1.0):samples.back().localPose);
  }
};

// Sampling criterion conservatively bounds travel of each point on a rigid
// footprint by |translation| + radius*|heading change|. In-place rotations
// therefore produce multiple poses despite exactly zero translation.
inline std::size_t sampleIntervals(double translation,double heading,
                                    const SampleLimits& limits) {
  if(!std::isfinite(translation)||!std::isfinite(heading)||
     !std::isfinite(limits.maxReferenceTravelMeters)||
     !std::isfinite(limits.maxBodyPointMotionMeters)||
     !std::isfinite(limits.maxBodyRadiusMeters)||
     !std::isfinite(limits.maxAngularStepRadians)||
     limits.maxReferenceTravelMeters<=0 || limits.maxBodyPointMotionMeters<=0 ||
     limits.maxBodyRadiusMeters<0 || limits.maxAngularStepRadians<=0 ||
     limits.maxSamples<2)
    throw std::invalid_argument("Invalid primitive sampling limits");
  const double traveled=std::abs(translation),turned=std::abs(heading);
  const double intervals=std::max({1.0,traveled/limits.maxReferenceTravelMeters,
      (traveled+limits.maxBodyRadiusMeters*turned)/limits.maxBodyPointMotionMeters,
      turned/limits.maxAngularStepRadians});
  // Validate before floating-point-to-integer conversion or allocation.
  if(!std::isfinite(intervals)||intervals>double(limits.maxSamples-1))
    throw std::length_error("Primitive sample budget exceeded");
  return std::max<std::size_t>(1,static_cast<std::size_t>(std::ceil(intervals)));
}
inline LocalPrimitive generateConstantTwist(double signedTranslationMeters,
                                            double headingChangeRadians,
                                            ExtentKind kind,const SampleLimits& limits) {
  if(kind!=ExtentKind::TranslationMeters && kind!=ExtentKind::RotationRadians)
    throw std::invalid_argument("Invalid primitive extent kind");
  if(kind==ExtentKind::TranslationMeters && !(std::abs(signedTranslationMeters)>0))
    throw std::invalid_argument("Translating primitive must travel");
  if(kind==ExtentKind::RotationRadians &&
     (!(std::abs(headingChangeRadians)>0) || signedTranslationMeters!=0))
    throw std::invalid_argument("Rotation primitive must be stationary and turn");
  const auto intervals=sampleIntervals(signedTranslationMeters,headingChangeRadians,limits);
  LocalPrimitive p;
  p.extentKind=kind;
  p.extent=kind==ExtentKind::TranslationMeters
           ? std::abs(signedTranslationMeters) : std::abs(headingChangeRadians);
  p.signedTranslationMeters=signedTranslationMeters;
  p.headingChangeRadians=headingChangeRadians;
  p.samples.reserve(intervals+1);
  for(std::size_t j=0;j<=intervals;++j) {
    const double lambda=double(j)/double(intervals);
    p.samples.push_back({lambda,constantTwist(signedTranslationMeters,headingChangeRadians,lambda)});
  }
  return p;
}
} // namespace hybrid_astar::motion
