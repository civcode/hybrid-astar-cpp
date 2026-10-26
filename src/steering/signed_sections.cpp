#include "hybrid_astar/steering/signed_sections.hpp"
#include <cmath>
#include <numbers>
#include <stdexcept>
namespace hybrid_astar::steering {
std::vector<ackermann::MotionSection> decodeSignedSections(
    const Pose2D& from,const Pose2D& goal,double radius,
    std::span<const SignedSegment> elements,double xyTolerance,double yawTolerance) {
  const auto finite=[](const Pose2D& p) {
    return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.yaw);
  };
  if (!finite(from)||!finite(goal)||!std::isfinite(radius)||radius<=0||
      !std::isfinite(xyTolerance)||xyTolerance<0||
      !std::isfinite(yawTolerance)||yawTolerance<0)
    throw std::invalid_argument("Invalid signed-section endpoint or geometry");
  std::vector<ackermann::MotionSection> sections;
  sections.reserve(elements.size());
  Pose2D pose=from;
  for (auto part:elements) {
    if (!std::isfinite(part.signedLengthMeters))
      throw std::invalid_argument("Nonfinite signed-section length");
    if (part.signedLengthMeters==0) continue;
    double curvature=0;
    switch(part.turn) {
      case SegmentTurn::Left: curvature=1.0/radius;break;
      case SegmentTurn::Right: curvature=-1.0/radius;break;
      case SegmentTurn::Straight: break;
      case SegmentTurn::None: throw std::invalid_argument("Nonzero no-op RS segment");
      default: throw std::invalid_argument("Unknown RS turn type");
    }
    const auto gear=part.signedLengthMeters>0?ackermann::Gear::Forward:ackermann::Gear::Reverse;
    sections.push_back(ackermann::makeSection(pose,gear,curvature,std::abs(part.signedLengthMeters)));
    pose=sections.back().end;
  }
  if (std::hypot(goal.x-pose.x,goal.y-pose.y)>xyTolerance||
      std::abs(std::remainder(goal.yaw-pose.yaw,2*std::numbers::pi_v<double>))>yawTolerance)
    throw std::runtime_error("Signed-section chain does not reach endpoint");
  return sections;
}
} // namespace hybrid_astar::steering
