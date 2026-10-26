#pragma once
#include "hybrid_astar/ackermann/planner.hpp"
#include <span>
#include <vector>
namespace hybrid_astar::steering {
// Native representation of an OMPL-style signed Reeds-Shepp segment.
// Positive distance is forward; negative is reverse. No OMPL types leak.
enum class SegmentTurn { None, Left, Straight, Right };
struct SignedSegment { SegmentTurn turn; double signedLengthMeters; };
// Enforces finite data, signed gears, exact analytic chaining, and final pose.
// Zero-length elements are discarded; nonzero None elements are invalid.
// Throws invalid_argument for bad inputs and runtime_error for mismatch.
std::vector<ackermann::MotionSection> decodeSignedSections(
    const Pose2D& from,const Pose2D& goal,double minimumTurningRadiusMeters,
    std::span<const SignedSegment> signedElements,
    double endpointToleranceMeters=1e-5,double headingToleranceRadians=1e-5);
} // namespace hybrid_astar::steering
