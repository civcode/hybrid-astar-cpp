#pragma once
#include "hybrid_astar/ackermann/planner.hpp"
#include <vector>

namespace hybrid_astar::ackermann {
// Deterministic local fixed-gear curvature-bounded C-S-C connectors.
// Four turn pairs per gear (LSL, LSR, RSL, RSR), plus a straight special case.
// These are geometric candidates, not collision checks or an exhaustive
// Dubins/Reeds-Shepp shortest-path solver (CCC and cusps are not generated).
// Endpoints are independently reconstructed and rejected unless within tol.
std::vector<std::vector<MotionSection>> generateCurvatureBoundedShortcuts(
    const Pose2D& start, const Pose2D& goal, double minimumTurningRadiusMeters,
    double endpointToleranceMeters = 1e-6);
} // namespace hybrid_astar::ackermann
