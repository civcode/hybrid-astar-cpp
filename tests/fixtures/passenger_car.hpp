#pragma once

namespace hybrid_astar::fixtures {
// Deliberately NOT public library defaults; representative compact passenger car.
inline constexpr double wheelbaseMeters = 2.7;
inline constexpr double minRearAxleTurningRadiusMeters = 5.0;
inline constexpr double bodyLengthMeters = 4.6;
inline constexpr double bodyWidthMeters = 1.8;
inline constexpr double bodyRearXMeters = -1.0;
inline constexpr double bodyFrontXMeters = 3.6;
inline constexpr double bodyHalfWidthMeters = 0.9;
inline constexpr double mapResolutionMeters = 0.2;
} // namespace hybrid_astar::fixtures
