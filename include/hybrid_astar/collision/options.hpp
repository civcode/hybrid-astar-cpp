#pragma once

namespace hybrid_astar::collision {
struct CheckOptions {
  bool checkFootprintInterior = true;  // full body is required for ordinary planning success
  bool adaptiveSubdivision = true;    // density control; does not certify the sweep
};
enum class Assurance { BoundaryOnlyPoseSampled, FullFootprintPoseSampled };
constexpr Assurance assurance(const CheckOptions& options) noexcept {
  return options.checkFootprintInterior ?
    Assurance::FullFootprintPoseSampled : Assurance::BoundaryOnlyPoseSampled;
}
} // namespace hybrid_astar::collision
