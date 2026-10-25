#pragma once

namespace hybrid_astar::collision {
struct CheckOptions {
  bool checkFootprintInterior = false; // boundary-only; may miss enclosed obstacles
  bool adaptiveSubdivision = false;   // primitive samples only by default
};
enum class Assurance { BoundaryOnlyPoseSampled, FullFootprintPoseSampled };
constexpr Assurance assurance(const CheckOptions& options) noexcept {
  return options.checkFootprintInterior ?
    Assurance::FullFootprintPoseSampled : Assurance::BoundaryOnlyPoseSampled;
}
} // namespace hybrid_astar::collision
