#pragma once
#include "hybrid_astar/ackermann/search_config.hpp"
#include "hybrid_astar/motion/primitive.hpp"
#include <array>

namespace hybrid_astar::ackermann {
// Six rigid local templates are generated ONCE for a fixed request catalog.
// Order remains forward/reverse x left/straight/right, matching the
// historical Ackermann search's deterministic successor ordering.
struct CachedPrimitive {
  Gear gear = Gear::None;
  double curvature = 0.0;
  motion::LocalPrimitive local;
};
using PrimitiveCatalog = std::array<CachedPrimitive,6>;

// When the requested sampling budget is insufficient, the template has no
// stored samples: caller must run the old evaluator/checker and report its
// ValidationIncomplete status, NEVER skip collision checks.
PrimitiveCatalog buildPrimitiveCatalog(const SearchConfig& search,
      double minimumTurningRadiusMeters,const motion::SampleLimits& limits);
} // namespace hybrid_astar::ackermann
