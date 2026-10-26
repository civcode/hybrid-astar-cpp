#include "hybrid_astar/ackermann/primitives.hpp"
#include <cmath>
#include <stdexcept>

namespace hybrid_astar::ackermann {
PrimitiveCatalog buildPrimitiveCatalog(const SearchConfig& search,
        double turningRadius,const motion::SampleLimits& limits) {
  validate(search);
  if(!std::isfinite(turningRadius)||turningRadius<=0)
    throw std::invalid_argument("Invalid Ackermann turning radius");
  PrimitiveCatalog result{};
  const double turn=1.0/turningRadius;
  std::size_t index=0;
  for(auto gear:{Gear::Forward,Gear::Reverse}) {
    for(double curvature:{-turn,0.0,turn}) {
      const double signedTravel=(gear==Gear::Reverse?-1.0:1.0)*search.primitiveLengthMeters;
      motion::LocalPrimitive local;
      try {
        local=motion::generateConstantTwist(signedTravel,curvature*signedTravel,
                         motion::ExtentKind::TranslationMeters,limits);
      } catch(const std::length_error&) {
        // Cannot sample within the configured limit. The search's existing
        // exact motion checker will report ValidationIncomplete as before.
        local.extentKind=motion::ExtentKind::TranslationMeters;
        local.extent=search.primitiveLengthMeters;
        local.signedTranslationMeters=signedTravel;
        local.headingChangeRadians=curvature*signedTravel;
      }
      result[index++]={gear,curvature,std::move(local)};
    }
  }
  return result;
}
} // namespace hybrid_astar::ackermann
