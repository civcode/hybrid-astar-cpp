#include "hybrid_astar/differential_drive/primitives.hpp"
#include <cmath>
#include <stdexcept>

namespace hybrid_astar::differential_drive {
std::vector<Primitive> buildCatalog(const Model& model,const PrimitiveConfig& c) {
  if(!std::isfinite(c.translationMeters)||c.translationMeters<=0 ||
     !std::isfinite(c.arcTurnRadians)||c.arcTurnRadians<=0 ||
     !std::isfinite(c.spinRadians)||c.spinRadians<=0 ||
     !std::isfinite(c.translationSpeed)||c.translationSpeed<=0 ||
     !std::isfinite(c.rotationSpeed)||c.rotationSpeed<=0)
    throw std::invalid_argument("Invalid differential-drive primitive catalog");
  std::vector<Primitive> primitives;
  primitives.reserve(8);
  const auto add=[&](Maneuver maneuver,Control control,double duration,
                     motion::ExtentKind extentKind) {
    if(!model.valid(control)||!std::isfinite(duration)||duration<=0)
      throw std::invalid_argument("Differential-drive primitive exceeds wheel limits");
    auto local=motion::generateConstantTwist(control.linearMetersPerSecond*duration,
                         control.angularRadiansPerSecond*duration,extentKind,c.sampling);
    primitives.push_back({maneuver,control,duration,std::move(local)});
  };
  const double travelTime=c.translationMeters/c.translationSpeed;
  const double spinTime=c.spinRadians/c.rotationSpeed;
  const double yawRate=c.arcTurnRadians/travelTime;
  add(Maneuver::ForwardStraight,{c.translationSpeed,0},travelTime,motion::ExtentKind::TranslationMeters);
  add(Maneuver::ReverseStraight,{-c.translationSpeed,0},travelTime,motion::ExtentKind::TranslationMeters);
  add(Maneuver::ForwardLeft,{c.translationSpeed,yawRate},travelTime,motion::ExtentKind::TranslationMeters);
  add(Maneuver::ForwardRight,{c.translationSpeed,-yawRate},travelTime,motion::ExtentKind::TranslationMeters);
  add(Maneuver::ReverseLeft,{-c.translationSpeed,yawRate},travelTime,motion::ExtentKind::TranslationMeters);
  add(Maneuver::ReverseRight,{-c.translationSpeed,-yawRate},travelTime,motion::ExtentKind::TranslationMeters);
  add(Maneuver::RotateLeft,{0,c.rotationSpeed},spinTime,motion::ExtentKind::RotationRadians);
  add(Maneuver::RotateRight,{0,-c.rotationSpeed},spinTime,motion::ExtentKind::RotationRadians);
  return primitives;
}
} // namespace hybrid_astar::differential_drive
