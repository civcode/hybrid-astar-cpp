#include "hybrid_astar/mecanum/primitives.hpp"
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace hybrid_astar::mecanum {
std::vector<Primitive> buildCatalog(const Model& model,const PrimitiveConfig& c) {
  if(!std::isfinite(c.translationMeters)||c.translationMeters<=0||
     !std::isfinite(c.translationSpeed)||c.translationSpeed<=0||
     !std::isfinite(c.arcTurnRadians)||c.arcTurnRadians<=0||
     !std::isfinite(c.spinRadians)||c.spinRadians<=0||
     !std::isfinite(c.rotationSpeed)||c.rotationSpeed<=0)
    throw std::invalid_argument("Invalid mecanum primitive catalog");
  const double t=c.translationMeters/c.translationSpeed;
  const double st=c.spinRadians/c.rotationSpeed;
  if(!std::isfinite(t)||t<=0||!std::isfinite(st)||st<=0)
    throw std::invalid_argument("Invalid mecanum primitive duration");
  const double v=c.translationSpeed;
  const double diag=v/std::sqrt(2.0);
  const double turn=c.arcTurnRadians/t;
  std::vector<Primitive> result;
  result.reserve(14);
  const auto add=[&](Maneuver maneuver,Control control,double seconds) {
    if(!model.valid(control))
      throw std::invalid_argument("Mecanum primitive exceeds wheel limits");
    const double forward=control.forwardMetersPerSecond*seconds;
    const double lateral=control.lateralMetersPerSecond*seconds;
    const double yaw=control.angularRadiansPerSecond*seconds;
    const double distance=std::hypot(forward,lateral);
    auto local=distance>0
        ? motion::generateHolonomicTwist(forward,lateral,yaw,c.sampling)
        : motion::generateConstantTwist(0,yaw,motion::ExtentKind::RotationRadians,c.sampling);
    result.push_back({maneuver,control,seconds,std::move(local)});
  };
  add(Maneuver::Forward,{v,0,0},t);
  add(Maneuver::Reverse,{-v,0,0},t);
  add(Maneuver::Left,{0,v,0},t);
  add(Maneuver::Right,{0,-v,0},t);
  add(Maneuver::ForwardLeft,{diag,diag,0},t);
  add(Maneuver::ForwardRight,{diag,-diag,0},t);
  add(Maneuver::ReverseLeft,{-diag,diag,0},t);
  add(Maneuver::ReverseRight,{-diag,-diag,0},t);
  add(Maneuver::ForwardRotateLeft,{v,0,turn},t);
  add(Maneuver::ForwardRotateRight,{v,0,-turn},t);
  add(Maneuver::StrafeRotateLeft,{0,v,turn},t);
  add(Maneuver::StrafeRotateRight,{0,v,-turn},t);
  add(Maneuver::RotateLeft,{0,0,c.rotationSpeed},st);
  add(Maneuver::RotateRight,{0,0,-c.rotationSpeed},st);
  return result;
}
} // namespace hybrid_astar::mecanum
