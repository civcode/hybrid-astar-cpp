#pragma once
#include "hybrid_astar/mecanum/model.hpp"
#include "hybrid_astar/motion/primitive.hpp"
#include <vector>

namespace hybrid_astar::mecanum {
enum class Maneuver {
  Forward,Reverse,Left,Right,ForwardLeft,ForwardRight,ReverseLeft,ReverseRight,
  ForwardRotateLeft,ForwardRotateRight,StrafeRotateLeft,StrafeRotateRight,
  RotateLeft,RotateRight
};
struct PrimitiveConfig {
  double translationMeters = 0.50;
  double translationSpeed = 0.50;
  double arcTurnRadians = 0.39269908169872414;
  double spinRadians = 0.39269908169872414;
  double rotationSpeed = 0.80;
  motion::SampleLimits sampling{};
};
struct Primitive {
  Maneuver maneuver;
  Control control;
  double durationSeconds;
  motion::LocalPrimitive local;
  Pose2D evaluate(const Pose2D& start,double lambda) const {
    return local.evaluateWorld(start,lambda);
  }
};
std::vector<Primitive> buildCatalog(const Model& model,const PrimitiveConfig& config);
} // namespace hybrid_astar::mecanum
