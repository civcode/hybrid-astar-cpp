#pragma once
#include "hybrid_astar/differential_drive/model.hpp"
#include "hybrid_astar/motion/primitive.hpp"
#include <vector>

namespace hybrid_astar::differential_drive {
enum class Maneuver { ForwardStraight, ReverseStraight, ForwardLeft, ForwardRight,
                      ReverseLeft, ReverseRight, RotateLeft, RotateRight };
struct PrimitiveConfig {
  double translationMeters = .50;          // per translating primitive
  double arcTurnRadians = .39269908169872414; // 22.5-degree heading delta
  double spinRadians = .39269908169872414;
  double translationSpeed = .50;           // |v| nominal, m/s
  double rotationSpeed = .80;              // |omega| nominal, rad/s
  motion::SampleLimits sampling{};         // includes full-body sweep bound
};
struct Primitive {
  Maneuver maneuver;
  Control control;
  double durationSeconds;
  motion::LocalPrimitive local; // immutable local [0,1] samples and evaluator
  Pose2D evaluate(const Pose2D& start,double lambda) const {
    return local.evaluateWorld(start,lambda);
  }
};
// Built ONCE for a model/configuration; rigid SE(2) transforms are the only
// per-pose operation. No Runge-Kutta calls occur in the expansion hot path.
std::vector<Primitive> buildCatalog(const Model& model,const PrimitiveConfig& config);
} // namespace hybrid_astar::differential_drive
