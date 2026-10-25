#include "hybrid_astar/ackermann/search_config.hpp"
#include "fixtures/passenger_car.hpp"
#include "hybrid_astar/collision/options.hpp"
#include "hybrid_astar/steering/analytic_expansion.hpp"
#include <cmath>
#include <numbers>
#include <stdexcept>

using namespace hybrid_astar;
using namespace hybrid_astar::ackermann;

int main() {
  SearchConfig c;
  validate(c);
  if (c.yawBins != 72 || c.rsTriggerXYMeters != 20.0) return 1;
  if (hybrid_astar::fixtures::minRearAxleTurningRadiusMeters != 4.0 ||
      hybrid_astar::fixtures::wheelbaseMeters != 2.7 ||
      hybrid_astar::fixtures::bodyFrontXMeters - hybrid_astar::fixtures::bodyRearXMeters != 4.6) return 17;
  // The max-curvature 0.50m primitive turns by ~7.16 degrees, larger than one 5-degree yaw bin.
  const double yawDelta = c.primitiveLengthMeters / hybrid_astar::fixtures::minRearAxleTurningRadiusMeters;
  if (quantize(Pose2D{0,0,yawDelta}, Gear::Forward, c).yawBin ==
      quantize(Pose2D{0,0,0}, Gear::Forward, c).yawBin) return 18;
  if (quantize(Pose2D{0,0,0}, Gear::Forward, c).yawBin != 0) return 2;
  if (quantize(Pose2D{-0.01,0,0}, Gear::Forward, c).xBin != -1) return 3;
  const Pose2D nearWrap{0,0,2.0 * std::numbers::pi_v<double> - 0.01};
  if (quantize(nearWrap, Gear::Forward, c).yawBin != 0) return 4;
  if (quantize(Pose2D{0,0,std::numbers::pi_v<double> / 36}, Gear::Reverse, c).yawBin != 1) return 5;
  if (quantize(Pose2D{0,0,0}, Gear::Forward, c) ==
      quantize(Pose2D{0,0,0}, Gear::Reverse, c)) return 6;
  if (std::abs(edgeCost(0.5, Gear::Forward, Gear::None, c)-0.5) > 1.e-12) return 7;
  if (std::abs(edgeCost(0.5, Gear::Reverse, Gear::Forward, c)-1.05) > 1.e-12) return 8;
  if (std::abs(edgeCost(0.5, Gear::Forward, Gear::Reverse, c)-1.0) > 1.e-12) return 9;
  try { edgeCost(-1.0, Gear::Forward, Gear::Forward, c); return 10; }
  catch (const std::invalid_argument&) {}
  hybrid_astar::collision::CheckOptions opts;
  if (hybrid_astar::collision::assurance(opts) != hybrid_astar::collision::Assurance::BoundaryOnlyPoseSampled) return 11;
  opts.checkFootprintInterior = true;
  if (hybrid_astar::collision::assurance(opts) != hybrid_astar::collision::Assurance::FullFootprintPoseSampled) return 12;
  hybrid_astar::steering::AnalyticExpansionOptions guards;
  using hybrid_astar::steering::AnalyticGate;
  using hybrid_astar::steering::evaluateAnalyticGuards;
  if (evaluateAnalyticGuards(Pose2D{}, Pose2D{19,0,1}, 12, guards) != AnalyticGate::AcceptableForFurtherValidation) return 13;
  if (evaluateAnalyticGuards(Pose2D{}, Pose2D{21,0,0}, 1, guards) != AnalyticGate::OutsideTrigger) return 14;
  // A large valid weighted cost is not rejected without an explicit cost cap.
  if (evaluateAnalyticGuards(Pose2D{}, Pose2D{1,0,0}, 230.0, guards) != AnalyticGate::AcceptableForFurtherValidation) return 15;
  guards.maxWeightedCostMeters = 15.0;
  if (evaluateAnalyticGuards(Pose2D{}, Pose2D{1,0,0}, 16, guards) != AnalyticGate::OverCostLimit) return 16;
  return 0;
}
