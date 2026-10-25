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
  if (c.yawBins != 72) return 1;
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
  using hybrid_astar::steering::AnalyticCostGate;
  using hybrid_astar::steering::AnalyticExpansionOptions;
  using hybrid_astar::steering::AnalyticExpansionScheduler;
  using hybrid_astar::steering::evaluateAnalyticCost;
  AnalyticExpansionOptions options;
  if (options.attemptRatio != 3.5) return 13;
  // One complete RS candidate is scheduled on the first expanded node,
  // even for a node far from the goal, without any fixed XY cutoff.
  AnalyticExpansionScheduler scheduler(options);
  if (!scheduler.shouldAttempt(100.0, 0.20)) return 14;
  // With 28.5 grid-cell equivalents, floor(28.5/3.5)=8; attempt every 8 pops.
  scheduler.reset();
  if (!scheduler.shouldAttempt(5.7, 0.20)) return 15;
  for (int i = 0; i < 7; ++i) {
    if (scheduler.shouldAttempt(5.7, 0.20)) return 16;
  }
  if (!scheduler.shouldAttempt(5.7, 0.20)) return 19;
  // Close to the goal, the attempt interval shortens to ceil(3.5)=4.
  scheduler.reset();
  if (!scheduler.shouldAttempt(0.4, 0.20)) return 20;
  for (int i = 0; i < 3; ++i) {
    if (scheduler.shouldAttempt(0.4, 0.20)) return 21;
  }
  if (!scheduler.shouldAttempt(0.4, 0.20)) return 22;
  // If the distance becomes smaller mid-cycle, the outstanding interval shortens.
  scheduler.reset();
  if (!scheduler.shouldAttempt(28.0, 0.20)) return 23;
  for (int i = 0; i < 3; ++i) {
    if (scheduler.shouldAttempt(28.0, 0.20)) return 24;
  }
  for (int i = 0; i < 3; ++i) {
    if (scheduler.shouldAttempt(0.2, 0.20)) return 25;
  }
  if (!scheduler.shouldAttempt(0.2, 0.20)) return 26;
  // No path-length gate; expensive valid paths are filtered only by an
  // explicitly enabled weighted-cost cap, separately from collision checks.
  if (evaluateAnalyticCost(230.0, options) != AnalyticCostGate::AcceptableForFurtherValidation) return 27;
  options.maxWeightedCostMeters = 15.0;
  if (evaluateAnalyticCost(16.0, options) != AnalyticCostGate::OverCostLimit) return 28;
  try {
    AnalyticExpansionOptions bad;
    bad.attemptRatio = 0.0;
    AnalyticExpansionScheduler rejected(bad);
    return 29;
  } catch (const std::invalid_argument&) {}
  return 0;
}
