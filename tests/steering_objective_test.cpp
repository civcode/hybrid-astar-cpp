#include <catch2/catch_test_macros.hpp>
#include "hybrid_astar/ackermann/refinement.hpp"
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

using namespace hybrid_astar;
using namespace hybrid_astar::ackermann;

TEST_CASE("Ackermann curvature variation is opt-in, nonnegative and revalidated",
          "[refinement][steering-objective]") {
  auto map=std::make_shared<MapSnapshot>();
  map->width=200;map->height=200;map->resolutionMeters=.25;
  map->cells.assign(map->width*map->height,0);
  map->revision=2026;map->frameId="test/map";map->sourceId="steering-fixture";
  PlanRequest req;
  req.map=map;
  req.start={20,20,0};
  const auto a=makeSection(req.start,Gear::Forward,.25,1);
  const auto b=makeSection(a.end,Gear::Forward,-.25,1);
  const auto c=makeSection(b.end,Gear::Forward,.25,1);
  req.goal=c.end;
  PlanResult raw;
  raw.status=PlanStatus::Success;
  raw.sections={a,b,c};
  raw.mapRevision=map->revision;
  raw.mapFrameId=map->frameId;
  raw.mapSourceId=map->sourceId;
  raw.effectiveOccupancy=req.occupancy;
  raw.modelGeometryValid=true;
  raw.collisionEvidence=CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled;

  RefinementOptions opts;
  opts.maxAttempts=1;
  opts.maxAccepted=1;
  opts.minimumCostReductionMeters=1000000; // revalidate; no candidate accepted
  auto legacy=refinePath(req,raw,{},opts);
  REQUIRE(legacy.path.status==PlanStatus::Success);
  REQUIRE(legacy.accepted==0);
  REQUIRE(std::abs(legacy.path.weightedCostMeters-3.0)<1e-8);

  opts.curvatureVariationWeightMetersSquared=1.5;
  auto penalized=refinePath(req,raw,{},opts);
  REQUIRE(penalized.path.status==PlanStatus::Success);
  REQUIRE(penalized.accepted==0);
  REQUIRE(penalized.path.collisionEvidence==
          CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
  // Two junctions, each |+0.25 - (-0.25)| = 0.5 inverse metres.
  // J = 3 m + 1.5 m^2 * (0.5 + 0.5) / m = 4.5 m.
  REQUIRE(std::abs(penalized.path.weightedCostMeters-4.5)<1e-8);
  REQUIRE(std::abs(penalized.path.geometricLengthMeters-3.0)<1e-8);
  REQUIRE(penalized.path.clearanceCostMeters==0);

  opts.curvatureVariationWeightMetersSquared=-0.1;
  REQUIRE(refinePath(req,raw,{},opts).status==RefinementStatus::InvalidInput);
  opts.curvatureVariationWeightMetersSquared=
      std::numeric_limits<double>::quiet_NaN();
  REQUIRE(refinePath(req,raw,{},opts).status==RefinementStatus::InvalidInput);
}
