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

TEST_CASE("Steering-aware lower bound prunes expensive full-path collision checks",
          "[refinement][steering-objective][pruning]") {
  auto map=std::make_shared<MapSnapshot>();
  map->width=200;map->height=200;map->resolutionMeters=.25;
  map->cells.assign(40000,0);
  map->revision=2027;map->frameId="test/map";map->sourceId="pruning-fixture";
  PlanRequest req;
  req.map=map;
  req.start={20,20,0};
  std::vector<MotionSection> originalSections;
  auto pose=req.start;
  const double pi=std::acos(-1.0);
  // 4 quarter-circle arcs: 16 m XY separation, 8*pi m length,
  // 1/m total curvature variation (+.25,-.25,-.25,+.25).
  for(double k:{.25,-.25,-.25,.25}) {
    auto part=makeSection(pose,Gear::Forward,k,2*pi);
    originalSections.push_back(part);
    pose=part.end;
  }
  req.goal=pose;
  REQUIRE(std::hypot(req.goal.x-36,req.goal.y-20)<1e-8);
  PlanResult original;
  original.status=PlanStatus::Success;
  original.sections=originalSections;
  original.mapRevision=map->revision;
  original.mapFrameId=map->frameId;
  original.mapSourceId=map->sourceId;
  original.effectiveOccupancy=req.occupancy;
  original.modelGeometryValid=true;
  original.collisionEvidence=CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled;

  // A shorter, geometrically valid but slightly wigglier connection:
  // arcs of 0.8 rad plus a straight tail. Curvature variation is 1.25/m.
  const RefinementConnector connector=[](const Pose2D& from,const Pose2D& to) {
    std::vector<MotionSection> parts;
    auto p=from;
    for(double k:{.25,-.25,-.25,.25}) {
      auto section=makeSection(p,Gear::Forward,k,3.2);
      parts.push_back(section);
      p=section.end;
    }
    parts.push_back(makeSection(p,Gear::Forward,0,to.x-p.x));
    return parts;
  };
  RefinementOptions options;
  options.maxAttempts=1;options.maxAccepted=1;
  options.maxTotalPoseChecks=20000;
  const auto lengthOnly=refinePath(req,original,connector,options);
  REQUIRE(lengthOnly.path.status==PlanStatus::Success);
  REQUIRE(lengthOnly.accepted==1);
  REQUIRE(lengthOnly.path.geometricLengthMeters<8*pi-1.0);
  REQUIRE(lengthOnly.prunedByCostLowerBound==0);

  options.curvatureVariationWeightMetersSquared=40.0;
  const auto steeringAware=refinePath(req,original,connector,options);
  REQUIRE(steeringAware.path.status==PlanStatus::Success);
  REQUIRE(steeringAware.accepted==0);
  REQUIRE(steeringAware.prunedByCostLowerBound==1);
  // Only original path is sampled: no expensive candidate evaluation needed.
  REQUIRE(steeringAware.exactPoseChecks<lengthOnly.exactPoseChecks);
  REQUIRE(steeringAware.path.sections.size()==originalSections.size());
  REQUIRE(std::abs(steeringAware.path.weightedCostMeters-(8*pi+40.0))<1e-7);
}
