#include <catch2/catch_test_macros.hpp>
#include "apps/demo/demo.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace hybrid_astar;
using namespace hybrid_astar::demo;
using namespace hybrid_astar::ackermann;
static void require(bool v,const char* msg) { INFO(msg); REQUIRE(v); }
static void runLegacyAssertions() {
  auto open=run({});
  require(open.original.status==PlanStatus::Success,"open plan success");
  require(!open.original.heuristicCertifiedAdmissible,"default demo uses obstacle grid");
  require(!Options{}.rsDistanceHeuristic &&
          toJson(open).find("\"rsDistanceHeuristic\":false")!=std::string::npos,
          "RS distance must be off by default, including exported configuration");
  Options certifiedOptions;
  certifiedOptions.performanceHeuristic=false;
  const auto certified=run(certifiedOptions);
  require(certified.original.status==PlanStatus::Success &&
          certified.original.heuristicCertifiedAdmissible,
          "demo explicitly selects certified Euclidean mode");
  require(open.original.collisionEvidence==CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled,
          "strict collision evidence");
  const auto json=toJson(open);
  require(json.find("\"mapRuns\":[]")!=std::string::npos,"empty occupancy serialized");
  require(json.find("\"status\":\"Success\"")!=std::string::npos,"JSON contains success");
  Options inflated;inflated.footprintInflationEnabled=true;
  inflated.footprintInflationMeters=.05;
  const auto marginResult=run(inflated);
  require(marginResult.original.status==PlanStatus::Success,
          "5cm geometric inflation preserves open scenario");
  require(std::abs(marginResult.request.options.footprintInflationMeters-.05)<1e-12,
          "demo propagates enabled metric inflation");
  require(toJson(marginResult).find("\"footprintInflationMeters\":0.05")!=std::string::npos,
          "demo serializes effective inflation margin");
  inflated.footprintInflationEnabled=false;
  require(run(inflated).request.options.footprintInflationMeters==0,
          "disabling inflation restores original footprint");
  // The optional distance guidance must never silently activate without the
  // analytic connector; incompatible combinations are rejected by the API.
  bool rejectedRs=false;
  try {Options bad;bad.rsDistanceHeuristic=true;run(bad);}
  catch(const std::invalid_argument&) {rejectedRs=true;}
  require(rejectedRs,"RS distance requires explicitly enabled connector");
  if(capabilitiesJson().find("\"ompl\":true")!=std::string::npos) {
    Options rs;rs.ompl=true;rs.scenario="reverse";
    const auto connectorOnly=run(rs);
    require(connectorOnly.original.status==PlanStatus::Success &&
            !connectorOnly.rsDistanceHeuristicEnabled,
            "RS connector defaults to grid guidance without RS distance");
    require(toJson(connectorOnly).find("\"rsDistanceHeuristic\":false")!=std::string::npos,
            "connector-only plan is exported as such");
    rs.rsDistanceHeuristic=true;
    const auto withDistance=run(rs);
    require(withDistance.original.status==PlanStatus::Success &&
            withDistance.rsDistanceHeuristicEnabled,
            "RS distance can be explicitly enabled");
    require(toJson(withDistance).find("\"rsDistanceHeuristic\":true")!=std::string::npos,
            "RS distance opt-in is exported");
    rs.rsDistanceHeuristic=false;
    rs.refine=true;rs.curvedRefinement=true;
    const auto combined=run(rs);
    require(combined.original.status==PlanStatus::Success &&
            combined.refinement.has_value() &&
            combined.refinement->path.status==PlanStatus::Success,
            "RS and native CSC can be enabled together in Planning Lab");
  }
  Options reverse;reverse.scenario="reverse";
  auto r=run(reverse);
  require(r.original.status==PlanStatus::Success,"reverse route");
  require(r.original.reverseLengthMeters>3.5,"reverse gear preserved");
  Options refine;refine.scenario="open";refine.refine=true;refine.curvedRefinement=true;
  auto refined=run(refine);
  require(refined.refinement.has_value(),"refinement invoked");
  require(refined.refinement->path.status==PlanStatus::Success,
          "refined or original route revalidated");
  require(refined.refinement->path.weightedCostMeters<=refined.original.weightedCostMeters+1e-8,
          "refinement never raises weighted cost");
  // Ackermann now defaults to 160 attempts, 8 acceptances and 200k
  // strict checks, but must NOT enable the steering objective implicitly.
  require(RefinementOptions{}.maxAttempts==160 &&
          RefinementOptions{}.maxAccepted==8 &&
          RefinementOptions{}.maxTotalPoseChecks==200000,
          "shared refinement defaults reflect the expanded benchmark profile");
  require(RefinementOptions{}.curvatureVariationWeightMetersSquared==0.0 &&
          !Options{}.steeringSmoothness,
          "steering penalty is strictly opt-in");
  require(refined.refinementSettings.maxAttempts==160 &&
          refined.refinementSettings.maxAccepted==8 &&
          refined.refinementSettings.maxTotalPoseChecks==200000 &&
          refined.appliedSteeringWeightMetersSquared==0,
          "demo applies 160/8/200k budget and zero steering penalty by default");
  require(toJson(refined).find("\"steeringSmoothness\":false")!=std::string::npos,
          "effective disabled steering objective is visible in exported JSON");
  Options withSteering=refine;
  withSteering.steeringSmoothness=true;
  withSteering.steeringWeightMetersSquared=2.5;
  const auto steered=run(withSteering);
  require(steered.refinement.has_value() &&
          steered.refinement->path.status==PlanStatus::Success,
          "opt-in steering objective keeps full-path validation");
  require(steered.refinementSettings.curvatureVariationWeightMetersSquared==2.5 &&
          steered.appliedSteeringWeightMetersSquared==2.5,
          "steering weight reaches the actual refiner");
  require(toJson(steered).find("\"steeringSmoothness\":true")!=std::string::npos &&
          toJson(steered).find("\"steeringWeightMetersSquared\":2.5")!=std::string::npos,
          "effective steering options are exported for comparison");
  for(const auto model:{std::string("differential"),std::string("mecanum")}) {
    bool invalid=false;
    try {auto bad=withSteering;bad.model=model;run(bad);}
    catch(const std::invalid_argument&) {invalid=true;}
    require(invalid,"steering-aware refinement is Ackermann-only");
  }
  bool disabledRefinementRejected=false;
  try {auto bad=withSteering;bad.refine=false;run(bad);}
  catch(const std::invalid_argument&) {disabledRefinementRejected=true;}
  require(disabledRefinementRejected,
          "steering objective requires enabled post-path refinement");
  for(const double badWeight:{0.0,-1.0,101.0}) {
    bool invalid=false;
    try {auto bad=withSteering;bad.steeringWeightMetersSquared=badWeight;run(bad);}
    catch(const std::invalid_argument&) {invalid=true;}
    require(invalid,"enabled steering weights have a positive bounded range");
  }
  Options differential;
  differential.model="differential";
  differential.scenario="open";
  differential.refine=true;
  differential.curvedRefinement=true;
  const auto smooth=run(differential);
  require(smooth.original.status==PlanStatus::Success,"differential plan succeeds");
  require(smooth.refinement.has_value(),"differential refinement is available");
  require(smooth.refinement->path.status==PlanStatus::Success,
          "differential refinement returns reverified original or improvement");
  require(smooth.refinement->path.collisionEvidence==
          CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled,
          "differential refined output is strictly validated");
  require(smooth.refinement->path.weightedCostMeters<=smooth.original.weightedCostMeters+1e-8,
          "differential refinement cannot increase path cost");
  require(toJson(smooth).find("\"refinement\":{")!=std::string::npos,
          "differential refined path serialized");
  // Reported warehouse case: a redundant internal spin near the first
  // partition should be removable when a shorter, strictly checked moving
  // twist exists. The cleanup cannot worsen cost or change search results.
  Options warehouse;
  warehouse.model="mecanum";warehouse.scenario="warehouse";
  warehouse.start=Pose2D{7.05,32.31,0.};
  warehouse.goal=Pose2D{30.02,33.30,0.};
  warehouse.footprintInflationEnabled=true;
  warehouse.footprintInflationMeters=.5;
  const auto optimizedWarehouse=run(warehouse);
  require(optimizedWarehouse.original.status==PlanStatus::Success,
          "mecanum warehouse with inflated footprint reaches goal");
  auto unsmoothedRequest=optimizedWarehouse.request;
  unsmoothedRequest.options.mecanumMergeIntermediateSpins=false;
  const auto rawWarehouse=plan(unsmoothedRequest);
  require(rawWarehouse.status==PlanStatus::Success,
          "same mecanum warehouse without spin cleanup also succeeds");
  require(rawWarehouse.diagnostics.expanded==
          optimizedWarehouse.original.diagnostics.expanded,
          "post-search mecanum cleanup leaves node expansion unchanged");
  require(optimizedWarehouse.original.weightedCostMeters<=
          rawWarehouse.weightedCostMeters+1e-7,
          "mecanum cleanup does not worsen search objective");
  if(optimizedWarehouse.original.diagnostics.mecanumSpinShortcutsAccepted>0) {
    std::size_t oldSpins=0,newSpins=0;
    for(const auto& section:rawWarehouse.sections)
      oldSpins+=section.kind==MotionKind::RotateInPlace;
    for(const auto& section:optimizedWarehouse.original.sections)
      newSpins+=section.kind==MotionKind::RotateInPlace;
    require(newSpins<oldSpins,
            "accepted moving-twist shortcut removes stationary spins");
  }
  Options mecanumSmoothed;
  mecanumSmoothed.model="mecanum";
  mecanumSmoothed.scenario="open";
  mecanumSmoothed.refine=true;
  const auto holonomicSmooth=run(mecanumSmoothed);
  require(holonomicSmooth.original.status==PlanStatus::Success,
          "mecanum search succeeds with refinement requested");
  require(holonomicSmooth.refinement.has_value(),
          "mecanum refinement is available in Planning Lab");
  require(holonomicSmooth.refinement->path.status==PlanStatus::Success,
          "mecanum refinement retains a strictly checked route");
  require(holonomicSmooth.refinement->path.weightedCostMeters<=
          holonomicSmooth.original.weightedCostMeters+1e-8,
          "mecanum refinement never increases weighted cost");
  Options unreachable;unreachable.scenario="blocked";
  unreachable.start=Pose2D{15,20,0};unreachable.goal=Pose2D{35,20,0};
  auto impossible=run(unreachable);
  require(impossible.original.status!=PlanStatus::Success,
          "blocked wall does not produce invalid success");
  require(toJson(impossible).find("\"validated\":false")!=std::string::npos,
          "failed plan serialized without claimed validation");
  bool rejected=false;
  try{Options bad;bad.scenario="invalid";run(bad);}catch(const std::invalid_argument&){rejected=true;}
  require(rejected,"invalid scenario rejected");
  rejected=false;
  try{Options bad;bad.start=Pose2D{1e10,20,0};run(bad);}catch(const std::invalid_argument&){rejected=true;}
  require(rejected,"invalid pose rejected");
  std::cout<<"DEMO PASS\n";
}

TEST_CASE("demo_test contracts", "[demo_test]") {
  runLegacyAssertions();
}
