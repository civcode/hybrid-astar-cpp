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
