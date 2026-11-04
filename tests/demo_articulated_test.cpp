#include <catch2/catch_test_macros.hpp>
#include "apps/demo/demo.hpp"
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>

namespace demo=hybrid_astar::demo;
namespace art=hybrid_astar::articulated_steering;

TEST_CASE("Planning Lab articulated adapter searches and serializes complete two-carriage path",
          "[articulated][demo][search]") {
  demo::Options o;
  o.model="articulated";
  o.scenario="open";
  o.start=hybrid_astar::Pose2D{10,15,0};
  o.goal=hybrid_astar::Pose2D{11,15,0};
  o.maxExpansions=1000;
  const auto result=demo::runArticulated(o);
  REQUIRE(result.result.status==art::Status::Success);
  REQUIRE(result.result.independentlySampledBodyValidated);
  REQUIRE_FALSE(result.result.motions.empty());
  REQUIRE(result.result.diagnostics.finalBodyChecks>0);
  REQUIRE(result.result.diagnostics.precomputedAnalyticDriveTemplates>0);
  REQUIRE(result.result.diagnostics.precomputedStationaryRk4Templates>0);
  REQUIRE(result.result.diagnostics.appliedPrecomputedTemplates>0);
  // No expanded driving section may change the articulation angle.
  for(const auto& motion:result.result.motions)
    if(motion.control.frontSpeedMetersPerSecond!=0.0)
      REQUIRE(motion.control.gammaRateRadiansPerSecond==0.0);
  REQUIRE(result.parameters.LfMeters==1.0);
  REQUIRE(result.parameters.LrMeters==1.0);
  REQUIRE(result.bodies.front.rearX==-0.5);
  REQUIRE(result.bodies.front.frontX==0.5);
  REQUIRE(result.bodies.front.halfWidth==0.3);
  REQUIRE(result.bodies.rear.rearX==-0.5);
  REQUIRE(result.bodies.rear.frontX==0.5);
  REQUIRE(result.bodies.rear.halfWidth==0.3);
  const auto json=demo::articulatedToJson(result);
  REQUIRE(json.find("\"model\":\"articulated\"")!=std::string::npos);
  REQUIRE(json.find("\"articulatedGeometry\"")!=std::string::npos);
  REQUIRE(json.find("\"articulatedPoints\"")!=std::string::npos);
  REQUIRE(json.find("\"precomputedAnalyticDriveTemplates\"")!=std::string::npos);
  REQUIRE(json.find("\"precomputedStationaryRk4Templates\"")!=std::string::npos);
  REQUIRE(json.find("\"appliedPrecomputedTemplates\"")!=std::string::npos);
  REQUIRE(json.find("\"collisionEvidence\":\"TwoCarriageIndependentAdaptiveSampled\"")!=std::string::npos);
  REQUIRE(json.find("\"actuatorFeasible\":false")!=std::string::npos);
  REQUIRE(json.find("\"validated\":true")!=std::string::npos);
  REQUIRE(json.find("\"refinement\":null")!=std::string::npos);
  REQUIRE(json.find("\"mapRuns\":[]")!=std::string::npos);
  REQUIRE(json.find("\"maxExpansions\":1000")!=std::string::npos);
}
TEST_CASE("Articulated Planning Lab honors one million expansions without a hidden cap",
          "[articulated][demo][limits]") {
  demo::Options opt;
  opt.model="articulated";
  opt.scenario="open";
  opt.start=hybrid_astar::Pose2D{10,15,0};
  opt.goal=opt.start; // zero-length valid goal keeps test fast
  opt.maxExpansions=1'000'000;
  const auto result=demo::runArticulated(opt);
  REQUIRE(result.result.status==art::Status::Success);
  REQUIRE(result.settings.maxExpansions==1'000'000);
  REQUIRE(result.settings.maxGenerated==4'000'000);
  const auto json=demo::articulatedToJson(result);
  REQUIRE(json.find("\"maxExpansions\":1000000")!=std::string::npos);
  REQUIRE(json.find("\"maxGeneratedNodes\":4000000")!=std::string::npos);

  opt.maxExpansions=80'000; // previously silently clamped to 50k
  const auto selected=demo::runArticulated(opt);
  REQUIRE(selected.result.status==art::Status::Success);
  REQUIRE(selected.settings.maxExpansions==80'000);
  REQUIRE(selected.settings.maxGenerated==320'000);

  opt.maxExpansions.reset(); // no explicit budget now uses the UI's 1M default
  const auto defaultBudget=demo::runArticulated(opt);
  REQUIRE(defaultBudget.result.status==art::Status::Success);
  REQUIRE(defaultBudget.settings.maxExpansions==1'000'000);
  REQUIRE(defaultBudget.settings.maxGenerated==4'000'000);
}

TEST_CASE("Planning Lab articulated gamma goals can require joint-only steering",
          "[articulated][demo][joint]") {
  demo::Options o;
  o.model="articulated";
  o.scenario="open";
  o.start=hybrid_astar::Pose2D{10,15,0};
  o.startGammaDegrees=0;
  const art::Parameters p{1.0,1.0,-1.2,1.2,1.0,.8,.1};
  art::Model model(p);
  const auto end=model.toRear(model.propagate(
    model.fromRear({10,15,0,0}),{0,.4},.5,.002));
  o.goal=hybrid_astar::Pose2D{end.xr,end.yr,end.thetaRear};
  o.goalGammaDegrees=end.gamma*180.0/std::numbers::pi_v<double>;
  o.maxExpansions=1000;
  const auto result=demo::runArticulated(o);
  REQUIRE(result.result.status==art::Status::Success);
  REQUIRE(result.result.independentlySampledBodyValidated);
  REQUIRE(std::abs(result.goal.gamma-.2)<1e-9);
  REQUIRE(std::abs(result.result.reached.gamma-.2)<.1);
  bool joint=false;
  for(const auto& motion:result.result.motions)
    if(motion.control.frontSpeedMetersPerSecond==0.0&&
       motion.control.gammaRateRadiansPerSecond!=0.0)
      joint=true;
  REQUIRE(joint);
  REQUIRE(demo::articulatedToJson(result).find(
    "\"kind\":\"articulation\"")!=std::string::npos);
}
TEST_CASE("Planning Lab articulated adapter rejects nonsensical geometry or unsupported features",
          "[articulated][demo][invalid]") {
  demo::Options o;
  o.model="articulated";
  o.LfMeters=0;
  REQUIRE_THROWS_AS(demo::runArticulated(o),std::invalid_argument);
  o.LfMeters=1.0;
  o.articulatedFront.halfWidth=0;
  REQUIRE_THROWS_AS(demo::runArticulated(o),std::invalid_argument);
  o.articulatedFront.halfWidth=.30;
  o.goalGammaDegrees=120;
  REQUIRE_THROWS_AS(demo::runArticulated(o),std::invalid_argument);
  o.goalGammaDegrees=0;
  o.refine=true;
  REQUIRE_THROWS_AS(demo::runArticulated(o),std::invalid_argument);
  o.refine=false;
  o.ompl=true;
  REQUIRE_THROWS_AS(demo::runArticulated(o),std::invalid_argument);
  o.ompl=false;
  o.model="mecanum";
  REQUIRE_THROWS_AS(demo::runArticulated(o),std::invalid_argument);
}
TEST_CASE("Planning Lab articulated adapter uses immutable edited map and inflated two-body geometry",
          "[articulated][demo][map]") {
  demo::Options o;
  o.model="articulated";
  o.scenario="open";
  o.start=hybrid_astar::Pose2D{10,15,0};
  o.goal=hybrid_astar::Pose2D{11,15,0};
  o.footprintInflationEnabled=true;
  o.footprintInflationMeters=.05;
  auto result=demo::runArticulated(o);
  REQUIRE(result.bodies.inflationMeters==.05);
  REQUIRE(result.result.status==art::Status::Success);
  REQUIRE(demo::articulatedToJson(result).find(
    "\"footprintInflationMeters\":0.05")!=std::string::npos);
  // Built-in open map starts empty; edit only the cell containing the
  // front carriage reference at x=12, y=15 and verify a start rejection.
  o.edits.push_back({std::size_t(60)*200+48,100});
  o.editGeneration=1;
  result=demo::runArticulated(o);
  REQUIRE(result.result.status==art::Status::StartInCollision);
  REQUIRE(result.map->revision>1);
}
