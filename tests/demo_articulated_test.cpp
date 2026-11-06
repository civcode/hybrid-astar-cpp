#include <catch2/catch_test_macros.hpp>
#include "apps/demo/demo.hpp"
#include <cmath>
#include <numbers>
#include <limits>
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
  REQUIRE(result.result.diagnostics.precomputedCoupledRk4Templates>0);
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
  REQUIRE(json.find("\"precomputedCoupledRk4Templates\"")!=std::string::npos);
  REQUIRE(json.find("\"searchSweptAabbAccepted\"")!=std::string::npos);
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

TEST_CASE("Planning Lab articulated optimizer settings preserve defaults and round-trip in JSON",
          "[articulated][demo][heuristic][options]") {
  demo::Options options;
  options.model="articulated";
  options.scenario="open";
  options.start=hybrid_astar::Pose2D{10,15,0};
  options.goal=options.start;
  options.maxExpansions=1000;
  auto defaultRun=demo::runArticulated(options);
  REQUIRE(defaultRun.result.status==art::Status::Success);
  REQUIRE(defaultRun.settings.heuristicMode==art::HeuristicMode::ObstacleGrid);
  REQUIRE(defaultRun.settings.heuristicWeight==1.0);
  REQUIRE(defaultRun.settings.coarseGridResolutionMeters==0.15);
  REQUIRE(defaultRun.settings.goalHeadingGuidanceWeight==0.8);
  REQUIRE(defaultRun.settings.enableCoupledMotion);
  REQUIRE(defaultRun.settings.useSweptAabbPrefilter);
  REQUIRE_FALSE(defaultRun.settings.enableGoalConnector);
  REQUIRE_FALSE(defaultRun.settings.enableAnytimeGoalRefinement);
  REQUIRE(defaultRun.settings.anytimeExtraExpansions==2000);
  REQUIRE(demo::articulatedToJson(defaultRun).find(
    "\"enableGoalConnector\":false")!=std::string::npos);
  const auto defaultJson=demo::articulatedToJson(defaultRun);
  REQUIRE(defaultJson.find(
    "\"articulatedSearch\":{\"heuristicMode\":\"grid\",\"heuristicWeight\":1,")!=
    std::string::npos);

  for(const auto mode:{art::HeuristicMode::AnalyticOnly,
                       art::HeuristicMode::Combined,
                       art::HeuristicMode::CombinedHeadingAware,
                       art::HeuristicMode::CombinedCoarseGrid}) {
    options.articulatedHeuristicMode=mode;
    options.articulatedHeuristicWeight=1.2;
    options.articulatedCoarseGridResolutionMeters=0.18;
    options.articulatedHeadingGuidanceWeight=1.4;
    options.articulatedEnableCoupledMotion=false;
    options.articulatedUseSweptAabbPrefilter=false;
    options.articulatedEnableGoalConnector=true;
    options.articulatedEnableAnytimeGoalRefinement=true;
    options.articulatedAnytimeExtraExpansions=16;
    const auto changed=demo::runArticulated(options);
    REQUIRE(changed.result.status==art::Status::Success);
    REQUIRE(changed.result.independentlySampledBodyValidated);
    REQUIRE(changed.settings.heuristicMode==mode);
    REQUIRE(changed.settings.heuristicWeight==1.2);
    REQUIRE(changed.settings.coarseGridResolutionMeters==0.18);
    REQUIRE(changed.settings.goalHeadingGuidanceWeight==1.4);
    REQUIRE_FALSE(changed.settings.enableCoupledMotion);
    REQUIRE_FALSE(changed.settings.useSweptAabbPrefilter);
    REQUIRE(changed.settings.enableGoalConnector);
    REQUIRE(changed.settings.enableAnytimeGoalRefinement);
    REQUIRE(changed.settings.anytimeExtraExpansions==16);
    REQUIRE(changed.result.diagnostics.anytimeFirstSolutionExpansion>=1);
    const auto json=demo::articulatedToJson(changed);
    const std::string expectedMode=mode==art::HeuristicMode::AnalyticOnly
        ?"analytic":mode==art::HeuristicMode::Combined?"combined":
        mode==art::HeuristicMode::CombinedHeadingAware?"heading":"coarse";
    REQUIRE(json.find("\"heuristicMode\":\""+expectedMode+"\"")!=
      std::string::npos);
    REQUIRE(json.find("\"headingGuidanceWeight\":1.4")!=std::string::npos);
    REQUIRE(json.find("\"coarseGridResolutionMeters\":0.18")!=std::string::npos);
    REQUIRE(json.find("\"coarseGridCells\"")!=std::string::npos);
    if(mode==art::HeuristicMode::CombinedCoarseGrid) {
      REQUIRE(changed.result.diagnostics.coarseGridSettlements>0);
      REQUIRE(changed.result.diagnostics.gridSettlements==0);
    }
    REQUIRE(json.find("\"heuristicWeight\":1.2")!=std::string::npos);
    REQUIRE(json.find("\"enableCoupledMotion\":false")!=std::string::npos);
    REQUIRE(json.find("\"useSweptAabbPrefilter\":false")!=std::string::npos);
    REQUIRE(json.find("\"enableGoalConnector\":true")!=std::string::npos);
    REQUIRE(json.find("\"enableAnytimeGoalRefinement\":true")!=std::string::npos);
    REQUIRE(json.find("\"anytimeExtraExpansions\":16")!=std::string::npos);
    REQUIRE(json.find("\"anytimeFirstCostMeters\"")!=std::string::npos);
    REQUIRE(json.find("\"anytimeBestCostMeters\"")!=std::string::npos);
    REQUIRE(json.find("\"goalConnectorAttempts\"")!=std::string::npos);
    REQUIRE(json.find("\"goalConnectorOneArcProposals\"")!=std::string::npos);
    REQUIRE(json.find("\"goalConnectorTwoArcProposals\"")!=std::string::npos);
    REQUIRE(json.find("\"goalConnectorThreeArcProposals\"")!=std::string::npos);
    REQUIRE(json.find("\"goalConnectorTailDrivingMeters\"")!=std::string::npos);
    REQUIRE(json.find("\"goalConnectorTailStationarySeconds\"")!=std::string::npos);
    REQUIRE(json.find("\"goalConnectorTailArticulationReversals\"")!=std::string::npos);
    REQUIRE(json.find("\"goalConnectorTailQualityScoreMeters\"")!=std::string::npos);
    REQUIRE(json.find("\"anytimePacedDeferrals\"")!=std::string::npos);
    REQUIRE(json.find("\"anytimeFirstExtraShotExpansion\"")!=std::string::npos);
    REQUIRE(json.find("\"anytimeLastExtraShotExpansion\"")!=std::string::npos);
  }
}

TEST_CASE("Planning Lab rejects invalid articulated heuristic weights and enum values",
          "[articulated][demo][heuristic][invalid]") {
  demo::Options options;
  options.model="articulated";
  options.scenario="open";
  options.start=hybrid_astar::Pose2D{10,15,0};
  options.goal=options.start;
  options.articulatedHeuristicWeight=0.0;
  REQUIRE_THROWS_AS(demo::runArticulated(options),std::invalid_argument);
  options.articulatedHeuristicWeight=-1.0;
  REQUIRE_THROWS_AS(demo::runArticulated(options),std::invalid_argument);
  options.articulatedHeuristicWeight=3.01;
  REQUIRE_THROWS_AS(demo::runArticulated(options),std::invalid_argument);
  options.articulatedHeuristicWeight=std::numeric_limits<double>::infinity();
  REQUIRE_THROWS_AS(demo::runArticulated(options),std::invalid_argument);
  options.articulatedHeuristicWeight=1.0;
  options.articulatedCoarseGridResolutionMeters=0.0;
  REQUIRE_THROWS_AS(demo::runArticulated(options),std::invalid_argument);
  options.articulatedCoarseGridResolutionMeters=2.1;
  REQUIRE_THROWS_AS(demo::runArticulated(options),std::invalid_argument);
  options.articulatedCoarseGridResolutionMeters=0.15;
  options.articulatedHeadingGuidanceWeight=-1.0;
  REQUIRE_THROWS_AS(demo::runArticulated(options),std::invalid_argument);
  options.articulatedHeadingGuidanceWeight=3.01;
  REQUIRE_THROWS_AS(demo::runArticulated(options),std::invalid_argument);
  options.articulatedHeadingGuidanceWeight=0.8;
  options.articulatedEnableAnytimeGoalRefinement=true;
  REQUIRE_THROWS_AS(demo::runArticulated(options),std::invalid_argument);
  options.articulatedEnableGoalConnector=true;
  options.articulatedAnytimeExtraExpansions=0;
  REQUIRE_THROWS_AS(demo::runArticulated(options),std::invalid_argument);
  options.articulatedAnytimeExtraExpansions=100001;
  REQUIRE_THROWS_AS(demo::runArticulated(options),std::invalid_argument);
  options.articulatedAnytimeExtraExpansions=2000;
  options.articulatedHeuristicMode=static_cast<art::HeuristicMode>(99);
  REQUIRE_THROWS_AS(demo::runArticulated(options),std::invalid_argument);
}
