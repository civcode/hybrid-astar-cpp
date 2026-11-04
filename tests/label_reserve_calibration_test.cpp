#include <catch2/catch_test_macros.hpp>
#include "hybrid_astar/ackermann/label_reserve_calibration.hpp"
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>

using namespace hybrid_astar::ackermann;

TEST_CASE("Versioned label tiers use four steps per doubling with legacy decode",
          "[labels][calibration]") {
  REQUIRE(LabelReserveTierSeriesVersion==2);
  const std::size_t expected[]{
      0,65536,81920,98304,114688,131072,163840,196608,229376,
      262144,327680,393216,458752,524288,655360,786432,917504,
      1048576,1310720};
  for(std::size_t tier=0;tier<sizeof(expected)/sizeof(expected[0]);++tier)
    REQUIRE(labelReserveForTier(tier)==expected[tier]);
  // Previously persisted v1 numeric tiers are never interpreted as v2
  // when the stored series-version tag is supplied.
  REQUIRE(labelReserveForTier(0,1)==0);
  REQUIRE(labelReserveForTier(3,1)==262144);
  REQUIRE(labelReserveForTier(4,1)==524288);
  REQUIRE(labelReserveForTier(5,1)==1048576);
  REQUIRE_THROWS_AS(labelReserveForTier(1,3),std::invalid_argument);
  REQUIRE_THROWS_AS(labelReserveForTier(std::size_t(-1)),std::out_of_range);
  REQUIRE(labelReserveTierForPeak(40000,0.20,100000)==1);
  REQUIRE(labelReserveTierForPeak(65000,0.20,150000)==2);
  REQUIRE(labelReserveTierForPeak(120000,0.20,300000)==6);
  REQUIRE(labelReserveTierForPeak(200000,0.20,300000)==9);
  REQUIRE(labelReserveTierForPeak(484534,.20,2000000)==14);
  REQUIRE(labelReserveTierForPeak(484534,0.0,2000000)==13);
  REQUIRE(labelReserveTierForPeak(655360,0.0,2000000)==14);
  REQUIRE(labelReserveTierForPeak(655361,0.0,2000000)==15);
  REQUIRE_THROWS_AS(labelReserveTierForPeak(0,.2,300000),std::invalid_argument);
  REQUIRE_THROWS_AS(labelReserveTierForPeak(200000,.3,200000),std::out_of_range);
  REQUIRE_THROWS_AS(labelReserveTierForPeak(1,0.,10000),std::out_of_range);
  REQUIRE_THROWS_AS(labelReserveTierForPeak(10,-.1,200000),std::invalid_argument);
}

TEST_CASE("Seeded rotated-map calibration covers multiple map strata and verifies tier",
          "[labels][calibration][deterministic]") {
  auto map=std::make_shared<MapSnapshot>();
  map->width=60;map->height=48;map->resolutionMeters=.25;
  map->origin={3.,-2.,.55};
  map->revision=52;
  map->cells.assign(map->width*map->height,0);
  for(std::size_t y=18;y<30;++y)
    for(std::size_t x=26;x<32;++x)
      map->cells[y*map->width+x]=100;
  for(std::size_t y=32;y<38;++y)
    for(std::size_t x=39;x<43;++x)
      map->cells[y*map->width+x]=-1;
  PlanRequest request;
  request.map=map;
  request.footprint={-.25,.25,.15};
  request.options.model=PlanOptions::Model::DifferentialDrive;
  request.options.footprintInflationMeters=.05;
  request.options.useEmptyRegionPrefixSum=true;
  request.options.useFastEmptyAabb=true;
  request.options.maxExpansions=350;
  request.options.maxGeneratedNodes=130000;
  request.options.gridMaxTotalSettlements=3000;
  LabelReserveCalibrationOptions settings;
  settings.seed=129837;
  settings.routeSamples=4;
  settings.strataX=4;settings.strataY=3;
  settings.poseAttemptsPerStratum=24;
  settings.minStartGoalDistanceMeters=1.0;
  settings.headroomFraction=.2;
  const auto first=calibrateLabelReserve(request,settings);
  REQUIRE(first.sampleTargetMet);
  REQUIRE(first.verificationPassed);
  REQUIRE(first.evaluatedRoutes==4);
  REQUIRE(first.validStrata>=6);
  REQUIRE(first.coveredStartStrata>=3);
  REQUIRE(first.coveredGoalStrata>=2);
  REQUIRE(first.coveredEitherStrata>=first.coveredStartStrata);
  REQUIRE(first.coveredEitherStrata>=first.coveredGoalStrata);
  REQUIRE(first.coveredEitherStrata<=first.validStrata);
  REQUIRE(first.recommendedTierIndex>=1);
  REQUIRE(first.tierSeriesVersion==LabelReserveTierSeriesVersion);
  REQUIRE(first.recommendedReserveLabels==labelReserveForTier(first.recommendedTierIndex));
  REQUIRE(first.maxObservedPeakLabels>0);
  REQUIRE(first.mapRevision==52);
  for(const auto& trial:first.trials) {
    REQUIRE(trial.peakLabels>0);
    REQUIRE(trial.verifiedRehashes==0);
    REQUIRE(trial.startStratum!=trial.goalStratum);
  }
  const auto again=calibrateLabelReserve(request,settings);
  REQUIRE(again.recommendedReserveLabels==first.recommendedReserveLabels);
  REQUIRE(again.maxObservedPeakLabels==first.maxObservedPeakLabels);
  REQUIRE(again.validStrata==first.validStrata);
  REQUIRE(again.trials.size()==first.trials.size());
  for(std::size_t i=0;i<first.trials.size();++i) {
    const auto& a=first.trials[i];const auto& b=again.trials[i];
    REQUIRE(a.start.x==b.start.x);
    REQUIRE(a.start.y==b.start.y);
    REQUIRE(a.start.yaw==b.start.yaw);
    REQUIRE(a.goal.x==b.goal.x);
    REQUIRE(a.goal.y==b.goal.y);
    REQUIRE(a.goal.yaw==b.goal.yaw);
    REQUIRE(a.peakLabels==b.peakLabels);
    REQUIRE(a.status==b.status);
  }
  // Opting out of verification still leaves a well-defined empirical tier.
  settings.verifyNoRehash=false;
  const auto unverified=calibrateLabelReserve(request,settings);
  REQUIRE_FALSE(unverified.verificationPassed);
  REQUIRE(unverified.recommendedReserveLabels==first.recommendedReserveLabels);
}

TEST_CASE("Calibration rejects unrepresentative or impossible sample configurations",
          "[labels][calibration][validation]") {
  auto map=std::make_shared<MapSnapshot>();
  map->width=24;map->height=24;map->resolutionMeters=.2;
  map->cells.assign(map->width*map->height,100);
  PlanRequest request;
  request.map=map;
  request.footprint={-.2,.2,.15};
  LabelReserveCalibrationOptions options;
  options.routeSamples=2;
  options.strataX=3;options.strataY=3;
  options.poseAttemptsPerStratum=4;
  REQUIRE_THROWS_AS(calibrateLabelReserve(request,options),std::runtime_error);
  map->cells.assign(map->width*map->height,0);
  options.strataY=0;
  REQUIRE_THROWS_AS(calibrateLabelReserve(request,options),std::invalid_argument);
  options.strataY=3;
  options.headroomFraction=-.1;
  REQUIRE_THROWS_AS(calibrateLabelReserve(request,options),std::invalid_argument);
}
