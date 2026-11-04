// Offline, manually invoked map-specific state-label reservation calibration.
// Emits one CSV row per sampled route to stdout; summary/tier to stderr.
#include "apps/demo/map_yaml.hpp"
#include "hybrid_astar/ackermann/label_reserve_calibration.hpp"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

using namespace hybrid_astar::ackermann;
namespace {
std::size_t integer(const std::string& s) {
  if(s.empty()||s[0]=='-')throw std::invalid_argument("Expected positive integer");
  std::size_t used=0;const auto n=std::stoull(s,&used);
  if(n==0||used!=s.size())throw std::invalid_argument("Expected positive integer");
  return static_cast<std::size_t>(n);
}
std::uint64_t seedInteger(const std::string& s) {
  if(s.empty()||s[0]=='-')throw std::invalid_argument("Expected nonnegative seed");
  std::size_t used=0;const auto n=std::stoull(s,&used);
  if(used!=s.size())throw std::invalid_argument("Invalid seed");
  return static_cast<std::uint64_t>(n);
}
double real(const std::string& s) {
  std::size_t used=0;const auto n=std::stod(s,&used);
  if(used!=s.size()||!std::isfinite(n)||n<0)
    throw std::invalid_argument("Expected finite nonnegative number");
  return n;
}
const char* status(PlanStatus s) {
  switch(s) {
  case PlanStatus::Success:return "Success";
  case PlanStatus::BudgetExceeded:return "BudgetExceeded";
  case PlanStatus::ValidationIncomplete:return "ValidationIncomplete";
  case PlanStatus::NoPath:return "NoPath";
  case PlanStatus::StrictValidationFailed:return "StrictValidationFailed";
  default:return "Other";
  }
}
void help() {
  std::cerr
    <<"Usage: hybrid_astar_label_reserve_map_calibration_benchmark "
      "[--map YAML] [--model differential|ackermann|mecanum] "
      "[--geometry large|small-inflated] [--samples 32] [--seed 20261104] "
      "[--strata-x N --strata-y N] [--pose-attempts 48] "
      "[--pair-attempts 32] [--min-distance 0=auto] [--headroom 0.2] "
      "[--max-expansions 500000] [--max-generated 2000000] "
      "[--grid-settlements 100000] [--fast-empty-aabb] "
      "[--no-verify] [--summary-json FILE]\n"
    <<"The output is empirical; re-run after map or planner option changes.\n";
}
} // namespace

int main(int argc,char** argv) {
  try {
    std::string yaml="apps/demo/maps/nav2_warehouse_full.yaml";
    std::string model="differential",geometry="large",summaryPath;
    bool fastAabb=false;
    std::size_t maxExpansions=500000,maxGenerated=2000000,gridSettlements=100000;
    LabelReserveCalibrationOptions settings;
    for(int i=1;i<argc;++i) {
      const std::string arg=argv[i];
      if(arg=="--help"||arg=="-h"){help();return 0;}
      if(arg=="--fast-empty-aabb"){fastAabb=true;continue;}
      if(arg=="--no-verify"){settings.verifyNoRehash=false;continue;}
      if(i+1>=argc)throw std::invalid_argument("Missing value for "+arg);
      const std::string v=argv[++i];
      if(arg=="--map")yaml=v;
      else if(arg=="--model")model=v;
      else if(arg=="--geometry")geometry=v;
      else if(arg=="--summary-json")summaryPath=v;
      else if(arg=="--samples")settings.routeSamples=integer(v);
      else if(arg=="--seed")settings.seed=seedInteger(v);
      else if(arg=="--strata-x")settings.strataX=integer(v);
      else if(arg=="--strata-y")settings.strataY=integer(v);
      else if(arg=="--pose-attempts")settings.poseAttemptsPerStratum=integer(v);
      else if(arg=="--pair-attempts")settings.maxPairAttemptsPerRoute=integer(v);
      else if(arg=="--min-distance")settings.minStartGoalDistanceMeters=real(v);
      else if(arg=="--headroom")settings.headroomFraction=real(v);
      else if(arg=="--max-expansions")maxExpansions=integer(v);
      else if(arg=="--max-generated")maxGenerated=integer(v);
      else if(arg=="--grid-settlements")gridSettlements=integer(v);
      else throw std::invalid_argument("Unknown option: "+arg);
    }
    if(model!="differential"&&model!="ackermann"&&model!="mecanum")
      throw std::invalid_argument("Unknown native search model");
    if(geometry!="large"&&geometry!="small-inflated")
      throw std::invalid_argument("Unknown footprint geometry");
    if(maxGenerated<65536)
      throw std::invalid_argument("max-generated must permit the first 65536-label tier");
    auto map=hybrid_astar::demo::loadRosMapYaml(yaml);
    PlanRequest request;
    request.map=map;
    request.options.maxExpansions=maxExpansions;
    request.options.maxGeneratedNodes=maxGenerated;
    request.options.gridMaxTotalSettlements=gridSettlements;
    request.options.useEmptyRegionPrefixSum=true;
    request.options.useFastEmptyAabb=fastAabb;
    request.options.heuristicMode=HeuristicMode::PerformanceReedsSheppAndObstacleGrid;
    request.footprint=geometry=="large"?
        Footprint{-.35,.35,.25}:Footprint{-.25,.25,.15};
    request.options.footprintInflationMeters=geometry=="large"?0.0:0.10;
    if(model=="differential")request.options.model=PlanOptions::Model::DifferentialDrive;
    else if(model=="mecanum")request.options.model=PlanOptions::Model::Mecanum;
    else request.options.model=PlanOptions::Model::Ackermann;
    request.options.rotationCostMetersPerRadian=.4;

    const auto result=calibrateLabelReserve(request,settings);
    std::cout<<std::fixed<<std::setprecision(6)
      <<"sample,model,geometry,start_stratum,goal_stratum,"
        "start_x,start_y,start_yaw,goal_x,goal_y,goal_yaw,"
        "status,expanded,generated,peak_labels,baseline_rehashes,"
        "verified_rehashes\n";
    for(std::size_t i=0;i<result.trials.size();++i) {
      const auto& t=result.trials[i];
      std::cout<<i<<','<<model<<','<<geometry<<','
        <<t.startStratum<<','<<t.goalStratum<<','
        <<t.start.x<<','<<t.start.y<<','<<t.start.yaw<<','
        <<t.goal.x<<','<<t.goal.y<<','<<t.goal.yaw<<','
        <<status(t.status)<<','<<t.expanded<<','<<t.generated<<','
        <<t.peakLabels<<','<<t.baselineRehashes<<','
        <<t.verifiedRehashes<<'\n';
    }
    std::cerr<<"Map calibration result (empirical): tier="
             <<result.recommendedTierIndex
             <<", reserve_labels="<<result.recommendedReserveLabels
             <<", max_observed_labels="<<result.maxObservedPeakLabels
             <<", map_fingerprint="<<result.mapFingerprint
             <<", successes="<<result.successfulRoutes<<'/'
             <<result.evaluatedRoutes<<", budget_limited="
             <<result.budgetLimitedRoutes
             <<", valid_strata="<<result.validStrata<<'/'
             <<result.strataX*result.strataY
             <<", covered_start_strata="<<result.coveredStartStrata
             <<", covered_goal_strata="<<result.coveredGoalStrata
             <<", verified="<<(result.verificationPassed?"yes":"no (opted out)")
             <<"\n";
    if(!summaryPath.empty()) {
      std::ofstream file(summaryPath);
      if(!file)throw std::runtime_error("Cannot open summary JSON file");
      file<<"{\n"
          <<"  \"map\": \""<<yaml<<"\",\n"
          <<"  \"model\": \""<<model<<"\",\n"
          <<"  \"geometry\": \""<<geometry<<"\",\n"
          <<"  \"map_revision\": "<<result.mapRevision<<",\n"
          <<"  \"map_fingerprint\": "<<result.mapFingerprint<<",\n"
          <<"  \"seed\": "<<settings.seed<<",\n"
          <<"  \"headroom_fraction\": "<<settings.headroomFraction<<",\n"
          <<"  \"max_expansions\": "<<maxExpansions<<",\n"
          <<"  \"max_generated_nodes\": "<<maxGenerated<<",\n"
          <<"  \"strata_x\": "<<result.strataX<<",\n"
          <<"  \"strata_y\": "<<result.strataY<<",\n"
          <<"  \"valid_strata\": "<<result.validStrata<<",\n"
          <<"  \"covered_start_strata\": "<<result.coveredStartStrata<<",\n"
          <<"  \"covered_goal_strata\": "<<result.coveredGoalStrata<<",\n"
          <<"  \"sampled_routes\": "<<result.evaluatedRoutes<<",\n"
          <<"  \"successful_routes\": "<<result.successfulRoutes<<",\n"
          <<"  \"budget_limited_routes\": "<<result.budgetLimitedRoutes<<",\n"
          <<"  \"max_observed_peak_labels\": "<<result.maxObservedPeakLabels<<",\n"
          <<"  \"recommended_tier_index\": "<<result.recommendedTierIndex<<",\n"
          <<"  \"recommended_reserve_labels\": "<<result.recommendedReserveLabels<<",\n"
          <<"  \"verification_passed\": "<<(result.verificationPassed?"true":"false")
          <<"\n}\n";
      if(!file)throw std::runtime_error("Failed to write summary JSON");
    }
    return 0;
  } catch(const std::exception& e) {
    std::cerr<<"Map label-reservation calibration failed: "<<e.what()<<'\n';
    return 2;
  }
}
