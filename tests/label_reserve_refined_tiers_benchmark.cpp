// Manual, unprofiled preferred, matched label-reservation performance sweep.
// Recreates the SAME stratified 32 routes as the map calibration by invoking
// calibrateLabelReserve with identical seed and map options; no CTest/Actions.
#include "apps/demo/map_yaml.hpp"
#include "hybrid_astar/ackermann/label_reserve_calibration.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace hybrid_astar::ackermann;
namespace {
using Clock=std::chrono::steady_clock;
constexpr std::array<std::size_t,4> tiers{13,14,15,17};
std::size_t integer(const std::string& s) {
  if(s.empty()||s.front()=='-')
    throw std::invalid_argument("Expected positive integer");
  std::size_t used=0;
  const auto n=std::stoull(s,&used);
  if(used!=s.size()||n==0)throw std::invalid_argument("Expected positive integer");
  return static_cast<std::size_t>(n);
}
std::uint64_t seedInteger(const std::string& s) {
  if(s.empty()||s.front()=='-')
    throw std::invalid_argument("Expected nonnegative seed");
  std::size_t used=0;
  const auto n=std::stoull(s,&used);
  if(used!=s.size())throw std::invalid_argument("Invalid seed");
  return static_cast<std::uint64_t>(n);
}
double percentile(std::vector<double> values,double fraction) {
  if(values.empty())throw std::invalid_argument("Empty timings");
  std::sort(values.begin(),values.end());
  return values[std::min(values.size()-1,
      static_cast<std::size_t>(std::ceil(fraction*values.size()))-1)];
}
bool near(double a,double b) {
  return std::abs(a-b)<=1e-9*(1.0+std::abs(a)+std::abs(b));
}
bool samePose(const hybrid_astar::Pose2D& a,const hybrid_astar::Pose2D& b) {
  return near(a.x,b.x)&&near(a.y,b.y)&&near(a.yaw,b.yaw);
}
void requireEquivalent(const PlanResult& a,const PlanResult& b) {
  if(a.status!=PlanStatus::Success||b.status!=PlanStatus::Success||
     a.sections.size()!=b.sections.size()||
     !near(a.weightedCostMeters,b.weightedCostMeters)||
     !near(a.clearanceCostMeters,b.clearanceCostMeters)||
     !near(a.geometricLengthMeters,b.geometricLengthMeters)||
     !samePose(a.reached,b.reached))
    throw std::runtime_error("Reservation changed path result or status");
  const auto& x=a.diagnostics;const auto& y=b.diagnostics;
  if(x.expanded!=y.expanded||x.generated!=y.generated||
     x.exactCollisionPoseCalls!=y.exactCollisionPoseCalls||
     x.searchPoseChecks!=y.searchPoseChecks||
     x.finalPoseChecks!=y.finalPoseChecks||
     x.searchMotionCalls!=y.searchMotionCalls||
     x.finalMotionCalls!=y.finalMotionCalls||
     x.labelPeakSize!=y.labelPeakSize||
     x.labelFinds!=y.labelFinds||x.labelFindHits!=y.labelFindHits||
     x.labelInsertions!=y.labelInsertions||x.labelUpdates!=y.labelUpdates||
     x.prunedByKey!=y.prunedByKey||x.reopened!=y.reopened||
     x.staleHeapPops!=y.staleHeapPops||
     x.openQueuePushes!=y.openQueuePushes||
     x.openQueuePops!=y.openQueuePops||
     x.gridSettlements!=y.gridSettlements||x.gridFallbacks!=y.gridFallbacks)
    throw std::runtime_error("Reservation changed search, hash or collision work");
  for(std::size_t i=0;i<a.sections.size();++i) {
    const auto& p=a.sections[i];const auto& q=b.sections[i];
    if(p.kind!=q.kind||p.gear!=q.gear||
       !samePose(p.start,q.start)||!samePose(p.end,q.end)||
       !near(p.curvature,q.curvature)||
       !near(p.lengthMeters,q.lengthMeters)||
       !near(p.turnRadians,q.turnRadians)||
       !near(p.forwardMeters,q.forwardMeters)||
       !near(p.lateralMeters,q.lateralMeters))
      throw std::runtime_error("Reservation changed motion section geometry");
  }
}
struct Measurement {
  PlanResult result;
  double searchP50=0.0,searchP95=0.0;
};
Measurement measure(const PlanRequest& req,std::size_t repeats) {
  // One complete warmup for EACH reserve option, excluded from wall timing.
  const auto warmup=plan(req);
  if(warmup.status!=PlanStatus::Success)
    throw std::runtime_error("Warehouse route did not succeed in warmup");
  std::vector<double> times;
  times.reserve(repeats);
  Measurement m;
  for(std::size_t r=0;r<repeats;++r) {
    const auto start=Clock::now();
    auto result=plan(req);
    const auto end=Clock::now();
    requireEquivalent(warmup,result);
    times.push_back(std::chrono::duration<double,std::milli>(end-start).count());
    m.result=std::move(result);
  }
  m.searchP50=percentile(times,.50);
  m.searchP95=percentile(times,.95);
  return m;
}
void header() {
  std::cout<<"sample,model,geometry,seed,tier_series_version,tier_index,"
      "label_reserve,repeats,status,search_p50_ms,search_p95_ms,"
      "expanded,generated,peak_labels,baseline_rehashes,rehashes,"
      "label_finds,label_find_hits,label_insertions,label_updates,"
      "pruned_by_key,exact_pose_calls,search_motion_calls,"
      "peak_buckets,bucket_bytes_estimate,fast_empty_aabb,"
      "map_fingerprint\n";
}
void row(std::size_t sample,const std::string& model,const std::string& geometry,
         std::uint64_t seed,std::size_t tier,std::size_t repeats,
         std::uint64_t mapFingerprint,std::size_t baselineRehashes,
         bool fastAabb,const Measurement& m) {
  const auto& d=m.result.diagnostics;
  std::cout<<std::fixed<<std::setprecision(6)
    <<sample<<','<<model<<','<<geometry<<','<<seed<<','
    <<LabelReserveTierSeriesVersion<<','<<tier<<','
    <<labelReserveForTier(tier)<<','<<repeats<<",Success,"
    <<m.searchP50<<','<<m.searchP95<<','
    <<d.expanded<<','<<d.generated<<','<<d.labelPeakSize<<','
    <<baselineRehashes<<','<<d.labelRehashes<<','
    <<d.labelFinds<<','<<d.labelFindHits<<','
    <<d.labelInsertions<<','<<d.labelUpdates<<','
    <<d.prunedByKey<<','<<d.exactCollisionPoseCalls<<','
    <<d.searchMotionCalls<<','<<d.labelPeakBuckets<<','
    <<d.labelPeakBuckets*sizeof(void*)<<','
    <<(fastAabb?1:0)<<','<<mapFingerprint<<'\n';
}
void help() {
  std::cerr<<"Usage: hybrid_astar_label_reserve_refined_tiers_benchmark "
    "[--map YAML] [--model differential|mecanum] "
    "[--geometry large|small-inflated] [--samples 32] [--seed 20261104] "
    "[--repeats 5] [--max-expansions 500000] [--max-generated 2000000] "
    "[--grid-settlements 100000] [--fast-empty-aabb]\n"
    "Manual four-way speed comparison, tiers 13/14/15/17; CSV to stdout.\n";
}
} // namespace

int main(int argc,char** argv) {
  try {
    std::string yaml="apps/demo/maps/nav2_warehouse_full.yaml";
    std::string model="differential",geometry="large";
    std::size_t repeats=5,maxExpansions=500000;
    std::size_t maxGenerated=2000000,gridSettlements=100000;
    bool fastAabb=false;
    LabelReserveCalibrationOptions settings;
    settings.verifyNoRehash=false; // map calibration is only route discovery
    for(int i=1;i<argc;++i) {
      const std::string arg=argv[i];
      if(arg=="--help"||arg=="-h"){help();return 0;}
      if(arg=="--fast-empty-aabb"){fastAabb=true;continue;}
      if(i+1>=argc)throw std::invalid_argument("Missing argument value: "+arg);
      const std::string v=argv[++i];
      if(arg=="--map")yaml=v;
      else if(arg=="--model")model=v;
      else if(arg=="--geometry")geometry=v;
      else if(arg=="--samples")settings.routeSamples=integer(v);
      else if(arg=="--seed")settings.seed=seedInteger(v);
      else if(arg=="--repeats")repeats=integer(v);
      else if(arg=="--max-expansions")maxExpansions=integer(v);
      else if(arg=="--max-generated")maxGenerated=integer(v);
      else if(arg=="--grid-settlements")gridSettlements=integer(v);
      else throw std::invalid_argument("Unknown option: "+arg);
    }
    if(model!="differential"&&model!="mecanum")
      throw std::invalid_argument("Only native differential/mecanum models supported");
    if(geometry!="large"&&geometry!="small-inflated")
      throw std::invalid_argument("Unknown footprint geometry");
    if(repeats>100||maxGenerated<labelReserveForTier(tiers.back()))
      throw std::invalid_argument("Invalid repeats or insufficient max-generated budget");
    const auto map=hybrid_astar::demo::loadRosMapYaml(yaml);
    PlanRequest req;
    req.map=map;
    req.options.model=model=="differential"?
        PlanOptions::Model::DifferentialDrive:PlanOptions::Model::Mecanum;
    req.options.rotationCostMetersPerRadian=.4;
    req.options.maxExpansions=maxExpansions;
    req.options.maxGeneratedNodes=maxGenerated;
    req.options.gridMaxTotalSettlements=gridSettlements;
    req.options.heuristicMode=HeuristicMode::PerformanceReedsSheppAndObstacleGrid;
    req.options.useEmptyRegionPrefixSum=true;
    req.options.useFastEmptyAabb=fastAabb;
    req.options.useInflatedFootprintBroadPhase=true;
    req.footprint=geometry=="large"?
        Footprint{-.35,.35,.25}:Footprint{-.25,.25,.15};
    req.options.footprintInflationMeters=geometry=="large"?0.0:0.10;
    req.options.clearanceWeight=0.0; // strictly isolate state-label reservation

    const auto calibration=calibrateLabelReserve(req,settings);
    if(!calibration.sampleTargetMet)
      throw std::runtime_error("Incomplete route discovery");
    std::cerr<<"Matched tier benchmark: "<<calibration.evaluatedRoutes
             <<" seeded routes, map fingerprint="<<calibration.mapFingerprint
             <<", valid strata="<<calibration.validStrata
             <<", covered endpoint strata="<<calibration.coveredEitherStrata
             <<", max observed labels="<<calibration.maxObservedPeakLabels
             <<"; manual timing includes allocations, excludes route discovery.\n";
    header();
    for(std::size_t sample=0;sample<calibration.trials.size();++sample) {
      const auto& trial=calibration.trials[sample];
      req.start=trial.start;req.goal=trial.goal;
      std::array<Measurement,4> measured;
      // Alternate both forward/reverse execution order and start position,
      // never use result order as the canonical CSV order.
      const std::size_t offset=sample%tiers.size();
      for(std::size_t j=0;j<tiers.size();++j) {
        const std::size_t step=sample%2 ? tiers.size()-j:j;
        const std::size_t index=(offset+step)%tiers.size();
        req.options.searchLabelReserve=labelReserveForTier(tiers[index]);
        measured[index]=measure(req,repeats);
      }
      for(const auto& m:measured) {
        if(m.result.status!=trial.status||
           m.result.diagnostics.expanded!=trial.expanded||
           m.result.diagnostics.generated!=trial.generated||
           m.result.diagnostics.labelPeakSize!=trial.peakLabels)
          throw std::runtime_error("Route differs from initial seeded calibration");
        requireEquivalent(measured[0].result,m.result);
        if(m.result.diagnostics.labelRehashes!=0)
          throw std::runtime_error("Reserved table rehashed in sampled route");
      }
      for(std::size_t i=0;i<tiers.size();++i)
        row(sample,model,geometry,settings.seed,tiers[i],repeats,
            calibration.mapFingerprint,trial.baselineRehashes,
            fastAabb,measured[i]);
      std::cerr<<"Route "<<sample<<" search p50 ms: "
               <<measured[0].searchP50<<"/"<<measured[1].searchP50
               <<"/"<<measured[2].searchP50<<"/"<<measured[3].searchP50
               <<"; peak labels="<<trial.peakLabels<<'\n';
    }
    return 0;
  } catch(const std::exception& e) {
    std::cerr<<"Refined label-reserve benchmark failed: "<<e.what()<<'\n';
    return 2;
  }
}
