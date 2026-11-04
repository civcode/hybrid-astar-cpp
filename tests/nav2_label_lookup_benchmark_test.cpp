// Manual Nav2 label-table pre-reserve A/B; never registered in CTest.
// Fixed routes, footprint, prefix and fast-AABB settings on both sides.
#include "tests/nav2_footprint_envelope_benchmark_support.hpp"
#include <array>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace pipeline=hybrid_astar::bench::nav2_pipeline;
namespace fe=hybrid_astar::bench::footprint_envelope;
using namespace hybrid_astar::ackermann;

namespace {
void checkProfile(const PlanDiagnostics& d) {
#ifdef HYBRID_ASTAR_NAV2_PIPELINE_PROFILE_ENABLED
  const auto calls=d.labelPopHits+d.labelPopMisses+
                   d.labelSuccessorHits+d.labelSuccessorMisses;
  if(calls!=d.labelFinds ||
     d.labelPopHitNanoseconds+d.labelPopMissNanoseconds+
     d.labelSuccessorHitNanoseconds+d.labelSuccessorMissNanoseconds!=
       d.labelLookupNanoseconds)
    throw std::runtime_error("Label lookup profile categories do not reconcile");
#else
  (void)d;
#endif
}
void verifyEquivalent(const pipeline::Measurement& off,
                      const pipeline::Measurement& on) {
  if(!pipeline::equivalentMeasurements(off,on))
    throw std::runtime_error("Pre-reserving labels changed search/refinement results");
  const auto& a=off.raw.diagnostics;
  const auto& b=on.raw.diagnostics;
  if(a.labelFinds!=b.labelFinds||a.labelFindHits!=b.labelFindHits||
     a.labelInsertions!=b.labelInsertions||a.labelUpdates!=b.labelUpdates||
     a.prunedByKey!=b.prunedByKey||a.reopened!=b.reopened||
     a.staleHeapPops!=b.staleHeapPops||
     a.openQueuePushes!=b.openQueuePushes||
     a.openQueuePops!=b.openQueuePops)
    throw std::runtime_error("Pre-reserving labels changed A* work or decisions");
  checkProfile(a);
  checkProfile(b);
}
void header() {
  std::cout<<"case_id,draw_index,model,geometry,variant,search_label_reserve,"
              "fast_empty_aabb,prefix_enabled,status,repeats,search_p50_ms,"
              "search_p95_ms,refine_p50_ms,total_p50_ms,expanded,generated,"
              "exact_pose_calls,pruned_by_key,stale_open_pops,label_finds,"
              "label_find_hits,label_insertions,label_updates,label_rehashes,"
              "label_peak_size,label_peak_buckets,label_bucket_bytes_estimate,"
              "hash_calls,profile_enabled,label_lookup_total_ms,"
              "label_pop_hit_calls,label_pop_miss_calls,"
              "label_successor_hit_calls,label_successor_miss_calls,"
              "label_pop_hit_ms,label_pop_miss_ms,"
              "label_successor_hit_ms,label_successor_miss_ms,"
              "label_write_ms,hash_hotpath_ms,exact_collision_hotpath_ms\n";
}
void row(const pipeline::Config& cfg,const pipeline::Scenario& scenario,
         fe::Variant geometry,const char* variant,
         const PlanRequest& req,const pipeline::Measurement& m) {
  const auto& d=m.raw.diagnostics;
#ifdef HYBRID_ASTAR_NAV2_PIPELINE_PROFILE_ENABLED
  constexpr int profiled=1;
#else
  constexpr int profiled=0;
#endif
  std::cout<<std::fixed<<std::setprecision(6)
    <<scenario.id<<','<<scenario.drawIndex<<','<<cfg.model<<','
    <<fe::label(geometry)<<','<<variant<<','
    <<req.options.searchLabelReserve<<','
    <<(req.options.useFastEmptyAabb?1:0)<<','
    <<(req.options.useEmptyRegionPrefixSum?1:0)<<','
    <<pipeline::statusName(m.raw.status)<<','<<cfg.repeats<<','
    <<m.searchP50<<','<<m.searchP95<<','<<m.refineP50<<','
    <<m.totalP50<<','<<d.expanded<<','<<d.generated<<','
    <<d.exactCollisionPoseCalls<<','<<d.prunedByKey<<','
    <<d.staleHeapPops<<','<<d.labelFinds<<','
    <<d.labelFindHits<<','<<d.labelInsertions<<','<<d.labelUpdates<<','
    <<d.labelRehashes<<','<<d.labelPeakSize<<','<<d.labelPeakBuckets<<','
    <<d.labelPeakBuckets*sizeof(void*)<<','<<d.hashCalls<<','
    <<profiled<<','<<pipeline::nsMs(d.labelLookupNanoseconds)<<','
    <<d.labelPopHits<<','<<d.labelPopMisses<<','
    <<d.labelSuccessorHits<<','<<d.labelSuccessorMisses<<','
    <<pipeline::nsMs(d.labelPopHitNanoseconds)<<','
    <<pipeline::nsMs(d.labelPopMissNanoseconds)<<','
    <<pipeline::nsMs(d.labelSuccessorHitNanoseconds)<<','
    <<pipeline::nsMs(d.labelSuccessorMissNanoseconds)<<','
    <<pipeline::nsMs(d.labelWriteNanoseconds)<<','
    <<pipeline::nsMs(d.hashNanoseconds)<<','
    <<pipeline::nsMs(d.exactCollisionNanoseconds)<<'\n';
}
} // namespace

int main(int argc,char** argv) {
  try {
    std::size_t reserve=262144;
    bool fastEmptyAabb=false;
    std::vector<char*> forwarded{argv[0]};
    for(int i=1;i<argc;++i) {
      const std::string arg=argv[i];
      if(arg=="--label-reserve") {
        if(i+1>=argc)throw std::invalid_argument("Missing value for --label-reserve");
        reserve=pipeline::integer(argv[++i]);
      } else if(arg=="--fast-empty-aabb") {
        fastEmptyAabb=true;
      } else {
        forwarded.push_back(argv[i]);
      }
    }
    const auto cfg=pipeline::parseArgs(
        static_cast<int>(forwarded.size()),forwarded.data());
    if((cfg.model!="differential"&&cfg.model!="mecanum")||
       cfg.connector!="native"||cfg.collectCollisionWork ||
       reserve>4*cfg.searchBudget)
      throw std::invalid_argument("Use differential/mecanum, native, valid reserve, no work counters");
    const auto map=hybrid_astar::demo::loadRosMapYaml(cfg.yamlPath);
    const auto base=pipeline::baseRequest(cfg,map);
    const auto scenarios=cfg.replayCsv.empty()?
        pipeline::discover(cfg,base):pipeline::replayScenarios(cfg,base);
    std::cerr<<"Label pre-reserve 0/"<<reserve
             <<", fast empty AABB "<<(fastEmptyAabb?"ON":"OFF")
             <<", prefix ON. Replaying "<<scenarios.size()
             <<" routes; map loading/discovery excluded from timings.\n";
    header();
    for(const auto& scenario:scenarios) {
      const std::array<fe::Variant,2> geometries=scenario.id%2?
          std::array{fe::Variant::SmallInflated,fe::Variant::LargeRectangle}:
          std::array{fe::Variant::LargeRectangle,fe::Variant::SmallInflated};
      for(auto geometry:geometries) {
        auto req=base;
        req.start=scenario.endpoints.start;
        req.goal=scenario.endpoints.goal;
        req.footprint=geometry==fe::Variant::LargeRectangle?
            Footprint{-.35,.35,.25}:Footprint{-.25,.25,.15};
        req.options.footprintInflationMeters=
            geometry==fe::Variant::LargeRectangle?0.:.10;
        req.options.useEmptyRegionPrefixSum=true;
        req.options.useFastEmptyAabb=fastEmptyAabb;
        req.options.useInflatedFootprintBroadPhase=true;
        std::optional<pipeline::Measurement> baseline,preReserved;
        // Alternate order to reduce temperature/frequency/order bias.
        for(auto value:scenario.id%2?
             std::array<std::size_t,2>{reserve,0}:
             std::array<std::size_t,2>{0,reserve}) {
          auto variant=req;
          variant.options.searchLabelReserve=value;
          auto result=pipeline::measure(cfg,variant);
          if(value)preReserved=std::move(result);
          else baseline=std::move(result);
        }
        if(!baseline||!preReserved)throw std::runtime_error("Missing A/B measurement");
        verifyEquivalent(*baseline,*preReserved);
        req.options.searchLabelReserve=0;
        row(cfg,scenario,geometry,"baseline",req,*baseline);
        req.options.searchLabelReserve=reserve;
        row(cfg,scenario,geometry,"pre_reserved",req,*preReserved);
        std::cerr<<"Case "<<scenario.id<<" "<<fe::label(geometry)
                 <<" search p50 baseline/reserved: "
                 <<baseline->searchP50<<"/"<<preReserved->searchP50<<" ms\n";
      }
    }
    return 0;
  } catch(const std::exception& e) {
    std::cerr<<"Nav2 label lookup experiment failed: "<<e.what()<<'\n';
    return 2;
  }
}
