// Manual Nav2 label-table reservation sweep; NOT registered in CTest.
// The only tested variable is searchLabelReserve: 0, 65536, 131072, 262144.
#include "tests/nav2_footprint_envelope_benchmark_support.hpp"
#include <array>
#include <cmath>
#include <cstddef>
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
constexpr std::array<std::size_t,4> reserves{0,65536,131072,262144};

void verifyProfile(const PlanDiagnostics& d) {
#ifdef HYBRID_ASTAR_NAV2_PIPELINE_PROFILE_ENABLED
  if(d.labelPopHits+d.labelPopMisses+
         d.labelSuccessorHits+d.labelSuccessorMisses!=d.labelFinds ||
     d.labelPopHitNanoseconds+d.labelPopMissNanoseconds+
         d.labelSuccessorHitNanoseconds+d.labelSuccessorMissNanoseconds!=
         d.labelLookupNanoseconds)
    throw std::runtime_error("Label lookup profiling categories do not reconcile");
#else
  (void)d;
#endif
}

void verifyEquivalent(const pipeline::Measurement& baseline,
                      const pipeline::Measurement& reserved) {
  if(!pipeline::equivalentMeasurements(baseline,reserved))
    throw std::runtime_error("Label reservation changed path, search work or refinement");
  const auto& a=baseline.raw.diagnostics;
  const auto& b=reserved.raw.diagnostics;
  if(a.labelFinds!=b.labelFinds ||
     a.labelFindHits!=b.labelFindHits ||
     a.labelInsertions!=b.labelInsertions ||
     a.labelUpdates!=b.labelUpdates ||
     a.prunedByKey!=b.prunedByKey ||
     a.reopened!=b.reopened ||
     a.staleHeapPops!=b.staleHeapPops ||
     a.openQueuePushes!=b.openQueuePushes ||
     a.openQueuePops!=b.openQueuePops ||
     a.openQueueReplacements!=b.openQueueReplacements ||
     a.searchMotionCalls!=b.searchMotionCalls ||
     a.finalMotionCalls!=b.finalMotionCalls)
    throw std::runtime_error("Label reservation changed A* decisions or motion work");
  verifyProfile(a);
  verifyProfile(b);
}

void header() {
  std::cout
    <<"case_id,draw_index,model,geometry,variant,label_reserve,"
      "fast_empty_aabb,prefix_enabled,repeats,status,"
      "search_p50_ms,search_p95_ms,refine_p50_ms,total_p50_ms,"
      "expanded,generated,exact_pose_calls,search_motion_calls,"
      "label_finds,label_find_hits,label_insertions,label_updates,"
      "pruned_by_key,stale_open_pops,label_rehashes,"
      "label_peak_size,label_peak_buckets,bucket_bytes_estimate,"
      "peak_label_load_factor,hash_calls,profile_enabled,"
      "label_lookup_ms,label_write_ms,hash_ms\n";
}

void row(const pipeline::Config& cfg,const pipeline::Scenario& scenario,
         fe::Variant geometry,const PlanRequest& request,
         const pipeline::Measurement& m) {
  const auto& d=m.raw.diagnostics;
#ifdef HYBRID_ASTAR_NAV2_PIPELINE_PROFILE_ENABLED
  constexpr int profiled=1;
#else
  constexpr int profiled=0;
#endif
  const auto reserve=request.options.searchLabelReserve;
  const double load=d.labelPeakBuckets?
      static_cast<double>(d.labelPeakSize)/d.labelPeakBuckets:0.0;
  std::cout<<std::fixed<<std::setprecision(6)
    <<scenario.id<<','<<scenario.drawIndex<<','<<cfg.model<<','
    <<fe::label(geometry)<<','
    <<(reserve==0?"baseline":"reserved")<<','<<reserve<<','
    <<(request.options.useFastEmptyAabb?1:0)<<','
    <<(request.options.useEmptyRegionPrefixSum?1:0)<<','
    <<cfg.repeats<<','<<pipeline::statusName(m.raw.status)<<','
    <<m.searchP50<<','<<m.searchP95<<','
    <<m.refineP50<<','<<m.totalP50<<','
    <<d.expanded<<','<<d.generated<<','<<d.exactCollisionPoseCalls<<','
    <<d.searchMotionCalls<<','
    <<d.labelFinds<<','<<d.labelFindHits<<','
    <<d.labelInsertions<<','<<d.labelUpdates<<','
    <<d.prunedByKey<<','<<d.staleHeapPops<<','
    <<d.labelRehashes<<','<<d.labelPeakSize<<','
    <<d.labelPeakBuckets<<','
    <<d.labelPeakBuckets*sizeof(void*)<<','<<load<<','
    <<d.hashCalls<<','<<profiled<<','
    <<pipeline::nsMs(d.labelLookupNanoseconds)<<','
    <<pipeline::nsMs(d.labelWriteNanoseconds)<<','
    <<pipeline::nsMs(d.hashNanoseconds)<<'\n';
}

} // namespace

int main(int argc,char** argv) {
  try {
    bool fastEmptyAabb=false;
    std::size_t replayLimit=0;
    std::vector<char*> forwarded{argv[0]};
    for(int i=1;i<argc;++i) {
      const std::string arg=argv[i];
      if(arg=="--fast-empty-aabb") {
        fastEmptyAabb=true;
      } else if(arg=="--replay-limit") {
        if(i+1>=argc)throw std::invalid_argument("Missing --replay-limit value");
        replayLimit=pipeline::integer(argv[++i]);
      } else {
        forwarded.push_back(argv[i]);
      }
    }
    const auto cfg=pipeline::parseArgs(
        static_cast<int>(forwarded.size()),forwarded.data());
    if((cfg.model!="differential"&&cfg.model!="mecanum")||
       cfg.connector!="native"||cfg.collectCollisionWork)
      throw std::invalid_argument("Use native differential/mecanum without work counters");
    const auto map=hybrid_astar::demo::loadRosMapYaml(cfg.yamlPath);
    const auto base=pipeline::baseRequest(cfg,map);
    for(const auto reserve:reserves) {
      if(reserve>base.options.maxGeneratedNodes)
        throw std::invalid_argument("Reservation exceeds maxGeneratedNodes");
    }
    auto scenarios=cfg.replayCsv.empty()?
        pipeline::discover(cfg,base):pipeline::replayScenarios(cfg,base);
    // --cases must match the FULL replay fixture, even for a short smoke test.
    if(replayLimit) {
      if(cfg.replayCsv.empty())
        throw std::invalid_argument("--replay-limit requires --replay-csv");
      if(replayLimit>scenarios.size())
        throw std::invalid_argument("--replay-limit exceeds validated routes");
      scenarios.resize(replayLimit);
    }
    std::cerr<<"Label reservation sweep: "<<scenarios.size()
             <<" routes, 2 geometries, 4 reservations (0/65536/131072/262144)"
             <<"; fast AABB="<<fastEmptyAabb
             <<"; complete planner and refiner timed, warmup excluded.\n";
    header();
    for(const auto& scenario:scenarios) {
      // Alternate geometry and reservation execution order to control bias.
      const std::array<fe::Variant,2> geometries=scenario.id%2?
          std::array{fe::Variant::SmallInflated,fe::Variant::LargeRectangle}:
          std::array{fe::Variant::LargeRectangle,fe::Variant::SmallInflated};
      for(const auto geometry:geometries) {
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
        req.options.clearanceWeight=0.0; // isolate label reservation
        std::array<std::optional<pipeline::Measurement>,4> measurements;
        const auto offset=(scenario.id+
            (geometry==fe::Variant::SmallInflated?1U:0U))%reserves.size();
        for(std::size_t j=0;j<reserves.size();++j) {
          const auto step=scenario.id%2? (reserves.size()-j):j;
          const auto index=(offset+step)%reserves.size();
          auto config=req;
          config.options.searchLabelReserve=reserves[index];
          measurements[index]=pipeline::measure(cfg,config);
        }
        for(std::size_t i=0;i<reserves.size();++i) {
          if(!measurements[i])
            throw std::runtime_error("Missing label-reserve sweep result");
          if(measurements[i]->raw.status!=PlanStatus::Success)
            throw std::runtime_error(
                "Unsuccessful route in reservation sweep case "+
                std::to_string(scenario.id)+" reserve="+
                std::to_string(reserves[i])+": "+
                pipeline::statusName(measurements[i]->raw.status));
          verifyEquivalent(*measurements[0],*measurements[i]);
        }
        for(std::size_t i=0;i<reserves.size();++i) {
          req.options.searchLabelReserve=reserves[i];
          row(cfg,scenario,geometry,req,*measurements[i]);
        }
        std::cerr<<"Case "<<scenario.id<<' '<<fe::label(geometry)
                 <<" p50 search (0/65k/131k/262k): "
                 <<measurements[0]->searchP50<<'/'
                 <<measurements[1]->searchP50<<'/'
                 <<measurements[2]->searchP50<<'/'
                 <<measurements[3]->searchP50<<" ms\n";
      }
    }
    return 0;
  } catch(const std::exception& ex) {
    std::cerr<<"Nav2 label reserve sweep failed: "<<ex.what()<<'\n';
    return 2;
  }
}
