// Opt-in wall-time sampling of Hybrid A* label-table, collision and queue hot paths.
// Build with -DHYBRID_ASTAR_PROFILE_HOTPATHS=ON to obtain nanosecond counters.
#include "bench/hard_scenarios.hpp"
#include "hybrid_astar/ackermann/planner.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace hybrid_astar;
using namespace hybrid_astar::ackermann;
namespace {
struct Variant {
  const char* name;
  std::size_t labelReserve;
  std::size_t discCount;
};
constexpr Variant variants[]={
    {"baseline",0,0},
    {"reserve_4096",4096,0},
    {"reserve_16384",16384,0},
    {"disc_4",0,4},
    {"reserve_4096_disc_4",4096,4}
};
double quantile(std::vector<double> v,double p) {
  std::sort(v.begin(),v.end());
  return v.at(std::max<std::size_t>(1,std::size_t(std::ceil(v.size()*p)))-1);
}
void equivalent(const PlanResult& a,const PlanResult& b) {
  if(a.status!=b.status || a.diagnostics.expanded!=b.diagnostics.expanded ||
     a.diagnostics.generated!=b.diagnostics.generated ||
     a.mapRevision!=b.mapRevision ||
     a.sections.size()!=b.sections.size() ||
     a.collisionEvidence!=b.collisionEvidence ||
     std::abs(a.weightedCostMeters-b.weightedCostMeters)>1e-9)
    throw std::runtime_error("Profiling variant changed deterministic plan");
}
}
int main(int argc,char** argv) {
  try {
    const int repeats=argc>1?std::stoi(argv[1]):7;
    if(repeats<1||repeats>100)throw std::invalid_argument("Repeat count must be 1..100");
    std::cout<<"scenario,variant,profile_timers_on,repeats,p50_ms,p95_ms,"
      "status,expanded,generated,weighted_cost,strict_evidence,"
      "label_finds,label_hits,label_inserts,label_updates,label_rehashes,"
      "label_peak_size,label_peak_buckets,hash_calls,"
      "exact_collision_pose_calls,search_motion_calls,final_motion_calls,"
      "search_pose_checks,final_pose_checks,disc_clear,disc_exact_fallback,"
      "open_pushes,open_pops,open_stale,grid_pushes,grid_pops,grid_stale,"
      "label_lookup_ns,label_write_ns,hash_ns,exact_collision_ns,"
      "search_motion_ns,final_motion_ns,open_queue_ns,grid_queue_ns\n";
    for(const auto& scene:bench::hardCases()) {
      // Run both certified and grid-assisted settings in a future matrix.
      // Use the same difficult-but-recoverable noncertified configuration to
      // isolate data-structure and collision-precheck costs here.
      auto query=scene.request;
      query.options.heuristicMode=HeuristicMode::PerformanceReedsSheppAndObstacleGrid;
      query.options.performanceHeuristicWeight=1.2;
      query.options.gridMaxTotalSettlements=20000;
      query.options.maxGeneratedNodes=60000;
      PlanResult baseline;
      bool first=true;
      for(const auto& v:variants) {
        query.options.searchLabelReserve=v.labelReserve;
        query.options.discPrefilterCount=v.discCount;
        std::vector<double> ms;
        PlanResult last;
        for(int r=0;r<=repeats;++r) {
          const auto start=std::chrono::steady_clock::now();
          auto result=plan(query);
          const auto end=std::chrono::steady_clock::now();
          if(!first)equivalent(baseline,result);
          if(r)ms.push_back(std::chrono::duration<double,std::milli>(end-start).count());
          last=std::move(result);
        }
        if(first){baseline=last;first=false;}
        const auto& d=last.diagnostics;
        const bool timings=d.labelLookupNanoseconds||d.exactCollisionNanoseconds||d.openQueueNanoseconds;
        std::cout<<scene.name<<','<<v.name<<','<<int(timings)<<','<<repeats
          <<','<<std::fixed<<std::setprecision(3)<<quantile(ms,.5)<<','<<quantile(ms,.95)
          <<','<<int(last.status)<<','<<d.expanded<<','<<d.generated<<','
          <<last.weightedCostMeters<<','
          <<int(last.collisionEvidence==CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled)
          <<','<<d.labelFinds<<','<<d.labelFindHits<<','<<d.labelInsertions
          <<','<<d.labelUpdates<<','<<d.labelRehashes<<','<<d.labelPeakSize
          <<','<<d.labelPeakBuckets<<','<<d.hashCalls<<','
          <<d.exactCollisionPoseCalls<<','<<d.searchMotionCalls<<','<<d.finalMotionCalls
          <<','<<d.searchPoseChecks<<','<<d.finalPoseChecks<<','
          <<d.discDefinitelyClear<<','<<d.discExactFallback<<','
          <<d.openQueuePushes<<','<<d.openQueuePops<<','<<d.staleHeapPops<<','
          <<d.gridQueuePushes<<','<<d.gridQueuePops<<','<<d.gridQueueStalePops<<','
          <<d.labelLookupNanoseconds<<','<<d.labelWriteNanoseconds<<','
          <<d.hashNanoseconds<<','<<d.exactCollisionNanoseconds<<','
          <<d.searchMotionNanoseconds<<','<<d.finalMotionNanoseconds<<','
          <<d.openQueueNanoseconds<<','<<d.gridQueueNanoseconds<<'\n';
      }
    }
  } catch(const std::exception& e) {
    std::cerr<<"Hotpath benchmark failure: "<<e.what()<<'\n';return 2;
  }
}
