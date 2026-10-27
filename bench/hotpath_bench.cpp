// Manual-only profiling and controlled heuristics A/B. No CTest registration.
// Profiling counters require -DHYBRID_ASTAR_PROFILE_HOTPATHS=ON.
#include "bench/hard_scenarios.hpp"
#include "bench/warehouse_scenarios.hpp"
#include "hybrid_astar/ackermann/planner.hpp"
#include <algorithm>
#include <array>
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
  std::size_t labelReserve,discCount;
};
constexpr std::array<Variant,5> variants{{
    {"baseline",0,0},{"reserve_4096",4096,0},{"reserve_16384",16384,0},
    {"disc_4",0,4},{"reserve_4096_disc_4",4096,4}}};
enum class Mode { Baseline, Warehouse, Heuristics, WarehouseHeuristics,
                  AckermannWarehouse, AckermannWarehouseHeuristics,
                  AckermannCollisionAB, AckermannCollisionWork };
struct Scenario {
  std::string name;
  PlanRequest request;
};
double quantile(std::vector<double> v,double p) {
  std::sort(v.begin(),v.end());
  const auto rank=std::max<std::size_t>(
      1,static_cast<std::size_t>(std::ceil(v.size()*p)));
  return v.at(rank-1);
}
void equivalent(const PlanResult& a,const PlanResult& b) {
  if(a.status!=b.status || a.diagnostics.expanded!=b.diagnostics.expanded ||
     a.diagnostics.generated!=b.diagnostics.generated ||
     a.mapRevision!=b.mapRevision || a.sections.size()!=b.sections.size() ||
     a.collisionEvidence!=b.collisionEvidence ||
     std::abs(a.weightedCostMeters-b.weightedCostMeters)>1e-9)
    throw std::runtime_error("Prefilter/reserve variant changed deterministic plan");
}
const char* modelName(PlanOptions::Model model) {
  switch(model) {
    case PlanOptions::Model::Ackermann:return "ackermann";
    case PlanOptions::Model::DifferentialDrive:return "differential";
    case PlanOptions::Model::Mecanum:return "mecanum";
  }
  return "unknown";
}
const char* heuristicName(HeuristicMode mode) {
  return mode==HeuristicMode::CertifiedRegionEuclidean?
      "certified_euclidean":"obstacle_grid";
}
int number(const char* arg,int min,int max) {
  const int value=std::stoi(arg);
  if(value<min||value>max)throw std::invalid_argument("Invalid repeats/budget argument");
  return value;
}
void csvHeader() {
  std::cout<<"scenario,variant,profile_timers_on,repeats,p50_ms,p95_ms,"
      "status,expanded,generated,weighted_cost,strict_evidence,"
      "label_finds,label_hits,label_inserts,label_updates,label_rehashes,"
      "label_peak_size,label_peak_buckets,hash_calls,"
      "exact_collision_pose_calls,search_motion_calls,final_motion_calls,"
      "search_pose_checks,final_pose_checks,disc_clear,disc_exact_fallback,"
      "open_pushes,open_pops,open_stale,grid_pushes,grid_pops,grid_stale,"
      "label_lookup_ns,label_write_ns,hash_ns,exact_collision_ns,"
      "search_motion_ns,final_motion_ns,open_queue_ns,grid_queue_ns,"
      "model,heuristic,max_expansions,grid_settlements,grid_fallbacks,"
      "terminal_state_attempts,terminal_candidates,terminal_validated,terminal_rejected,"
      "start_x,start_y,start_yaw,goal_x,goal_y,goal_yaw,footprint_margin_m,"
      "reopened,pruned_by_key,strict_goal_candidates,strict_rejected_collision,"
      "label_update_count,open_peak_size,open_peak_storage_bytes,"
      "grid_peak_size,grid_peak_storage_bytes,"
      "collision_work_enabled,collision_candidate_cells,collision_occupied_cells,"
      "collision_sat_tests,collision_distance_tests,collision_boundary_rejections\n";
}
void csvRow(const Scenario& scene,const char* variant,const PlanOptions& options,
            int repeats,const std::vector<double>& times,const PlanResult& last) {
  const auto& d=last.diagnostics;
  const bool timings=d.labelLookupNanoseconds||d.exactCollisionNanoseconds||d.openQueueNanoseconds;
  std::cout<<scene.name<<','<<variant<<','<<int(timings)<<','<<repeats
    <<','<<std::fixed<<std::setprecision(3)<<quantile(times,.5)<<','<<quantile(times,.95)
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
    <<d.openQueueNanoseconds<<','<<d.gridQueueNanoseconds<<','
    <<modelName(options.model)<<','<<heuristicName(options.heuristicMode)<<','
    <<options.maxExpansions<<','<<d.gridSettlements<<','<<d.gridFallbacks<<','
    <<d.terminalConnectionAttempts<<','<<d.terminalCandidates<<','
    <<d.terminalValidated<<','<<d.terminalRejected<<','
    <<scene.request.start.x<<','<<scene.request.start.y<<','
    <<scene.request.start.yaw<<','<<scene.request.goal.x<<','
    <<scene.request.goal.y<<','<<scene.request.goal.yaw<<','
    <<options.footprintInflationMeters<<','
    <<d.reopened<<','<<d.prunedByKey<<','<<d.strictGoalCandidates<<','
    <<d.strictRejectedCollision<<','<<d.labelUpdates<<','
    <<d.openQueuePeakSize<<','<<d.openQueuePeakStorageBytes<<','
    <<d.gridQueuePeakSize<<','<<d.gridQueuePeakStorageBytes<<','
    <<int(d.collectCollisionWork)<<','<<d.collisionWork.candidateCells<<','
    <<d.collisionWork.occupiedCells<<','<<d.collisionWork.satTests<<','
    <<d.collisionWork.distanceTests<<','
    <<d.collisionWork.boundaryRejections<<'\n';
}
PlanResult measure(const Scenario& scene,PlanRequest q,const char* variant,int repeats,
                   bool warmup) {
  std::vector<double> ms;
  PlanResult last;
  for(int run=0;run<repeats+(warmup?1:0);++run) {
    const auto start=std::chrono::steady_clock::now();
    auto result=plan(q);
    const auto end=std::chrono::steady_clock::now();
    if(!warmup||run)ms.push_back(std::chrono::duration<double,std::milli>(end-start).count());
    last=std::move(result);
  }
  csvRow(scene,variant,q.options,repeats,ms,last);
  return last;
}
}
int main(int argc,char** argv) {
  try {
    Mode mode=Mode::Baseline;
    int first=1;
    if(argc>1 && argv[1][0]=='-') {
      const std::string flag=argv[1];first=2;
      if(flag=="--warehouse")mode=Mode::Warehouse;
      else if(flag=="--heuristics")mode=Mode::Heuristics;
      else if(flag=="--warehouse-heuristics")mode=Mode::WarehouseHeuristics;
      else if(flag=="--ackermann-warehouse")mode=Mode::AckermannWarehouse;
      else if(flag=="--ackermann-warehouse-heuristics")
        mode=Mode::AckermannWarehouseHeuristics;
      else if(flag=="--ackermann-collision-ab")
        mode=Mode::AckermannCollisionAB;
      else if(flag=="--ackermann-collision-work")
        mode=Mode::AckermannCollisionWork;
      else throw std::invalid_argument("Unknown benchmark mode");
    }
    const bool warehouse=mode==Mode::Warehouse||mode==Mode::WarehouseHeuristics;
    const bool collisionMatrix=mode==Mode::AckermannCollisionAB||
                               mode==Mode::AckermannCollisionWork;
    const bool ackermannWarehouse=mode==Mode::AckermannWarehouse||
                                  mode==Mode::AckermannWarehouseHeuristics||
                                  collisionMatrix;
    const bool largeCase=warehouse||ackermannWarehouse;
    const bool heuristicAB=mode==Mode::Heuristics||
        mode==Mode::WarehouseHeuristics||mode==Mode::AckermannWarehouseHeuristics;
    const int repeats=argc>first?number(argv[first],1,100):(largeCase?1:7);
    const int budget=largeCase && argc>first+1?
        number(argv[first+1],100,1000000):1000000;
    const std::string filter=ackermannWarehouse&&argc>first+2?argv[first+2]:"";
    if(argc>first+1+int(largeCase)+int(ackermannWarehouse))
      throw std::invalid_argument(
          "Usage: [repeats] | --warehouse [repeats] [budget] | "
          "--heuristics [repeats] | --warehouse-heuristics [repeats] [budget] | "
          "--ackermann-warehouse[(-heuristics)] [repeats] [budget] [case_name] | "
          "--ackermann-collision-(ab|work) [repeats] [budget] [case_name]");
    std::vector<Scenario> scenes;
    if(warehouse) scenes.push_back({"mecanum_warehouse_off_grid",
                                     bench::mecanumWarehouseCase(std::size_t(budget))});
    else if(ackermannWarehouse) {
      for(auto&& [name,request]:
          bench::ackermannWarehouseCases(std::size_t(budget))) {
        // Default collision matrix profiles two distinct Ackermann routes,
        // not all five closely related margin/improvement variations.
        const bool selected=filter.empty()?
            (!collisionMatrix||name=="ack_across_first_margin_050"||
             name=="ack_corner_margin_000"):
            filter==name;
        if(selected)
          scenes.push_back({std::move(name),std::move(request)});
      }
      if(scenes.empty())
        throw std::invalid_argument("Unknown Ackermann warehouse case name");
    } else for(auto&& s:bench::hardCases())
      scenes.push_back({std::move(s.name),std::move(s.request)});
    csvHeader();
    for(const auto& scene:scenes) {
      if(collisionMatrix) {
        // Full factorial: identical warehouse request for each heuristic,
        // margin and disc-count pair. The AB mode is the unbiased wall-clock
        // measurement; "work" separately counts grid visits and SAT/distance
        // tests, which necessarily adds instrumentation overhead.
        for(const auto heuristic:{HeuristicMode::CertifiedRegionEuclidean,
                                  HeuristicMode::PerformanceReedsSheppAndObstacleGrid})
          for(const double margin:{0.,.5}) {
            PlanResult baseline;
            for(const std::size_t discCount:{std::size_t{0},std::size_t{4}}) {
              auto q=scene.request;
              q.options.heuristicMode=heuristic;
              q.options.performanceHeuristicWeight=1.0;
              q.options.footprintInflationMeters=margin;
              q.options.discPrefilterCount=discCount;
              q.options.searchLabelReserve=0;
              q.options.collectCollisionWork=mode==Mode::AckermannCollisionWork;
              const std::string label=std::string(heuristicName(heuristic))+
                  (discCount?"_disc4":"_disc0")+
                  (margin>0?"_margin050":"_margin000");
              auto outcome=measure(scene,q,label.c_str(),repeats,false);
              if(discCount==0)baseline=std::move(outcome);
              else {
                equivalent(baseline,outcome);
                if(outcome.diagnostics.exactCollisionPoseCalls>
                   baseline.diagnostics.exactCollisionPoseCalls)
                  throw std::runtime_error("Cover increased exact collision calls");
              }
            }
          }
        continue;
      }
      // A/B keeps the request, map, budget, model, collision settings, and
      // noncertified heuristic weight (1.0) identical apart from heuristic mode.
      // Different heuristic ordering may legitimately yield different paths.
      if(heuristicAB) {
        for(auto heuristic:{HeuristicMode::CertifiedRegionEuclidean,
                            HeuristicMode::PerformanceReedsSheppAndObstacleGrid}) {
          auto q=scene.request;
          q.options.heuristicMode=heuristic;
          q.options.performanceHeuristicWeight=1.0;
          q.options.discPrefilterCount=0;
          q.options.searchLabelReserve=0;
          measure(scene,q,heuristicName(heuristic),repeats,!largeCase);
        }
        continue;
      }
      auto q=scene.request;
      if(!largeCase) {
        // Historical five-way baseline for comparison to existing CSV runs.
        q.options.heuristicMode=HeuristicMode::PerformanceReedsSheppAndObstacleGrid;
        q.options.performanceHeuristicWeight=1.2;
        q.options.gridMaxTotalSettlements=20000;
        q.options.maxGeneratedNodes=60000;
      }
      PlanResult reference;
      bool hasReference=false;
      const std::size_t count=largeCase?3:variants.size();
      for(std::size_t i=0;i<count;++i) {
        // Warehouse tests baseline, conservative discs and discs+reserve.
        const auto& v=largeCase?
            (i==0?variants[0]:i==1?variants[3]:variants[4]):variants[i];
        q.options.searchLabelReserve=v.labelReserve;
        q.options.discPrefilterCount=v.discCount;
        std::vector<double> ms;
        PlanResult last;
        for(int run=0;run<repeats+(largeCase?0:1);++run) {
          const auto start=std::chrono::steady_clock::now();
          auto result=plan(q);
          const auto end=std::chrono::steady_clock::now();
          if(hasReference)equivalent(reference,result);
          if(largeCase||run)ms.push_back(
              std::chrono::duration<double,std::milli>(end-start).count());
          last=std::move(result);
        }
        if(!hasReference){reference=last;hasReference=true;}
        csvRow(scene,v.name,q.options,repeats,ms,last);
      }
    }
  } catch(const std::exception& error) {
    std::cerr<<"Hotpath benchmark failure: "<<error.what()<<'\n';return 2;
  }
}
