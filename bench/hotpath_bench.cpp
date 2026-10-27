// Manual-only profiling and controlled heuristics A/B. No CTest registration.
// Profiling counters require -DHYBRID_ASTAR_PROFILE_HOTPATHS=ON.
#include "bench/hard_scenarios.hpp"
#include "bench/warehouse_scenarios.hpp"
#include "hybrid_astar/ackermann/planner.hpp"
#include "hybrid_astar/ackermann/cover_prefilter.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
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
                  AckermannCollisionAB, AckermannCollisionWork,
                  AckermannEmptyRegionAB };
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
      "collision_sat_tests,collision_distance_tests,collision_boundary_rejections,"
      "effective_disc_count,empty_region_prefix_enabled,"
      "empty_region_queries,empty_region_bypasses\n";
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
    <<d.collisionWork.boundaryRejections<<','
    <<d.discPrefilterCountUsed<<','<<int(d.emptyRegionPrefixSumUsed)<<','
    <<d.collisionWork.emptyRegionQueries<<','
    <<d.collisionWork.emptyRegionBypasses<<'\n';
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
// Fixed-pose collision microbenchmark. Unlike whole search comparisons,
 // this holds the exact list of pose transforms constant across margins.
 // Prefilter construction and counter-collection are excluded from timed
 // checker loops. A counted verification pass reports work independently.
void collisionMicro(int repeats) {
  const auto cases=bench::ackermannWarehouseCases(1000);
  const auto& request=cases.front().second;
  const auto& map=*request.map;
  std::vector<Pose2D> poses;
  for(std::size_t iy=4;iy+4<map.height;iy+=4)
    for(std::size_t ix=4;ix+4<map.width;ix+=4)
      for(double yaw:{0.,.39269908169872414,.7853981633974483,
                      1.5707963267948966})
        poses.push_back({(ix+.31)*map.resolutionMeters,
                         (iy+.19)*map.resolutionMeters,yaw});
  std::cout<<"benchmark,margin_m,discs,prefix_enabled,poses,repeats,p50_ms,p95_ms,"
      "exact_calls,disc_clear,collisions,timed_collisions_total,"
      "candidate_cells,occupied_cells,sat_tests,distance_tests,"
      "boundary_rejections,empty_region_queries,empty_region_bypasses\n";
  for(double margin:{0.,.5}) {
    // Always compare with the scanner, independent of prefix/prefilter.
    const ExactCollisionChecker oracle(
        map,request.occupancy,request.footprint,margin,false);
    std::vector<PoseCollision> reference;
    reference.reserve(poses.size());
    for(const auto& pose:poses)reference.push_back(oracle.check(pose));
    for(std::size_t discs:{std::size_t{0},std::size_t{4}}) {
      std::unique_ptr<ConservativeDiscPrefilter> prefilter;
      if(discs)prefilter=std::make_unique<ConservativeDiscPrefilter>(
          map,request.occupancy,request.footprint,discs,margin);
      for(bool prefix:{false,true}) {
        // Checker/prefix construction and EDT build are excluded from the
        // micro-timer; whole-plan benchmarks DO include those costs.
        const ExactCollisionChecker checker(
            map,request.occupancy,request.footprint,margin,prefix);
        std::vector<double> ms;
        std::size_t timedCollisions=0;
        for(int run=0;run<repeats;++run) {
          std::size_t collisionCount=0;
          const auto t0=std::chrono::steady_clock::now();
          for(const auto& pose:poses) {
            if(prefilter&&prefilter->definitelyClear(pose))continue;
            collisionCount+=checker.check(pose)==PoseCollision::Colliding;
          }
          const auto t1=std::chrono::steady_clock::now();
          timedCollisions+=collisionCount;
          ms.push_back(std::chrono::duration<double,std::milli>(t1-t0).count());
        }
        CollisionWorkCounters work;
        std::size_t bypassed=0,exactCalls=0,collisions=0;
        for(std::size_t k=0;k<poses.size();++k) {
          if(prefilter&&prefilter->definitelyClear(poses[k])) {
            if(reference[k]!=PoseCollision::Clear)
              throw std::runtime_error("Disc prefilter bypassed a collision");
            ++bypassed;
            continue;
          }
          ++exactCalls;
          const auto verdict=checker.check(poses[k],&work);
          if(verdict!=reference[k])
            throw std::runtime_error("Empty region fast path changed collision");
          collisions+=verdict==PoseCollision::Colliding;
        }
        if(timedCollisions!=collisions*std::size_t(repeats))
          throw std::runtime_error("Timed and counted collision verdicts differed");
        std::cout<<"ackermann_warehouse_fixed_poses,"
                 <<std::fixed<<std::setprecision(3)
                 <<margin<<','<<discs<<','<<int(prefix)<<','
                 <<poses.size()<<','<<repeats<<','
                 <<quantile(ms,.5)<<','<<quantile(ms,.95)<<','
                 <<exactCalls<<','<<bypassed<<','<<collisions<<','
                 <<timedCollisions<<','<<work.candidateCells<<','
                 <<work.occupiedCells<<','<<work.satTests<<','
                 <<work.distanceTests<<','<<work.boundaryRejections<<','
                 <<work.emptyRegionQueries<<','
                 <<work.emptyRegionBypasses<<'\n';
      }
    }
  }
}
}
void ackermannEasyAB(int repeats) {
  // Deliberately simple open-map control case: broad-phase setup may cost
  // more than the subsequent short search. Include it in timed plan().
  auto map=std::make_shared<MapSnapshot>();
  map->width=map->height=100;
  map->resolutionMeters=.25;
  map->cells.assign(map->width*map->height,0);
  map->revision=1;map->frameId="easy-bench";map->sourceId="open";
  PlanRequest request;
  request.map=map;
  request.start={6,6,0};
  request.goal={9,6,0};
  request.options.model=PlanOptions::Model::Ackermann;
  request.options.maxExpansions=20000;
  request.options.maxGeneratedNodes=80000;
  request.options.collectCollisionWork=false;
  csvHeader();
  for(double margin:{0.,.5}) {
    PlanResult reference;
    bool haveReference=false;
    for(std::size_t discs:{std::size_t{0},std::size_t{4}})
      for(bool prefix:{false,true}) {
        auto q=request;
        q.options.footprintInflationMeters=margin;
        q.options.discPrefilterCount=discs;
        q.options.useEmptyRegionPrefixSum=prefix;
        std::vector<double> times;
        PlanResult last;
        for(int run=0;run<repeats;++run) {
          const auto t0=std::chrono::steady_clock::now();
          auto result=plan(q);
          const auto t1=std::chrono::steady_clock::now();
          if(result.status!=PlanStatus::Success)
            throw std::runtime_error("Open Ackermann control route not solved");
          if(haveReference)equivalent(reference,result);
          times.push_back(std::chrono::duration<double,std::milli>(t1-t0).count());
          last=std::move(result);
        }
        if(!haveReference) {
          reference=last;
          haveReference=true;
        }
        const std::string variant=std::string(prefix?"prefix1":"prefix0")+
            (discs?"_disc4":"_disc0")+
            (margin>0?"_margin050":"_margin000");
        csvRow({"ackermann_easy_open",request},variant.c_str(),
               q.options,repeats,times,last);
      }
  }
}
int main(int argc,char** argv) {
  try {
    if(argc>1&&std::string(argv[1])=="--ackermann-easy-ab") {
      if(argc>3)throw std::invalid_argument("Usage: --ackermann-easy-ab [repeats]");
      ackermannEasyAB(argc==3?number(argv[2],1,100):7);
      return 0;
    }
    if(argc>1&&std::string(argv[1])=="--ackermann-collision-micro") {
      if(argc>3)throw std::invalid_argument(
          "Usage: --ackermann-collision-micro [repeats]");
      const int repeats=argc==3?number(argv[2],1,100):5;
      collisionMicro(repeats);
      return 0;
    }
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
      else if(flag=="--ackermann-empty-region-ab")
        mode=Mode::AckermannEmptyRegionAB;
      else throw std::invalid_argument("Unknown benchmark mode");
    }
    const bool warehouse=mode==Mode::Warehouse||mode==Mode::WarehouseHeuristics;
    const bool collisionMatrix=mode==Mode::AckermannCollisionAB||
                               mode==Mode::AckermannCollisionWork||
                               mode==Mode::AckermannEmptyRegionAB;
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
          "--ackermann-collision-(ab|work) [repeats] [budget] [case_name] | "
          "--ackermann-empty-region-ab [repeats] [budget] [case_name]");
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
      if(mode==Mode::AckermannEmptyRegionAB) {
        // Compare integral-image bypass OFF vs ON for fixed disc and
        // margin settings. The original grid scanner is the oracle.
        for(double margin:{0.,.5})
          for(std::size_t discs:{std::size_t{0},std::size_t{4}}) {
            PlanResult baseline;
            for(bool prefix:{false,true}) {
              auto q=scene.request;
              q.options.heuristicMode=
                  HeuristicMode::PerformanceReedsSheppAndObstacleGrid;
              q.options.footprintInflationMeters=margin;
              q.options.discPrefilterCount=discs;
              q.options.useEmptyRegionPrefixSum=prefix;
              // Work counts in this mode are intentional; compare timings
              // with prefix-off while counters stay identically enabled.
              q.options.collectCollisionWork=true;
              const std::string label=std::string(prefix?"prefix1":"prefix0")+
                  (discs?"_disc4":"_disc0")+
                  (margin>0?"_margin050":"_margin000");
              auto outcome=measure(scene,q,label.c_str(),repeats,false);
              if(!prefix)baseline=std::move(outcome);
              else {
                equivalent(baseline,outcome);
                if(outcome.diagnostics.collisionWork.candidateCells>
                   baseline.diagnostics.collisionWork.candidateCells)
                  throw std::runtime_error("Integral AABB increased cell scans");
              }
            }
          }
        continue;
      }
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
              q.options.useEmptyRegionPrefixSum=false; // historical A/B
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
          q.options.useEmptyRegionPrefixSum=false; // historical heuristic A/B
          q.options.searchLabelReserve=0;
          measure(scene,q,heuristicName(heuristic),repeats,!largeCase);
        }
        continue;
      }
      auto q=scene.request;
      q.options.useEmptyRegionPrefixSum=false; // preserve prior variant baseline
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
