// Manual full-resolution Nav2 prefix-sum OFF/ON benchmark (not CTest).
#include "tests/nav2_footprint_envelope_benchmark_support.hpp"
#include <array>
#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>
namespace fe=hybrid_astar::bench::footprint_envelope;
using namespace hybrid_astar::ackermann;
namespace {
namespace pipe=hybrid_astar::bench::nav2_pipeline;
struct Micro {
  double ctorP50Ms=0,poseP50Us=0;
  std::size_t collisions=0;
  CollisionWorkCounters work;
};
struct Result {
  pipe::Measurement pipeline;
  CollisionWorkCounters work;
  Micro micro;
};
Micro measureMicro(const PlanRequest& req,
                   const std::vector<hybrid_astar::Pose2D>& poses,
                   std::size_t repeats) {
  Micro out;
  std::vector<double> ctor;
  std::unique_ptr<ExactCollisionChecker> checker;
  for(std::size_t i=0;i<=repeats;++i) {
    const auto begin=pipe::Clock::now();
    auto instance=std::make_unique<ExactCollisionChecker>(
        *req.map,req.occupancy,req.footprint,
        req.options.footprintInflationMeters,
        req.options.useEmptyRegionPrefixSum,
        req.options.useInflatedFootprintBroadPhase);
    const auto end=pipe::Clock::now();
    if(i==0)checker=std::move(instance);
    else ctor.push_back(pipe::millis(begin,end));
  }
  // Both configurations validate the map; only ON builds the prefix table.
  out.ctorP50Ms=pipe::percentile(ctor,.5);
  for(const auto& pose:poses)
    out.collisions+=checker->check(pose,&out.work)==PoseCollision::Colliding;
  const auto checkAll=[&]() {
    std::size_t hits=0;
    for(const auto& pose:poses)
      hits+=checker->check(pose)==PoseCollision::Colliding;
    return hits;
  };
  if(checkAll()!=out.collisions)
    throw std::runtime_error("Collision-work counters changed results");
  std::vector<double> micro;
  for(std::size_t i=0;i<repeats;++i) {
    const auto begin=pipe::Clock::now();
    const auto hits=checkAll();
    const auto end=pipe::Clock::now();
    if(hits!=out.collisions)
      throw std::runtime_error("Non-deterministic collision microbenchmark");
    micro.push_back(1000*pipe::millis(begin,end)/poses.size());
  }
  out.poseP50Us=pipe::percentile(micro,.5);
  return out;
}
Result measure(const pipe::Config& cfg,const PlanRequest& req,
               const std::vector<hybrid_astar::Pose2D>& poses,
               std::size_t repeats) {
  Result result;
  result.pipeline=pipe::measure(cfg,req);
  auto instrumented=req;
  instrumented.options.collectCollisionWork=true;
  const auto counted=pipe::planOnce(instrumented,cfg);
  if(!pipe::equivalentPaths(counted,result.pipeline.raw) ||
     counted.diagnostics.expanded!=result.pipeline.raw.diagnostics.expanded ||
     counted.diagnostics.exactCollisionPoseCalls!=
        result.pipeline.raw.diagnostics.exactCollisionPoseCalls)
    throw std::runtime_error("Counter collection changed planning");
  result.work=counted.diagnostics.collisionWork;
  result.micro=measureMicro(req,poses,repeats);
  return result;
}
void printRow(const pipe::Config& cfg,const pipe::Scenario& scenario,
              fe::Variant geometry,bool prefix,const PlanRequest& req,
              const Result& r,std::size_t poses) {
  const auto& d=r.pipeline.raw.diagnostics;
  const auto& w=r.work;
  const auto& m=r.micro;
  const auto& t=d.collisionTiming;
#ifdef HYBRID_ASTAR_NAV2_PIPELINE_PROFILE_ENABLED
  constexpr int profiling=1;
  if(t.emptyBypassCalls+t.nonemptyFallbackCalls+t.boundaryCalls!=
     d.exactCollisionPoseCalls)
    throw std::runtime_error("Collision outcome timing missed an exact pose call");
#else
  constexpr int profiling=0;
#endif
  std::cout<<std::fixed<<std::setprecision(6)
      <<scenario.id<<','<<scenario.drawIndex<<','<<cfg.seed<<','
      <<cfg.model<<','<<fe::label(geometry)<<','<<(prefix?1:0)<<','
      <<req.options.footprintInflationMeters<<','<<cfg.gridSettlements<<','
      <<pipe::statusName(r.pipeline.raw.status)<<','<<cfg.repeats<<','
      <<r.pipeline.searchP50<<','<<r.pipeline.refineP50<<','
      <<r.pipeline.totalP50<<','
      <<d.expanded<<','<<d.generated<<','<<d.exactCollisionPoseCalls<<','
      <<m.ctorP50Ms<<','<<poses<<','<<m.collisions<<','<<m.poseP50Us<<','
      <<w.candidateCells<<','<<w.occupiedCells<<','<<w.satTests<<','
      <<w.distanceTests<<','<<w.emptyRegionQueries<<','
      <<w.emptyRegionBypasses<<','
      <<m.work.candidateCells<<','<<m.work.occupiedCells<<','
      <<m.work.emptyRegionQueries<<','<<m.work.emptyRegionBypasses<<','
      <<profiling<<','<<t.emptyBypassCalls<<','<<t.nonemptyFallbackCalls<<','
      <<t.boundaryCalls<<','
      <<pipe::nsMs(t.emptyBypassNanoseconds)<<','
      <<pipe::nsMs(t.nonemptyFallbackNanoseconds)<<','
      <<pipe::nsMs(t.boundaryNanoseconds)<<'\n';
}
}
int main(int argc,char** argv) {
  namespace pipe=hybrid_astar::bench::nav2_pipeline;
  try {
    std::size_t microPoses=20000,microRepeats=5;
    std::vector<char*> forwarded{argv[0]};
    for(int i=1;i<argc;++i) {
      const std::string a=argv[i];
      if(a=="--micro-poses"||a=="--micro-repeats") {
        if(i+1>=argc)throw std::invalid_argument("Missing value for "+a);
        const auto n=pipe::integer(argv[++i]);
        if(a=="--micro-poses")microPoses=n;else microRepeats=n;
      } else forwarded.push_back(argv[i]);
    }
    if(microPoses<100||microPoses>1000000||microRepeats>100)
      throw std::invalid_argument("Micro poses 100..1000000; repeats 1..100");
    const auto cfg=pipe::parseArgs(
        static_cast<int>(forwarded.size()),forwarded.data());
    if((cfg.model!="differential"&&cfg.model!="mecanum")||
       cfg.connector!="native"||cfg.collectCollisionWork)
      throw std::invalid_argument("Use differential/mecanum, native, no work counters");
    const auto map=hybrid_astar::demo::loadRosMapYaml(cfg.yamlPath);
    const auto base=pipe::baseRequest(cfg,map);
    const auto cases=cfg.replayCsv.empty()?
        pipe::discover(cfg,base):pipe::replayScenarios(cfg,base);
    std::cerr<<"Prefix sum OFF/ON: "<<cases.size()<<" routes; search includes build cost\n";
    std::cout<<"case_id,draw_index,seed,model,geometry,prefix_enabled,margin_m,"
                "grid_settlements,status,repeats,search_p50_ms,refine_p50_ms,"
                "total_p50_ms,expanded,generated,exact_pose_calls,"
                "checker_construction_p50_ms,micro_poses,micro_collisions,"
                "micro_p50_us_per_pose,search_candidate_cells,search_occupied_cells,"
                "search_sat_tests,search_distance_tests,search_prefix_queries,"
                "search_prefix_bypasses,micro_candidate_cells,micro_occupied_cells,"
                "micro_prefix_queries,micro_prefix_bypasses,"
                "profile_enabled,collision_empty_bypass_calls,"
                "collision_nonempty_fallback_calls,collision_boundary_calls,"
                "collision_empty_bypass_ms,collision_nonempty_fallback_ms,"
                "collision_boundary_ms\n";
    for(const auto& scenario:cases) {
      const auto poses=fe::fixedPoseBank(cfg,scenario,*map,microPoses);
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
        req.options.useInflatedFootprintBroadPhase=true;
        std::optional<Result> off,on;
        for(bool prefix:scenario.id%2?
             std::array{true,false}:std::array{false,true}) {
          req.options.useEmptyRegionPrefixSum=prefix;
          auto r=measure(cfg,req,poses,microRepeats);
          if(prefix)on=std::move(r);else off=std::move(r);
        }
        if(!off||!on||
           !pipe::equivalentMeasurements(off->pipeline,on->pipeline)||
           off->micro.collisions!=on->micro.collisions)
          throw std::runtime_error("Prefix ON/OFF altered plan or collision result");
        req.options.useEmptyRegionPrefixSum=false;
        printRow(cfg,scenario,geometry,false,req,*off,poses.size());
        req.options.useEmptyRegionPrefixSum=true;
        printRow(cfg,scenario,geometry,true,req,*on,poses.size());
        std::cerr<<"Case "<<scenario.id<<' '<<fe::label(geometry)
                 <<" search off/on "<<off->pipeline.searchP50<<'/'
                 <<on->pipeline.searchP50<<" ms; micro off/on "
                 <<off->micro.poseP50Us<<'/'<<on->micro.poseP50Us<<" us/pose\n";
      }
    }
    return 0;
  } catch(const std::exception& ex) {
    std::cerr<<"Nav2 prefix A/B failed: "<<ex.what()<<'\n';
    return 2;
  }
}
