#pragma once
// Manual, controlled geometry-equivalence benchmark, not part of normal CTest.
// All timed searches and refinements disable collision-work instrumentation.
// Detailed counters and per-pose microtiming are collected separately.
#include "tests/nav2_warehouse_pipeline_benchmark_support.hpp"
#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace hybrid_astar::bench::footprint_envelope {
using namespace hybrid_astar::ackermann;
namespace pipeline=hybrid_astar::bench::nav2_pipeline;
using pipeline::Clock;
using pipeline::Config;
using pipeline::Scenario;
using pipeline::millis;

enum class Variant { LargeRectangle, SmallInflated };
inline const char* label(Variant variant) {
  return variant==Variant::LargeRectangle?"large_rectangle_70x50":
                                         "small_rounded_50x30_margin10";
}
inline PlanRequest configure(PlanRequest request,Variant variant) {
  if(request.options.model!=PlanOptions::Model::DifferentialDrive)
    throw std::invalid_argument("Footprint experiment requires differential drive");
  request.footprint=variant==Variant::LargeRectangle?
      Footprint{-.35,.35,.25}:Footprint{-.25,.25,.15};
  request.options.footprintInflationMeters=
      variant==Variant::LargeRectangle?0.0:0.10;
  // Identical implementation for both: only geometry and inflation differ.
  request.options.useInflatedFootprintBroadPhase=true;
  request.options.collectCollisionWork=false;
  return request;
}
inline double length(const PlanRequest& req) {
  return req.footprint.frontX-req.footprint.rearX;
}
inline double width(const PlanRequest& req) {
  return 2*req.footprint.halfWidth;
}
inline std::vector<Pose2D> fixedPoseBank(const Config& config,
                                        const Scenario& scenario,
                                        const MapSnapshot& map,
                                        std::size_t count) {
  // Sampling is independent of either vehicle footprint, margin or outcome.
  // Exactly the same world-frame poses are given to both checkers.
  pipeline::RandomStream rng(config.seed ^
      (static_cast<std::uint64_t>(scenario.drawIndex)+1)*0xd1342543de82ef95ULL);
  std::vector<Pose2D> poses;
  poses.reserve(count);
  const double xspan=map.width*map.resolutionMeters;
  const double yspan=map.height*map.resolutionMeters;
  constexpr double border=.7;
  if(xspan<=2*border||yspan<=2*border)
    throw std::invalid_argument("Map too small for footprint microbenchmark");
  const double c=std::cos(map.origin.yaw),s=std::sin(map.origin.yaw);
  for(std::size_t i=0;i<count;++i) {
    const double x=border+(xspan-2*border)*rng.unit();
    const double y=border+(yspan-2*border)*rng.unit();
    poses.push_back({map.origin.x+c*x-s*y,map.origin.y+s*x+c*y,
        -std::numbers::pi_v<double>+
        2*std::numbers::pi_v<double>*rng.unit()+map.origin.yaw});
  }
  return poses;
}

struct CollisionMicro {
  double medianUsPerPose=0,p95UsPerPose=0;
  std::size_t poses=0,collisions=0;
  CollisionWorkCounters counters{};
};
inline CollisionMicro benchmarkCollision(const PlanRequest& req,
                                         const std::vector<Pose2D>& poses,
                                         std::size_t repeats) {
  if(poses.empty()||repeats==0)
    throw std::invalid_argument("No microbenchmark poses or repeats");
  // Construct/validate once, outside both timed and instrumented loops.
  ExactCollisionChecker checker(*req.map,req.occupancy,req.footprint,
      req.options.footprintInflationMeters,false,true);
  CollisionMicro out;
  out.poses=poses.size();
  // Work counters are measured once OUTSIDE timing.
  for(const auto& pose:poses)
    if(checker.check(pose,&out.counters)==PoseCollision::Colliding)
      ++out.collisions;
  // Warm up with no counters; prevent dead code elimination via checksum.
  std::uint64_t checksum=0;
  const auto scan=[&]() {
    std::uint64_t hits=0;
    for(const auto& pose:poses)
      hits+=checker.check(pose)==PoseCollision::Colliding;
    return hits;
  };
  checksum^=scan();
  if(checksum!=out.collisions)
    throw std::runtime_error("Microbenchmark collision-work pass changed outcome");
  std::vector<double> times;
  times.reserve(repeats);
  for(std::size_t i=0;i<repeats;++i) {
    const auto start=Clock::now();
    const auto hits=scan();
    const auto stop=Clock::now();
    if(hits!=out.collisions)
      throw std::runtime_error("Non-deterministic microbenchmark collision decisions");
    checksum^=hits+static_cast<std::uint64_t>(i);
    times.push_back(1000.0*millis(start,stop)/
                    static_cast<double>(poses.size()));
  }
  // Observable checksum prevents eliding the collision call loop.
  if(checksum==std::numeric_limits<std::uint64_t>::max())
    std::cerr<<"microbench checksum "<<checksum<<'\n';
  out.medianUsPerPose=pipeline::percentile(times,.5);
  out.p95UsPerPose=pipeline::percentile(times,.95);
  return out;
}

struct Result {
  pipeline::Measurement pipeline;
  CollisionWorkCounters pipelineWork{};
  CollisionMicro micro{};
};
inline Result run(const Config& cfg,const PlanRequest& req,
                  const std::vector<Pose2D>& poses,std::size_t microRepeats) {
  if(req.options.collectCollisionWork)
    throw std::invalid_argument("Detailed collision counters must be untimed");
  Result result;
  // A warmup and each complete repeated search+refinement run happen here.
  result.pipeline=pipeline::measure(cfg,req);

  // Separate untimed full search with collision-work counters. This captures
  // actual planned motion, not synthetic collision workloads. Confirm that
  // instrumentation changes neither the selected route nor search effort.
  auto instrumented=req;
  instrumented.options.collectCollisionWork=true;
  const auto counted=pipeline::planOnce(instrumented,cfg);
  if(!pipeline::equivalentPaths(result.pipeline.raw,counted)||
     counted.diagnostics.expanded!=result.pipeline.raw.diagnostics.expanded||
     counted.diagnostics.generated!=result.pipeline.raw.diagnostics.generated||
     counted.diagnostics.exactCollisionPoseCalls!=
       result.pipeline.raw.diagnostics.exactCollisionPoseCalls)
    throw std::runtime_error("Counter collection changed search result");
  result.pipelineWork=counted.diagnostics.collisionWork;
  result.micro=benchmarkCollision(req,poses,microRepeats);
  return result;
}
inline void printHeader() {
  std::cout
    <<"case_id,draw_index,seed,variant,model,map_cells,resolution_m,grid_cap,"
      "start_x_m,start_y_m,start_yaw_rad,goal_x_m,goal_y_m,goal_yaw_rad,"
      "discovery_expansions,body_length_m,body_width_m,margin_m,"
      "envelope_length_m,envelope_width_m,status,repeats,"
      "search_p50_ms,search_p95_ms,refine_p50_ms,refine_p95_ms,"
      "total_p50_ms,total_p95_ms,expanded,generated,search_pose_checks,"
      "exact_collision_pose_calls,grid_settlements,grid_fallbacks,"
      "raw_length_m,refine_status,refine_attempted,refine_accepted,"
      "refine_exact_pose_checks,refined_length_m,"
      "work_candidate_cells,work_occupied_cells,work_sat_tests,"
      "work_exact_distance_tests,work_aabb_rejections,work_axis_rejections,"
      "micro_poses,micro_collisions,micro_p50_us_per_pose,micro_p95_us_per_pose,"
      "micro_candidate_cells,micro_occupied_cells,micro_sat_tests,"
      "micro_exact_distance_tests,micro_aabb_rejections,micro_axis_rejections\n";
}
inline void printRow(const Config& cfg,const Scenario& scenario,
                     Variant variant,const PlanRequest& request,
                     const Result& result) {
  const auto& m=result.pipeline;
  const auto& d=m.raw.diagnostics;
  const auto& w=result.pipelineWork;
  const auto& x=result.micro;
  const bool refined=m.refinement&&pipeline::strictSuccess(m.refinement->path);
  std::cout<<std::fixed<<std::setprecision(6)
    <<scenario.id<<','<<scenario.drawIndex<<','<<cfg.seed<<','
    <<label(variant)<<','<<cfg.model<<','
    <<request.map->cells.size()<<','<<request.map->resolutionMeters<<','
    <<request.options.gridMaxTotalSettlements<<','
    <<request.start.x<<','<<request.start.y<<','<<request.start.yaw<<','
    <<request.goal.x<<','<<request.goal.y<<','<<request.goal.yaw<<','
    <<scenario.discoveryExpansions<<','
    <<length(request)<<','<<width(request)<<','
    <<request.options.footprintInflationMeters<<','
    <<length(request)+2*request.options.footprintInflationMeters<<','
    <<width(request)+2*request.options.footprintInflationMeters<<','
    <<pipeline::statusName(m.raw.status)<<','<<cfg.repeats<<','
    <<m.searchP50<<','<<m.searchP95<<','
    <<m.refineP50<<','<<m.refineP95<<','
    <<m.totalP50<<','<<m.totalP95<<','
    <<d.expanded<<','<<d.generated<<','<<d.searchPoseChecks<<','
    <<d.exactCollisionPoseCalls<<','<<d.gridSettlements<<','
    <<d.gridFallbacks<<','<<m.raw.geometricLengthMeters<<','
    <<(refined?pipeline::refinerName(m.refinement->status):"NotRun")<<','
    <<(refined?m.refinement->attempted:0)<<','
    <<(refined?m.refinement->accepted:0)<<','
    <<(refined?m.refinement->exactPoseChecks:0)<<','
    <<(refined?m.refinement->path.geometricLengthMeters:0)<<','
    <<w.candidateCells<<','<<w.occupiedCells<<','
    <<w.satTests<<','<<w.distanceTests<<','
    <<w.inflatedAabbRejections<<','<<w.inflatedAxisRejections<<','
    <<x.poses<<','<<x.collisions<<','
    <<x.medianUsPerPose<<','<<x.p95UsPerPose<<','
    <<x.counters.candidateCells<<','<<x.counters.occupiedCells<<','
    <<x.counters.satTests<<','<<x.counters.distanceTests<<','
    <<x.counters.inflatedAabbRejections<<','
    <<x.counters.inflatedAxisRejections<<'\n';
}
} // namespace hybrid_astar::bench::footprint_envelope
