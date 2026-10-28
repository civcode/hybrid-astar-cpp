#pragma once
// Shared deterministic, opt-in benchmark harness; no performance threshold is
// asserted and these workloads are not part of normal CTest runs.
#include "bench/warehouse_scenarios.hpp"
#include "hybrid_astar/ackermann/refinement.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace hybrid_astar::bench::ackermann_refinement {
using namespace hybrid_astar::ackermann;

struct Config {
  std::size_t repeats=5;
  std::size_t minimumExpansions=1000;
  std::size_t maxCases=3;
  std::size_t searchBudget=120000;
};
inline Config parseArgs(int argc,char** argv) {
  if(argc>5)throw std::invalid_argument(
      "Usage: BENCH [repeats=5] [minimum_expansions=1000] [max_cases=3] [search_budget=120000]");
  auto read=[&](int index,std::size_t fallback) {
    if(argc<=index)return fallback;
    const std::string text=argv[index];
    if(text.empty()||text[0]=='-')throw std::invalid_argument("Expected a positive integer");
    std::size_t consumed=0;
    const auto parsed=std::stoull(text,&consumed);
    if(consumed!=text.size())throw std::invalid_argument("Expected an integer");
    return static_cast<std::size_t>(parsed);
  };
  Config c;
  c.repeats=read(1,c.repeats);
  c.minimumExpansions=read(2,c.minimumExpansions);
  c.maxCases=read(3,c.maxCases);
  c.searchBudget=read(4,c.searchBudget);
  if(c.repeats==0||c.repeats>100||c.maxCases==0||c.maxCases>8||
     c.minimumExpansions==0||c.minimumExpansions>1000000||
     c.searchBudget<100||c.searchBudget>1000000||
     c.minimumExpansions>c.searchBudget)
    throw std::invalid_argument("Out-of-range benchmark arguments");
  return c;
}
struct Scenario {
  std::string name;
  PlanRequest request;
  PlanResult original;
};

// Follow the hard-scenario/warehouse benchmark pattern: search a deterministic
// list of candidate pose pairs, retain ONLY successful, strictly verified
// planning problems requiring at least the requested number of expansions.
// The first pose is the reported Planning Lab 7k-expansion Ackermann case.
// Search/selection is done ONCE and excluded from timed refinement measurements.
inline std::vector<Scenario> discover(const Config& config) {
  const auto pi=std::numbers::pi_v<double>;
  struct Candidate {const char* name;Pose2D start,goal;};
  const std::array<Candidate,8> candidates{{
    {"exported_warehouse",{4.92,6.68,0},{46.75,31.86,.5*pi}},
    {"warehouse_nearby_start",{4.92,8.68,0},{46.75,31.86,.5*pi}},
    {"warehouse_nearby_goal",{4.92,6.68,0},{45.5,30.5,.5*pi}},
    {"warehouse_shifted_start",{5.5,6.68,0},{46.75,31.86,.5*pi}},
    {"warehouse_nearby_goal_2",{4.92,6.68,0},{44.5,28.5,.5*pi}},
    {"warehouse_upper_left",{4.77,32.84,0},{44.62,33.53,-.5*pi}},
    {"warehouse_left_center",{4.77,20,0},{43,20,0}},
    {"warehouse_middle_left",{5.75,11.75,0},{45.5,31,.5*pi}}
  }};
  auto common=mecanumWarehouseCase(config.searchBudget);
  common.options.model=PlanOptions::Model::Ackermann;
  common.footprint=PlanRequest{}.footprint;
  common.options.footprintInflationMeters=.5;
  common.options.heuristicMode=HeuristicMode::PerformanceReedsSheppAndObstacleGrid;
  common.options.maxGeneratedNodes=4*config.searchBudget;
  std::vector<Scenario> selected;
  for(const auto& candidate:candidates) {
    auto request=common;
    request.start=candidate.start;request.goal=candidate.goal;
    auto original=plan(request);
    if(original.status==PlanStatus::Success &&
       original.modelGeometryValid &&
       original.collisionEvidence==CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled &&
       original.diagnostics.expanded>=config.minimumExpansions) {
      selected.push_back({candidate.name,std::move(request),std::move(original)});
      if(selected.size()>=config.maxCases)break;
    }
  }
  if(selected.empty())
    throw std::runtime_error("No validated warehouse route met minimum_expansions; "
                             "raise search_budget or lower minimum_expansions");
  std::cerr<<"Selected "<<selected.size()<<" validated Ackermann routes (min "
           <<config.minimumExpansions<<" expansions); search excluded from timings\n";
  return selected;
}

struct Smoothness {
  double totalVariation=0;
  double maxJump=0;
  std::size_t nonzeroDirectionReversals=0;
};
inline Smoothness measure(const std::vector<MotionSection>& sections) {
  Smoothness s;
  int previousNonzeroSign=0;
  for(std::size_t i=0;i<sections.size();++i) {
    const auto& sec=sections[i];
    if(sec.kind!=MotionKind::Translate)continue;
    if(i>0&&sections[i-1].kind==MotionKind::Translate) {
      const auto jump=std::abs(sec.curvature-sections[i-1].curvature);
      s.totalVariation+=jump;
      s.maxJump=std::max(s.maxJump,jump);
    }
    const int sign=sec.curvature>1e-9?1:sec.curvature< -1e-9?-1:0;
    if(sign && previousNonzeroSign && sign!=previousNonzeroSign)
      ++s.nonzeroDirectionReversals;
    if(sign)previousNonzeroSign=sign;
  }
  return s;
}
inline double percentile(std::vector<double> values,double p) {
  std::sort(values.begin(),values.end());
  const auto n=std::max<std::size_t>(1,
      static_cast<std::size_t>(std::ceil(values.size()*p)));
  return values[std::min(values.size()-1,n-1)];
}
struct Variant {
  const char* name;
  std::size_t maxAttempts, maxAccepted, maxTotalPoseChecks;
  double curvatureVariationWeightMetersSquared;
};
struct Measurement {
  RefinementResult result;
  double p50Millis=0,p95Millis=0;
};
inline Measurement run(const Scenario& scenario,const Config& config,
                       const Variant& variant) {
  RefinementOptions options;
  options.maxAttempts=variant.maxAttempts;
  options.maxAccepted=variant.maxAccepted;
  options.maxTotalPoseChecks=variant.maxTotalPoseChecks; // match actual demo profile
  options.useCurvatureBoundedShortcuts=true; // same native CSC generator
  options.curvatureVariationWeightMetersSquared=
      variant.curvatureVariationWeightMetersSquared;
  const auto originalSmoothness=measure(scenario.original.sections);
  const double initialObjective=scenario.original.weightedCostMeters+
      variant.curvatureVariationWeightMetersSquared*originalSmoothness.totalVariation;
  // Warm up this profile once (unmeasured) to reduce first-use allocator and
  // instruction-cache bias; timed runs all refine the unchanged raw path.
  const auto warmup=refinePath(scenario.request,scenario.original,{},options);
  if(warmup.path.status!=PlanStatus::Success ||
     warmup.path.weightedCostMeters>initialObjective+1e-6)
    throw std::runtime_error(std::string("Invalid warmup refinement: ")+variant.name);
  std::vector<double> timings;
  RefinementResult last;
  for(std::size_t i=0;i<config.repeats;++i) {
    const auto start=std::chrono::steady_clock::now();
    auto result=refinePath(scenario.request,scenario.original,{},options);
    const auto end=std::chrono::steady_clock::now();
    timings.push_back(std::chrono::duration<double,std::milli>(end-start).count());
    if(result.path.status!=PlanStatus::Success ||
       result.path.collisionEvidence!=CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled ||
       !result.path.modelGeometryValid ||
       result.path.mapRevision!=scenario.request.map->revision ||
       result.path.weightedCostMeters>initialObjective+1e-6)
      throw std::runtime_error(std::string("Invalid or cost-regressing refinement: ")+variant.name);
    if(i>0&&(result.path.sections.size()!=last.path.sections.size()||
             std::abs(result.path.weightedCostMeters-last.path.weightedCostMeters)>1e-8))
      throw std::runtime_error("Non-deterministic refinement output across repeats");
    last=std::move(result);
  }
  return {std::move(last),percentile(timings,.5),percentile(timings,.95)};
}
inline void printHeader() {
  std::cout<<"scenario,variant,search_expanded,raw_sections,raw_length_m,"
      "raw_curvature_tv_inv_m,raw_steering_reversals,max_attempts,max_accepted,max_pose_checks,"
      "curvature_weight_m2,attempted,accepted,rejected,cost_pruned,strict_checks,"
      "refine_status,refined_sections,refined_length_m,refined_curvature_tv_inv_m,"
      "refined_max_curvature_jump_inv_m,refined_steering_reversals,"
      "refined_objective_m,repeats,refine_p50_ms,refine_p95_ms\n";
}
inline void printRow(const Scenario& scenario,const Config& config,
                     const Variant& variant,const Measurement& measurement) {
  const auto& out=measurement.result;
  const auto originalSmoothness=measure(scenario.original.sections);
  const auto smoothness=measure(out.path.sections);
  std::cout<<scenario.name<<','<<variant.name<<','
      <<scenario.original.diagnostics.expanded<<','
      <<scenario.original.sections.size()<<','
      <<std::fixed<<std::setprecision(6)<<scenario.original.geometricLengthMeters<<','
      <<originalSmoothness.totalVariation<<','
      <<originalSmoothness.nonzeroDirectionReversals<<','
      <<variant.maxAttempts<<','<<variant.maxAccepted<<','<<variant.maxTotalPoseChecks<<','
      <<variant.curvatureVariationWeightMetersSquared<<','
      <<out.attempted<<','<<out.accepted<<','<<out.rejected<<','
      <<out.prunedByCostLowerBound<<','<<out.exactPoseChecks<<','
      <<static_cast<int>(out.status)<<','
      <<out.path.sections.size()<<','<<out.path.geometricLengthMeters<<','
      <<smoothness.totalVariation<<','<<smoothness.maxJump<<','
      <<smoothness.nonzeroDirectionReversals<<','
      <<out.path.weightedCostMeters<<','<<config.repeats<<','
      <<measurement.p50Millis<<','<<measurement.p95Millis<<'\n';
}
} // namespace hybrid_astar::bench::ackermann_refinement
