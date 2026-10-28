#pragma once
// Shared deterministic, opt-in benchmark harness; no performance threshold is
// asserted and these workloads are not part of normal CTest runs.
#include "bench/warehouse_scenarios.hpp"
#include "hybrid_astar/ackermann/refinement.hpp"
#ifdef HYBRID_ASTAR_REFINEMENT_BENCH_WITH_OMPL
#include "hybrid_astar/steering/reeds_shepp_evaluator.hpp"
#endif
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <set>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace hybrid_astar::bench::ackermann_refinement {
using namespace hybrid_astar::ackermann;

struct Config {
  // The production A/B suite needs 100 DISTINCT strictly validated routes
  // whose searches EACH required at least ten thousand expansions.
  std::size_t repeats=5;
  std::size_t minimumExpansions=10000;
  std::size_t maxCases=100;
  std::size_t searchBudget=250000;
  std::size_t maxCandidateSearches=0; // 0: search until target is reached
  double steeringWeightMetersSquared=1.0;
  std::uint64_t seed=20261028;
};
inline Config parseArgs(int argc,char** argv) {
  if(argc>8)throw std::invalid_argument(
      "Usage: BENCH [repeats=5] [minimum_expansions=10000] "
      "[distinct_paths=100] [search_budget=250000] [max_candidates=0] "
      "[steering_weight_m2=1.0] [seed=20261028]");
  const auto read=[&](int index,std::size_t fallback) {
    if(argc<=index)return fallback;
    const std::string value=argv[index];
    if(value.empty()||value[0]=='-')
      throw std::invalid_argument("Expected a positive integer");
    std::size_t consumed=0;
    const auto parsed=std::stoull(value,&consumed);
    if(consumed!=value.size())throw std::invalid_argument("Expected an integer");
    if(parsed>std::numeric_limits<std::size_t>::max())
      throw std::invalid_argument("Value exceeds size_t");
    return static_cast<std::size_t>(parsed);
  };
  Config c;
  c.repeats=read(1,c.repeats);
  c.minimumExpansions=read(2,c.minimumExpansions);
  c.maxCases=read(3,c.maxCases);
  c.searchBudget=read(4,c.searchBudget);
  c.maxCandidateSearches=read(5,c.maxCandidateSearches);
  if(argc>6) {
    const std::string value=argv[6];
    std::size_t consumed=0;
    c.steeringWeightMetersSquared=std::stod(value,&consumed);
    if(consumed!=value.size())throw std::invalid_argument("Expected floating steering weight");
  }
  if(argc>7) {
    const std::string value=argv[7];
    if(value.empty()||value[0]=='-')throw std::invalid_argument("Invalid random seed");
    std::size_t consumed=0;
    c.seed=std::stoull(value,&consumed);
    if(consumed!=value.size())throw std::invalid_argument("Invalid random seed");
  }
  if(!std::isfinite(c.steeringWeightMetersSquared)||
     c.steeringWeightMetersSquared<=0||c.steeringWeightMetersSquared>100||
     c.repeats==0||c.repeats>100||
     c.minimumExpansions==0||c.minimumExpansions>1000000||
     c.maxCases==0||c.maxCases>1000||
     c.searchBudget<100||c.searchBudget>1000000||
     c.minimumExpansions>c.searchBudget||
     (c.maxCandidateSearches!=0 && c.maxCandidateSearches<c.maxCases))
    throw std::invalid_argument("Out-of-range benchmark arguments");
  return c;
}
struct Scenario {
  std::string name;
  PlanRequest request;
  PlanResult original;
  std::uint64_t pathFingerprint=0;
  std::size_t startRegion=0,goalRegion=0;
};
// The map remains unchanged: only sample robot poses in its free space.
// A fixed-seed pseudo-random generator makes both benchmark binaries select
// the same 100 paths, independent of platform-specific standard distributions.
inline constexpr std::size_t coverageColumns=3,coverageRows=3;
inline constexpr std::size_t coverageRegions=coverageColumns*coverageRows;
using CoverageCounts=std::array<std::size_t,coverageRegions>;
struct Endpoints {Pose2D start,goal;std::size_t startRegion,goalRegion;};
struct RandomStream {
  std::uint64_t state;
  explicit RandomStream(std::uint64_t seed):state(seed) {}
  std::uint64_t next() {
    state+=0x9e3779b97f4a7c15ULL;
    auto v=state;
    v=(v^(v>>30))*0xbf58476d1ce4e5b9ULL;
    v=(v^(v>>27))*0x94d049bb133111ebULL;
    return v^(v>>31);
  }
  double unit() {return static_cast<double>(next()>>11)*0x1.0p-53;}
  std::size_t index(std::size_t n) {
    return static_cast<std::size_t>(unit()*static_cast<double>(n));
  }
};
inline std::size_t regionOf(const Pose2D& p,const MapSnapshot& map) {
  const double w=map.width*map.resolutionMeters;
  const double h=map.height*map.resolutionMeters;
  const auto x=std::min(coverageColumns-1,
      static_cast<std::size_t>(std::clamp(p.x/w,0.0,1.0)*coverageColumns));
  const auto y=std::min(coverageRows-1,
      static_cast<std::size_t>(std::clamp(p.y/h,0.0,1.0)*coverageRows));
  return y*coverageColumns+x;
}
inline std::size_t chooseRegion(RandomStream& rng,const CoverageCounts& counts) {
  // 75%: favor regions with the fewest ACCEPTED starts/goals;
  // 25%: explore freely so a temporarily difficult region cannot monopolize
  // all the candidates. Balance is based on validated results, not attempts.
  if(rng.unit()<0.75) {
    const auto minimum=*std::min_element(counts.begin(),counts.end());
    std::array<std::size_t,coverageRegions> tied{};
    std::size_t n=0;
    for(std::size_t i=0;i<coverageRegions;++i)
      if(counts[i]==minimum)tied[n++]=i;
    return tied[rng.index(n)];
  }
  return rng.index(coverageRegions);
}
inline Pose2D drawPose(std::size_t region,const MapSnapshot& map,
                       RandomStream& rng) {
  const double w=map.width*map.resolutionMeters;
  const double h=map.height*map.resolutionMeters;
  // Conservative 4.5m interior margin for the inflated full-sized Ackermann
  // footprint at ANY sampled heading. ExactCollisionChecker still decides.
  constexpr double border=4.5;
  const auto col=region%coverageColumns,row=region/coverageColumns;
  const double left=std::max(border,(w*col)/coverageColumns+0.05);
  const double right=std::min(w-border,(w*(col+1))/coverageColumns-0.05);
  const double bottom=std::max(border,(h*row)/coverageRows+0.05);
  const double top=std::min(h-border,(h*(row+1))/coverageRows-0.05);
  if(left>=right||bottom>=top)
    throw std::invalid_argument("Map too small for stratified robot poses");
  const double pi=std::numbers::pi_v<double>;
  return {left+(right-left)*rng.unit(),
          bottom+(top-bottom)*rng.unit(),
          -pi+2.0*pi*rng.unit()};
}
inline std::optional<Endpoints> candidateAt(
    std::size_t attempt,const Config& config,const CoverageCounts& starts,
    const CoverageCounts& goals,const MapSnapshot& map) {
  // An attempt is independently reproducible even if previous candidates
  // were rejected. Both x/y positions AND BOTH headings are randomized.
  RandomStream rng(config.seed+
      static_cast<std::uint64_t>(attempt)*0xd1342543de82ef95ULL);
  for(std::size_t sample=0;sample<128;++sample) {
    const auto s=chooseRegion(rng,starts),g=chooseRegion(rng,goals);
    if(s==g)continue;
    const auto from=drawPose(s,map,rng),to=drawPose(g,map,rng);
    // Favor routes that traverse warehouse wall columns/corridors. Do not
    // select short, trivial routes unlikely to demand 10k expansions.
    if(std::hypot(from.x-to.x,from.y-to.y)<18.0)continue;
    return Endpoints{from,to,s,g};
  }
  return std::nullopt;
}
// Hash quantized, full section geometry (not the arbitrary scenario name or
// merely the requested start/goal). The full quantized signature is also kept
// in a set to prevent hash collisions from affecting the uniqueness check.
inline std::vector<std::int64_t> pathSignature(const PlanResult& path) {
  const auto quantize=[](double x,double scale) {
    return static_cast<std::int64_t>(std::llround(x*scale));
  };
  std::vector<std::int64_t> signature;
  signature.reserve(path.sections.size()*10+1);
  signature.push_back(static_cast<std::int64_t>(path.sections.size()));
  for(const auto& s:path.sections) {
    signature.push_back(static_cast<std::int64_t>(s.kind));
    signature.push_back(static_cast<std::int64_t>(s.gear));
    signature.push_back(quantize(s.start.x,20.0)); // 5 cm resolution
    signature.push_back(quantize(s.start.y,20.0));
    signature.push_back(quantize(s.start.yaw,100.0)); // ~0.01 radian
    signature.push_back(quantize(s.end.x,20.0));
    signature.push_back(quantize(s.end.y,20.0));
    signature.push_back(quantize(s.end.yaw,100.0));
    signature.push_back(quantize(s.curvature,1000.0));
    signature.push_back(quantize(s.lengthMeters,20.0));
  }
  return signature;
}
inline std::uint64_t signatureHash(const std::vector<std::int64_t>& sig) {
  std::uint64_t hash=14695981039346656037ULL;
  for(const auto value:sig) {
    const auto u=static_cast<std::uint64_t>(value);
    for(unsigned shift=0;shift<64;shift+=8) {
      hash^=(u>>shift)&0xffULL;
      hash*=1099511628211ULL;
    }
  }
  return hash;
}

// Search/discovery happens ONCE per executable, outside optimizer timing.
// Keep the map and Ackermann geometry fixed; use new random poses every
// attempt. Enforce both endpoint coverage and whole-path uniqueness.
inline std::size_t minCoveragePerRegion(const Config& config) {
  return config.maxCases>=5*coverageRegions?5:
         config.maxCases>=3*coverageRegions?3:
         config.maxCases>=coverageRegions?1:0;
}
inline bool coverageSatisfied(const CoverageCounts& starts,
                              const CoverageCounts& goals,std::size_t minCount) {
  for(std::size_t i=0;i<coverageRegions;++i)
    if(starts[i]<minCount||goals[i]<minCount)return false;
  return true;
}
inline void printCoverage(const CoverageCounts& starts,
                          const CoverageCounts& goals) {
  std::cerr<<"Accepted start/goal coverage by warehouse 3x3 sector "
           <<"(lower row first):\n";
  for(std::size_t y=0;y<coverageRows;++y) {
    for(std::size_t x=0;x<coverageColumns;++x) {
      const auto k=y*coverageColumns+x;
      std::cerr<<" ["<<k<<":"<<starts[k]<<"/"<<goals[k]<<"]";
    }
    std::cerr<<'\n';
  }
}
inline std::vector<Scenario> discover(const Config& config) {
  auto common=mecanumWarehouseCase(config.searchBudget);
  common.options.model=PlanOptions::Model::Ackermann;
  common.footprint=PlanRequest{}.footprint;
  common.options.footprintInflationMeters=.5;
  common.options.heuristicMode=HeuristicMode::PerformanceReedsSheppAndObstacleGrid;
  common.options.maxGeneratedNodes=4*config.searchBudget;
  ExactCollisionChecker checker(*common.map,common.occupancy,common.footprint,
                                common.options.footprintInflationMeters);
  std::vector<Scenario> selected;
  selected.reserve(config.maxCases);
  std::set<std::vector<std::int64_t>> distinctSignatures;
  std::set<std::array<std::int64_t,6>> distinctEndpoints;
  CoverageCounts startCounts{},goalCounts{};
  std::array<std::size_t,coverageRegions*coverageRegions> pairs{};
  const auto coverageMinimum=minCoveragePerRegion(config);
  // Prevent one successful corridor from dominating the sample while still
  // allowing a few repeated region pairs with distinct path geometry.
  const auto maxPerRegion=std::max<std::size_t>(
      3,(config.maxCases+4)/5);  // <=20% of target, rounded up
  const auto maxPerPair=std::max<std::size_t>(
      2,(config.maxCases+9)/10); // <=10% per ordered region pair
  std::size_t blocked=0,failed=0,tooFew=0,duplicates=0,
              coverageSkipped=0,unusable=0,attempts=0;
  std::cerr<<"Randomized warehouse selection: seed="<<config.seed
           <<", target="<<config.maxCases
           <<", min_expansions="<<config.minimumExpansions
           <<", minimum_start_and_goal_per_sector="<<coverageMinimum
           <<", max_candidates="<<config.maxCandidateSearches
           <<" (0 means unbounded)\n";
  while(selected.size()<config.maxCases &&
        (config.maxCandidateSearches==0||attempts<config.maxCandidateSearches)) {
    if(attempts==std::numeric_limits<std::size_t>::max())
      throw std::runtime_error("Candidate counter overflow");
    const auto index=attempts++;
    if(index && index%250==0)
      std::cerr<<"Discovery progress: "<<selected.size()<<"/"<<config.maxCases
               <<" paths after "<<index<<" candidates; below 10k="
               <<tooFew<<", failed="<<failed<<"\n";
    const auto maybe=candidateAt(index,config,startCounts,goalCounts,*common.map);
    if(!maybe) {++unusable;continue;}
    const auto pair=*maybe;
    if(checker.check(pair.start)!=PoseCollision::Clear ||
       checker.check(pair.goal)!=PoseCollision::Clear) {++blocked;continue;}
    if(startCounts[pair.startRegion]>=maxPerRegion ||
       goalCounts[pair.goalRegion]>=maxPerRegion ||
       pairs[pair.startRegion*coverageRegions+pair.goalRegion]>=maxPerPair) {
      ++coverageSkipped;continue;
    }
    // Don't fill the 100 slots before all 9 start AND goal sectors have
    // reached their minimum: spend early slots on missing coverage.
    if(!coverageSatisfied(startCounts,goalCounts,coverageMinimum) &&
       startCounts[pair.startRegion]>=coverageMinimum &&
       goalCounts[pair.goalRegion]>=coverageMinimum) {
      ++coverageSkipped;continue;
    }
    const auto rounded=[](double v) {
      return static_cast<std::int64_t>(std::llround(v*1000.0));
    };
    const std::array<std::int64_t,6> key{rounded(pair.start.x),
        rounded(pair.start.y),rounded(pair.start.yaw),rounded(pair.goal.x),
        rounded(pair.goal.y),rounded(pair.goal.yaw)};
    // Only retain keys of ACCEPTED routes. With unlimited candidates,
    // storing every failed pose pair would grow memory without bound.
    if(distinctEndpoints.find(key)!=distinctEndpoints.end()) {++duplicates;continue;}
    auto request=common;
    request.start=pair.start;request.goal=pair.goal;
#ifdef HYBRID_ASTAR_REFINEMENT_BENCH_WITH_OMPL
    auto original=steering::planWithReedsShepp(request);
#else
    auto original=plan(request);
#endif
    if(original.status!=PlanStatus::Success ||
       !original.modelGeometryValid ||
       original.collisionEvidence!=CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled) {
      ++failed;continue;
    }
    if(original.diagnostics.expanded<config.minimumExpansions) {
      ++tooFew;continue;
    }
    auto signature=pathSignature(original);
    const auto [it,inserted]=distinctSignatures.insert(signature);
    if(!inserted) {++duplicates;continue;}
    const auto pathId=signatureHash(*it);
    distinctEndpoints.insert(key);
    const auto number=selected.size();
    selected.push_back({"warehouse_"+std::to_string(number),
                        std::move(request),std::move(original),pathId,
                        pair.startRegion,pair.goalRegion});
    ++startCounts[pair.startRegion];
    ++goalCounts[pair.goalRegion];
    ++pairs[pair.startRegion*coverageRegions+pair.goalRegion];
    if(selected.size()%10==0||selected.size()==config.maxCases)
      std::cerr<<"Validated distinct paths: "<<selected.size()<<"/"
               <<config.maxCases<<" (candidate attempts "<<attempts<<")\n";
  }
  std::cerr<<"Discovery: selected="<<selected.size()
           <<", target="<<config.maxCases
           <<", attempts="<<attempts
           <<", blocked="<<blocked
           <<", plan_failed="<<failed
           <<", below_expansion_threshold="<<tooFew
           <<", duplicate="<<duplicates
           <<", coverage_skipped="<<coverageSkipped
           <<", unusable_pose_draws="<<unusable<<"\n";
  printCoverage(startCounts,goalCounts);
  if(selected.size()!=config.maxCases ||
     !coverageSatisfied(startCounts,goalCounts,coverageMinimum))
    throw std::runtime_error("Only "+std::to_string(selected.size())+" of "+
       std::to_string(config.maxCases)+" distinct paths met >= "+
       std::to_string(config.minimumExpansions)+" search expansions AND "
       "the 3x3 start/goal coverage requirements. No partial CSV is produced. "
       "Increase search_budget or max_candidates, or use 0 for unlimited candidates.");
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
  // Match the Planning Lab connector when OMPL is enabled in this build.
  // One evaluator per variant is reused across warmup and timed repetitions.
  RefinementConnector connector;
#ifdef HYBRID_ASTAR_REFINEMENT_BENCH_WITH_OMPL
  steering::ReedsSheppEvaluator evaluator(
      scenario.request.options.minimumTurningRadiusMeters);
  connector=[&evaluator](const Pose2D& from,const Pose2D& to) {
    return evaluator.shortestPath(from,to);
  };
#endif
  // Warm up this profile once (unmeasured) to reduce first-use allocator and
  // instruction-cache bias; timed runs all refine the unchanged raw path.
  const auto warmup=refinePath(scenario.request,scenario.original,connector,options);
  if(warmup.path.status!=PlanStatus::Success ||
     warmup.path.weightedCostMeters>initialObjective+1e-6)
    throw std::runtime_error(std::string("Invalid warmup refinement: ")+variant.name);
  std::vector<double> timings;
  RefinementResult last;
  for(std::size_t i=0;i<config.repeats;++i) {
    const auto start=std::chrono::steady_clock::now();
    auto result=refinePath(scenario.request,scenario.original,connector,options);
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
// Aggregate over distinct paths, not over repeated timing samples. This keeps
// a five-repeat 100-route benchmark from hiding route-specific regressions.
struct Summary {
  std::string label;
  std::vector<double> medianRuntimesMillis;
  std::vector<double> curvatureReductions;
  std::vector<double> lengthReductionsMeters;
  std::size_t smoothnessWins=0;
  std::size_t workBudgetExceeded=0;
  void add(const Scenario& scenario,const Measurement& m) {
    const double before=measure(scenario.original.sections).totalVariation;
    const double after=measure(m.result.path.sections).totalVariation;
    curvatureReductions.push_back(before-after);
    lengthReductionsMeters.push_back(
        scenario.original.geometricLengthMeters-m.result.path.geometricLengthMeters);
    medianRuntimesMillis.push_back(m.p50Millis);
    if(after<before-1e-9)++smoothnessWins;
    if(m.result.status==RefinementStatus::WorkBudgetExceeded)++workBudgetExceeded;
  }
  void report() const {
    if(medianRuntimesMillis.empty())return;
    const auto average=[](const std::vector<double>& xs) {
      double total=0;
      for(const auto v:xs)total+=v;
      return total/static_cast<double>(xs.size());
    };
    std::cerr<<std::fixed<<std::setprecision(3)
             <<"Summary "<<label
             <<": paths="<<medianRuntimesMillis.size()
             <<", median_path_p50_ms="<<percentile(medianRuntimesMillis,.5)
             <<", p95_path_p50_ms="<<percentile(medianRuntimesMillis,.95)
             <<", mean_curvature_tv_reduction_inv_m="<<average(curvatureReductions)
             <<", mean_length_reduction_m="<<average(lengthReductionsMeters)
             <<", smoother_paths="<<smoothnessWins
             <<", work_budget_exceeded="<<workBudgetExceeded<<"\n";
  }
};
inline void printHeader() {
  std::cout<<"scenario,path_fingerprint,random_seed,start_region,goal_region,start_x_m,start_y_m,start_yaw_rad,"
      "goal_x_m,goal_y_m,goal_yaw_rad,connector,variant,search_expanded,raw_sections,raw_length_m,"
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
  std::cout<<scenario.name<<','<<scenario.pathFingerprint<<','
      <<config.seed<<','<<scenario.startRegion<<','<<scenario.goalRegion<<','
      <<std::fixed<<std::setprecision(6)
      <<scenario.request.start.x<<','<<scenario.request.start.y<<','
      <<scenario.request.start.yaw<<','<<scenario.request.goal.x<<','
      <<scenario.request.goal.y<<','<<scenario.request.goal.yaw<<','
#ifdef HYBRID_ASTAR_REFINEMENT_BENCH_WITH_OMPL
      <<"ompl_rs_plus_native_csc,"
#else
      <<"native_csc_only,"
#endif
      <<variant.name<<','
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
