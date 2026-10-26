// Manual-only benchmark: never registered in CTest or GitHub Actions.
// CSV comparisons use identical maps/poses/budgets for each algorithm mode.
#include "bench/hard_scenarios.hpp"
#include "hybrid_astar/ackermann/refinement.hpp"
#ifdef HYBRID_ASTAR_HARD_CASES_WITH_OMPL
#include "hybrid_astar/steering/reeds_shepp_evaluator.hpp"
#endif
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
const char* status(PlanStatus s) {
  switch(s) {
  case PlanStatus::Success:return "success";
  case PlanStatus::NoPath:return "no_path";
  case PlanStatus::BudgetExceeded:return "budget_exceeded";
  case PlanStatus::ValidationIncomplete:return "validation_incomplete";
  case PlanStatus::StrictValidationFailed:return "strict_validation_failed";
  case PlanStatus::StartInCollision:return "start_in_collision";
  case PlanStatus::InvalidMap:return "invalid_map";
  case PlanStatus::InvalidRequest:return "invalid_request";
  case PlanStatus::RejectedUnknownMap:return "unknown_map_rejected";
  case PlanStatus::Cancelled:return "cancelled";
  default:return "internal_error";
  }
}
double quantile(std::vector<double> v,double q) {
  if(v.empty())return 0;
  std::sort(v.begin(),v.end());
  return v[std::min(v.size()-1,static_cast<std::size_t>(std::ceil(q*v.size()))-1)];
}
struct Variant {const char* name;bool performance,ompl,refine,allowCornerCut=false; double weight=1.0; std::size_t gridBudget=0;};
PlanResult run(PlanRequest q,const Variant& variant) {
  if(variant.performance)
    q.options.heuristicMode=HeuristicMode::PerformanceReedsSheppAndObstacleGrid;
  q.options.gridPreventDiagonalCornerCutting=!variant.allowCornerCut;
  q.options.performanceHeuristicWeight=variant.weight;
  if(variant.gridBudget)q.options.gridMaxTotalSettlements=variant.gridBudget;
  PlanResult path;
#ifdef HYBRID_ASTAR_HARD_CASES_WITH_OMPL
  if(variant.ompl)path=steering::planWithReedsShepp(q,variant.performance);
  else
#endif
  {
    path=plan(q);
  }
  if(variant.refine&&path.status==PlanStatus::Success) {
    RefinementOptions opts;
    opts.useCurvatureBoundedShortcuts=true;
    opts.maxAttempts=16;opts.maxAccepted=2;
    auto refined=refinePath(q,path,{},opts);
    if(refined.path.status==PlanStatus::Success)path=std::move(refined.path);
  }
  return path;
}
}
int main(int argc,char** argv) {
  try {
    const int repetitions=argc>1?std::stoi(argv[1]):3;
    if(repetitions<1||repetitions>100)
      throw std::invalid_argument("Expected repeats in [1,100]");
    const std::vector<Variant> variants={
      {"primitive",false,false,false},
      {"primitive_grid",true,false,false},
      {"primitive_grid_funded",true,false,false,false,1.0,20000},
      {"primitive_grid_weighted_1_2",true,false,false,false,1.2,20000},
      {"primitive_grid_weighted_1_5",true,false,false,false,1.5,20000},
      {"primitive_grid_legacy_diagonals",true,false,false,true},
      {"primitive_grid_refined",true,false,true},
#ifdef HYBRID_ASTAR_HARD_CASES_WITH_OMPL
      {"ompl_rs",false,true,false},
      {"ompl_rs_grid",true,true,false},
      {"ompl_rs_grid_weighted_1_2",true,true,false,false,1.2,20000},
      {"ompl_rs_grid_refined",true,true,true}
#endif
    };
    std::cout<<"scenario,variant,runs,status,successes,p50_ms,p95_ms,p99_ms,"
      "mean_expanded,mean_generated,mean_final_pose_checks,"
      "mean_grid_settlements,mean_grid_fallbacks,seed_cells_examined,"
      "grid_budget_fallbacks,validated_cost,validated_length,"
      "heuristic_certified,strict_success,expansion_budget,grid_total_budget,heuristic_weight\n";
    for(const auto& scenario:bench::hardCases())for(const auto& v:variants) {
      std::vector<double> times;
      std::size_t successful=0,expanded=0,generated=0,finalChecks=0,gridWork=0,
                  fallbacks=0,seedCells=0,budgetFallbacks=0,strict=0;
      PlanResult last;
      for(int i=0;i<repetitions;++i) {
        const auto begin=std::chrono::steady_clock::now();
        auto result=run(scenario.request,v);
        const auto end=std::chrono::steady_clock::now();
        times.push_back(std::chrono::duration<double,std::milli>(end-begin).count());
        if(result.status==PlanStatus::Success) {
          if(result.collisionEvidence!=CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled||
             result.mapRevision!=scenario.request.map->revision)
            throw std::runtime_error("Unverified success: "+scenario.name);
          ++successful;++strict;
        }
        expanded+=result.diagnostics.expanded;
        generated+=result.diagnostics.generated;
        finalChecks+=result.diagnostics.finalPoseChecks;
        gridWork+=result.diagnostics.gridSettlements;
        fallbacks+=result.diagnostics.gridFallbacks;
        seedCells+=result.diagnostics.gridSeedCellsExamined;
        budgetFallbacks+=result.diagnostics.gridWorkBudgetFallbacks;
        last=std::move(result);
      }
      std::cout<<scenario.name<<','<<v.name<<','<<repetitions<<','<<status(last.status)
        <<','<<successful<<','<<std::fixed<<std::setprecision(3)
        <<quantile(times,.5)<<','<<quantile(times,.95)<<','<<quantile(times,.99)
        <<','<<double(expanded)/repetitions<<','<<double(generated)/repetitions
        <<','<<double(finalChecks)/repetitions<<','<<double(gridWork)/repetitions
        <<','<<double(fallbacks)/repetitions<<','<<double(seedCells)/repetitions
        <<','<<double(budgetFallbacks)/repetitions<<',';
      if(last.status==PlanStatus::Success)
        std::cout<<last.weightedCostMeters<<','<<last.geometricLengthMeters;
      else std::cout<<"NA,NA";
      std::cout<<','<<(last.heuristicCertifiedAdmissible?1:0)<<','<<strict
        <<','<<scenario.request.options.maxExpansions
        <<','<<(v.gridBudget?v.gridBudget:scenario.request.options.gridMaxTotalSettlements)
        <<','<<v.weight<<'\n';
    }
  } catch(const std::exception& e) {
    std::cerr<<"Hard scenario benchmark failed: "<<e.what()<<'\n';
    return 2;
  }
}
