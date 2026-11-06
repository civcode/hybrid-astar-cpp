// Manual reproducible baseline from exported Planning Lab warehouse JSON.
// This benchmark is not registered with CTest or GitHub Actions.
#include "hybrid_astar/articulated_steering/planner.hpp"
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
namespace as=hybrid_astar::articulated_steering;
namespace ac=hybrid_astar::ackermann;
int main(int argc,char** argv) {
  try {
    const std::string mode=argc>1?argv[1]:"coupled";
    if(mode!="legacy"&&mode!="coupled"&&mode!="coupled-no-fast"&&
       mode!="analytic-only"&&mode!="combined"&&mode!="weighted"&&
       mode!="grid-weighted"&&mode!="connector"&&mode!="weighted-connector"&&
       mode!="heading"&&mode!="heading-connector"&&
       mode!="anytime"&&mode!="heading-anytime"&&
       mode!="coarse"&&mode!="coarse-connector"&&mode!="coarse-anytime")
      throw std::invalid_argument(
          "Use legacy | coupled | coupled-no-fast | analytic-only | combined | weighted | grid-weighted | connector | weighted-connector | heading | heading-connector | anytime | heading-anytime | coarse | coarse-connector | coarse-anytime");
    if(argc>6)throw std::invalid_argument(
        "Usage: articulated_warehouse_bench [mode] [heuristic_weight] [baseline|warehouse-heading|open-heading|uploaded-heading] [extra_expansions] [coarse_resolution_m]");
    const std::string scenario=argc>=4?argv[3]:"baseline";
    if(scenario!="baseline"&&scenario!="warehouse-heading"&&
       scenario!="open-heading"&&scenario!="uploaded-heading")
      throw std::invalid_argument("Unknown articulated warehouse scenario");
    // Preserve all historical default runs. The control differs from
    // 'weighted' only in using the grid heuristic rather than Combined.
    double weight=(mode=="weighted"||mode=="grid-weighted"||
                   mode=="weighted-connector")?1.5:
                  (mode=="connector"||mode=="heading"||
                   mode=="heading-connector"||mode=="anytime"||
                   mode=="heading-anytime"||mode=="coarse"||
                   mode=="coarse-connector"||mode=="coarse-anytime")?1.2:1.0;
    if(argc>=3) {
      const std::string arg=argv[2];
      std::size_t parsed=0;
      weight=std::stod(arg,&parsed);
      if(parsed!=arg.size()||!std::isfinite(weight)||weight<=0.0||weight>3.0)
        throw std::invalid_argument("heuristic_weight must be finite and in (0, 3]");
    }
    auto map=std::make_shared<ac::MapSnapshot>();
    map->width=200;map->height=160;map->resolutionMeters=.25;
    map->origin={0,0,0};
    map->frameId="demo/map";map->sourceId="built-in";map->revision=1;
    map->cells.assign(map->width*map->height,0);
    const auto rectangle=[&](int x0,int y0,int x1,int y1) {
      for(int y=y0;y<y1;++y)for(int x=x0;x<x1;++x)
        map->cells[std::size_t(y)*map->width+std::size_t(x)]=100;
    };
    if(scenario!="open-heading") {
      rectangle(48,12,53,48);rectangle(48,92,53,145);
      rectangle(94,0,99,46);rectangle(94,100,99,160);
      rectangle(141,12,146,55);rectangle(141,104,146,145);
    }
    as::Request request;
    request.map=map;
    request.start={5.0,30.56,0.0,0.0};
    request.goal={18.16,27.98,0.0,0.0};
    if(scenario=="warehouse-heading") {
      // Exact 2026-11-06 uploaded warehouse obstacle/heading regression.
      request.start={4.92,33.53,0.0,0.0};
      request.goal={30.32,31.48,-std::acos(-1.0)/2.0,0.0};
    }
    if(scenario=="open-heading") {
      request.start={8.42,25.39,0.0,0.0};
      request.goal={40.13,5.85,-std::acos(-1.0)/2.0,0.0};
    }
    if(scenario=="uploaded-heading") {
      // Matches both uploaded (5) and (6) warehouse plans exactly.
      request.start={4.70,30.49,0.0,0.0};
      request.goal={31.24,5.85,-std::acos(-1.0)/2.0,0.0};
    }
    request.parameters={1.0,1.0,-1.2,1.2,1.0,.8,
                        .95*(1.0+std::cos(1.2))};
    request.bodies.front={-.5,.5,.3};
    request.bodies.rear={-.5,.5,.3};
    request.options.maxExpansions=1000000;
    request.options.maxGenerated=4000000;
    request.options.enableCoupledMotion=mode!="legacy";
    request.options.useSweptAabbPrefilter=
        mode!="legacy"&&mode!="coupled-no-fast";
    if(mode=="legacy")request.options.stationaryArticulationPenaltyMeters=0;
    if(mode=="analytic-only")
      request.options.heuristicMode=as::HeuristicMode::AnalyticOnly;
    if(mode=="combined"||mode=="weighted"||
       mode=="connector"||mode=="weighted-connector"||
       mode=="anytime")
      request.options.heuristicMode=as::HeuristicMode::Combined;
    if(mode=="heading"||mode=="heading-connector"||
       mode=="heading-anytime")
      request.options.heuristicMode=as::HeuristicMode::CombinedHeadingAware;
    if(mode=="coarse"||mode=="coarse-connector"||mode=="coarse-anytime")
      request.options.heuristicMode=as::HeuristicMode::CombinedCoarseGrid;
    if(mode=="connector"||mode=="weighted-connector"||
       mode=="heading-connector"||mode=="anytime"||
       mode=="heading-anytime"||mode=="coarse-connector"||
       mode=="coarse-anytime")
      request.options.enableGoalConnector=true;
    if(mode=="anytime"||mode=="heading-anytime"||mode=="coarse-anytime")
      request.options.enableAnytimeGoalRefinement=true;
    if(argc==5) {
      const std::string extra=argv[4];
      std::size_t used=0;
      const auto value=std::stoull(extra,&used);
      if(used!=extra.size()||value<1||value>100000)
        throw std::invalid_argument("Extra expansion budget must be in [1, 100000]");
      request.options.anytimeExtraExpansions=static_cast<std::size_t>(value);
    }
    if(argc==6) {
      const std::string arg=argv[5];
      std::size_t used=0;
      const double resolution=std::stod(arg,&used);
      if(used!=arg.size()||!std::isfinite(resolution)||
         resolution<=0.0||resolution>2.0)
        throw std::invalid_argument("coarse grid resolution must be in (0, 2] m");
      request.options.coarseGridResolutionMeters=resolution;
    }
    // Both weighted modes have the same kinematics, costs, map and budgets;
    // only the heuristic changes. Weight >1 is an experiment, not a bound.
    if(mode=="grid-weighted")
      request.options.heuristicMode=as::HeuristicMode::ObstacleGrid;
    request.options.heuristicWeight=weight;
    const auto started=std::chrono::steady_clock::now();
    const auto result=as::plan(request);
    const double elapsed=std::chrono::duration<double,std::milli>(
        std::chrono::steady_clock::now()-started).count();
    double stationary=0,simultaneous=0,distance=0;
    for(const auto& motion:result.motions) {
      const double v=motion.control.frontSpeedMetersPerSecond;
      const double u=motion.control.gammaRateRadiansPerSecond;
      if(v==0.0)stationary+=motion.durationSeconds;
      else {
        distance+=std::abs(v)*motion.durationSeconds;
        if(u!=0.0)simultaneous+=motion.durationSeconds;
      }
    }
    std::cout<<"mode,status,validated,expanded,generated,search_body_checks,"
      "elapsed_ms,stationary_s,coupled_s,drive_m,swept_free,grid_settlements,"
      "heuristic_weight,plan_cost_m,goal_xy_error_m,goal_yaw_error_rad,goal_gamma_error_rad,"
      "connector_attempts,connector_candidates,connector_validated,connector_checks,connector_evaluations,"
      "scenario,connector_rejected_collision,connector_rejected_terminal,connector_rejected_strict,"
      "connector_global_used,connector_near_used,connector_deferred,"
      "connector_coupled_candidates,connector_clearance_penalized,"
      "connector_first_expansion,connector_last_expansion,connector_closest_xy_m,"
      "anytime_first_solution_expansion,anytime_extra_expansions,"
      "anytime_extra_connector_attempts,anytime_validated_candidates,"
      "anytime_improvements,anytime_cost_bound_prunes,"
      "anytime_first_cost_m,anytime_best_cost_m,"
      "coarse_resolution_m,coarse_cells,coarse_blocked,coarse_seed_cells,"
      "coarse_settlements,coarse_queries,coarse_fallbacks,"
      "coarse_build_ms,coarse_skipped,"
      "connector_1arc_proposals,connector_2arc_proposals,connector_3arc_proposals,"
      "tail_drive_m,tail_reverse_m,tail_steering_rad,tail_stationary_s,"
      "tail_peak_gamma_rad,tail_gear_changes,tail_steering_reversals,"
      "tail_drive_arcs,tail_fitted_arcs,tail_operational_penalty_m,"
      "tail_quality_score_m,anytime_paced_deferrals,"
      "anytime_first_extra_shot_expansion,anytime_last_extra_shot_expansion\n";
    std::cout<<mode<<','<<static_cast<int>(result.status)<<','
      <<result.independentlySampledBodyValidated<<','
      <<result.diagnostics.expanded<<','<<result.diagnostics.generated<<','
      <<result.diagnostics.searchBodyChecks<<','<<elapsed<<','
      <<stationary<<','<<simultaneous<<','<<distance<<','
      <<result.diagnostics.searchSweptAabbAccepted<<','
      <<result.diagnostics.gridSettlements<<','
      <<weight<<','<<result.costMeters<<','
      <<result.goalXYErrorMeters<<','<<result.goalYawErrorRadians<<','
      <<result.goalGammaErrorRadians<<','
      <<result.diagnostics.goalConnectorAttempts<<','
      <<result.diagnostics.goalConnectorCandidates<<','
      <<result.diagnostics.goalConnectorValidated<<','
      <<result.diagnostics.goalConnectorBodyChecks<<','
      <<result.diagnostics.goalConnectorEvaluations<<','
      <<scenario<<','
      <<result.diagnostics.goalConnectorRejectedCollision<<','
      <<result.diagnostics.goalConnectorRejectedTerminal<<','
      <<result.diagnostics.goalConnectorRejectedStrict<<','
      <<result.diagnostics.goalConnectorGlobalUsed<<','
      <<result.diagnostics.goalConnectorNearUsed<<','
      <<result.diagnostics.goalConnectorDeferred<<','
      <<result.diagnostics.goalConnectorCoupledCandidates<<','
      <<result.diagnostics.goalConnectorClearancePenalized<<','
      <<result.diagnostics.goalConnectorFirstExpansion<<','
      <<result.diagnostics.goalConnectorLastExpansion<<','
      <<result.diagnostics.goalConnectorClosestXYMeters<<','
      <<result.diagnostics.anytimeFirstSolutionExpansion<<','
      <<result.diagnostics.anytimeExtraExpansions<<','
      <<result.diagnostics.anytimeExtraConnectorAttempts<<','
      <<result.diagnostics.anytimeValidatedCandidates<<','
      <<result.diagnostics.anytimeImprovements<<','
      <<result.diagnostics.anytimeCostBoundPrunes<<','
      <<result.diagnostics.anytimeFirstCostMeters<<','
      <<result.diagnostics.anytimeBestCostMeters<<','
      <<result.diagnostics.coarseGridResolutionMeters<<','
      <<result.diagnostics.coarseGridCells<<','
      <<result.diagnostics.coarseGridBlockedCells<<','
      <<result.diagnostics.coarseGridSeedCells<<','
      <<result.diagnostics.coarseGridSettlements<<','
      <<result.diagnostics.coarseGridQueries<<','
      <<result.diagnostics.coarseGridFallbacks<<','
      <<result.diagnostics.coarseGridBuildMilliseconds<<','
      <<result.diagnostics.coarseGridSkipped<<','
      <<result.diagnostics.goalConnectorOneArcProposals<<','
      <<result.diagnostics.goalConnectorTwoArcProposals<<','
      <<result.diagnostics.goalConnectorThreeArcProposals<<','
      <<result.diagnostics.goalConnectorTailDrivingMeters<<','
      <<result.diagnostics.goalConnectorTailReverseMeters<<','
      <<result.diagnostics.goalConnectorTailArticulationRadians<<','
      <<result.diagnostics.goalConnectorTailStationarySeconds<<','
      <<result.diagnostics.goalConnectorTailPeakGammaRadians<<','
      <<result.diagnostics.goalConnectorTailGearChanges<<','
      <<result.diagnostics.goalConnectorTailArticulationReversals<<','
      <<result.diagnostics.goalConnectorTailDriveArcs<<','
      <<result.diagnostics.goalConnectorTailFittedArcs<<','
      <<result.diagnostics.goalConnectorTailOperationalPenaltyMeters<<','
      <<result.diagnostics.goalConnectorTailQualityScoreMeters<<','
      <<result.diagnostics.anytimePacedDeferrals<<','
      <<result.diagnostics.anytimeFirstExtraShotExpansion<<','
      <<result.diagnostics.anytimeLastExtraShotExpansion<<'\n';
    return result.status==as::Status::Success?0:2;
  } catch(const std::exception& e) {
    std::cerr<<e.what()<<'\n';
    return 1;
  }
}
