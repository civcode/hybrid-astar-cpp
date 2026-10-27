// Opt-in local benchmark; no GitHub Actions target, no latency guarantee.
#include "hybrid_astar/ackermann/refinement.hpp"
#include "warehouse_scenarios.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>
using namespace hybrid_astar;
using namespace hybrid_astar::ackermann;
struct Scenario {std::string name; PlanRequest request; PlanResult raw;};
static Scenario fixture(std::string name) {
  auto m=std::make_shared<MapSnapshot>();m->width=m->height=400;
  m->resolutionMeters=.2;m->cells.assign(160000,0);m->revision=7;
  PlanRequest q;q.map=m;q.start={20,20,0};
  std::vector<MotionSection> sections;Pose2D p=q.start;
  auto append=[&](Gear gear,double k,double length) {
    auto part=makeSection(p,gear,k,length);sections.push_back(part);p=part.end;
  };
  if(name=="straight_cusp") {
    append(Gear::Forward,0,2);append(Gear::Reverse,0,1);append(Gear::Forward,0,1);
  } else if(name=="curved_cusp") {
    append(Gear::Forward,.2,2);append(Gear::Reverse,.2,1);append(Gear::Forward,.2,1);
  } else {
    for(double k:{.25,-.25,-.25,.25}) append(Gear::Forward,k,2*std::numbers::pi);
    m->cells[100*400+140]=100;
  }
  q.goal=p;
  PlanResult raw;raw.status=PlanStatus::Success;raw.sections=sections;
  raw.mapRevision=q.map->revision;raw.mapFrameId=q.map->frameId;raw.mapSourceId=q.map->sourceId;
  raw.effectiveOccupancy=q.occupancy;raw.modelGeometryValid=true;
  raw.collisionEvidence=CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled;
  return {name,q,raw};
}
static double percentile(std::vector<double> times,double fraction) {
  std::sort(times.begin(),times.end());
  const auto i=static_cast<std::size_t>(std::ceil(double(times.size())*fraction));
  return times[std::min(times.size()-1,i?i-1:0)];
}
// High-expansion Planning Lab reproduction. Manual only: CTest never runs
// this workload. A/B holds the map, footprint inflation, original search
// result, validation budget and accepted-merge budget constant.
static int differentialWarehouse(int argc,char** argv) {
  if(argc>4)throw std::invalid_argument(
      "Usage: --warehouse-differential [repeats] [search_expansion_budget]");
  const int repetitions=argc>2?std::stoi(argv[2]):3;
  const int budget=argc>3?std::stoi(argv[3]):1000000;
  if(repetitions<1||repetitions>100||budget<100||budget>1000000)
    throw std::invalid_argument("Invalid repeat count or work budget");
  auto request=bench::mecanumWarehouseCase(std::size_t(budget));
  request.options.model=PlanOptions::Model::DifferentialDrive;
  request.start={4.77,32.84,0};
  request.goal={29.94,6.23,-.5*std::numbers::pi_v<double>};
  request.options.footprintInflationMeters=.5;
  const auto base=plan(request);
  if(base.status!=PlanStatus::Success) {
    std::cerr<<"Differential warehouse search did not return a validated route; "
             <<"status="<<int(base.status)<<" expanded="<<base.diagnostics.expanded<<"\n";
    return 2;
  }
  struct Variant {const char* label;bool curved;std::size_t radii;};
  constexpr Variant variants[]{
      {"straight",false,1},
      {"csc_fixed_radius",true,1},
      {"csc_multi_radius",true,4}};
  std::cout<<"scenario,mode,repeats,search_expanded,original_sections,"
      "original_cost,p50_ms,p95_ms,refinement_status,refined_sections,"
      "refined_cost,attempted,accepted,rejected,cost_pruned,strict_pose_checks\n";
  for(const auto variant:variants) {
    RefinementOptions opts;
    opts.maxAttempts=160;opts.maxAccepted=8;opts.maxTotalPoseChecks=200000;
    opts.useCurvatureBoundedShortcuts=variant.curved;
    opts.differentialRadiusVariants=variant.radii;
    std::vector<double> elapsed;
    RefinementResult last;
    for(int run=0;run<repetitions;++run) {
      const auto t0=std::chrono::steady_clock::now();
      auto result=refinePath(request,base,{},opts);
      const auto t1=std::chrono::steady_clock::now();
      if(result.path.status!=PlanStatus::Success ||
         result.path.collisionEvidence!=
             CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled ||
         result.path.weightedCostMeters>base.weightedCostMeters+1e-8)
        throw std::runtime_error("Refinement lost its strictly validated original");
      elapsed.push_back(std::chrono::duration<double,std::milli>(t1-t0).count());
      last=std::move(result);
    }
    std::cout<<"diff_warehouse_diag_margin_050,"<<variant.label<<','<<repetitions
             <<','<<base.diagnostics.expanded<<','<<base.sections.size()<<','
             <<std::fixed<<std::setprecision(4)<<base.weightedCostMeters<<','
             <<percentile(elapsed,.5)<<','<<percentile(elapsed,.95)<<','
             <<int(last.status)<<','<<last.path.sections.size()<<','
             <<last.path.weightedCostMeters<<','<<last.attempted<<','
             <<last.accepted<<','<<last.rejected<<','
             <<last.prunedByCostLowerBound<<','<<last.exactPoseChecks<<'\n';
  }
  return 0;
}
// Separates Ackermann search from post-path refinement: Planning Lab's
// elapsed timer includes BOTH. This fixture matches the reported successful
// warehouse case (bounded improvement + CSC enabled + 0.50m inflation).
static int ackermannWarehouse(int argc,char** argv) {
  if(argc>4)throw std::invalid_argument(
      "Usage: --warehouse-ackermann [repeats] [search_expansion_budget]");
  const int repetitions=argc>2?std::stoi(argv[2]):3;
  const int budget=argc>3?std::stoi(argv[3]):1000000;
  if(repetitions<1||repetitions>100||budget<100||budget>1000000)
    throw std::invalid_argument("Invalid repeats or search work budget");
  const auto cases=bench::ackermannWarehouseCases(std::size_t(budget));
  const auto& request=cases.at(1).second; // reported "improve" screenshot
  const auto t0=std::chrono::steady_clock::now();
  const auto original=plan(request);
  const auto t1=std::chrono::steady_clock::now();
  if(original.status!=PlanStatus::Success) {
    std::cerr<<"Ackermann warehouse did not find a strictly validated route; "
             <<"status="<<int(original.status)
             <<" expansions="<<original.diagnostics.expanded<<'\n';
    return 2;
  }
  RefinementOptions options;
  options.useCurvatureBoundedShortcuts=true;
  options.maxAttempts=35;options.maxAccepted=3;
  std::vector<double> elapsed;
  RefinementResult last;
  for(int run=0;run<repetitions;++run) {
    const auto r0=std::chrono::steady_clock::now();
    auto refined=refinePath(request,original,{},options);
    const auto r1=std::chrono::steady_clock::now();
    if(refined.path.status!=PlanStatus::Success ||
       refined.path.collisionEvidence!=
           CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled ||
       refined.path.weightedCostMeters>original.weightedCostMeters+1e-8)
      throw std::runtime_error("Ackermann post-refinement lost verified incumbent");
    elapsed.push_back(std::chrono::duration<double,std::milli>(r1-r0).count());
    last=std::move(refined);
  }
  std::cout<<"scenario,search_ms,search_expanded,search_generated,"
      "search_cost,original_sections,refine_repeats,refine_p50_ms,"
      "refine_p95_ms,refine_status,refined_cost,refined_sections,"
      "refine_attempted,refine_accepted,refine_rejected,"
      "refine_cost_pruned,refine_pose_checks\n";
  std::cout<<"ack_across_improve_margin_050,"
      <<std::fixed<<std::setprecision(4)
      <<std::chrono::duration<double,std::milli>(t1-t0).count()<<','
      <<original.diagnostics.expanded<<','<<original.diagnostics.generated<<','
      <<original.weightedCostMeters<<','<<original.sections.size()<<','
      <<repetitions<<','<<percentile(elapsed,.5)<<','
      <<percentile(elapsed,.95)<<','<<int(last.status)<<','
      <<last.path.weightedCostMeters<<','<<last.path.sections.size()<<','
      <<last.attempted<<','<<last.accepted<<','<<last.rejected<<','
      <<last.prunedByCostLowerBound<<','<<last.exactPoseChecks<<'\n';
  return 0;
}
int main(int argc,char** argv) {
  if(argc>1&&std::string(argv[1])=="--warehouse-ackermann") {
    try{return ackermannWarehouse(argc,argv);}
    catch(const std::exception& e) {
      std::cerr<<"Ackermann warehouse refinement benchmark: "
               <<e.what()<<'\n';return 2;
    }
  }
  if(argc>1&&std::string(argv[1])=="--warehouse-differential") {
    try{return differentialWarehouse(argc,argv);}
    catch(const std::exception& e) {
      std::cerr<<"Differential warehouse refinement benchmark: "
               <<e.what()<<'\n';return 2;
    }
  }
  int repetitions=argc>1?std::max(1,std::stoi(argv[1])):15;
  std::cout << "scenario,mode,runs,validated,improved,p50_ms,p95_ms,mean_checks,final_cost\n";
  for(const auto& name:{"straight_cusp","curved_cusp","blocked_direct"}) {
    auto scenario=fixture(name);
    for(bool curve:{false,true}) {
      std::vector<double> elapsed;
      int valid=0,improved=0;std::size_t checks=0;double lastCost=-1;
      RefinementOptions options;options.useCurvatureBoundedShortcuts=curve;
      for(int i=0;i<repetitions;++i) {
        const auto begin=std::chrono::steady_clock::now();
        const auto result=refinePath(scenario.request,scenario.raw,{},options);
        const auto end=std::chrono::steady_clock::now();
        elapsed.push_back(std::chrono::duration<double,std::milli>(end-begin).count());
        valid+=result.path.status==PlanStatus::Success;
        improved+=result.accepted>0;
        checks+=result.exactPoseChecks;
        lastCost=result.path.status==PlanStatus::Success?result.path.weightedCostMeters:-1;
      }
      std::cout<<scenario.name<<','<<(curve?"curvature_CSC":"straight_only")<<','<<repetitions
               <<','<<valid<<','<<improved<<','<<std::fixed<<std::setprecision(3)
               <<percentile(elapsed,.5)<<','<<percentile(elapsed,.95)<<','
               <<double(checks)/repetitions<<','<<lastCost<<'\n';
    }
  }
}
