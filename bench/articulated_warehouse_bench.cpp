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
       mode!="analytic-only"&&mode!="combined"&&mode!="weighted")
      throw std::invalid_argument(
          "Use legacy | coupled | coupled-no-fast | analytic-only | combined | weighted");
    auto map=std::make_shared<ac::MapSnapshot>();
    map->width=200;map->height=160;map->resolutionMeters=.25;
    map->origin={0,0,0};
    map->frameId="demo/map";map->sourceId="built-in";map->revision=1;
    map->cells.assign(map->width*map->height,0);
    const auto rectangle=[&](int x0,int y0,int x1,int y1) {
      for(int y=y0;y<y1;++y)for(int x=x0;x<x1;++x)
        map->cells[std::size_t(y)*map->width+std::size_t(x)]=100;
    };
    rectangle(48,12,53,48);rectangle(48,92,53,145);
    rectangle(94,0,99,46);rectangle(94,100,99,160);
    rectangle(141,12,146,55);rectangle(141,104,146,145);
    as::Request request;
    request.map=map;
    request.start={5.0,30.56,0.0,0.0};
    request.goal={18.16,27.98,0.0,0.0};
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
    if(mode=="combined"||mode=="weighted")
      request.options.heuristicMode=as::HeuristicMode::Combined;
    // Weighted is explicitly a non-optimal, experimental search ordering.
    if(mode=="weighted")request.options.heuristicWeight=1.5;
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
      "elapsed_ms,stationary_s,coupled_s,drive_m,swept_free,grid_settlements\n";
    std::cout<<mode<<','<<static_cast<int>(result.status)<<','
      <<result.independentlySampledBodyValidated<<','
      <<result.diagnostics.expanded<<','<<result.diagnostics.generated<<','
      <<result.diagnostics.searchBodyChecks<<','<<elapsed<<','
      <<stationary<<','<<simultaneous<<','<<distance<<','
      <<result.diagnostics.searchSweptAabbAccepted<<','
      <<result.diagnostics.gridSettlements<<'\n';
    return result.status==as::Status::Success?0:2;
  } catch(const std::exception& e) {
    std::cerr<<e.what()<<'\n';
    return 1;
  }
}
