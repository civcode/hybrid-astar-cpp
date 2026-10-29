// Heavy manual benchmark, not automatically executed by ordinary CTest.
// CSV to stdout, discovery/profiling diagnostics to stderr.
#include "tests/nav2_warehouse_pipeline_benchmark_support.hpp"
#include <exception>
#include <iostream>

int main(int argc,char** argv) {
  using namespace hybrid_astar::bench::nav2_pipeline;
  try {
    const auto config=parseArgs(argc,argv);
    const auto loadStart=Clock::now();
    const auto map=hybrid_astar::demo::loadRosMapYaml(config.yamlPath);
    const auto loadEnd=Clock::now();
    if(map->width!=1006||map->height!=1674||
       std::abs(map->resolutionMeters-.03)>1e-12)
      throw std::invalid_argument("Expected ORIGINAL Nav2 warehouse PGM: 1006x1674 at 0.03 m/cell, not resampled map");
    const auto request=baseRequest(config,map);
    const auto scenarios=discover(config,request);
    std::cerr<<std::fixed<<std::setprecision(3)
      <<"Map load (untimed in pipeline)="<<millis(loadStart,loadEnd)<<" ms"
      <<", bytes of occupancy="<<map->cells.size()
      <<", model="<<config.model
      <<", connector="<<config.connector
      <<", curved="<<config.curved
      <<", max_accepted="<<config.acceptedLimit
      <<", cases="<<scenarios.size()
      <<", repeats="<<config.repeats
      <<". Each row is a repeated full plan+refine, not just optimization.\n";
#ifdef HYBRID_ASTAR_NAV2_PIPELINE_PROFILE_ENABLED
    std::cerr<<"Hotpath profiling ENABLED. Timed build has clock overhead; "
                "motion includes nested collision timings. Never sum overlapping hotpaths.\n";
#else
    std::cerr<<"Hotpath profiling disabled (fast/low-overhead timing). "
                "Build with -DHYBRID_ASTAR_PROFILE_HOTPATHS=ON for hotpath breakdown.\n";
#endif
    header(); // no partial CSV when randomized discovery can't meet thresholds
    for(const auto& scenario:scenarios) {
      const auto runMargin=[&](double margin) {
        auto paired=request;
        paired.start=scenario.endpoints.start;
        paired.goal=scenario.endpoints.goal;
        paired.options.footprintInflationMeters=margin;
        auto m=measure(config,paired);
        row(config,scenario,paired,m);
        std::cerr<<"Case "<<scenario.id<<", margin="<<margin
          <<", status="<<statusName(m.raw.status)
          <<", expanded="<<m.raw.diagnostics.expanded
          <<", search_p50="<<m.searchP50<<" ms, refine_p50="<<m.refineP50
          <<" ms, total_p50="<<m.totalP50<<" ms\n";
      };
      // Balance variant order across cases to reduce systematic cache bias.
      if(scenario.id%2==0){runMargin(0.0);runMargin(.1);}
      else{runMargin(.1);runMargin(0.0);}
    }
    return 0;
  } catch(const std::exception& e) {
    std::cerr<<"Nav2 warehouse pipeline benchmark failed: "<<e.what()<<'\n';
    return 2;
  }
}
