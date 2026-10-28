// Manual/opt-in CTest benchmark: two budgets x two steering objectives.
// No claim of universal monotonic smoothness: report independent geometric
// metrics and let the data show where fixed-curvature CSC proposals help.
#include "tests/ackermann_refinement_benchmark_support.hpp"
#include <exception>
#include <iostream>

int main(int argc,char** argv) {
  using namespace hybrid_astar::bench::ackermann_refinement;
  try {
    const auto config=parseArgs(argc,argv);
    const auto selected=discover(config);
    printHeader();
    for(const auto& scenario:selected) {
      for(const Variant variant: {
          Variant{"legacy_budget_length",35,3,0.0},
          Variant{"expanded_budget_length",160,8,0.0},
          Variant{"legacy_budget_steering",35,3,1.0},
          Variant{"expanded_budget_steering",160,8,1.0}}) {
        const auto outcome=run(scenario,config,variant);
        printRow(scenario,config,variant,outcome);
      }
    }
    return 0;
  } catch(const std::exception& e) {
    std::cerr<<"Ackermann steering-objective benchmark failed: "<<e.what()<<'\n';
    return 2;
  }
}
