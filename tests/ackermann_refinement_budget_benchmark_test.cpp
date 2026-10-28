// Manual/opt-in CTest benchmark: keep the objective at its historical default.
// Both budgets see the identical precomputed original path and strict pose cap.
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
          Variant{"legacy_35_3",35,3,0.0},
          Variant{"expanded_160_8",160,8,0.0}}) {
        const auto outcome=run(scenario,config,variant);
        printRow(scenario,config,variant,outcome);
      }
    }
    return 0;
  } catch(const std::exception& e) {
    std::cerr<<"Ackermann budget benchmark failed: "<<e.what()<<'\n';
    return 2;
  }
}
