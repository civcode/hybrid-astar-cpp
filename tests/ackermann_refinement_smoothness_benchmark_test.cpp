// Manual/opt-in CTest: 35/3 legacy plus 160/8, 160/12 and 160/16,
// each with the length-only and steering-aware objective.
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
    const auto variants=smoothnessAcceptanceBudgetVariants(config);
    std::array<Summary,8> summaries{};
    for(std::size_t i=0;i<variants.size();++i)
      summaries[i].label=variants[i].name;
    for(const auto& scenario:selected) {
      for(std::size_t i=0;i<variants.size();++i) {
        const auto outcome=run(scenario,config,variants[i]);
        printRow(scenario,config,variants[i],outcome);
        summaries[i].add(scenario,outcome,variants[i]);
      }
    }
    for(const auto& summary:summaries)summary.report();
    return 0;
  } catch(const std::exception& e) {
    std::cerr<<"Ackermann steering-objective benchmark failed: "<<e.what()<<'\n';
    return 2;
  }
}
