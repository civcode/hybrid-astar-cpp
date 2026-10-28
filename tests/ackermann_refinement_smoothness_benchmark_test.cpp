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
    const std::array<Variant,4> variants{{
      {"legacy_budget_length",35,3,100000,0.0},
      {"expanded_budget_length",160,8,200000,0.0},
      {"legacy_budget_steering",35,3,100000,config.steeringWeightMetersSquared},
      {"expanded_budget_steering",160,8,200000,config.steeringWeightMetersSquared}
    }};
    std::array<Summary,4> summaries{};
    for(std::size_t i=0;i<variants.size();++i)
      summaries[i].label=variants[i].name;
    for(const auto& scenario:selected) {
      for(std::size_t i=0;i<variants.size();++i) {
        const auto outcome=run(scenario,config,variants[i]);
        printRow(scenario,config,variants[i],outcome);
        summaries[i].add(scenario,outcome);
      }
    }
    for(const auto& summary:summaries)summary.report();
    return 0;
  } catch(const std::exception& e) {
    std::cerr<<"Ackermann steering-objective benchmark failed: "<<e.what()<<'\n';
    return 2;
  }
}
