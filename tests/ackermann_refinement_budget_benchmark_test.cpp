// Manual/opt-in CTest benchmark: keep the objective at its historical default.
// Both budgets refine the same validated raw path with their respective
// historical strict pose-check caps (100k versus 200k).
#include "tests/ackermann_refinement_benchmark_support.hpp"
#include <exception>
#include <iostream>

int main(int argc,char** argv) {
  using namespace hybrid_astar::bench::ackermann_refinement;
  try {
    const auto config=parseArgs(argc,argv);
    const auto selected=discover(config);
    printHeader();
    const std::array<Variant,2> variants{{
      {"legacy_35_3",35,3,100000,0.0},
      {"expanded_160_8",160,8,200000,0.0}
    }};
    std::array<Summary,2> summaries{};
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
    std::cerr<<"Ackermann budget benchmark failed: "<<e.what()<<'\n';
    return 2;
  }
}
