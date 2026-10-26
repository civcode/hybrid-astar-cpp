// Manually invoked C++20 benchmark; never runs in GitHub Actions.
#include "bench/hard_scenarios.hpp"
#include "hybrid_astar/ackermann/planner.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace hybrid_astar;
using namespace hybrid_astar::ackermann;
namespace {
struct Variant {const char* label;OpenQueueMode open,grid;};
constexpr Variant variants[]={
  {"binary_baseline",OpenQueueMode::BinaryLazy,OpenQueueMode::BinaryLazy},
  {"four_ary_open",OpenQueueMode::FourAryLazy,OpenQueueMode::BinaryLazy},
  {"indexed_binary_open",OpenQueueMode::IndexedBinary,OpenQueueMode::BinaryLazy},
  {"indexed_four_ary_open",OpenQueueMode::IndexedFourAry,OpenQueueMode::BinaryLazy},
  {"four_ary_grid",OpenQueueMode::BinaryLazy,OpenQueueMode::FourAryLazy},
  {"indexed_binary_grid",OpenQueueMode::BinaryLazy,OpenQueueMode::IndexedBinary},
  {"indexed_four_ary_grid",OpenQueueMode::BinaryLazy,OpenQueueMode::IndexedFourAry},
  {"four_ary_open_indexed_grid",OpenQueueMode::FourAryLazy,OpenQueueMode::IndexedBinary}
};
double percentile(std::vector<double> v,double fraction) {
  std::sort(v.begin(),v.end());
  const auto index=std::max(std::size_t(1),std::size_t(std::ceil(v.size()*fraction)))-1;
  return v.at(index);
}
void equivalent(const PlanResult& left,const PlanResult& right) {
  if(left.status!=right.status || left.diagnostics.expanded!=right.diagnostics.expanded ||
     left.diagnostics.generated!=right.diagnostics.generated ||
     left.diagnostics.gridSettlements!=right.diagnostics.gridSettlements ||
     left.sections.size()!=right.sections.size() ||
     std::abs(left.weightedCostMeters-right.weightedCostMeters)>1e-9 ||
     left.collisionEvidence!=right.collisionEvidence)
    throw std::runtime_error("Queue implementation changed geometric search result");
}
}
int main(int argc,char** argv) {
  try {
    const int repetitions=argc>1?std::stoi(argv[1]):5;
    if(repetitions<1||repetitions>100)throw std::invalid_argument("repetitions must be 1..100");
    std::cout<<"case,variant,repeats,p50_ms,p95_ms,status,expanded,generated,"
      "cost,open_pushes,open_pops,open_stale,open_replaced,open_peak_entries,open_peak_bytes,"
      "grid_pushes,grid_pops,grid_stale,grid_peak_entries,grid_peak_bytes\n";
    for(const auto& item:bench::hardCases()) {
      auto request=item.request;
      request.options.heuristicMode=HeuristicMode::PerformanceReedsSheppAndObstacleGrid;
      request.options.performanceHeuristicWeight=1.2;
      request.options.gridMaxTotalSettlements=20000;
      PlanResult baseline;
      bool baselineReady=false;
      for(const auto& v:variants) {
        request.options.openQueueMode=v.open;
        request.options.gridQueueMode=v.grid;
        std::vector<double> timings;
        PlanResult last;
        for(int i=0;i<repetitions+1;++i) {
          const auto begin=std::chrono::steady_clock::now();
          auto result=plan(request);
          const auto end=std::chrono::steady_clock::now();
          if(baselineReady)equivalent(baseline,result);
          if(i!=0)timings.push_back(std::chrono::duration<double,std::milli>(end-begin).count());
          last=std::move(result);
        }
        if(!baselineReady){baseline=last;baselineReady=true;}
        const auto& d=last.diagnostics;
        std::cout<<item.name<<','<<v.label<<','<<repetitions<<','
           <<std::fixed<<std::setprecision(3)<<percentile(timings,.5)<<','
           <<percentile(timings,.95)<<','<<int(last.status)<<','<<d.expanded<<','
           <<d.generated<<','<<last.weightedCostMeters<<','
           <<d.openQueuePushes<<','<<d.openQueuePops<<','<<d.staleHeapPops<<','
           <<d.openQueueReplacements<<','<<d.openQueuePeakSize<<','
           <<d.openQueuePeakStorageBytes<<','<<d.gridQueuePushes<<','
           <<d.gridQueuePops<<','<<d.gridQueueStalePops<<','
           <<d.gridQueuePeakSize<<','<<d.gridQueuePeakStorageBytes<<'\n';
      }
    }
  }catch(const std::exception& e) {
    std::cerr<<"Queue benchmark failed: "<<e.what()<<'\n';return 2;
  }
}
