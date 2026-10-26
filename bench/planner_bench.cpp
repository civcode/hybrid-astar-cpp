// Manually run only; no GitHub Actions / unattended CI benchmark trigger.
#include "hybrid_astar/ackermann/planner.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

using hybrid_astar::Pose2D;
using namespace hybrid_astar::ackermann;

struct Scenario {
  std::string name;
  std::shared_ptr<const MapSnapshot> map;
  Pose2D start,goal;
  bool performance=false;
  std::size_t expansionBudget=10000;
};
static std::shared_ptr<MapSnapshot> emptyMap() {
  auto map=std::make_shared<MapSnapshot>();
  map->resolutionMeters=.2;map->width=map->height=400;
  map->cells.assign(map->width*map->height,0);
  map->revision=1;map->sourceId="bench";map->frameId="map";
  return map;
}
static std::vector<Scenario> cases() {
  auto empty=emptyMap();
  auto wall=std::make_shared<MapSnapshot>(*empty);
  wall->revision=2;
  for (std::size_t y=60;y<340;++y) {
    if (y>=270&&y<315) continue; // 9 m passage, not a point-mass slit
    wall->cells[y*wall->width+200]=100;
  }
  return {{"free_forward",empty,{20,20,0},{26,20,0}},
          {"free_reverse",empty,{20,20,0},{14,20,0}},
          {"distant_doorway_euclidean",wall,{32,40,0},{48,40,0},false,10000},
          {"distant_doorway_grid",wall,{32,40,0},{48,40,0},true,50000}};
}
static double quantile(const std::vector<double>& sorted,double fraction) {
  if (sorted.empty()) throw std::runtime_error("No observations");
  const std::size_t index=static_cast<std::size_t>(std::ceil(fraction*sorted.size()));
  return sorted[std::max<std::size_t>(1,index)-1];
}
int main(int argc,char** argv) {
  try {
    const int repeats=argc>1?std::stoi(argv[1]):10;
    if (repeats<1||repeats>1000) throw std::invalid_argument("Repetitions must be 1..1000");
    for (const auto& scenario:cases()) {
      std::vector<double> milliseconds;
      std::size_t success=0,expanded=0,generated=0,finalChecks=0;
      double lastValidatedCost=0;
      for (int run=0;run<repeats;++run) {
        PlanRequest request;
        request.map=scenario.map;request.start=scenario.start;request.goal=scenario.goal;
        request.options.maxExpansions=scenario.expansionBudget;
        if(scenario.performance)
          request.options.heuristicMode=HeuristicMode::PerformanceReedsSheppAndObstacleGrid;
        const auto start=std::chrono::steady_clock::now();
        const auto result=plan(request);
        const auto finish=std::chrono::steady_clock::now();
        milliseconds.push_back(std::chrono::duration<double,std::milli>(finish-start).count());
        if (result.status==PlanStatus::Success) {++success;lastValidatedCost=result.weightedCostMeters;}
        expanded+=result.diagnostics.expanded;
        generated+=result.diagnostics.generated;
        finalChecks+=result.diagnostics.finalPoseChecks;
      }
      std::sort(milliseconds.begin(),milliseconds.end());
      std::cout << scenario.name << ",runs=" << repeats
                << ",heuristic=" << (scenario.performance?"performance_grid":"certified_euclidean")
                << ",expansion_budget=" << scenario.expansionBudget << ",success=" << success
                << ",p50_ms=" << quantile(milliseconds,.50)
                << ",p95_ms=" << quantile(milliseconds,.95)
                << ",p99_ms=" << quantile(milliseconds,.99)
                << ",mean_expanded=" << expanded/repeats
                << ",mean_generated=" << generated/repeats
                << ",mean_final_pose_checks=" << finalChecks/repeats
                << ",last_validated_cost_m=" << lastValidatedCost << '\n';
    }
  } catch (const std::exception& e) {
    std::cerr << "Benchmark configuration error: " << e.what() << '\n';
    return 2;
  }
}
