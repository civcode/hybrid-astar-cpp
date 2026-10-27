#pragma once
#include "hybrid_astar/ackermann/planner.hpp"
#include <cstddef>
#include <cstdint>
#include <memory>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

namespace hybrid_astar::bench {
// Shared reproduction for manual performance measurements and Catch2.
// Matches the Planning Lab 200 x 160, 0.25 m warehouse obstacle layout.
inline ackermann::PlanRequest mecanumWarehouseCase(
    std::size_t maxExpansions = 1'000'000) {
  using namespace ackermann;
  auto map=std::make_shared<MapSnapshot>();
  map->width=200;map->height=160;map->resolutionMeters=.25;
  map->frameId="warehouse/map";map->sourceId="warehouse-repro";map->revision=1313;
  map->cells.assign(map->width*map->height,0);
  const auto occupied=[&](std::size_t x0,std::size_t y0,
                          std::size_t x1,std::size_t y1) {
    for(std::size_t y=y0;y<y1;++y)
      for(std::size_t x=x0;x<x1;++x)map->cells[y*map->width+x]=100;
  };
  occupied(48,12,53,48);occupied(48,92,53,145);
  occupied(94,0,99,46);occupied(94,100,99,160);
  occupied(141,12,146,55);occupied(141,104,146,145);
  PlanRequest request;
  request.map=map;
  request.start={4.77,5.62,0};
  request.goal={31.01,3.41,-.5*std::numbers::pi_v<double>};
  request.footprint={-.35,.35,.25};
  request.options.model=PlanOptions::Model::Mecanum;
  request.options.maxExpansions=maxExpansions;
  // Keep this benchmark's generated-node ceiling proportional to its budget.
  request.options.maxGeneratedNodes=4*maxExpansions;
  return request;
}
// Manual hot-path workloads based on two Planning Lab screenshots. They use
// the SAME occupancy layout as the app and the default full-size Ackermann
// rectangle. No long-running case is registered in CTest.
// The previously reported far-corner screenshot did not show the numeric
// inflation value: run 0, 0.1 and 0.5 m rather than assuming one.
inline std::vector<std::pair<std::string,ackermann::PlanRequest>>
ackermannWarehouseCases(std::size_t maxExpansions=1'000'000) {
  using namespace ackermann;
  auto common=mecanumWarehouseCase(maxExpansions);
  common.options.model=PlanOptions::Model::Ackermann;
  common.footprint=PlanRequest{}.footprint;
  common.options.heuristicMode=
      HeuristicMode::PerformanceReedsSheppAndObstacleGrid;
  std::vector<std::pair<std::string,PlanRequest>> cases;
  const auto add=[&](std::string label,Pose2D start,Pose2D goal,
                     double inflation,bool improve=false) {
    auto r=common;
    r.start=start;r.goal=goal;
    r.options.footprintInflationMeters=inflation;
    r.options.solutionPolicy=improve?SolutionPolicy::ImproveIncumbent:
                                      SolutionPolicy::FirstStrictlyValidated;
    if(improve)r.options.maxExtraExpansionsAfterFirst=700;
    cases.emplace_back(std::move(label),std::move(r));
  };
  const double down=-.5*std::numbers::pi_v<double>;
  // Reported success after 428853 expansions at 0.50 m inflation.
  add("ack_across_first_margin_050",{4.77,32.84,0},
      {44.62,33.53,down},.5);
  add("ack_across_improve_margin_050",{4.77,32.84,0},
      {44.62,33.53,down},.5,true);
  // Reported BudgetExceeded at 1M expansions; margin unknown in screenshot.
  add("ack_corner_margin_000",{3.25,3.64,0},
      {30.93,38.17,down},0);
  add("ack_corner_margin_010",{3.25,3.64,0},
      {30.93,38.17,down},.1);
  add("ack_corner_margin_050",{3.25,3.64,0},
      {30.93,38.17,down},.5);
  return cases;
}
} // namespace hybrid_astar::bench
