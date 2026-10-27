#pragma once

#include "hybrid_astar/ackermann/planner.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <numbers>

namespace hybrid_astar::bench {

// The 200x160, 0.25 m warehouse map with the 143 occupied-cell edits exported
// from Planning Lab (revision 187). Reused unchanged by the fast A/B/C
// regression and the manual full-budget benchmark.
inline ackermann::PlanRequest warehouseReedsSheppRequest() {
  using namespace ackermann;
  auto map=std::make_shared<MapSnapshot>();
  map->width=200;
  map->height=160;
  map->resolutionMeters=.25;
  map->frameId="demo/map";
  map->sourceId="built-in";
  map->revision=187;
  map->cells.assign(map->width*map->height,0);
  const auto rectangle=[&](int x0,int y0,int x1,int y1) {
    for(int y=y0;y<y1;++y)
      for(int x=x0;x<x1;++x)
        map->cells[std::size_t(y)*map->width+std::size_t(x)]=100;
  };
  // Built-in warehouse.
  rectangle(48,12,53,48);rectangle(48,92,53,145);
  rectangle(94,0,99,46);rectangle(94,100,99,160);
  rectangle(141,12,146,55);rectangle(141,104,146,145);

  struct Run {int x,y,length;};
  constexpr std::array<Run,34> edits{{
    {126,99,1},
    {125,100,3},{125,101,3},{125,102,3},{125,103,3},
    {125,104,3},{125,105,3},{125,106,3},{125,107,3},
    {125,108,3},{125,109,3},{125,110,3},{125,111,3},
    {125,112,3},{125,113,3},{125,114,4},
    {126,115,3},{126,116,3},{126,117,3},{126,118,3},
    {100,119,1},{112,119,2},{115,119,4},{120,119,10},
    {99,120,8},{108,120,2},{111,120,19},
    {99,121,15},{115,121,4},{120,121,9},
    {100,122,1},{102,122,5},{108,122,2},{111,122,2}
  }};
  for(const auto edit:edits)
    rectangle(edit.x,edit.y,edit.x+edit.length,edit.y+1);

  PlanRequest request;
  request.map=map;
  request.start={5.68,6.68,0.0};
  request.goal={30.32,35.13,-std::numbers::pi_v<double>/2};
  request.options.model=PlanOptions::Model::Ackermann;
  request.options.heuristicMode=HeuristicMode::PerformanceReedsSheppAndObstacleGrid;
  request.options.footprintInflationMeters=.5;
  request.options.maxExpansions=1'000'000;
  request.options.maxGeneratedNodes=4'000'000;
  return request;
}

} // namespace hybrid_astar::bench
