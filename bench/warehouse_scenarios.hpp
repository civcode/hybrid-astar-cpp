#pragma once
#include "hybrid_astar/ackermann/planner.hpp"
#include <cstddef>
#include <cstdint>
#include <memory>
#include <numbers>

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
} // namespace hybrid_astar::bench
