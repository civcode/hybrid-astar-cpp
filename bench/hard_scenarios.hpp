#pragma once
#include "hybrid_astar/ackermann/planner.hpp"
#include <algorithm>
#include <cmath>
#include <utility>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>
namespace hybrid_astar::bench {
struct HardCase {
  std::string name;
  ackermann::PlanRequest request;
  bool baselineExpectedSuccess = false;
  bool impossibleByConstruction = false;
};
inline void fillRect(ackermann::MapSnapshot& map,double x0,double y0,
                     double x1,double y1) {
  const double r=map.resolutionMeters;
  const auto left=static_cast<std::size_t>(std::max(0.,std::floor(x0/r)));
  const auto right=static_cast<std::size_t>(std::min(double(map.width),std::ceil(x1/r)));
  const auto bottom=static_cast<std::size_t>(std::max(0.,std::floor(y0/r)));
  const auto top=static_cast<std::size_t>(std::min(double(map.height),std::ceil(y1/r)));
  for(std::size_t y=bottom;y<top;++y)
    for(std::size_t x=left;x<right;++x)map.cells[y*map.width+x]=100;
}
inline std::vector<HardCase> hardCases() {
  using namespace ackermann;
  auto open=std::make_shared<MapSnapshot>();
  open->width=180;open->height=140;open->resolutionMeters=.25;
  open->frameId="hard/map";open->sourceId="hard-fixtures";
  open->cells.assign(open->width*open->height,0);
  auto make=[&](std::string name,Pose2D start,Pose2D goal,
                std::size_t budget,bool expected,bool impossible=false) {
    auto map=std::make_shared<MapSnapshot>(*open);
    map->revision=1+std::size_t(name.size())*100+budget;
    PlanRequest request;
    request.map=map;request.start=start;request.goal=goal;
    request.options.maxExpansions=budget;
    request.options.maxGeneratedNodes=60000;
    request.options.gridMaxTotalSettlements=7000;
    request.options.gridMaxSettlementsPerQuery=200;
    return HardCase{std::move(name),std::move(request),expected,impossible};
  };
  std::vector<HardCase> cases;
  cases.push_back(make("reverse_approach",{20,17,0},{16,17,0},1500,true));
  cases.push_back(make("three_point_turn",{20,17,0},{18,18,0},12000,true));
  auto parking=make("parallel_parking",{23,13,0},{17,14,0},6000,false);
  auto parked=std::make_shared<MapSnapshot>(*parking.request.map);
  fillRect(*parked,11,15.7,14,17.8);
  fillRect(*parked,21,15.7,24,17.8);
  parking.request.map=parked;cases.push_back(parking);
  auto corridor=make("narrow_corridor",{10,17.5,0},{29,17.5,0},5000,true);
  auto hallway=std::make_shared<MapSnapshot>(*corridor.request.map);
  fillRect(*hallway,6,12,37,15.5);
  fillRect(*hallway,6,19.5,37,23);
  corridor.request.map=hallway;cases.push_back(corridor);
  auto dead=make("dead_end",{18,17,0},{31,17,0},3500,false);
  auto culdesac=std::make_shared<MapSnapshot>(*dead.request.map);
  fillRect(*culdesac,13,11,31,13);
  fillRect(*culdesac,13,22,31,24);
  fillRect(*culdesac,29,11,31,24);
  dead.request.map=culdesac;cases.push_back(dead);
  auto detour=make("obstacle_detour",{12,15,0},{34,15,0},8000,false);
  auto wall=std::make_shared<MapSnapshot>(*detour.request.map);
  fillRect(*wall,23,5,23.5,24); // drive around top or bottom end
  detour.request.map=wall;cases.push_back(detour);
  auto blocked=make("sub_vehicle_width_gap",{11,17,0},{33,17,0},1600,false,true);
  auto impassable=std::make_shared<MapSnapshot>(*blocked.request.map);
  fillRect(*impassable,22,0,22.5,16.875);
  fillRect(*impassable,22,17.125,22.5,35);
  blocked.request.map=impassable;cases.push_back(blocked);
  return cases;
}
} // namespace hybrid_astar::bench
