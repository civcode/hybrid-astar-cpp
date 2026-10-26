#include "hybrid_astar/steering/reeds_shepp.hpp"
#include "hybrid_astar/steering/reeds_shepp_evaluator.hpp"
#include <array>
#include <cmath>
#include <memory>
#include <numbers>
#include <stdexcept>
using namespace hybrid_astar;
using namespace hybrid_astar::ackermann;
using namespace hybrid_astar::steering;
static void check(bool yes,const char* reason){if(!yes) throw std::runtime_error(reason);}
int main() {
  ReedsSheppEvaluator evaluator(4.0);
  const Pose2D start{20,20,0};
  check(evaluator.shortestPath(start,start).empty(),"zero RS path");
  const std::array<Pose2D,9> goals{{
    {21,20,0},{19,20,0},{23,22,.5},{17,18,-.7},
    {20,21,std::numbers::pi},{20,20,std::numbers::pi},
    {21,21,-std::numbers::pi/2},{25,20,1.0},{16,23,2.2}
  }};
  bool observedCusp=false;
  for(const auto& goal:goals) {
    const auto path=evaluator.shortestPath(start,goal);
    check(!path.empty(),"nonzero RS chain");
    double total=0;auto pose=start;Gear previous=Gear::None;
    for(const auto& section:path) {
      check(section.gear!=Gear::None&&section.lengthMeters>0,"signed section gear and positive arc progress");
      check(std::hypot(pose.x-section.start.x,pose.y-section.start.y)<1e-8,"section continuity");
      check(std::abs(section.curvature)<=.250000001,"curvature bounded by radius");
      if(previous!=Gear::None&&previous!=section.gear) observedCusp=true;
      previous=section.gear;
      total+=section.lengthMeters;
      pose=section.evaluate(section.lengthMeters);
    }
    check(std::hypot(pose.x-goal.x,pose.y-goal.y)<1e-5,"RS exact terminal XY");
    check(std::abs(std::remainder(pose.yaw-goal.yaw,2*std::numbers::pi))<1e-5,"RS exact terminal yaw");
    check(std::abs(total-evaluator.distance(start,goal))<1e-5,"RS length matches OMPL metric");
  }
  check(observedCusp,"test targets must exercise actual Reeds-Shepp gear reversal");
  check(std::abs(reedsSheppDistance(start,{19,20,0},4)-1)<1e-5,"convenience reverse distance");
  auto map=std::make_shared<MapSnapshot>();
  map->width=map->height=200;map->resolutionMeters=.2;
  map->cells.assign(40000,0);
  PlanRequest request;request.map=map;request.start=start;request.goal={19,20,0};
  const auto basic=planWithReedsShepp(request);
  check(basic.status==PlanStatus::Success&&basic.diagnostics.analyticAttempts==1&&
        basic.diagnostics.analyticAccepted==1,"OMPL automatically connected to complete planner");
  check(basic.heuristicCertifiedAdmissible,"OMPL connector retains certified heuristic default");
  check(basic.finalGear==Gear::Reverse&&basic.reverseLengthMeters>.9,"RS reverse path metadata");
  const auto performance=planWithReedsShepp(request,true);
  check(performance.status==PlanStatus::Success&&!performance.heuristicCertifiedAdmissible,
        "explicit noncertified RS performance ordering");
  request.goal=start;
  const auto identical=planWithReedsShepp(request);
  check(identical.status==PlanStatus::Success&&identical.sections.empty(),"zero-path goal completion");
  return 0;
}
