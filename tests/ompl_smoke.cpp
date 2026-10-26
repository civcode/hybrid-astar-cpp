#include "hybrid_astar/steering/reeds_shepp.hpp"
#include "hybrid_astar/steering/reeds_shepp_evaluator.hpp"
#include <cmath>
#include <memory>
#include <stdexcept>

int main() {
  using hybrid_astar::Pose2D;
  using hybrid_astar::steering::ReedsSheppEvaluator;
  using hybrid_astar::steering::reedsSheppDistance;
  const Pose2D origin{};
  ReedsSheppEvaluator evaluator(4.5);
  if (std::abs(evaluator.distance(origin,origin))>1.e-8) return 1;
  if (std::abs(reedsSheppDistance(origin,Pose2D{1,0,0},4.5)-1.)>1.e-6) return 2;
  if (std::abs(evaluator.distance(origin,Pose2D{-1,0,0})-1.)>1.e-6) return 3;

  for (const Pose2D target : {Pose2D{1,0,0},Pose2D{-1,0,0},Pose2D{4,2,0.7}}) {
    auto sections=evaluator.shortestPath(origin,target);
    double length=0;
    Pose2D reached=origin;
    if (sections.empty()) return 4;
    for (const auto& section : sections) {
      if (std::hypot(section.start.x-reached.x,section.start.y-reached.y)>1.e-6) return 5;
      reached=section.evaluate(section.lengthMeters);
      length+=section.lengthMeters;
    }
    if (std::hypot(reached.x-target.x,reached.y-target.y)>1.e-5 ||
        std::abs(std::remainder(reached.yaw-target.yaw,2*3.14159265358979323846))>1.e-5) return 6;
    if (std::abs(length-evaluator.distance(origin,target))>1.e-5) return 7;
  }

  auto m=std::make_shared<hybrid_astar::ackermann::MapSnapshot>();
  m->width=m->height=200; m->resolutionMeters=0.2;
  m->cells.assign(m->width*m->height,0);
  hybrid_astar::ackermann::PlanRequest request;
  request.map=m;
  request.start={20,20,0};request.goal={19,20,0};
  request.analyticConnector=[&](const Pose2D& p,const Pose2D& g) {
    return evaluator.shortestPath(p,g);
  };
  auto result=hybrid_astar::ackermann::plan(request);
  if (result.status!=hybrid_astar::ackermann::PlanStatus::Success ||
      result.diagnostics.analyticAttempts!=1 || result.diagnostics.analyticAccepted!=1)
    return 8;
  return 0;
}
