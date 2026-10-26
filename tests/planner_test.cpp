#include "hybrid_astar/ackermann/planner.hpp"
#include <cmath>
#include <iostream>
#include <memory>
#include <numbers>
#include <stdexcept>

using namespace hybrid_astar;
using namespace hybrid_astar::ackermann;

static MapSnapshot makeMap(double resolution=.2) {
  MapSnapshot m;
  m.width=static_cast<std::size_t>(std::round(40/resolution));
  m.height=m.width; m.resolutionMeters=resolution;
  m.cells.assign(m.width*m.height,0);
  m.revision=7;m.frameId="map";m.sourceId="test";
  return m;
}
static void expect(bool x,const char* message) { if(!x) throw std::runtime_error(message); }

int main() {
  const double pi=std::numbers::pi;
  {
    auto forward=makeSection({1,2,0},Gear::Forward,0,0.5);
    expect(std::abs(forward.end.x-1.5)<1e-12,"forward exact");
    auto reverse=makeSection({1,2,0},Gear::Reverse,0.25,0.5);
    expect(std::abs(reverse.end.y-2.0-(std::cos(0)-std::cos(-0.125))/.25)<1e-12,"reverse arc");
    expect(std::abs(reverse.end.y-2.)>1e-5,"reverse lateral motion");
    expect(std::abs(reverse.end.yaw+0.125)<1e-12,"reverse yaw");
    auto rotated=makeSection({1,2,pi/2},Gear::Forward,0,.5);
    expect(std::abs(rotated.end.x-1)<1e-12 && std::abs(rotated.end.y-2.5)<1e-12,"SE2 exact");
  }
  for(double r:{.2,.1,.05,.025}) {
    auto m=makeMap(r);
    const std::size_t ix=static_cast<std::size_t>(20/r),iy=static_cast<std::size_t>(20/r);
    m.cells[iy*m.width+ix]=100; // blocked cell enclosed by entire body
    ExactCollisionChecker c(m,{},{});
    expect(c.check({20,20,0})==PoseCollision::Colliding,"enclosed cell");
    m.cells[iy*m.width+ix]=0;
    ExactCollisionChecker clear(m,{},{});
    expect(clear.check({20,20,0})==PoseCollision::Clear,"free body");
  }
  {
    auto m=makeMap();m.origin={-20,-20,pi/2};
    ExactCollisionChecker c(m,{},{});
    expect(c.check({-40,0,pi/2})==PoseCollision::Clear,"rotated map");
    expect(c.check({-20,0,0})==PoseCollision::Colliding,"map boundary");
  }
  {
    auto m=makeMap();m.cells[100*m.width+100]=-1;
    expect(ExactCollisionChecker(m,{},{}).check({20,20,0})==PoseCollision::Colliding,"unknown occupied");
    OccupancyPolicy free{0,UnknownPolicy::Free};
    expect(ExactCollisionChecker(m,free,{}).check({20,20,0})==PoseCollision::Clear,"unknown free");
    try {
      ExactCollisionChecker rejected(m,{0,UnknownPolicy::Reject},{});
      throw std::runtime_error("Reject accepted");
    } catch(const std::domain_error&) {}
    m.cells[0]=101;
    try {
      ExactCollisionChecker invalid(m,{},{});
      throw std::runtime_error("Malformed occupancy accepted");
    } catch(const std::invalid_argument&) {}
  }
  {
    auto m=std::make_shared<MapSnapshot>(makeMap());
    PlanRequest request;
    request.map=m;request.start={12,12,0};request.goal={15,12,0};
    request.options.maxExpansions=15000;
    auto result=plan(request);
    expect(result.status==PlanStatus::Success,"free planner succeeds");
    expect(!result.sections.empty(),"has path sections");
    expect(result.goalXYErrorMeters<=request.options.goalXYToleranceMeters,"goal xy");
    expect(result.goalYawErrorRadians<=request.options.goalYawToleranceRadians,"goal yaw");
    expect(result.collisionEvidence==CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled,"sampled evidence");
    expect(!result.trajectoryActuatorFeasible,"no actuator assertion");
    expect(result.mapRevision==7,"map provenance");
    expect(result.weightedCostMeters>0,"costed motion");
    request.options.maxExpansions=1;
    expect(plan(request).status==PlanStatus::BudgetExceeded,"bounded work not no path");
    request.options.maxExpansions=15000;
    m->cells[60*m->width+60]=100;
    expect(plan(request).status==PlanStatus::StartInCollision,"start collision");
    request.occupancy.unknown=UnknownPolicy::Reject;
    m->cells[5]=-1;
    expect(plan(request).status==PlanStatus::RejectedUnknownMap,"whole unknown rejection");
    request.occupancy.unknown=UnknownPolicy::Occupied;
    request.options.maxSamplesPerSection=2;
    m->cells[60*m->width+60]=0;
    expect(plan(request).status==PlanStatus::ValidationIncomplete,"sampling budget incomplete");
    m->cells[0]=101;
    expect(plan(request).status==PlanStatus::InvalidMap,"malformed snapshot distinct");
  }
  std::cout<<"planner tests passed\n";
}
