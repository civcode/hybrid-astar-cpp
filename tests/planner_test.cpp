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
    auto m=makeMap();
    OccupancyPolicy p{25,UnknownPolicy::Occupied};
    m.cells[100*m.width+100]=25;
    expect(ExactCollisionChecker(m,p,{}).check({20,20,0})==PoseCollision::Clear,"threshold equality free");
    p.maxTraversableOccupancy=24;
    expect(ExactCollisionChecker(m,p,{}).check({20,20,0})==PoseCollision::Colliding,"threshold greater blocked");
  }
  {
    // Nominal goal collides, but an actual in-tolerance pose is free.
    auto m=std::make_shared<MapSnapshot>(makeMap());
    m->cells[75*m->width+94]=100;
    PlanRequest q;
    q.map=m;q.start={15.12,15,0};q.goal={15.30,15,0};
    expect(ExactCollisionChecker(*m,{},{}).check(q.goal)==PoseCollision::Colliding,"goal nominal blocked");
    auto result=plan(q);
    expect(result.status==PlanStatus::Success,"free tolerance pose accepted");
    expect(result.sections.empty()&&std::abs(result.reached.x-q.start.x)<1e-12,"no fake nominal endpoint");
  }
  {
    auto m=std::make_shared<MapSnapshot>(makeMap());
    PlanRequest q;
    q.map=m;q.start={20,20,0};q.goal={18,20,0};
    const auto reverse=plan(q);
    expect(reverse.status==PlanStatus::Success,"reverse plan accepted");
    expect(reverse.finalGear==Gear::Reverse,"incoming reverse gear preserved");
    expect(reverse.sections.size()==4,"four reverse arc sections");
    expect(std::abs(reverse.weightedCostMeters-2.20)<1e-9,"reverse weighted length");
    for (const auto& s: reverse.sections) expect(s.gear==Gear::Reverse,"gear for each section");
  }
  {
    auto m=std::make_shared<MapSnapshot>(makeMap());
    PlanRequest q;
    q.map=m;q.start={20,20,0};q.goal={21,20,0};
    q.analyticConnector=[](const Pose2D& start,const Pose2D&) {
      auto reverse=makeSection(start,Gear::Reverse,0,1);
      auto forward=makeSection(reverse.end,Gear::Forward,0,2);
      return std::vector<MotionSection>{reverse,forward};
    };
    auto result=plan(q);
    expect(result.status==PlanStatus::Success,"analytic callback connected");
    expect(result.diagnostics.analyticAttempts==1 && result.diagnostics.analyticAccepted==1,"first-node attempt");
    expect(result.sections.size()==2 && result.finalGear==Gear::Forward,"analytic cusp preserved");
    expect(std::abs(result.weightedCostMeters-3.6)<1e-9,"analytic cost including cusp");
    expect(std::abs(result.geometricLengthMeters-3.0)<1e-9,"analytic geometric length");
    expect(std::abs(result.reverseLengthMeters-1.0)<1e-9,"reverse section length");
    expect(result.gearChangeSectionIndices.size()==1 && result.gearChangeSectionIndices[0]==1,"explicit cusp event");
    q.options.analytic.maxWeightedCostMeters=2.0;
    auto fallback=plan(q);
    expect(fallback.status==PlanStatus::Success,"cost-gated connector falls back");
    expect(fallback.diagnostics.analyticAccepted==0,"analytic cost cap respected");
    q.options.analytic.maxWeightedCostMeters.reset();
    q.analyticConnector=[](const Pose2D& p,const Pose2D&) {
      auto invalid=makeSection(p,Gear::Forward,0,1);
      invalid.end.x+=2.0;
      return std::vector<MotionSection>{invalid};
    };
    expect(plan(q).status==PlanStatus::Success,"invalid analytic output cannot abort search");
  }
  {
    // Deliberately undersampled search misses an obstacle; independent strict
    // final validation MUST reject the candidate, never return Success.
    auto m=std::make_shared<MapSnapshot>();
    m->width=m->height=300;m->resolutionMeters=.01;
    m->cells.assign(90000,0);m->cells[100*300+125]=100;
    PlanRequest q;
    q.map=m;q.footprint={-.02,.02,.02};q.start={1,1,0};q.goal={1.5,1,0};
    q.options.goalXYToleranceMeters=.02;
    q.options.searchSampleSpacingMeters=1;
    q.options.searchMaxBodyPointMotionMeters=1;
    q.options.finalSampleSpacingMeters=.01;
    q.options.finalMaxBodyPointMotionMeters=.01;
    q.options.maxExpansions=2;
    const auto result=plan(q);
    expect(result.status!=PlanStatus::Success,"strict final gate rejects missed obstacle");
    expect(result.diagnostics.finalPoseChecks>0,"strict validation actually ran");
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
    request.cancelled=std::make_shared<std::atomic_bool>(true);
    expect(plan(request).status==PlanStatus::Cancelled,"cancelled status");
    request.cancelled.reset();
    request.options.heuristicMode=HeuristicMode::PerformanceReedsSheppAndObstacleGrid;
    request.options.gridMaxSettlementsPerQuery=0;
    auto partial=plan(request);
    expect(partial.status==PlanStatus::Success,"limited lazy grid falls back");
    expect(!partial.heuristicCertifiedAdmissible,"grid performance not certified");
    expect(partial.diagnostics.gridFallbacks>0,"budget uses fallback");
    request.options.gridMaxSettlementsPerQuery=500;
    auto settled=plan(request);
    expect(settled.status==PlanStatus::Success && settled.diagnostics.gridSettlements>0,"grid Dijkstra settlement");
    request.options.heuristicMode=HeuristicMode::CertifiedRegionEuclidean;
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
