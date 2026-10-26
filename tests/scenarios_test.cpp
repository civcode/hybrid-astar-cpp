#include "hybrid_astar/ackermann/planner.hpp"
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
using namespace hybrid_astar;using namespace hybrid_astar::ackermann;
static void require(bool yes,const char* what){if(!yes) throw std::runtime_error(what);}
static std::shared_ptr<MapSnapshot> map() {
  auto m=std::make_shared<MapSnapshot>();
  m->width=m->height=200;m->resolutionMeters=.2;
  m->cells.assign(m->width*m->height,0);m->revision=21;
  return m;
}
int main() {
  {
    auto m=map();PlanRequest q;
    q.map=m;q.start={20,20,0};q.goal={18,20,0};
    auto result=plan(q);
    require(result.status==PlanStatus::Success,"primitive-only backward parking");
    require(result.finalGear==Gear::Reverse&&result.reverseLengthMeters>1.5,
            "primitive-only reverse sections and final gear");
    require(result.diagnostics.analyticAttempts==0,"no hidden RS provider");
  }
  {
    auto m=map();PlanRequest q;
    q.map=m;q.start={20,20,0};q.goal={18,21,0};
    q.options.maxExpansions=10000;
    auto result=plan(q);
    require(result.status==PlanStatus::Success,"primitive-only multi-cusp parking");
    require(result.gearChangeSectionIndices.size()>=2,"two or more parking direction changes");
    require(result.goalXYErrorMeters<=q.options.goalXYToleranceMeters &&
            result.goalYawErrorRadians<=q.options.goalYawToleranceRadians,
            "parking actual continuous goal tolerances");
    require(result.collisionEvidence==CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled,
            "parking full path independently sampled");
  }
  {
    auto snapshot=map();PlanRequest q;
    q.map=snapshot;q.start={20,20,0};q.goal={21,20,0};
    auto old=plan(q);
    require(old.status==PlanStatus::Success&&old.mapRevision==21,"old immutable snapshot");
    auto next=std::make_shared<MapSnapshot>(*snapshot);
    next->revision=22;next->cells[100*next->width+100]=100;
    q.map=next;
    auto blocked=plan(q);
    require(blocked.status==PlanStatus::StartInCollision&&blocked.mapRevision==22,
            "new snapshot revision has new occupancy");
    require(old.status==PlanStatus::Success&&old.mapRevision==21,
            "previous validated result remains tied to old revision");
  }
  {
    auto m=map();
    m->cells[100*m->width+118]=100; // left edge = 23.6 m; front = 20+3.6 m
    ExactCollisionChecker checker(*m,{},{});
    require(checker.check({20,20,0})==PoseCollision::Colliding,
            "blocked square touching front edge is collision");
    m->cells[100*m->width+118]=0;
    ExactCollisionChecker free(*m,{},{});
    require(free.check({20,20,0})==PoseCollision::Clear,"nonblocked contact cell clear");
    require(free.check({36.4,20,0})==PoseCollision::Colliding,
            "map right-edge tangency is collision");
  }
  {
    auto m=map();
    for(std::size_t y=0;y<m->height;++y) {
      if(y==100) continue; // 0.2m opening, vehicle is 1.8m wide
      m->cells[y*m->width+125]=100;
    }
    PlanRequest q;q.map=m;q.start={20,20,0};q.goal={30,20,0};
    q.options.maxExpansions=2000;
    const auto blocked=plan(q);
    require(blocked.status!=PlanStatus::Success,"sub-vehicle-width gap never a solution");
    require(blocked.collisionEvidence==CollisionEvidence::None,
            "failed search never advertises validated path");
  }
  {
    // Search has ample samples but the strict validator has an independent cap.
    auto m=map();PlanRequest q;q.map=m;
    q.start={20,20,0};q.goal={21,20,0};
    q.options.maxFinalSamplesPerSection=3;
    auto bounded=plan(q);
    require(bounded.status==PlanStatus::ValidationIncomplete,
            "strict sample cap cannot be silently bypassed");
    require(bounded.collisionEvidence==CollisionEvidence::None,
            "strict budget abort never claims validated geometry");
    require(bounded.diagnostics.finalPoseChecks>0,
            "strict checking actually attempted");
    q.options.maxFinalSamplesPerSection=10000;
    require(plan(q).status==PlanStatus::Success,
            "larger independent strict cap permits complete validation");
  }
  {
    auto m=map();PlanRequest q;q.map=m;
    q.start={20,20,0};q.goal={21,20,0};
    q.options.maxTotalFinalPoseChecks=1;
    const auto incomplete=plan(q);
    require(incomplete.status==PlanStatus::ValidationIncomplete,
            "global final-check cap cannot return unchecked success");
    require(incomplete.collisionEvidence==CollisionEvidence::None,
            "no collision evidence after incomplete required work");
    require(incomplete.diagnostics.finalPoseChecks<=1,
            "strict whole-request check count never exceeds limit");
    q.options.maxTotalFinalPoseChecks=0;
    require(plan(q).status==PlanStatus::InvalidRequest,
            "zero strict final work allowance is invalid");
  }
  std::cout<<"SCENARIOS PASS\n";
}
