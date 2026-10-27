#include <catch2/catch_test_macros.hpp>
#include "hybrid_astar/ackermann/refinement.hpp"
#include <atomic>
#include <cmath>
#include <iostream>
#include <memory>
#include <numbers>
#include <stdexcept>
using namespace hybrid_astar;
using namespace hybrid_astar::ackermann;
static void must(bool x,const char* why) { INFO(why); REQUIRE(x); }
static std::shared_ptr<MapSnapshot> map() {
  auto m=std::make_shared<MapSnapshot>();
  m->width=m->height=400;
  m->resolutionMeters=.2;
  m->cells.assign(160000,0);
  m->revision=50;
  m->frameId="map";
  return m;
}
static PlanResult raw(const PlanRequest& req,const std::vector<MotionSection>& parts) {
  PlanResult r;
  r.status=PlanStatus::Success;
  r.sections=parts;
  r.collisionEvidence=CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled;
  r.modelGeometryValid=true;
  r.mapRevision=req.map->revision;
  r.mapFrameId=req.map->frameId;
  r.mapSourceId=req.map->sourceId;
  r.effectiveOccupancy=req.occupancy;
  // The refiner does NOT trust original cost, path summary or final state.
  r.weightedCostMeters=-999;
  return r;
}
static void runLegacyAssertions() {
  auto clear=map();
  PlanRequest req;req.map=clear;req.start={20,20,0};req.goal={22,20,0};
  const auto a=makeSection(req.start,Gear::Forward,0,2);
  const auto b=makeSection(a.end,Gear::Reverse,0,1);
  const auto c=makeSection(b.end,Gear::Forward,0,1);
  const auto given=raw(req,{a,b,c});
  auto shortened=refinePath(req,given);
  must(shortened.status==RefinementStatus::Improved,"builtin chord accepted");
  must(shortened.accepted==1&&shortened.attempted>=1,"bounded iterations");
  must(shortened.path.status==PlanStatus::Success &&
       shortened.path.collisionEvidence==CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled,
       "strictly verified output evidence");
  must(shortened.path.solutionQuality==SolutionQuality::FeasibleAfterRefinement,
       "result reflects post-path refinement separately from search incumbent quality");
  must(shortened.path.sections.size()==1&&
       shortened.path.sections[0].gear==Gear::Forward,"exact straight one segment");
  must(std::abs(shortened.path.weightedCostMeters-2)<1e-8,"quality beats two gear switches");
  must(shortened.path.gearChangeSectionIndices.empty()&&
       shortened.path.reverseLengthMeters==0,"all cusp metadata recomputed");
  must(shortened.exactPoseChecks>100,"full strict baseline and optimized checking");
  must(shortened.path.mapRevision==50&&shortened.path.mapFrameId=="map",
       "output tied to original map revision");
  must(!shortened.path.trajectoryActuatorFeasible,"geometric refinement cannot imply actuator feasibility");
  const auto external=refinePath(req,given,[](const Pose2D& from,const Pose2D& to) {
    return std::vector<MotionSection>{makeSection(from,Gear::Forward,0,to.x-from.x)};
  });
  must(external.status==RefinementStatus::Improved&&external.path.sections.size()==1,
       "provider-generated exact sections use the same strict improvement gate");
  auto plannerOutput=plan(req);
  must(plannerOutput.status==PlanStatus::Success,
       "ordinary planner produces a validated path for refinement");
  auto roundtrip=refinePath(req,plannerOutput);
  must(roundtrip.path.status==PlanStatus::Success &&
       roundtrip.path.collisionEvidence==CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled,
       "actual planner output revalidated end-to-end");
  must(roundtrip.path.weightedCostMeters<=plannerOutput.weightedCostMeters+1e-9,
       "post-search refinement never worsens the planner objective");
  auto corrupted=given;
  corrupted.sections[1].end.x+=.25;
  auto invalid=refinePath(req,corrupted);
  must(invalid.status==RefinementStatus::OriginalNotVerified&&
       invalid.path.sections.empty(),"malformed raw path never retained");
  auto stale=given;
  stale.mapRevision=49;
  auto old=refinePath(req,stale);
  must(old.status==RefinementStatus::InvalidInput&&old.path.sections.empty(),"stale snapshot forbidden");
  RefinementOptions small;small.maxTotalPoseChecks=4;
  auto scarce=refinePath(req,given,{},small);
  must(scarce.status==RefinementStatus::OriginalNotVerified&&scarce.path.sections.empty(),
       "cannot claim original if unable to revalidate it");
  auto ignored=refinePath(req,given,[](const Pose2D& s,const Pose2D&) {
    return std::vector<MotionSection>{makeSection(s,Gear::Forward,.25,1)};
  });
  must(ignored.status==RefinementStatus::Unchanged&&ignored.path.sections.size()==3,
       "endpoint-disconnected candidate falls back to verified raw");
  auto token=std::make_shared<std::atomic_bool>(false);
  req.cancelled=token;
  auto aborted=refinePath(req,given,[token](const Pose2D& s,const Pose2D&) {
    token->store(true);
    return std::vector<MotionSection>{makeSection(s,Gear::Forward,0,2)};
  });
  must(aborted.status==RefinementStatus::Cancelled&&aborted.path.status==PlanStatus::Success&&
       aborted.path.sections.size()==3,"cancel after verified raw retains raw");
  req.cancelled.reset();
  // A four-quarter-arc obstacle detour reaches the SAME heading and y.
  // The shorter direct straight shortcut would intersect a blocked square.
  auto clutter=map();
  clutter->cells[100*400+140]=100; // (28,20), clear of the curved upper detour
  req.map=clutter;req.goal={36,20,0};
  auto p=req.start;
  std::vector<MotionSection> detour;
  for(double k:{.25,-.25,-.25,.25}) {
    auto section=makeSection(p,Gear::Forward,k,2*std::numbers::pi);
    detour.push_back(section);
    p=section.end;
  }
  must(std::hypot(p.x-36,p.y-20)<1e-8&&std::abs(p.yaw)<1e-8,
       "quarter arc benchmark reaches nominal goal");
  auto blocked=refinePath(req,raw(req,detour));
  must(blocked.status==RefinementStatus::Unchanged&&blocked.path.sections.size()==4,
       "strict collision checker rejects shorter obstacle-cutting shortcut");
  must(blocked.rejected>0&&blocked.exactPoseChecks>0,"collision proposal really inspected");
  RefinementOptions finiteAttempts;finiteAttempts.maxAttempts=1;
  auto bounded=refinePath(req,raw(req,detour),{},finiteAttempts);
  must(bounded.status==RefinementStatus::WorkBudgetExceeded&&
       bounded.path.status==PlanStatus::Success&&bounded.path.sections.size()==4,
       "work cap preserves independently rechecked detour");
  // If there is only one eligible window, exactly one allowed attempt
  // exhausts the search space rather than being a work-budget failure.
  req.map=clear;req.goal={22,20,0};
  const auto firstHalf=makeSection(req.start,Gear::Forward,0,1);
  const auto secondHalf=makeSection(firstHalf.end,Gear::Forward,0,1);
  const auto alreadyStraight=refinePath(
      req,raw(req,{firstHalf,secondHalf}),{},finiteAttempts);
  must(alreadyStraight.status==RefinementStatus::Unchanged &&
       alreadyStraight.attempted==1 &&
       alreadyStraight.path.status==PlanStatus::Success,
       "fully enumerated refinement windows do not falsely exhaust budget");
  // Optional provider may suggest reverse/cusp sections, but each must pass
  // the same full-path kinematic, objective, and strict collision gate.
  req.map=clear;req.goal={22,20,0};
  req.options.clearanceWeight=.1;
  const auto withClearance=refinePath(req,given);
  must(withClearance.status==RefinementStatus::Improved,
       "positive clearance penalty still permits strict improvement");
  must(withClearance.path.clearanceCostMeters>=0&&
       withClearance.path.weightedCostMeters>=2,
       "weighted cost includes nonnegative oriented clearance objective");
  std::cout<<"REFINEMENT PASS\n";
}

TEST_CASE("refinement_test contracts", "[refinement_test]") {
  runLegacyAssertions();
}
