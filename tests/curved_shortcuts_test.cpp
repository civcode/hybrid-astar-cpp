#include <catch2/catch_test_macros.hpp>
#include "hybrid_astar/ackermann/curved_shortcuts.hpp"
#include "hybrid_astar/ackermann/refinement.hpp"
#include <cmath>
#include <iostream>
#include <memory>
#include <numbers>
#include <random>
#include <stdexcept>
using namespace hybrid_astar;
using namespace hybrid_astar::ackermann;
static void require(bool c,const char* why) { INFO(why); REQUIRE(c); }
static std::shared_ptr<MapSnapshot> openMap() {
  auto m=std::make_shared<MapSnapshot>();m->width=m->height=400;m->resolutionMeters=.2;
  m->cells.assign(160000,0);m->revision=77;return m;
}
static PlanResult given(const PlanRequest& r,const std::vector<MotionSection>& parts) {
  PlanResult p;p.status=PlanStatus::Success;p.sections=parts;p.mapRevision=r.map->revision;
  p.mapFrameId=r.map->frameId;p.mapSourceId=r.map->sourceId;p.effectiveOccupancy=r.occupancy;
  p.modelGeometryValid=true;p.collisionEvidence=CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled;
  return p;
}
static void runLegacyAssertions() {
  const Pose2D start{20,20,0};
  std::mt19937 gen(75642);
  std::uniform_real_distribution<double> coord(-10.,10.),angle(-3.1,3.1);
  std::size_t forwardSeen=0,reverseSeen=0,nonzeroCurved=0;
  for(int i=0;i<2000;++i) {
    const Pose2D finish{start.x+coord(gen),start.y+coord(gen),angle(gen)};
    auto alternatives=generateCurvatureBoundedShortcuts(start,finish,4.0);
    require(!alternatives.empty(),"CSC must produce at least one path for random poses");
    double prev=-1;
    for(const auto& parts:alternatives) {
      require(!parts.empty()&&parts.size()<=3,"CSC arc cardinality");
      Pose2D pose=start;Gear gear=parts.front().gear;
      double total=0;
      for(const auto& part:parts) {
        require(part.lengthMeters>0 && std::isfinite(part.lengthMeters),"positive finite arc lengths");
        require(part.gear==gear,"single gear throughout CSC");
        require(std::abs(part.curvature)<=.25+1e-12,"curvature feasibility");
        require(std::hypot(part.start.x-pose.x,part.start.y-pose.y)<1e-6,
                "native arc start continuity");
        if(std::abs(part.curvature)>1e-9) ++nonzeroCurved;
        pose=part.evaluate(part.lengthMeters);total+=part.lengthMeters;
      }
      require(std::hypot(pose.x-finish.x,pose.y-finish.y)<1e-6,
              "CSC reaches exact goal XY");
      require(std::abs(std::remainder(pose.yaw-finish.yaw,2*std::numbers::pi))<1e-6,
              "CSC reaches exact goal yaw");
      require(prev<=total+1e-9,"sorted geometric length");prev=total;
      if(gear==Gear::Forward)++forwardSeen;else ++reverseSeen;
    }
  }
  require(forwardSeen>500&&reverseSeen>500&&nonzeroCurved>1000,"directions and arcs exercised");
  bool invalid=false;
  try {generateCurvatureBoundedShortcuts(start,start,-1);}catch(const std::invalid_argument&) {invalid=true;}
  require(invalid,"invalid curvature radius rejected");
  // Arc path deliberately drives forward, reverses one meter, then drives
  // forward again along exactly the same circle. Its one-arc form is cheaper.
  PlanRequest req;req.map=openMap();req.start=start;
  const auto first=makeSection(start,Gear::Forward,.20,2.0);
  const auto second=makeSection(first.end,Gear::Reverse,.20,1.0);
  const auto third=makeSection(second.end,Gear::Forward,.20,1.0);
  req.goal=third.end;
  RefinementOptions settings;settings.useCurvatureBoundedShortcuts=true;
  const auto ref=refinePath(req,given(req,{first,second,third}),{},settings);
  require(ref.status==RefinementStatus::Improved,"curved shortcut improved cost");
  require(ref.path.sections.size()<=3&&ref.path.reverseLengthMeters<1e-6,
          "redundant reverse cusp removed");
  require(ref.path.weightedCostMeters<4.6-1e-3,"cost strictly reduced");
  require(ref.path.solutionQuality==SolutionQuality::FeasibleAfterRefinement,
          "curved section evidence");
  require(ref.prunedByCostLowerBound>0,
          "nonnegative cost bound prunes long CSC candidates");
  require(std::abs(ref.path.reached.yaw-req.goal.yaw)<1e-6,"arc final heading");
  // With the built-in straight-only mode, that curved same-position target is
  // intentionally not shortcuttable (backwards-compatible default).
  const auto straightOnly=refinePath(req,given(req,{first,second,third}));
  require(straightOnly.status==RefinementStatus::Unchanged,"default straight-only behavior");
  // A finite check budget may stop optimization but must never erase the
  // completely revalidated original, or claim unverified collision evidence.
  settings.maxTotalPoseChecks=220;
  const auto bounded=refinePath(req,given(req,{first,second,third}),{},settings);
  require(bounded.path.status==PlanStatus::Success,"bounded curved refiner keeps verified original");
  require(bounded.exactPoseChecks<=settings.maxTotalPoseChecks,
          "global strict pose-check budget never exceeded");
  std::cout<<"CURVED SHORTCUTS PASS\n";
}

TEST_CASE("curved_shortcuts_test contracts", "[curved_shortcuts_test]") {
  runLegacyAssertions();
}

TEST_CASE("Ackermann local-window scheduling reaches a mid-route reversible arc",
          "[ackermann][refinement][local-window]") {
  // 32 ordinary sections precede a forward/reverse/forward backtrack;
  // the remaining 29 sections make a 64-section route. With the historical
  // widest-window-first, maxAttempts=35 never tried the exact [32,35) window.
  PlanRequest req;
  req.map=openMap();
  req.start={20,20,0};
  Pose2D pose=req.start;
  std::vector<MotionSection> sections;
  const auto add=[&](Gear gear,double curvature,double length) {
    sections.push_back(makeSection(pose,gear,curvature,length));
    pose=sections.back().end;
  };
  for(int i=0;i<32;++i)add(Gear::Forward,0,.35);
  const Pose2D cuspStart=pose;
  add(Gear::Forward,.20,2.0);
  add(Gear::Reverse,.20,1.0);
  add(Gear::Forward,.20,1.0);
  const Pose2D cuspEnd=pose;
  for(int i=0;i<29;++i)add(Gear::Forward,0,.35);
  req.goal=pose;
  REQUIRE(sections.size()==64);
  RefinementOptions settings;
  settings.maxAttempts=35;
  settings.maxAccepted=1;
  std::size_t matchingWindows=0;
  const auto result=refinePath(req,given(req,sections),
    [&](const Pose2D& from,const Pose2D& to) -> std::vector<MotionSection> {
      const auto near=[](const Pose2D& a,const Pose2D& b) {
        return std::hypot(a.x-b.x,a.y-b.y)<1e-8 &&
               std::abs(std::remainder(a.yaw-b.yaw,2*std::numbers::pi))<1e-8;
      };
      if(!near(from,cuspStart)||!near(to,cuspEnd))return {};
      ++matchingWindows;
      return {makeSection(from,Gear::Forward,.20,2.0)};
    },settings);
  REQUIRE(matchingWindows>0);
  REQUIRE(result.accepted==1);
  REQUIRE(result.attempted<=settings.maxAttempts);
  REQUIRE(result.status==RefinementStatus::Improved);
  REQUIRE(result.path.status==PlanStatus::Success);
  REQUIRE(result.path.collisionEvidence==
          CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
  REQUIRE(result.path.weightedCostMeters<result.path.geometricLengthMeters+1.);
  REQUIRE(result.path.sections.size()==62); // replace 3-section cusp with one arc
  REQUIRE(result.path.reverseLengthMeters==0);
}

TEST_CASE("Ackermann RS provider does not suppress optional native CSC",
          "[ackermann][refinement][rs-csc]") {
  PlanRequest req;
  req.map=openMap();
  req.start={20,20,0};
  const auto first=makeSection(req.start,Gear::Forward,.20,2.0);
  const auto second=makeSection(first.end,Gear::Reverse,.20,1.0);
  const auto third=makeSection(second.end,Gear::Forward,.20,1.0);
  req.goal=third.end;
  const auto original=given(req,{first,second,third});
  int connectorCalls=0;
  const auto rsCandidate=[&](const Pose2D&,const Pose2D&) {
    ++connectorCalls;
    // The provider returns the original path, which is collision-free but
    // does not improve cost. Native CSC must still be considered.
    return std::vector<MotionSection>{first,second,third};
  };
  RefinementOptions settings;
  settings.useCurvatureBoundedShortcuts=false;
  const auto withoutCsc=refinePath(req,original,rsCandidate,settings);
  REQUIRE(withoutCsc.accepted==0);
  REQUIRE(withoutCsc.path.status==PlanStatus::Success);
  REQUIRE(connectorCalls>0);
  connectorCalls=0;
  settings.useCurvatureBoundedShortcuts=true;
  const auto withCsc=refinePath(req,original,rsCandidate,settings);
  REQUIRE(connectorCalls>0);
  REQUIRE(withCsc.accepted>0);
  REQUIRE(withCsc.path.status==PlanStatus::Success);
  REQUIRE(withCsc.path.weightedCostMeters<withoutCsc.path.weightedCostMeters-1e-3);
  REQUIRE(withCsc.path.reverseLengthMeters<1e-6);
  REQUIRE(withCsc.path.collisionEvidence==
          CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
  REQUIRE(withCsc.exactPoseChecks<=settings.maxTotalPoseChecks);
  // Even a malformed external candidate cannot bypass path validation or
  // prevent valid native CSC from being evaluated.
  const auto invalidProvider=[&](const Pose2D& from,const Pose2D&) {
    return std::vector<MotionSection>{makeSection(from,Gear::Forward,0,.5)};
  };
  const auto withInvalid=refinePath(req,original,invalidProvider,settings);
  REQUIRE(withInvalid.accepted>0);
  REQUIRE(withInvalid.path.weightedCostMeters<withoutCsc.path.weightedCostMeters);
  REQUIRE(withInvalid.path.collisionEvidence==
          CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
}
