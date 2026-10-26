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
