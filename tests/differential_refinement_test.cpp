#include <catch2/catch_test_macros.hpp>
#include "hybrid_astar/ackermann/refinement.hpp"
#include <cmath>
#include <memory>
#include <numbers>
#include <vector>
#include <string_view>
#include <initializer_list>

using namespace hybrid_astar;
using namespace hybrid_astar::ackermann;

namespace {
constexpr double pi=std::numbers::pi_v<double>;
PlanRequest openRequest() {
  auto m=std::make_shared<MapSnapshot>();
  m->width=m->height=200;
  m->resolutionMeters=.1;
  m->cells.assign(m->width*m->height,0);
  m->frameId="diff-refine/map";m->sourceId="diff-refine";m->revision=44;
  PlanRequest req;
  req.map=m;req.start={10,10,0};req.goal={12,10,0};
  req.footprint={-.35,.35,.25};
  req.options.model=PlanOptions::Model::DifferentialDrive;
  return req;
}
PlanResult raw(const PlanRequest& req,std::vector<MotionSection> sections) {
  PlanResult out;
  out.status=PlanStatus::Success;
  out.modelGeometryValid=true;
  out.collisionEvidence=CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled;
  out.mapRevision=req.map->revision;
  out.mapFrameId=req.map->frameId;
  out.mapSourceId=req.map->sourceId;
  out.effectiveOccupancy=req.occupancy;
  out.sections=std::move(sections);
  // A forged cost cannot manipulate the strict refiner.
  out.weightedCostMeters=-400;
  return out;
}
std::vector<MotionSection> squiggle(PlanRequest& req) {
  std::vector<MotionSection> sections;
  auto addSpin=[&](double angle) {
    auto section=makeRotationSection(sections.empty()?req.start:sections.back().end,angle);
    sections.push_back(section);
  };
  auto addStraight=[&](double length) {
    auto section=makeSection(sections.back().end,Gear::Forward,0,length);
    sections.push_back(section);
  };
  addSpin(.2);addStraight(1);
  addSpin(-.4);addStraight(1);addSpin(.2);
  req.goal=sections.back().end;
  return sections;
}
void checkDense(const PlanRequest& req,const PlanResult& path) {
  ExactCollisionChecker checker(*req.map,req.occupancy,req.footprint,
                                req.options.footprintInflationMeters);
  Pose2D end=req.start;
  for(const auto& part:path.sections) {
    REQUIRE(std::hypot(part.start.x-end.x,part.start.y-end.y)<1e-6);
    REQUIRE(std::abs(std::remainder(part.start.yaw-end.yaw,2*pi))<1e-6);
    const auto progress=part.progressExtent();
    for(int k=0;k<=80;++k) {
      // Multiplying by 80 and then dividing by 80 can round one ULP
      // beyond progress and spuriously throw "Invalid Ackermann section
      // evaluation". The final sample must use the exact endpoint.
      const auto at=k==80?progress:progress*(double(k)/80.);
      INFO("kind="<<int(part.kind)<<" length="<<progress
           <<" sample="<<k<<" progress="<<at);
      const auto sample=part.evaluate(at);
      REQUIRE(checker.check(sample)==PoseCollision::Clear);
    }
    end=part.end;
  }
  REQUIRE(std::hypot(end.x-req.goal.x,end.y-req.goal.y)<=
          req.options.goalXYToleranceMeters+1e-8);
  REQUIRE(std::abs(std::remainder(end.yaw-req.goal.yaw,2*pi))<=
          req.options.goalYawToleranceRadians+1e-8);
}
}

TEST_CASE("Differential zigzags shorten to validated direct motions",
          "[differential][refinement][smoothing]") {
  for(const bool curved:{false,true}) {
    auto req=openRequest();
    const auto sections=squiggle(req);
    const auto original=raw(req,sections);
    RefinementOptions settings;
    settings.useCurvatureBoundedShortcuts=curved;
    settings.maxAttempts=120;
    const auto refined=refinePath(req,original,{},settings);
    INFO("curved="<<curved<<" status="<<int(refined.status)
         <<" accepted="<<refined.accepted);
    REQUIRE(refined.status==RefinementStatus::Improved);
    REQUIRE(refined.accepted>0);
    REQUIRE(refined.path.status==PlanStatus::Success);
    REQUIRE(refined.path.collisionEvidence==
            CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
    REQUIRE(refined.path.solutionQuality==SolutionQuality::FeasibleAfterRefinement);
    REQUIRE(refined.path.weightedCostMeters<2.32-1e-3);
    REQUIRE(refined.path.sections.size()<sections.size());
    for(const auto& part:refined.path.sections)
      REQUIRE(part.kind!=MotionKind::RotateInPlace);
    REQUIRE(refined.path.geometricLengthMeters<2);
    REQUIRE(refined.path.finalGear==Gear::Forward);
    REQUIRE(refined.exactPoseChecks<=settings.maxTotalPoseChecks);
    REQUIRE_FALSE(refined.path.trajectoryActuatorFeasible);
    checkDense(req,refined.path);
  }
}

TEST_CASE("Differential refinement never invents a stop-and-spin connector",
          "[differential][refinement][no-stop]") {
  auto req=openRequest();
  req.start={8,8,0};
  const auto a=makeSection(req.start,Gear::Forward,.4,1.5);
  const auto b=makeSection(a.end,Gear::Forward,0,1.0);
  req.goal=b.end;
  const auto original=raw(req,{a,b});
  for(const bool curves:{false,true}) {
    RefinementOptions options;
    options.useCurvatureBoundedShortcuts=curves;
    options.maxAttempts=25;
    const auto result=refinePath(req,original,{},options);
    REQUIRE(result.path.status==PlanStatus::Success);
    REQUIRE(result.path.collisionEvidence==
            CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
    for(const auto& section:result.path.sections)
      REQUIRE(section.kind!=MotionKind::RotateInPlace);
    checkDense(req,result.path);
  }
}

TEST_CASE("Differential native CSC removes a redundant reverse cusp",
          "[differential][refinement][curved]") {
  auto req=openRequest();
  req.start={6,6,0};
  std::vector<MotionSection> sections;
  auto append=[&](Gear gear,double curvature,double length) {
    sections.push_back(makeSection(
        sections.empty()?req.start:sections.back().end,gear,curvature,length));
  };
  append(Gear::Forward,1/.75,.45);
  append(Gear::Forward,0,2);
  append(Gear::Forward,-1/.75,.45);
  append(Gear::Forward,0,1);
  append(Gear::Reverse,0,1);
  req.goal=sections.back().end;
  RefinementOptions options;
  options.useCurvatureBoundedShortcuts=true;
  options.differentialCurveRadiusMeters=.75;
  const auto result=refinePath(req,raw(req,sections),{},options);
  INFO("status="<<int(result.status)<<" attempts="<<result.attempted
       <<" accepted="<<result.accepted);
  REQUIRE(result.path.status==PlanStatus::Success);
  REQUIRE(result.accepted>0);
  REQUIRE(result.path.weightedCostMeters<5);
  bool curved=false;
  for(const auto& section:result.path.sections)
    curved|=section.kind==MotionKind::Translate&&std::abs(section.curvature)>.5;
  REQUIRE(curved);
  for(const auto& part:result.path.sections)
    REQUIRE(part.kind!=MotionKind::RotateInPlace);
  checkDense(req,result.path);
}

TEST_CASE("Differential CSC refinement is invariant under world rotation",
          "[differential][refinement][rotated-frame]") {
  // The same relative SE(2) motion rotated by 45 degrees should generate
  // the same costs, number of accepted improvements and collision verdicts.
  // A diagonal in the WORLD map does not change the kinematic problem.
  auto base=openRequest();
  base.start={8,8,0};
  auto originalSections=squiggle(base);
  RefinementOptions options;
  options.useCurvatureBoundedShortcuts=true;
  options.maxAttempts=80;
  options.maxAccepted=3;
  options.differentialRadiusVariants=4;
  const auto a=refinePath(base,raw(base,originalSections),{},options);
  REQUIRE(a.path.status==PlanStatus::Success);
  const double turn=pi/4;
  const double cs=std::cos(turn),sn=std::sin(turn);
  const Pose2D origin{10,10,0};
  const auto rotate=[&](Pose2D pose)->Pose2D {
    const double x=pose.x-origin.x,y=pose.y-origin.y;
    return {origin.x+cs*x-sn*y,origin.y+sn*x+cs*y,pose.yaw+turn};
  };
  auto rotated=base;
  // Rotate the WORLD coordinate frame and the occupancy grid together:
  // leaving the map axis-aligned would not be an equivalent problem.
  auto rotatedMap=std::make_shared<MapSnapshot>(*base.map);
  rotatedMap->origin=rotate(base.map->origin);
  rotated.map=rotatedMap;
  rotated.start=rotate(base.start);
  rotated.goal=rotate(base.goal);
  std::vector<MotionSection> sections;
  Pose2D pose=rotated.start;
  for(const auto& section:originalSections) {
    MotionSection next;
    if(section.kind==MotionKind::RotateInPlace)
      next=makeRotationSection(pose,section.turnRadians);
    else
      next=makeSection(pose,section.gear,section.curvature,
                       section.lengthMeters);
    pose=next.end;
    sections.push_back(next);
  }
  const auto b=refinePath(rotated,raw(rotated,sections),{},options);
  REQUIRE(b.path.status==PlanStatus::Success);
  REQUIRE(a.accepted==b.accepted);
  REQUIRE(a.attempted==b.attempted);
  REQUIRE(std::abs(a.path.weightedCostMeters-b.path.weightedCostMeters)<1e-6);
  REQUIRE(std::abs(a.path.geometricLengthMeters-b.path.geometricLengthMeters)<1e-6);
  REQUIRE(a.path.collisionEvidence==
          CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
  REQUIRE(b.path.collisionEvidence==
          CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
  checkDense(base,a.path);
  checkDense(rotated,b.path);
}

TEST_CASE("Differential refinement supports fixed and multiple curve radii",
          "[differential][refinement][radius-ab]") {
  auto req=openRequest();
  const auto sections=squiggle(req);
  for(std::size_t variants:{std::size_t{1},std::size_t{4}}) {
    RefinementOptions options;
    options.useCurvatureBoundedShortcuts=true;
    options.differentialRadiusVariants=variants;
    const auto result=refinePath(req,raw(req,sections),{},options);
    INFO("radii="<<variants);
    REQUIRE(result.path.status==PlanStatus::Success);
    REQUIRE(result.path.collisionEvidence==
            CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
    REQUIRE(result.path.weightedCostMeters<=2.32+1e-8);
    std::size_t retainedSpins=0;
    for(const auto& part:result.path.sections)
      retainedSpins+=part.kind==MotionKind::RotateInPlace;
    // A rejected shortcut may retain the original stops, but the refiner
    // cannot introduce additional stops of its own.
    REQUIRE(retainedSpins<=3);
  }
  RefinementOptions invalid;
  invalid.differentialRadiusVariants=0;
  REQUIRE(refinePath(req,raw(req,sections),{},invalid).status==
          RefinementStatus::InvalidInput);
  invalid.differentialRadiusVariants=5;
  REQUIRE(refinePath(req,raw(req,sections),{},invalid).status==
          RefinementStatus::InvalidInput);
}

TEST_CASE("Differential refiner keeps original if stricter work cap prevents verification",
          "[differential][refinement][budget]") {
  auto req=openRequest();
  auto original=raw(req,squiggle(req));
  RefinementOptions insufficient;
  insufficient.maxTotalPoseChecks=3;
  const auto result=refinePath(req,original,{},insufficient);
  REQUIRE(result.status==RefinementStatus::OriginalNotVerified);
  REQUIRE(result.path.sections.empty());
  REQUIRE(result.exactPoseChecks<=insufficient.maxTotalPoseChecks);
  original.mapRevision++;
  REQUIRE(refinePath(req,original).status==RefinementStatus::InvalidInput);
}

TEST_CASE("Differential shortcut proposals never cut through occupied wall",
          "[differential][refinement][collision]") {
  auto req=openRequest();
  req.start={3,3,0};
  std::vector<MotionSection> sections;
  auto spin=[&](double angle) {
    sections.push_back(makeRotationSection(sections.empty()?req.start:sections.back().end,angle));
  };
  auto move=[&](double length) {
    sections.push_back(makeSection(sections.back().end,Gear::Forward,0,length));
  };
  spin(pi/2);move(5);spin(-pi/2);move(8);spin(-pi/2);move(5);spin(pi/2);
  req.goal=sections.back().end;
  auto wall=std::make_shared<MapSnapshot>(*req.map);
  for(std::size_t y=20;y<65;++y)wall->cells[y*wall->width+70]=100;
  req.map=wall;
  ExactCollisionChecker checker(*req.map,req.occupancy,req.footprint);
  REQUIRE(checker.check(req.start)==PoseCollision::Clear);
  REQUIRE(checker.check(req.goal)==PoseCollision::Clear);
  REQUIRE(checker.check({7,3,0})==PoseCollision::Colliding);
  RefinementOptions settings;
  settings.useCurvatureBoundedShortcuts=true;
  settings.maxAttempts=120;
  const auto refined=refinePath(req,raw(req,sections),{},settings);
  REQUIRE(refined.path.status==PlanStatus::Success);
  REQUIRE(refined.path.collisionEvidence==
          CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
  REQUIRE(refined.path.weightedCostMeters<=18+2*pi*.4+1e-8);
  REQUIRE(refined.exactPoseChecks<=settings.maxTotalPoseChecks);
  checkDense(req,refined.path);
}

TEST_CASE("Differential post-refiner rejects a colliding swept rotation",
          "[differential][refinement][collision][spin]") {
  auto req=openRequest();
  req.start={10,10,0};
  const auto spin=makeRotationSection(req.start,pi);
  req.goal=spin.end;
  req.footprint={-1.,1.,.10};
  auto obstacle=std::make_shared<MapSnapshot>(*req.map);
  obstacle->cells[std::size_t(10.85/.1)*obstacle->width+
                  std::size_t(10/.1)]=100;
  req.map=obstacle;
  ExactCollisionChecker checker(*req.map,req.occupancy,req.footprint);
  REQUIRE(checker.check(req.start)==PoseCollision::Clear);
  REQUIRE(checker.check(spin.end)==PoseCollision::Clear);
  const auto result=refinePath(req,raw(req,{spin}));
  REQUIRE(result.status==RefinementStatus::OriginalNotVerified);
  REQUIRE(result.path.sections.empty());
}

TEST_CASE("Mecanum geometric refinement removes zigzags using moving twists",
          "[mecanum][refinement][smoothing]") {
  auto req=openRequest();
  req.options.model=PlanOptions::Model::Mecanum;
  req.options.footprintInflationMeters=.5;
  req.options.mecanumStationarySpinPenaltyMeters=.35;
  const auto first=makeHolonomicSection(req.start,0,1,0);
  const auto second=makeHolonomicSection(first.end,2,0,0);
  const auto third=makeHolonomicSection(second.end,0,-1,0);
  req.goal=third.end;
  const auto result=refinePath(req,raw(req,{first,second,third}));
  INFO("status="<<int(result.status)<<" accepted="<<result.accepted
       <<" attempted="<<result.attempted);
  REQUIRE(result.status==RefinementStatus::Improved);
  REQUIRE(result.accepted>0);
  REQUIRE(result.path.status==PlanStatus::Success);
  REQUIRE(result.path.sections.size()<3);
  REQUIRE(result.path.weightedCostMeters<4.0);
  REQUIRE(result.path.collisionEvidence==
          CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
  for(const auto& section:result.path.sections)
    REQUIRE(section.kind==MotionKind::HolonomicTwist);
  checkDense(req,result.path);
}

TEST_CASE("Mecanum strict refinements cannot cut through inflated occupied cells",
          "[mecanum][refinement][inflation][collision]") {
  auto req=openRequest();
  req.options.model=PlanOptions::Model::Mecanum;
  req.options.footprintInflationMeters=.15;
  req.start={4,4,0};
  const auto first=makeHolonomicSection(req.start,0,3,0);
  const auto second=makeHolonomicSection(first.end,6,0,0);
  const auto third=makeHolonomicSection(second.end,0,-3,0);
  req.goal=third.end;
  auto blocked=std::make_shared<MapSnapshot>(*req.map);
  for(std::size_t j=28;j<55;++j)
    blocked->cells[j*blocked->width+70]=100; // crosses straight shortcut
  req.map=blocked;
  const auto result=refinePath(req,raw(req,{first,second,third}));
  REQUIRE(result.path.status==PlanStatus::Success);
  REQUIRE(result.path.collisionEvidence==
          CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
  REQUIRE(result.path.weightedCostMeters<=12.0+1e-8);
  ExactCollisionChecker checker(*req.map,req.occupancy,req.footprint,
                                req.options.footprintInflationMeters);
  for(const auto& section:result.path.sections)
    for(int k=0;k<=100;++k)
      REQUIRE(checker.check(section.evaluate(
          k==100?section.progressExtent():
                 section.progressExtent()*(double(k)/100)))==PoseCollision::Clear);
}

TEST_CASE("Differential rolling refinement avoids a splice immediately beside a stop",
          "[differential][refinement][no-stop][splice]") {
  auto req=openRequest();
  req.start={6,6,0};
  const auto approach=makeSection(req.start,Gear::Forward,0,1);
  const auto stop=makeRotationSection(approach.end,.4);
  const auto curve=makeSection(stop.end,Gear::Forward,0,2);
  const auto finish=makeRotationSection(curve.end,-.4);
  req.goal=finish.end;
  RefinementOptions opt;
  opt.maxAttempts=50;
  const auto result=refinePath(req,raw(req,{approach,stop,curve,finish}),{},opt);
  REQUIRE(result.path.status==PlanStatus::Success);
  // The refiner can replace a stop only when its replacement window includes
  // that stop. It cannot decorate the stop with a new shorter rolling chord.
  for(std::size_t j=1;j<result.path.sections.size();++j) {
    const auto& prev=result.path.sections[j-1];
    const auto& next=result.path.sections[j];
    if(prev.kind==MotionKind::RotateInPlace)
      REQUIRE(next.kind!=MotionKind::RotateInPlace);
  }
  checkDense(req,result.path);
}

namespace {
// Compact, deterministic reproductions of the two user-exported warehouse
// routes. The codes reproduce exact 0.5 m differential primitives; the
// added occupancy runs reproduce all 143 browser-painted cells without a
// large JSON test artifact. Both exports use a 0.50 m footprint margin.
PlanRequest exportedWarehouseRequest(bool good) {
  auto map=std::make_shared<MapSnapshot>();
  map->width=200;map->height=160;map->resolutionMeters=.25;
  map->frameId="demo/map";map->sourceId="built-in";map->revision=187;
  map->cells.assign(map->width*map->height,0);
  const auto paint=[&](int x0,int y0,int x1,int y1) {
    for(int y=y0;y<y1;++y)
      for(int x=x0;x<x1;++x)
        map->cells[std::size_t(y)*map->width+std::size_t(x)]=100;
  };
  paint(48,12,53,48);paint(48,92,53,145);
  paint(94,0,99,46);paint(94,100,99,160);
  paint(141,12,146,55);paint(141,104,146,145);
  constexpr int additions[][3]={
    {126,99,1},{125,100,3},{125,101,3},{125,102,3},{125,103,3},
    {125,104,3},{125,105,3},{125,106,3},{125,107,3},{125,108,3},
    {125,109,3},{125,110,3},{125,111,3},{125,112,3},{125,113,3},
    {125,114,4},{126,115,3},{126,116,3},{126,117,3},{126,118,3},
    {100,119,1},{112,119,2},{115,119,4},{120,119,10},
    {99,120,8},{108,120,2},{111,120,19},
    {99,121,15},{115,121,4},{120,121,9},
    {100,122,1},{102,122,5},{108,122,2},{111,122,2}
  };
  for(const auto& run:additions)
    paint(run[0],run[1],run[0]+run[2],run[1]+1);
  PlanRequest req;
  req.map=map;
  req.start={5,32.08,0};
  req.goal=good?Pose2D{44.54,7.37,pi/2}:Pose2D{31.77,35.96,pi/2};
  req.footprint={-.35,.35,.25};
  req.options.model=PlanOptions::Model::DifferentialDrive;
  req.options.footprintInflationMeters=.5;
  req.options.rotationCostMetersPerRadian=.4;
  return req;
}

std::vector<MotionSection> exportedWarehouseSections(const PlanRequest& req,bool good) {
  // T/L/S are clockwise/counterclockwise/straight forward 0.5 m sections.
  // r is a -pi/8 in-place turn; R is the terminal +3pi/4 turn.
  constexpr std::string_view bad="rrTSSLSSTLSSSTLTSLTSLTSSLLSLSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSLSTLLSSTTLLLSLSSSSSTLTLSSSSSLTLLSTSSr";
  constexpr std::string_view clean="TTTTLLTSSLTSLTSLSSTLTSLSLSSTSSLTSSSLTSLTSSLTSLSLSSSSSSSSSSSSSSSSSSSSSSSSSSSTSTLSTSSLTSSSLTSSSSSSSSSSSR";
  std::vector<MotionSection> sections;
  Pose2D at=req.start;
  for(const char code:(good?clean:bad)) {
    MotionSection part;
    if(code=='r'||code=='R')
      part=makeRotationSection(at,code=='r'?-pi/8:3*pi/4);
    else
      part=makeSection(at,Gear::Forward,
          code=='T'?-pi/4:code=='L'?pi/4:0.0,.5);
    at=part.end;
    sections.push_back(part);
  }
  return sections;
}
} // namespace

TEST_CASE("Differential local windows refine the initial warehouse approach",
          "[differential][refinement][warehouse][local-window]") {
  for(const bool good:{false,true}) {
    auto req=exportedWarehouseRequest(good);
    auto originalSections=exportedWarehouseSections(req,good);
    REQUIRE(originalSections.size()==(good?102:95));
    RefinementOptions settings;
    settings.maxAttempts=160;
    settings.maxAccepted=8;
    settings.maxTotalPoseChecks=200000;
    settings.useCurvatureBoundedShortcuts=true;
    settings.differentialCurveRadiusMeters=.75;
    settings.differentialRadiusVariants=4;
    const auto refined=refinePath(req,raw(req,originalSections),{},settings);
    INFO("good="<<good<<" status="<<int(refined.status)
         <<" attempts="<<refined.attempted<<" accepted="<<refined.accepted
         <<" cost="<<refined.path.weightedCostMeters);
    REQUIRE(refined.path.status==PlanStatus::Success);
    REQUIRE(refined.path.collisionEvidence==
            CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
    REQUIRE(refined.attempted<=settings.maxAttempts);
    REQUIRE(refined.exactPoseChecks<=settings.maxTotalPoseChecks);
    REQUIRE(refined.accepted>0);
    REQUIRE(refined.path.weightedCostMeters<
            (good?51.4424777961:46.471238898));
    if(!good) {
      // Before the local-window pass, the first two stationary rotations
      // remained untouched even though an early rolling CSC is available.
      REQUIRE(refined.path.sections.front().kind==MotionKind::Translate);
    }
    checkDense(req,refined.path);
  }
}
