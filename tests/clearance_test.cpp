#include <catch2/catch_test_macros.hpp>
#include "hybrid_astar/ackermann/clearance.hpp"
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
using namespace hybrid_astar;
using namespace hybrid_astar::ackermann;
static void require(bool ok,const char* why) { INFO(why); REQUIRE(ok); }
static void runLegacyAssertions() {
  auto map=std::make_shared<MapSnapshot>();
  map->width=map->height=400;map->resolutionMeters=.1;
  map->cells.assign(160000,0);
  map->cells[110*400+120]=100; // obstacle square y=[11.0,11.1]
  ClearanceScorer scorer(*map,{},{},.15,.4);
  const double near=scorer.poseClearanceMeters({10,10,0});
  require(near>.09 && near<.11,"actual oriented rectangle distance");
  require(std::abs(scorer.poseClearanceMeters({10,20,0})-.4)<1e-9,"influence cap");
  const auto section=makeSection({10,10,0},Gear::Forward,0,.5);
  std::size_t samples=0;
  const auto cost=scorer.sectionCost(section,.1,.15,10000,samples);
  require(cost>0 && cost<.15*.5,"nonnegative bounded clearance integral");
  require(samples>1,"trapezoid integrates progress");
  PlanRequest request;
  request.map=map;request.start={10,10,0};request.goal={10.5,10,0};
  request.analyticConnector=[](const Pose2D& start,const Pose2D&) {
    return std::vector<MotionSection>{makeSection(start,Gear::Forward,0,.5)};
  };
  const auto base=plan(request);
  require(base.status==PlanStatus::Success && std::abs(base.weightedCostMeters-.5)<1e-8,"default zero weight");
  request.options.clearanceWeight=.15;
  auto with=plan(request);
  require(with.status==PlanStatus::Success,"clearance enabled path");
  require(with.clearanceCostMeters>0 && with.diagnostics.clearanceSamples>0,"clearance tracked");
  require(std::abs(with.weightedCostMeters-.5-with.clearanceCostMeters)<1e-8,"analytic weighted cost includes clearance");
  request.options.clearanceWeight=-1;
  require(plan(request).status==PlanStatus::InvalidRequest,"negative objective forbidden");
  std::cout<<"CLEARANCE PASS\n";
}

TEST_CASE("clearance_test contracts", "[clearance_test]") {
  runLegacyAssertions();
}


TEST_CASE("EDT zero-penalty certificate matches exact rotated inflated clearance",
          "[clearance][edt][rotation][unknown]") {
  MapSnapshot map;
  map.width=96;map.height=84;map.resolutionMeters=.12;
  map.origin={4.,-3.,1.1};
  map.cells.assign(map.width*map.height,0);
  for(std::size_t y=18;y<70;++y)map.cells[y*map.width+46]=100;
  for(std::size_t y=20;y<36;++y)map.cells[y*map.width+70]=-1;
  for(std::size_t x=14;x<29;++x)map.cells[60*map.width+x]=75;
  auto world=[&](double x,double y,double yaw) {
    const double c=std::cos(map.origin.yaw),s=std::sin(map.origin.yaw);
    return Pose2D{map.origin.x+c*x-s*y,
                  map.origin.y+s*x+c*y,yaw+map.origin.yaw};
  };
  for(auto unknown:{UnknownPolicy::Occupied,UnknownPolicy::Free}) {
    for(auto footprint:{Footprint{-.35,.35,.25},Footprint{-.25,.25,.15}}) {
      for(double inflation:{0.,.1}) {
        const OccupancyPolicy policy{0,unknown};
        const ClearanceScorer exact(map,policy,footprint,.15,.4,false,inflation);
        const ClearanceScorer edt(map,policy,footprint,.15,.4,true,inflation);
        REQUIRE(edt.distanceTransformBytes()==map.cells.size()*sizeof(double));
        PlanDiagnostics diag;
        std::size_t equalCount=0;
        for(int iy=0;iy<84;iy+=3)for(int ix=0;ix<96;ix+=3)for(int yaw=0;yaw<8;++yaw) {
          const auto pose=world((ix+.37)*map.resolutionMeters,
                                 (iy+.41)*map.resolutionMeters,
                                 yaw*.39269908169872414);
          const double baseline=exact.poseClearanceMeters(pose);
          const double accelerated=edt.poseClearanceMeters(pose,&diag);
          INFO("x="<<ix<<" y="<<iy<<" yaw="<<yaw<<" inflation="<<inflation);
          REQUIRE(std::abs(baseline-accelerated)<1e-12);
          ++equalCount;
        }
        REQUIRE(diag.clearanceDistanceQueries==equalCount);
        REQUIRE(diag.clearanceEdtZeroCostBypasses>0);
        REQUIRE(diag.clearanceExactFallbacks>0);
        REQUIRE(diag.clearanceEdtZeroCostBypasses+
                diag.clearanceExactFallbacks==diag.clearanceDistanceQueries);
      }
    }
  }
}

TEST_CASE("EDT handles all-free, occupied-policy and near-map-edge clearance",
          "[clearance][edt][boundary]") {
  MapSnapshot map;
  map.width=map.height=80;map.resolutionMeters=.1;
  map.cells.assign(map.width*map.height,0);
  const Footprint footprint{-.25,.25,.15};
  const ClearanceScorer exact(map,{},footprint,.15,.4,false,.1);
  const ClearanceScorer edt(map,{},footprint,.15,.4,true,.1);
  REQUIRE(edt.distanceTransformBytes()==0);
  PlanDiagnostics d;
  for(double x:{.1,.35,.5,.7,1.,2.,5.,7.6,7.9})
    for(double y:{.1,.35,.5,1.,2.,4.,7.9})
      for(double yaw:{0.,.5,1.2}) {
        const Pose2D pose{x,y,yaw};
        REQUIRE(std::abs(exact.poseClearanceMeters(pose)-
                         edt.poseClearanceMeters(pose,&d))<1e-12);
      }
  REQUIRE(d.clearanceEdtZeroCostBypasses>0);
  REQUIRE(d.clearanceExactFallbacks>0);
  // Unknown cells and occupancy thresholds must use identical blocked
  // definitions for the exact scanner and the distance transform.
  map.cells[25*map.width+25]=-1;
  map.cells[30*map.width+30]=55;
  for(auto policy:{OccupancyPolicy{0,UnknownPolicy::Occupied},
                   OccupancyPolicy{60,UnknownPolicy::Free}}) {
    const ClearanceScorer a(map,policy,footprint,.15,.4,false,0.);
    const ClearanceScorer b(map,policy,footprint,.15,.4,true,0.);
    for(double x:{1.5,2.2,2.5,3.,3.5,5.,6.})
      for(double y:{1.5,2.5,3.,3.5,6.})
        REQUIRE(std::abs(a.poseClearanceMeters({x,y,.7})-
                         b.poseClearanceMeters({x,y,.7}))<1e-12);
  }
}

TEST_CASE("EDT-assisted clearance integrates exactly the same penalized sections",
          "[clearance][edt][objective]") {
  MapSnapshot map;
  map.width=map.height=100;map.resolutionMeters=.1;
  map.cells.assign(map.width*map.height,0);
  for(int y=20;y<75;++y)map.cells[y*map.width+57]=100;
  const Footprint footprint{-.25,.25,.15};
  const ClearanceScorer exact(map,{},footprint,.15,.4,false,.1);
  const ClearanceScorer edt(map,{},footprint,.15,.4,true,.1);
  PlanDiagnostics diagnostics;
  for(double x:{1.8,2.7,3.5,4.5,5.2,6.4,7.4}) {
    const auto section=makeSection({x,3.,0.},Gear::Forward,0.,.5);
    std::size_t n1=0,n2=0;
    const auto a=exact.sectionCost(section,.1,.15,1000,n1);
    const auto b=edt.sectionCost(section,.1,.15,1000,n2,&diagnostics);
    REQUIRE(n1==n2);
    REQUIRE(std::abs(a-b)<1e-12);
  }
  REQUIRE(diagnostics.clearanceDistanceQueries>0);
  REQUIRE(diagnostics.clearanceDistanceQueries==
          diagnostics.clearanceEdtZeroCostBypasses+
          diagnostics.clearanceExactFallbacks);
}


TEST_CASE("EDT and exact clearance produce identical differential search and path",
          "[clearance][edt][planner]") {
  auto map=std::make_shared<MapSnapshot>();
  map->width=96;map->height=96;map->resolutionMeters=.2;
  map->cells.assign(map->width*map->height,0);
  for(std::size_t y=22;y<34;++y)map->cells[y*map->width+41]=100;
  PlanRequest req;
  req.map=map;
  req.start={4.,5.,0.};
  req.goal={8.,5.,0.};
  req.footprint={-.25,.25,.15};
  req.options.model=PlanOptions::Model::DifferentialDrive;
  req.options.footprintInflationMeters=.1;
  req.options.clearanceWeight=.15;
  req.options.clearanceInfluenceRadiusMeters=.4;
  req.options.maxExpansions=18000;
  req.options.maxGeneratedNodes=72000;
  const auto exact=plan(req);
  req.options.useClearanceDistanceTransform=true;
  const auto edt=plan(req);
  REQUIRE(exact.status==PlanStatus::Success);
  REQUIRE(edt.status==exact.status);
  REQUIRE(exact.diagnostics.expanded==edt.diagnostics.expanded);
  REQUIRE(exact.diagnostics.generated==edt.diagnostics.generated);
  REQUIRE(exact.diagnostics.exactCollisionPoseCalls==
          edt.diagnostics.exactCollisionPoseCalls);
  REQUIRE(exact.diagnostics.clearanceSamples==edt.diagnostics.clearanceSamples);
  REQUIRE(exact.diagnostics.clearanceDistanceQueries==
          edt.diagnostics.clearanceDistanceQueries);
  REQUIRE(edt.diagnostics.clearanceDistanceQueries==
          edt.diagnostics.clearanceEdtZeroCostBypasses+
          edt.diagnostics.clearanceExactFallbacks);
  REQUIRE(edt.diagnostics.clearanceEdtBytes==
          map->cells.size()*sizeof(double));
  REQUIRE(edt.diagnostics.clearanceEdtZeroCostBypasses>0);
  REQUIRE(edt.diagnostics.clearanceExactFallbacks>0);
  REQUIRE(std::abs(exact.weightedCostMeters-edt.weightedCostMeters)<1e-9);
  REQUIRE(std::abs(exact.clearanceCostMeters-edt.clearanceCostMeters)<1e-9);
  REQUIRE(exact.sections.size()==edt.sections.size());
  for(std::size_t i=0;i<exact.sections.size();++i) {
    const auto& a=exact.sections[i];
    const auto& b=edt.sections[i];
    REQUIRE(a.kind==b.kind);
    REQUIRE(a.gear==b.gear);
    REQUIRE(std::abs(a.start.x-b.start.x)<1e-9);
    REQUIRE(std::abs(a.start.y-b.start.y)<1e-9);
    REQUIRE(std::abs(a.start.yaw-b.start.yaw)<1e-9);
    REQUIRE(std::abs(a.end.x-b.end.x)<1e-9);
    REQUIRE(std::abs(a.end.y-b.end.y)<1e-9);
    REQUIRE(std::abs(a.end.yaw-b.end.yaw)<1e-9);
  }
}
