// Opt-in local benchmark; no GitHub Actions target, no latency guarantee.
#include "hybrid_astar/ackermann/refinement.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numbers>
#include <string>
#include <vector>
using namespace hybrid_astar;
using namespace hybrid_astar::ackermann;
struct Scenario {std::string name; PlanRequest request; PlanResult raw;};
static Scenario fixture(std::string name) {
  auto m=std::make_shared<MapSnapshot>();m->width=m->height=400;
  m->resolutionMeters=.2;m->cells.assign(160000,0);m->revision=7;
  PlanRequest q;q.map=m;q.start={20,20,0};
  std::vector<MotionSection> sections;Pose2D p=q.start;
  auto append=[&](Gear gear,double k,double length) {
    auto part=makeSection(p,gear,k,length);sections.push_back(part);p=part.end;
  };
  if(name=="straight_cusp") {
    append(Gear::Forward,0,2);append(Gear::Reverse,0,1);append(Gear::Forward,0,1);
  } else if(name=="curved_cusp") {
    append(Gear::Forward,.2,2);append(Gear::Reverse,.2,1);append(Gear::Forward,.2,1);
  } else {
    for(double k:{.25,-.25,-.25,.25}) append(Gear::Forward,k,2*std::numbers::pi);
    m->cells[100*400+140]=100;
  }
  q.goal=p;
  PlanResult raw;raw.status=PlanStatus::Success;raw.sections=sections;
  raw.mapRevision=q.map->revision;raw.mapFrameId=q.map->frameId;raw.mapSourceId=q.map->sourceId;
  raw.effectiveOccupancy=q.occupancy;raw.modelGeometryValid=true;
  raw.collisionEvidence=CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled;
  return {name,q,raw};
}
static double percentile(std::vector<double> times,double fraction) {
  std::sort(times.begin(),times.end());
  const auto i=static_cast<std::size_t>(std::ceil(double(times.size())*fraction));
  return times[std::min(times.size()-1,i?i-1:0)];
}
int main(int argc,char** argv) {
  int repetitions=argc>1?std::max(1,std::stoi(argv[1])):15;
  std::cout << "scenario,mode,runs,validated,improved,p50_ms,p95_ms,mean_checks,final_cost\n";
  for(const auto& name:{"straight_cusp","curved_cusp","blocked_direct"}) {
    auto scenario=fixture(name);
    for(bool curve:{false,true}) {
      std::vector<double> elapsed;
      int valid=0,improved=0;std::size_t checks=0;double lastCost=-1;
      RefinementOptions options;options.useCurvatureBoundedShortcuts=curve;
      for(int i=0;i<repetitions;++i) {
        const auto begin=std::chrono::steady_clock::now();
        const auto result=refinePath(scenario.request,scenario.raw,{},options);
        const auto end=std::chrono::steady_clock::now();
        elapsed.push_back(std::chrono::duration<double,std::milli>(end-begin).count());
        valid+=result.path.status==PlanStatus::Success;
        improved+=result.accepted>0;
        checks+=result.exactPoseChecks;
        lastCost=result.path.status==PlanStatus::Success?result.path.weightedCostMeters:-1;
      }
      std::cout<<scenario.name<<','<<(curve?"curvature_CSC":"straight_only")<<','<<repetitions
               <<','<<valid<<','<<improved<<','<<std::fixed<<std::setprecision(3)
               <<percentile(elapsed,.5)<<','<<percentile(elapsed,.95)<<','
               <<double(checks)/repetitions<<','<<lastCost<<'\n';
    }
  }
}
