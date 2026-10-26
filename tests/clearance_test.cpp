#include "hybrid_astar/ackermann/clearance.hpp"
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
using namespace hybrid_astar;
using namespace hybrid_astar::ackermann;
static void require(bool ok,const char* why) {if(!ok) throw std::runtime_error(why);}
int main() {
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
