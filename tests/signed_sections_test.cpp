#include "hybrid_astar/steering/signed_sections.hpp"
#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
using namespace hybrid_astar;
using namespace hybrid_astar::ackermann;
using namespace hybrid_astar::steering;
static void require(bool okay,const char* msg) {if(!okay) throw std::runtime_error(msg);}
int main() {
  const Pose2D start{20,20,0};
  const std::array<SignedSegment,5> parts{{{SegmentTurn::Straight,2},
    {SegmentTurn::Straight,-2},{SegmentTurn::Straight,1},
    {SegmentTurn::None,0},{SegmentTurn::Left,0}}};
  auto decoded=decodeSignedSections(start,{21,20,0},4,parts);
  require(decoded.size()==3,"zero sections removed");
  require(decoded[0].gear==Gear::Forward&&decoded[1].gear==Gear::Reverse&&
          decoded[2].gear==Gear::Forward,"two cusps retain signed gear");
  require(std::abs(decoded[1].start.x-decoded[0].end.x)<1e-12,"chain continuous");
  const std::array<SignedSegment,2> arc{{{SegmentTurn::Left,.8},{SegmentTurn::Left,-.8}}};
  auto reverse=decodeSignedSections(start,start,4,arc);
  require(reverse.size()==2&&reverse[0].curvature>.24&&reverse[1].gear==Gear::Reverse,
          "reverse curvature and endpoint exact");
  auto changed=parts;
  changed[2].turn=SegmentTurn::None;
  bool thrown=false;
  try {decodeSignedSections(start,{21,20,0},4,changed);}catch(const std::invalid_argument&) {thrown=true;}
  require(thrown,"nonzero no-op rejected");
  thrown=false;
  try {decodeSignedSections(start,{22,20,0},4,parts);}catch(const std::runtime_error&) {thrown=true;}
  require(thrown,"endpoint mismatch rejected");
  auto map=std::make_shared<MapSnapshot>();
  map->width=map->height=200;map->resolutionMeters=.2;map->cells.assign(40000,0);
  PlanRequest request;
  request.map=map;request.start=start;request.goal={21,20,0};
  request.analyticConnector=[&](const Pose2D& from,const Pose2D& goal) {
    return decodeSignedSections(from,goal,4,parts);
  };
  const auto planResult=plan(request);
  require(planResult.status==PlanStatus::Success,"decoded chain accepted by strict planner");
  require(planResult.diagnostics.analyticAttempts==1&&planResult.diagnostics.analyticAccepted==1,
          "first-node signed analytic accepted");
  require(planResult.gearChangeSectionIndices.size()==2&&
          planResult.gearChangeSectionIndices[0]==1&&planResult.gearChangeSectionIndices[1]==2,
          "two explicit cusp locations");
  require(std::abs(planResult.weightedCostMeters-6.2)<1e-9,"hand-calculated signed weighted cost");
  require(std::abs(planResult.reverseLengthMeters-2.)<1e-9,"reverse mileage");
  request.options.analytic.maxWeightedCostMeters=5.0;
  const auto fallback=plan(request);
  require(fallback.status==PlanStatus::Success&&fallback.diagnostics.analyticAccepted==0,
          "cost-gated path falls back to primitive search");
  std::cout<<"SIGNED SECTIONS PASS\n";
}
