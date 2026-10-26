#include "hybrid_astar/steering/reeds_shepp_evaluator.hpp"
#include <cmath>
#include <iostream>
#include <memory>
#include <numbers>
#include <random>
#include <stdexcept>

using namespace hybrid_astar;
using namespace hybrid_astar::ackermann;
using namespace hybrid_astar::steering;

static void require(bool c,const char* msg){if(!c) throw std::runtime_error(msg);}
static double yawError(double a,double b) {
  return std::abs(std::remainder(a-b,2*std::numbers::pi));
}

int main() {
  std::mt19937_64 rng(20261026);
  std::uniform_real_distribution<double> pos(-10,10),yaw(-3.14159,3.14159);
  for(double radius:{.5,1.,4.,9.}) {
    ReedsSheppEvaluator evaluator(radius);
    std::size_t sections=0,cusps=0;
    for(int i=0;i<3000;++i) {
      Pose2D a{pos(rng),pos(rng),yaw(rng)},b{pos(rng),pos(rng),yaw(rng)};
      if(i==0)b=a;
      if(i==1)b={a.x+1e-10,a.y-1e-10,a.yaw+1e-10};
      auto path=evaluator.shortestPath(a,b);
      const double distance=evaluator.distance(a,b);
      require(std::isfinite(distance)&&distance>=0,"finite RS metric");
      require(std::abs(distance-evaluator.distance(b,a))<1e-6,"symmetric RS metric");
      Gear previous=Gear::None;
      double length=0;
      auto reached=a;
      for(const auto& part:path) {
        require(part.lengthMeters>0&&std::isfinite(part.lengthMeters),"nonzero signed section");
        require(part.gear!=Gear::None&&std::abs(part.curvature)<=1/radius+1e-8,"gear and turn radius");
        require(std::hypot(reached.x-part.start.x,reached.y-part.start.y)<1e-7 &&
                yawError(reached.yaw,part.start.yaw)<1e-7,"exact section continuity");
        const auto end=part.evaluate(part.lengthMeters);
        require(std::hypot(end.x-part.end.x,end.y-part.end.y)<1e-8,"exact evaluator");
        if(previous!=Gear::None&&previous!=part.gear)++cusps;
        previous=part.gear;
        reached=part.end;
        length+=part.lengthMeters;
        ++sections;
      }
      require(std::abs(length-distance)<1e-5*std::max(1.,distance),"OMPL path length vs signed sum");
      require(std::hypot(reached.x-b.x,reached.y-b.y)<1e-5 &&
              yawError(reached.yaw,b.yaw)<1e-5,"actual analytic goal");
    }
    require(sections>1000&&cusps>50,"random samples exercise gear cusps");
  }
  auto map=std::make_shared<MapSnapshot>();
  map->width=map->height=300;map->resolutionMeters=.2;
  map->cells.assign(90000,0);
  PlanRequest q;q.map=map;q.start={25,25,0};q.goal={24,25,0};
  const auto basic=planWithReedsShepp(q);
  require(basic.status==PlanStatus::Success&&basic.diagnostics.analyticAccepted==1,
          "real reverse analytic connection");
  q.options.solutionPolicy=SolutionPolicy::ImproveIncumbent;
  q.options.maxExtraExpansionsAfterFirst=8;
  const auto improved=planWithReedsShepp(q,true);
  require(improved.status==PlanStatus::Success&&!improved.heuristicCertifiedAdmissible,
          "bounded incumbent with real optional RS ordering");
  PlanRequest capped=q;
  capped.options.maxTotalFinalPoseChecks=22; // 1 start + 21 samples for 1m reverse
  capped.options.maxExtraExpansionsAfterFirst=50;
  const auto cappedPlan=planWithReedsShepp(capped);
  require(cappedPlan.status==PlanStatus::Success &&
          cappedPlan.terminationReason==TerminationReason::ValidationBudgetExhausted,
          "global final cap retains previously validated incumbent");
  require(cappedPlan.diagnostics.finalPoseChecks==22,
          "global final budget used exactly without an unchecked evaluation");
  // The combined workflow must preserve a validated original even when
  // postprocessing is unable to spend the required independent check budget.
  auto combined=planAndRefineWithReedsShepp(q);
  require(combined.original.status==PlanStatus::Success&&combined.refinement.has_value(),
          "one-evaluator plan/refine entry point");
  require(combined.refinement->path.status==PlanStatus::Success&&
          combined.refinement->path.weightedCostMeters<=combined.original.weightedCostMeters+1e-9,
          "postprocessor does not worsen the original");
  RefinementOptions tooSmall;tooSmall.maxTotalPoseChecks=1;
  auto bounded=planAndRefineWithReedsShepp(q,tooSmall,true);
  require(bounded.original.status==PlanStatus::Success&&bounded.refinement.has_value()&&
          bounded.refinement->status==RefinementStatus::OriginalNotVerified&&
          bounded.refinement->path.sections.empty(),
          "failed postprocessing cannot erase separately returned validated original");
  // Interposed obstacle: no analytic candidate can bypass the strict validator.
  for(std::size_t y=0;y<300;++y)
    if(y<115||y>135)map->cells[y*300+150]=100;
  q.options.solutionPolicy=SolutionPolicy::FirstStrictlyValidated;
  q.options.maxExpansions=2000;q.goal={29,25,0};
  const auto blocked=planWithReedsShepp(q);
  require(blocked.status!=PlanStatus::InternalError,"obstacle never causes adapter crash");
  if(blocked.status==PlanStatus::Success)
    require(blocked.collisionEvidence==CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled,
            "collision evidence from independent strict path validation");
  std::cout<<"OMPL RANDOMIZED INTEGRATION PASS\n";
}
