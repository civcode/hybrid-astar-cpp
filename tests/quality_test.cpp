#include "hybrid_astar/ackermann/planner.hpp"
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
using namespace hybrid_astar;
using namespace hybrid_astar::ackermann;
static void require(bool yes,const char* why) {if(!yes) throw std::runtime_error(why);}
int main() {
  auto map=std::make_shared<MapSnapshot>();
  map->width=map->height=200;map->resolutionMeters=.2;
  map->cells.assign(40000,0);
  PlanRequest req;
  req.map=map;req.start={20,20,0};req.goal={22,20,0};
  req.options.maxExpansions=200;
  req.analyticConnector=[](const Pose2D& p,const Pose2D&) {
    if (std::abs(p.x-20)<1e-8 && std::abs(p.y-20)<1e-8) {
      auto a=makeSection(p,Gear::Reverse,0,1.0);
      auto b=makeSection(a.end,Gear::Forward,0,3.0);
      return std::vector<MotionSection>{a,b};
    }
    return std::vector<MotionSection>{};
  };
  auto first=plan(req);
  require(first.status==PlanStatus::Success,"first valid");
  require(first.terminationReason==TerminationReason::FirstValidated,"first reason");
  require(std::abs(first.weightedCostMeters-4.6)<1e-8,"expensive first connector");
  req.options.solutionPolicy=SolutionPolicy::ImproveIncumbent;
  req.options.maxExtraExpansionsAfterFirst=150;
  auto improved=plan(req);
  require(improved.status==PlanStatus::Success,"incumbent success");
  require(improved.weightedCostMeters<first.weightedCostMeters-1e-9,"strictly better incumbent");
  require(improved.diagnostics.incumbentImprovements>0,"count improved");
  require(improved.solutionQuality==SolutionQuality::FeasibleAfterImprovement,"quality metadata");
  require(improved.terminationReason==TerminationReason::ImprovementBudgetExhausted||
          improved.terminationReason==TerminationReason::WorkBudgetExceeded||
          improved.terminationReason==TerminationReason::SearchExhausted,"termination metadata");
  req.options.maxExtraExpansionsAfterFirst=0;
  auto zero=plan(req);
  require(zero.weightedCostMeters==first.weightedCostMeters,"no extra work");
  require(zero.terminationReason==TerminationReason::ImprovementBudgetExhausted,"bounded zero");
  req.options.maxExtraExpansionsAfterFirst=150;
  req.options.maxExpansions=1;
  auto work=plan(req);
  require(work.status==PlanStatus::Success,"validated incumbent kept at cap");
  require(work.terminationReason==TerminationReason::WorkBudgetExceeded,"work cap explicit");
  require(work.collisionEvidence==CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled,"no evidence downgrade");
  std::cout << "QUALITY PASS\n";
}
