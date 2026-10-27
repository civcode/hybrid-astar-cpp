#include "hybrid_astar/steering/reeds_shepp.hpp"
#include "hybrid_astar/steering/reeds_shepp_evaluator.hpp"
#include "hybrid_astar/steering/signed_sections.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <utility>

#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

namespace hybrid_astar::steering {
namespace {
void validatePose(const Pose2D& p) {
  if (!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.yaw))
    throw std::invalid_argument("Reeds-Shepp requires finite poses");
}
double headingError(double a,double b) {
  return std::abs(std::remainder(a-b,2.0*std::numbers::pi_v<double>));
}
}

struct ReedsSheppEvaluator::Impl {
  double radius;
  std::shared_ptr<ompl::base::ReedsSheppStateSpace> space;
  ompl::base::ScopedState<> fromState, goalState;

  explicit Impl(double r)
      : radius(r),
        space(std::make_shared<ompl::base::ReedsSheppStateSpace>(r)),
        fromState(space),goalState(space) {}

  void setStates(const Pose2D& from,const Pose2D& goal) {
    validatePose(from);validatePose(goal);
    fromState[0]=from.x;fromState[1]=from.y;fromState[2]=from.yaw;
    goalState[0]=goal.x;goalState[1]=goal.y;goalState[2]=goal.yaw;
  }
};

ReedsSheppEvaluator::ReedsSheppEvaluator(double radius) {
  if (!std::isfinite(radius)||radius<=0)
    throw std::invalid_argument("Reeds-Shepp turning radius must be finite and positive");
  impl_=std::make_unique<Impl>(radius);
}
ReedsSheppEvaluator::~ReedsSheppEvaluator()=default;
ReedsSheppEvaluator::ReedsSheppEvaluator(ReedsSheppEvaluator&&) noexcept=default;
ReedsSheppEvaluator& ReedsSheppEvaluator::operator=(ReedsSheppEvaluator&&) noexcept=default;

double ReedsSheppEvaluator::distance(const Pose2D& from,const Pose2D& goal) const {
  auto& i=*impl_;
  i.setStates(from,goal);
  const double d=i.space->distance(i.fromState.get(),i.goalState.get());
  if (!std::isfinite(d)||d<0) throw std::runtime_error("OMPL returned invalid RS distance");
  return d;
}

std::vector<ackermann::MotionSection>
ReedsSheppEvaluator::shortestPath(const Pose2D& from,const Pose2D& goal) const {
  auto& i=*impl_;
  i.setStates(from,goal);
  // Public OMPL API. The five path elements have SIGNED length in units
  // of turning radius; preserve signs and section types before decoding.
  const auto path=i.space->getPath(i.fromState.get(),i.goalState.get());
  std::array<SignedSegment,5> native{};
  for(std::size_t n=0;n<native.size();++n) {
    SegmentTurn kind=SegmentTurn::None;
    switch(path.type_[n]) {
      case ompl::base::ReedsSheppStateSpace::RS_NOP:kind=SegmentTurn::None;break;
      case ompl::base::ReedsSheppStateSpace::RS_LEFT:kind=SegmentTurn::Left;break;
      case ompl::base::ReedsSheppStateSpace::RS_RIGHT:kind=SegmentTurn::Right;break;
      case ompl::base::ReedsSheppStateSpace::RS_STRAIGHT:kind=SegmentTurn::Straight;break;
      default:throw std::runtime_error("Unexpected OMPL RS section kind");
    }
    native[n]={kind,path.length_[n]*i.radius};
  }
  auto chain=decodeSignedSections(from,goal,i.radius,native);
  double reconstructed=0;
  for(const auto& part:chain) reconstructed+=part.lengthMeters;
  const double reported=path.length()*i.radius;
  if(!std::isfinite(reported)||!std::isfinite(reconstructed)||
     std::abs(reported-reconstructed)>1e-7*std::max(1.0,reported))
    throw std::runtime_error("OMPL RS signed-section length mismatch");
  return chain;
}

// Backwards-compatible convenience API; use ReedsSheppEvaluator for hot paths
// so the state space and scratch states are reused instead of reallocated.
double reedsSheppDistance(const Pose2D& from,const Pose2D& goal,
                          double minimumTurningRadiusMeters) {
  return ReedsSheppEvaluator(minimumTurningRadiusMeters).distance(from,goal);
}
// One reusable OMPL evaluator, with both callbacks scoped to this synchronous
// call. Core defaults to obstacle-grid ordering; an explicit certified
// heuristic mode on the request remains honored when the flag is false.
// The true flag additionally installs the OMPL RS distance estimate.
ackermann::PlanResult planWithReedsShepp(ackermann::PlanRequest request,
                                        bool useReedsSheppDistanceHeuristic) {
  ReedsSheppEvaluator evaluator(request.options.minimumTurningRadiusMeters);
  request.analyticConnector=[&evaluator](const Pose2D& from,const Pose2D& goal) {
    return evaluator.shortestPath(from,goal);
  };
  if(useReedsSheppDistanceHeuristic) {
    request.options.heuristicMode=ackermann::HeuristicMode::PerformanceReedsSheppAndObstacleGrid;
    request.analyticDistance=[&evaluator](const Pose2D& from,const Pose2D& goal) {
      return evaluator.distance(from,goal);
    };
  }
  return ackermann::plan(request);
}
ReedsSheppRefinementResult planAndRefineWithReedsShepp(
    ackermann::PlanRequest request,
    ackermann::RefinementOptions refinementOptions,
    bool useReedsSheppDistanceHeuristic) {
  ReedsSheppEvaluator evaluator(request.options.minimumTurningRadiusMeters);
  request.analyticConnector=[&evaluator](const Pose2D& from,const Pose2D& goal) {
    return evaluator.shortestPath(from,goal);
  };
  if(useReedsSheppDistanceHeuristic) {
    request.options.heuristicMode=ackermann::HeuristicMode::PerformanceReedsSheppAndObstacleGrid;
    request.analyticDistance=[&evaluator](const Pose2D& from,const Pose2D& goal) {
      return evaluator.distance(from,goal);
    };
  }
  ReedsSheppRefinementResult result;
  result.original=ackermann::plan(request);
  if(result.original.status==ackermann::PlanStatus::Success) {
    result.refinement=ackermann::refinePath(request,result.original,
      [&evaluator](const Pose2D& from,const Pose2D& to) {
        return evaluator.shortestPath(from,to);
      },refinementOptions);
  }
  return result;
}
} // namespace hybrid_astar::steering
