#include "hybrid_astar/steering/reeds_shepp.hpp"
#include "hybrid_astar/steering/reeds_shepp_evaluator.hpp"

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
  // OMPL has called this path accessor 'reedsShepp' and 'getPath'
  // in different versions. Select whichever the installed header provides.
  const auto path=[&]<typename Space>(const Space& space) {
    if constexpr (requires(const Space& s,const ompl::base::State* a,const ompl::base::State* b) {
      s.getPath(a,b);
    }) {
      return space.getPath(i.fromState.get(),i.goalState.get());
    } else {
      return space.reedsShepp(i.fromState.get(),i.goalState.get());
    }
  }(*i.space);

  std::vector<ackermann::MotionSection> sections;
  sections.reserve(5);
  Pose2D position=from;
  double distanceMeters=0.0;
  for (int n=0;n<5;++n) {
    const double signedLengthMeters=path.length_[n]*i.radius;
    if (!std::isfinite(signedLengthMeters))
      throw std::runtime_error("OMPL returned nonfinite RS section");
    if (std::abs(signedLengthMeters)<=1e-12) continue;
    const auto type=path.type_[n];
    double curvature=0.0;
    switch(type) {
      case ompl::base::ReedsSheppStateSpace::RS_LEFT:
        curvature=1.0/i.radius;break;
      case ompl::base::ReedsSheppStateSpace::RS_RIGHT:
        curvature=-1.0/i.radius;break;
      case ompl::base::ReedsSheppStateSpace::RS_STRAIGHT:
        break;
      default:
        throw std::runtime_error("OMPL returned unsupported nonzero RS section");
    }
    const auto gear=signedLengthMeters>0?ackermann::Gear::Forward:ackermann::Gear::Reverse;
    auto section=ackermann::makeSection(position,gear,curvature,std::abs(signedLengthMeters));
    position=section.end;
    distanceMeters+=section.lengthMeters;
    sections.push_back(section);
  }
  const double reported=path.length()*i.radius;
  if (!std::isfinite(reported)||!std::isfinite(distanceMeters)||
      std::abs(reported-distanceMeters)>1e-7*std::max(1.0,reported)||
      std::hypot(position.x-goal.x,position.y-goal.y)>1e-5||
      headingError(position.yaw,goal.yaw)>1e-5)
    throw std::runtime_error("OMPL RS section chain failed exact endpoint/length check");
  return sections;
}

// Backwards-compatible convenience API; use ReedsSheppEvaluator for hot paths
// so the state space and scratch states are reused instead of reallocated.
double reedsSheppDistance(const Pose2D& from,const Pose2D& goal,
                          double minimumTurningRadiusMeters) {
  return ReedsSheppEvaluator(minimumTurningRadiusMeters).distance(from,goal);
}
} // namespace hybrid_astar::steering
