#pragma once

#include "hybrid_astar/articulated_steering/model.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace hybrid_astar::articulated_steering {

struct GoalHeuristicTolerances {
  double xyMeters=0.0;
  double yawRadians=0.0;
  double gammaRadians=0.0;
};

// Obstacle-free lower bound on the planner's driving + articulation effort.
// Valid for the no-slip articulated ODE, a feasible positive D(gamma),
// nonnegative tolerances/costs and reverse driving multiplier >= 1.
// Does not model collisions, steering transition penalties, or time.
class AnalyticGoalHeuristic {
public:
  AnalyticGoalHeuristic(Parameters p, RearState goal,
                        GoalHeuristicTolerances tolerance,
                        double articulationCostMetersPerRadian)
      : p_(p),goal_(goal),tolerance_(tolerance),
        articulationCost_(articulationCostMetersPerRadian) {
    const double a=p_.LrMeters,b=p_.LfMeters;
    const auto curvature=[&](double gamma) {
      return std::abs(std::sin(gamma)/(a+b*std::cos(gamma)));
    };
    maxHeadingPerFrontMeter_=std::max(curvature(p_.minGammaRadians),
                                     curvature(p_.maxGammaRadians));
    // d/dgamma [sin(gamma)/D(gamma)] =
    // (Lr*cos(gamma)+Lf)/D(gamma)^2. Consider interior extrema.
    if(a>b) {
      const double critical=std::acos(-b/a);
      for(const double gamma:{-critical,critical})
        if(gamma>=p_.minGammaRadians&&gamma<=p_.maxGammaRadians)
          maxHeadingPerFrontMeter_=std::max(maxHeadingPerFrontMeter_,curvature(gamma));
    }
    const auto sensitivity=[&](double gamma) {
      return std::abs(b*std::cos(gamma)/(a+b*std::cos(gamma)));
    };
    maxHeadingOffsetPerGamma_=std::max(sensitivity(p_.minGammaRadians),
                                      sensitivity(p_.maxGammaRadians));
    if(p_.minGammaRadians<=0.0&&p_.maxGammaRadians>=0.0)
      maxHeadingOffsetPerGamma_=std::max(maxHeadingOffsetPerGamma_,sensitivity(0.0));
    goalHeading_=goal_.thetaRear+headingOffset(goal_.gamma);
    goalFrontX_=goal_.xr+a*std::cos(goal_.thetaRear)+
                b*std::cos(goal_.thetaRear+goal_.gamma);
    goalFrontY_=goal_.yr+a*std::sin(goal_.thetaRear)+
                b*std::sin(goal_.thetaRear+goal_.gamma);
    frontTolerance_=tolerance_.xyMeters+(a+b)*tolerance_.yawRadians+
                    b*tolerance_.gammaRadians;
    headingTolerance_=tolerance_.yawRadians+
                      maxHeadingOffsetPerGamma_*tolerance_.gammaRadians;
  }

  // B'(gamma) = Lf*cos(gamma)/(Lr+Lf*cos(gamma)), B(0)=0.
  // Closed-form primitive; unlike the vehicle's RK4 motion integration,
  // this is evaluated once per lattice articulation angle.
  double headingOffset(double gamma) const noexcept {
    const double a=p_.LrMeters,b=p_.LfMeters;
    const double t=std::tan(0.5*gamma);
    const double d=a*a-b*b;
    double integral=0.0;
    if(std::abs(d)<=1e-12*(a+b)*(a+b)) {
      integral=t/a;
    } else if(d>0.0) {
      const double root=std::sqrt(d);
      integral=2.0/root*std::atan(root*t/(a+b));
    } else {
      const double root=std::sqrt(-d);
      integral=2.0/root*std::atanh(root*t/(a+b));
    }
    return gamma-a*integral;
  }

  double estimate(RearState from) const noexcept {
    return estimate(from,headingOffset(from.gamma));
  }
  // Reuse B(gamma) precomputed for the request-local gamma lattice.
  double estimate(RearState from,double fromHeadingOffset) const noexcept {
    const double a=p_.LrMeters,b=p_.LfMeters;
    const double fx=from.xr+a*std::cos(from.thetaRear)+
                    b*std::cos(from.thetaRear+from.gamma);
    const double fy=from.yr+a*std::sin(from.thetaRear)+
                    b*std::sin(from.thetaRear+from.gamma);
    const double front=std::max(0.0,std::hypot(fx-goalFrontX_,fy-goalFrontY_)-frontTolerance_);
    const double angle=std::abs(std::remainder(
        goalHeading_-(from.thetaRear+fromHeadingOffset),2.0*std::numbers::pi_v<double>));
    const double heading=maxHeadingPerFrontMeter_>0.0
        ?std::max(0.0,angle-headingTolerance_)/maxHeadingPerFrontMeter_:0.0;
    const double joint=articulationCost_*std::max(0.0,
        std::abs(goal_.gamma-from.gamma)-tolerance_.gammaRadians);
    return std::max(front,heading)+joint;
  }

private:
  Parameters p_;
  RearState goal_;
  GoalHeuristicTolerances tolerance_;
  double articulationCost_;
  double maxHeadingPerFrontMeter_=0.0;
  double maxHeadingOffsetPerGamma_=0.0;
  double goalHeading_=0.0,goalFrontX_=0.0,goalFrontY_=0.0;
  double frontTolerance_=0.0,headingTolerance_=0.0;
};

} // namespace hybrid_astar::articulated_steering
