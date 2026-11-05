#include "hybrid_astar/articulated_steering/goal_connector.hpp"
#include "hybrid_astar/articulated_steering/heuristic.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>

namespace hybrid_astar::articulated_steering {
namespace {
constexpr double pi=std::numbers::pi_v<double>;
using Vector=std::array<double,6>;
using Error=std::array<double,3>;
using Gears=std::array<int,3>;

// The generator uses an exact, RK4-free proposal model. A steering phase
// holds the FRONT reference fixed, and a drive phase follows a fixed-gamma
// circular arc. Crucially, stationary steering is NOT a no-motion curvature
// change: the rear chassis moves around the fixed front reference.
RearState steer(RearState r,double gamma,const Parameters& p,
                const AnalyticGoalHeuristic& offset) {
  const double frontX=r.xr+p.LrMeters*std::cos(r.thetaRear)+
      p.LfMeters*std::cos(r.thetaRear+r.gamma);
  const double frontY=r.yr+p.LrMeters*std::sin(r.thetaRear)+
      p.LfMeters*std::sin(r.thetaRear+r.gamma);
  r.thetaRear-=offset.headingOffset(gamma)-offset.headingOffset(r.gamma);
  r.gamma=gamma;
  r.xr=frontX-p.LrMeters*std::cos(r.thetaRear)-
      p.LfMeters*std::cos(r.thetaRear+gamma);
  r.yr=frontY-p.LrMeters*std::sin(r.thetaRear)-
      p.LfMeters*std::sin(r.thetaRear+gamma);
  return r;
}
RearState drive(RearState r,double signedFrontMeters,const Parameters& p) {
  const double D=p.LrMeters+p.LfMeters*std::cos(r.gamma);
  const double yaw=signedFrontMeters*std::sin(r.gamma)/D;
  const double rearMeters=signedFrontMeters*
      (p.LfMeters+p.LrMeters*std::cos(r.gamma))/D;
  const double sinc=std::abs(yaw)<1e-5
      ?1-yaw*yaw/6+yaw*yaw*yaw*yaw/120:std::sin(yaw)/yaw;
  const double cosc=std::abs(yaw)<1e-5
      ?yaw/2-yaw*yaw*yaw/24+yaw*yaw*yaw*yaw*yaw/720
      :(1-std::cos(yaw))/yaw;
  const double c=std::cos(r.thetaRear),s=std::sin(r.thetaRear);
  r.xr+=rearMeters*(c*sinc-s*cosc);
  r.yr+=rearMeters*(s*sinc+c*cosc);
  r.thetaRear+=yaw;
  return r;
}
struct Eval {
  RearState end;
  Error error{};
  double norm=std::numeric_limits<double>::infinity();
};
Eval evaluate(RearState from,RearState goal,const Vector& x,const Gears& signs,
              const Parameters& p,const AnalyticGoalHeuristic& offset) {
  for(int i=0;i<3;++i) {
    from=steer(from,x[i],p,offset);
    from=drive(from,signs[i]*x[i+3],p);
  }
  from=steer(from,goal.gamma,p,offset);
  const Error e{from.xr-goal.xr,from.yr-goal.yr,
                2.0*std::remainder(from.thetaRear-goal.thetaRear,2*pi)};
  return {from,e,e[0]*e[0]+e[1]*e[1]+e[2]*e[2]};
}
// Small, pivoted 3x3 solve for a damped least-squares step.
// A=J*J^T+lambda*I is SPD for positive lambda, including singular J.
bool solve3(std::array<std::array<double,3>,3> A,Error rhs,Error& out) {
  for(int k=0;k<3;++k) {
    int pivot=k;
    for(int i=k+1;i<3;++i)
      if(std::abs(A[i][k])>std::abs(A[pivot][k]))pivot=i;
    if(std::abs(A[pivot][k])<1e-14||!std::isfinite(A[pivot][k]))
      return false;
    if(pivot!=k){std::swap(A[k],A[pivot]);std::swap(rhs[k],rhs[pivot]);}
    for(int i=k+1;i<3;++i) {
      const double factor=A[i][k]/A[k][k];
      for(int j=k;j<3;++j)A[i][j]-=factor*A[k][j];
      rhs[i]-=factor*rhs[k];
    }
  }
  for(int i=2;i>=0;--i) {
    double value=rhs[i];
    for(int j=i+1;j<3;++j)value-=A[i][j]*out[j];
    out[i]=value/A[i][i];
    if(!std::isfinite(out[i]))return false;
  }
  return true;
}
void clampVector(Vector& x,const Parameters& p,double maxArc) {
  for(int i=0;i<3;++i)x[i]=std::clamp(x[i],p.minGammaRadians,p.maxGammaRadians);
  for(int i=3;i<6;++i)x[i]=std::clamp(x[i],0.0,maxArc);
}
bool nearTarget(const Eval& e,const SearchOptions& o) {
  return std::hypot(e.error[0],e.error[1])<
      .65*o.goalXYToleranceMeters &&
      std::abs(e.error[2])<1.3*o.goalYawToleranceRadians;
}
std::vector<GoalControl> makeControls(const Vector& x,const Gears& signs,
                                     RearState from,RearState goal,
                                     const SearchOptions& o) {
  std::vector<GoalControl> out;
  out.reserve(32);
  const double v=o.translationSpeedMetersPerSecond;
  const double u=o.articulationRateRadiansPerSecond;
  const auto addSteer=[&](double to) {
    const double delta=to-from.gamma;
    if(std::abs(delta)>1e-8) {
      out.push_back({{0.0,std::copysign(u,delta)},std::abs(delta)/u});
      from.gamma=to;
    }
  };
  for(int i=0;i<3;++i) {
    addSteer(x[i]);
    // Split long exact arcs into bounded-duration pieces. This retains the
    // same physical motion but respects adaptive sample limits in both
    // preliminary collision checks and independent final RK4 validation.
    double remaining=x[i+3];
    const double chunkMeters=std::clamp(o.translationPrimitiveMeters*2.0,.25,1.0);
    while(remaining>1e-6) {
      const double distance=std::min(remaining,chunkMeters);
      out.push_back({{signs[i]*v,0.0},distance/v});
      remaining-=distance;
    }
  }
  addSteer(goal.gamma);
  return out;
}
double costOf(const std::vector<GoalControl>& segments,const SearchOptions& o) {
  double cost=0.0;
  for(const auto& segment:segments) {
    const double distance=std::abs(segment.control.frontSpeedMetersPerSecond)*
        segment.durationSeconds;
    const double joint=std::abs(segment.control.gammaRateRadiansPerSecond)*
        segment.durationSeconds;
    cost+=distance*(segment.control.frontSpeedMetersPerSecond<0.0
        ?o.reverseCostMultiplier:1.0)+
        joint*o.articulationCostMetersPerRadian;
    if(distance==0.0&&joint>0.0)cost+=o.stationaryArticulationPenaltyMeters;
  }
  return cost;
}
} // namespace

std::vector<GoalConnection> proposeGoalConnections(
    const Model& model,RearState from,RearState goal,
    const SearchOptions& options,std::size_t& evaluations) {
  evaluations=0;
  std::vector<GoalConnection> found;
  const auto& p=model.parameters();
  const auto maxArc=options.maxGoalConnectorArcMeters;
  const auto budget=options.maxGoalConnectorEvaluationsPerAttempt;
  if(budget==0||maxArc<=0.0)return found;
  const AnalyticGoalHeuristic B(p,goal,{},0.0);
  const double distance=std::hypot(goal.xr-from.xr,goal.yr-from.yr);
  const double ahead=(goal.xr-from.xr)*std::cos(from.thetaRear)+
                     (goal.yr-from.yr)*std::sin(from.thetaRear);
  // Reverse-first ordering matters for small work budgets and goals behind
  // the vehicle. Every family is tried with bounded signed arc lengths.
  const std::array<Gears,8> patterns=ahead<0.0
      ?std::array<Gears,8>{{{-1,-1,-1},{-1,-1,1},{-1,1,1},
                            {-1,1,-1},{1,-1,-1},{1,1,-1},
                            {1,-1,1},{1,1,1}}}
      :std::array<Gears,8>{{{1,1,1},{1,1,-1},{1,-1,-1},
                            {1,-1,1},{-1,1,1},{-1,-1,1},
                            {-1,1,-1},{-1,-1,-1}}};
  const double bearing=std::atan2(goal.yr-from.yr,goal.xr-from.xr);
  const double aimed=std::clamp(.5*std::remainder(
      bearing-from.thetaRear,2*pi),p.minGammaRadians,p.maxGammaRadians);
  const double turn=std::remainder(goal.thetaRear-from.thetaRear,2*pi);
  const double steerSign=turn<0?-1.0:1.0;
  const double amp=std::min(.85,std::max(0.0,p.maxGammaRadians));
  const double minusAmp=std::max(-.85,p.minGammaRadians);
  // Small S-shaped arcs often provide a clean path between separated
  // poses with the SAME heading; try them before large-articulation seeds.
  // This also helps eliminate avoidable open-field steering oscillations.
  const double approachSign=aimed<0.0?-1.0:1.0;
  const std::array<std::array<double,3>,5> seeds{{
      {approachSign*.12,0.0,-approachSign*.12},
      {aimed,aimed,aimed},
      {steerSign*amp,-steerSign*amp,steerSign*amp},
      {-steerSign*amp,steerSign*amp,-steerSign*amp},
      {minusAmp,0.0,amp}
  }};
  for(const Gears& signs:patterns) {
    for(const auto& seed:seeds) {
      if(evaluations>=budget)break;
      Vector x{seed[0],seed[1],seed[2],
               distance/3.0+.2,distance/3.0+.2,distance/3.0+.2};
      clampVector(x,p,maxArc);
      Eval best=evaluate(from,goal,x,signs,p,B);++evaluations;
      double lambda=.03;
      for(int iteration=0;iteration<21&&!nearTarget(best,options);++iteration) {
        if(evaluations+6>=budget)break;
        std::array<std::array<double,6>,3> J{};
        for(int j=0;j<6;++j) {
          auto perturbed=x;
          const double step=j<3?1e-4:1e-3;
          perturbed[j]+=step;
          clampVector(perturbed,p,maxArc);
          const double actual=perturbed[j]-x[j];
          if(std::abs(actual)<1e-10) {
            perturbed=x;perturbed[j]-=step;
            clampVector(perturbed,p,maxArc);
          }
          const double d=perturbed[j]-x[j];
          if(std::abs(d)<1e-10)continue;
          const Eval next=evaluate(from,goal,perturbed,signs,p,B);
          ++evaluations;
          for(int k=0;k<3;++k)
            J[k][j]=k==2
                ?2.0*std::remainder((next.error[k]-best.error[k])/2.0,2*pi)/d
                :(next.error[k]-best.error[k])/d;
        }
        std::array<std::array<double,3>,3> A{};
        for(int row=0;row<3;++row)for(int col=0;col<3;++col) {
          for(int j=0;j<6;++j)A[row][col]+=J[row][j]*J[col][j];
          if(row==col)A[row][col]+=lambda;
        }
        Error direction{};
        if(!solve3(A,best.error,direction))break;
        Vector delta{};
        for(int j=0;j<6;++j) {
          for(int k=0;k<3;++k)delta[j]-=J[k][j]*direction[k];
          delta[j]=std::clamp(delta[j],j<3?-.5:-3.0,j<3?.5:3.0);
        }
        bool improved=false;
        for(double scale:{1.0,.5,.25,.125}) {
          if(evaluations>=budget)break;
          auto trial=x;
          for(int j=0;j<6;++j)trial[j]+=scale*delta[j];
          clampVector(trial,p,maxArc);
          const Eval next=evaluate(from,goal,trial,signs,p,B);
          ++evaluations;
          if(next.norm+1e-10<best.norm) {
            x=trial;best=next;improved=true;break;
          }
        }
        lambda=std::clamp(lambda*(improved?.7:4.0),1e-5,1e7);
        if(lambda>=1e6)break;
      }
      if(!nearTarget(best,options))continue;
      auto controls=makeControls(x,signs,from,goal,options);
      if(controls.empty())continue;
      const double cost=costOf(controls,options);
      if(std::isfinite(cost))
        found.push_back({std::move(controls),best.end,cost});
    }
    if(evaluations>=budget)break;
  }
  std::stable_sort(found.begin(),found.end(),[](const auto& a,const auto& b) {
    return a.costMeters<b.costMeters;
  });
  if(found.size()>options.maxGoalConnectorCandidatesPerAttempt)
    found.resize(options.maxGoalConnectorCandidatesPerAttempt);
  return found;
}
} // namespace hybrid_astar::articulated_steering
