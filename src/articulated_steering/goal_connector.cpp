#include "hybrid_astar/articulated_steering/goal_connector.hpp"
#include "hybrid_astar/articulated_steering/heuristic.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <utility>

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
// Coupled steering: integrate front-reference XY by fixed Simpson quadrature,
// while transforming rear heading analytically. This avoids per-evaluation
// RK4 and respects nonzero chassis motion during articulation changes.
// Full independent RK4 and exact two-body checks still decide acceptance.
RearState coupledDrive(RearState r,double signedFrontMeters,double gamma,
                       const Parameters& p,const AnalyticGoalHeuristic& B) {
  const double delta=gamma-r.gamma;
  if(std::abs(delta)<1e-10)return drive(r,signedFrontMeters,p);
  const double d0=p.LrMeters+p.LfMeters*std::cos(r.gamma);
  const double beta0=r.thetaRear+B.headingOffset(r.gamma);
  const double ratio=signedFrontMeters/(delta*p.LfMeters);
  const auto beta=[&](double g) {
    return beta0-ratio*std::log((p.LrMeters+p.LfMeters*std::cos(g))/d0);
  };
  const auto thetaFront=[&](double s) {
    const double g=r.gamma+s*delta;
    return beta(g)-B.headingOffset(g)+g;
  };
  double fx=r.xr+p.LrMeters*std::cos(r.thetaRear)+
            p.LfMeters*std::cos(r.thetaRear+r.gamma);
  double fy=r.yr+p.LrMeters*std::sin(r.thetaRear)+
            p.LfMeters*std::sin(r.thetaRear+r.gamma);
  constexpr int steps=8;
  for(int i=0;i<steps;++i) {
    const double s0=double(i)/steps,s1=double(i+1)/steps;
    const double t0=thetaFront(s0),tm=thetaFront((s0+s1)/2),t1=thetaFront(s1);
    const double ds=signedFrontMeters/(6.0*steps);
    fx+=ds*(std::cos(t0)+4.0*std::cos(tm)+std::cos(t1));
    fy+=ds*(std::sin(t0)+4.0*std::sin(tm)+std::sin(t1));
  }
  r.thetaRear=beta(gamma)-B.headingOffset(gamma);
  r.gamma=gamma;
  r.xr=fx-p.LrMeters*std::cos(r.thetaRear)-
       p.LfMeters*std::cos(r.thetaRear+gamma);
  r.yr=fy-p.LrMeters*std::sin(r.thetaRear)-
       p.LfMeters*std::sin(r.thetaRear+gamma);
  return r;
}
struct Phase {
  double preGamma=0.0, coupledMeters=0.0, remainingMeters=0.0;
};
Phase phasePlan(double currentGamma,double targetGamma,double length,
                bool coupled,const SearchOptions& o) {
  const double delta=targetGamma-currentGamma;
  const double coupledMeters=coupled?std::min(length,
      std::abs(delta)*o.translationSpeedMetersPerSecond/
                      o.articulationRateRadiansPerSecond):0.0;
  const double jointCoupled=coupledMeters*
      o.articulationRateRadiansPerSecond/o.translationSpeedMetersPerSecond;
  const double preGamma=targetGamma-std::copysign(jointCoupled,delta);
  return {preGamma,coupledMeters,length-coupledMeters};
}
// Operational quality is a secondary objective, NOT a substitute for
// geometric/clearance feasibility. The fitting stage first reaches a valid
// endpoint; only then does projected nullspace descent shorten maneuvers.
// The weights are metre-equivalent preferences, not actuator certification.
double operationalPenalty(double stationarySeconds,double jointRadians,
                          std::size_t gearChanges,std::size_t reversals,
                          double peakGamma,std::size_t arcs) {
  const double excess=std::max(0.0,peakGamma-0.75);
  return 0.12*stationarySeconds+0.12*jointRadians+
         0.60*double(gearChanges)+0.55*double(reversals)+
         1.4*excess*excess+
         0.12*double(arcs>0?arcs-1:0);
}
struct EstimatedEffort {
  double physicalCost=0.0,operationalCost=0.0,qualityCost=0.0;
};
EstimatedEffort estimateEffort(RearState start,RearState goal,
                               const Vector& x,const Gears& signs,
                               const SearchOptions& o,bool coupled) {
  double gamma=start.gamma,peak=std::abs(gamma),joint=0.0,stationary=0.0;
  double physical=0.0;
  std::size_t gearChanges=0,reversals=0,arcs=0;
  int previousGear=0,previousSteer=0;
  const auto changeSteer=[&](double delta,double stationaryDelta) {
    if(std::abs(delta)<=1e-9)return;
    const int direction=delta>0.0?1:-1;
    if(previousSteer&&direction!=previousSteer)++reversals;
    previousSteer=direction;
    joint+=std::abs(delta);
    stationary+=std::abs(stationaryDelta)/
                o.articulationRateRadiansPerSecond;
    if(std::abs(stationaryDelta)>1e-9)
      physical+=o.stationaryArticulationPenaltyMeters;
  };
  for(int i=0;i<3;++i) {
    const auto phase=phasePlan(gamma,x[i],x[i+3],coupled,o);
    changeSteer(phase.preGamma-gamma,phase.preGamma-gamma);
    changeSteer(x[i]-phase.preGamma,0.0);
    peak=std::max({peak,std::abs(phase.preGamma),std::abs(x[i])});
    gamma=x[i];
    const double distance=x[i+3];
    if(distance<=1e-9)continue;
    ++arcs;
    if(previousGear&&previousGear!=signs[i])++gearChanges;
    previousGear=signs[i];
    physical+=distance*(signs[i]<0?o.reverseCostMultiplier:1.0);
  }
  changeSteer(goal.gamma-gamma,goal.gamma-gamma);
  peak=std::max(peak,std::abs(goal.gamma));
  physical+=joint*o.articulationCostMetersPerRadian;
  const double extra=operationalPenalty(stationary,joint,gearChanges,
                                         reversals,peak,arcs);
  return {physical,extra,physical+extra};
}
struct Eval {
  RearState end;
  Error error{};
  double norm=std::numeric_limits<double>::infinity();
  double obstaclePenalty=0.0;
  double score=std::numeric_limits<double>::infinity();
  double qualityScore=std::numeric_limits<double>::infinity();
};
Eval evaluate(RearState from,RearState goal,const Vector& x,const Gears& signs,
              const Parameters& p,const AnalyticGoalHeuristic& B,
              const SearchOptions& o,bool coupled,
              const GoalConnectorObstacleGuide* guide) {
  const RearState startPose=from;
  double obstacle=0.0;
  std::size_t samples=0;
  const auto sample=[&](RearState r) {
    if(guide) {obstacle+=guide->penalty(r,p);++samples;}
  };
  for(int i=0;i<3;++i) {
    const auto phase=phasePlan(from.gamma,x[i],x[i+3],coupled,o);
    const auto begin=from;
    if(guide)for(int j=1;j<=3;++j)
      sample(steer(begin,begin.gamma+
               (phase.preGamma-begin.gamma)*double(j)/3.0,p,B));
    from=steer(from,phase.preGamma,p,B);
    if(phase.coupledMeters>1e-10) {
      const auto start=from;
      const int steps=guide?std::clamp(
          int(std::ceil(phase.coupledMeters/.5)),3,16):1;
      if(guide)for(int j=1;j<=steps;++j) {
        const double fraction=double(j)/steps;
        sample(coupledDrive(start,
          signs[i]*phase.coupledMeters*fraction,
          start.gamma+(x[i]-start.gamma)*fraction,p,B));
      }
      from=coupledDrive(start,signs[i]*phase.coupledMeters,x[i],p,B);
    }
    if(phase.remainingMeters>1e-10) {
      const auto start=from;
      if(guide) {
        const int steps=std::clamp(int(
            std::ceil(phase.remainingMeters/.5)),3,60);
        for(int j=1;j<=steps;++j)
          sample(drive(start,signs[i]*phase.remainingMeters*
                        double(j)/steps,p));
      }
      from=drive(from,signs[i]*phase.remainingMeters,p);
    }
  }
  if(guide)for(int j=1;j<=3;++j)
    sample(steer(from,from.gamma+(goal.gamma-from.gamma)*
        double(j)/3.0,p,B));
  from=steer(from,goal.gamma,p,B);
  const Error e{from.xr-goal.xr,from.yr-goal.yr,
                2.0*std::remainder(from.thetaRear-goal.thetaRear,2*pi)};
  const double norm=e[0]*e[0]+e[1]*e[1]+e[2]*e[2];
  const double penalty=samples?obstacle/double(samples):0.0;
  const auto effort=estimateEffort(startPose,goal,x,signs,o,coupled);
  return {from,e,norm,penalty,norm+8.0*penalty,
          effort.qualityCost+12.0*penalty};
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
                                     const SearchOptions& o,bool coupled) {
  std::vector<GoalControl> out;
  out.reserve(64);
  const double v=o.translationSpeedMetersPerSecond;
  const double u=o.articulationRateRadiansPerSecond;
  const double chunkMeters=std::clamp(o.translationPrimitiveMeters*2.0,.25,1.0);
  const auto addSteer=[&](double to) {
    const double delta=to-from.gamma;
    if(std::abs(delta)>1e-8) {
      out.push_back({{0.0,std::copysign(u,delta)},std::abs(delta)/u});
      from.gamma=to;
    }
  };
  for(int i=0;i<3;++i) {
    const auto phase=phasePlan(from.gamma,x[i],x[i+3],coupled,o);
    addSteer(phase.preGamma);
    if(phase.coupledMeters>1e-8) {
      const double rate=std::copysign(u,x[i]-phase.preGamma);
      double remaining=phase.coupledMeters;
      while(remaining>1e-6) {
        const double distance=std::min(remaining,chunkMeters);
        out.push_back({{signs[i]*v,rate},distance/v});
        remaining-=distance;
      }
      from.gamma=x[i];
    }
    // Split long arcs to respect both the preliminary and strict sample
    // budgets. Identical adjacent controls remain physically continuous.
    double remaining=phase.remainingMeters;
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

GoalConnectorEffort summarizeGoalConnectorEffort(
    const std::vector<GoalControl>& controls,RearState start,
    const SearchOptions& options) {
  GoalConnectorEffort effort;
  double gamma=start.gamma;
  effort.peakAbsGammaRadians=std::abs(gamma);
  int previousGear=0,previousSteer=0;
  bool wasDriving=false;
  double priorDriveRate=0.0,priorDriveJoint=0.0;
  for(const auto& section:controls) {
    const double v=section.control.frontSpeedMetersPerSecond;
    const double u=section.control.gammaRateRadiansPerSecond;
    const double length=std::abs(v)*section.durationSeconds;
    const double steering=std::abs(u)*section.durationSeconds;
    if(length>1e-9) {
      effort.drivingMeters+=length;
      if(v<0.0)effort.reverseMeters+=length;
      const int gear=v<0.0?-1:1;
      if(previousGear&&gear!=previousGear)++effort.gearChanges;
      previousGear=gear;
      // Chunked identical controls are one continuous arc, NOT several.
      if(!wasDriving||priorDriveRate!=double(gear)||
         std::abs(priorDriveJoint-u)>1e-10)++effort.driveArcs;
      wasDriving=true;
      priorDriveRate=double(gear);
      priorDriveJoint=u;
    } else if(steering>1e-9) {
      effort.stationarySteeringSeconds+=section.durationSeconds;
      wasDriving=false;
    }
    if(steering>1e-9) {
      effort.totalArticulationRadians+=steering;
      const int direction=u<0.0?-1:1;
      if(previousSteer&&direction!=previousSteer)
        ++effort.articulationReversals;
      previousSteer=direction;
    }
    gamma+=u*section.durationSeconds;
    effort.peakAbsGammaRadians=std::max(
        effort.peakAbsGammaRadians,std::abs(gamma));
  }
  effort.operationalPenaltyMeters=operationalPenalty(
      effort.stationarySteeringSeconds,effort.totalArticulationRadians,
      effort.gearChanges,effort.articulationReversals,
      effort.peakAbsGammaRadians,effort.driveArcs);
  effort.qualityScoreMeters=costOf(controls,options)+
      effort.operationalPenaltyMeters;
  return effort;
}

GoalConnectorObstacleGuide::GoalConnectorObstacleGuide(
    const ackermann::MapSnapshot& map,ackermann::OccupancyPolicy occupancy,
    BodyGeometry geometry)
    : width_(map.width),height_(map.height),resolution_(map.resolutionMeters),
      originX_(map.origin.x),originY_(map.origin.y),
      cosYaw_(std::cos(map.origin.yaw)),sinYaw_(std::sin(map.origin.yaw)) {
  const auto radius=[](ackermann::Footprint b,double inflation) {
    return std::hypot(std::max(std::abs(b.frontX),std::abs(b.rearX)),
                      b.halfWidth)+inflation;
  };
  frontRadius_=radius(geometry.front,geometry.inflationMeters);
  rearRadius_=radius(geometry.rear,geometry.inflationMeters);
  clearance_.assign(width_*height_,std::numeric_limits<double>::infinity());
  for(std::size_t i=0;i<clearance_.size();++i) {
    const int cell=map.cells[i];
    const bool blocked=cell<0
        ?occupancy.unknown!=ackermann::UnknownPolicy::Free
        :cell>occupancy.maxTraversableOccupancy;
    if(blocked) {
      clearance_[i]=0.0;
      hasObstacles_=true;
    }
  }
  if(!hasObstacles_)return;
  const auto improve=[&](std::size_t x,std::size_t y,
                         std::size_t nx,std::size_t ny,double step) {
    auto& d=clearance_[y*width_+x];
    d=std::min(d,clearance_[ny*width_+nx]+step);
  };
  // Two-pass 8-neighbor chamfer distance transform, O(map cells).
  // It is only a smooth optimization guide, never safety evidence.
  for(std::size_t y=0;y<height_;++y)for(std::size_t x=0;x<width_;++x) {
    if(x>0)improve(x,y,x-1,y,1.0);
    if(y>0) {
      improve(x,y,x,y-1,1.0);
      if(x>0)improve(x,y,x-1,y-1,std::numbers::sqrt2_v<double>);
      if(x+1<width_)improve(x,y,x+1,y-1,std::numbers::sqrt2_v<double>);
    }
  }
  for(std::size_t y=height_;y-->0;)for(std::size_t x=width_;x-->0;) {
    if(x+1<width_)improve(x,y,x+1,y,1.0);
    if(y+1<height_) {
      improve(x,y,x,y+1,1.0);
      if(x>0)improve(x,y,x-1,y+1,std::numbers::sqrt2_v<double>);
      if(x+1<width_)improve(x,y,x+1,y+1,std::numbers::sqrt2_v<double>);
    }
  }
}

double GoalConnectorObstacleGuide::at(double worldX,double worldY,
                                      double radius) const noexcept {
  const double dx=worldX-originX_,dy=worldY-originY_;
  const double x=cosYaw_*dx+sinYaw_*dy,y=-sinYaw_*dx+cosYaw_*dy;
  const double edge=std::min({x,y,width_*resolution_-x,height_*resolution_-y});
  if(edge<=0.0)return 16.0;
  double occupied=std::numeric_limits<double>::infinity();
  if(hasObstacles_) {
    const double gx=std::clamp(x/resolution_-0.5,0.0,double(width_-1));
    const double gy=std::clamp(y/resolution_-0.5,0.0,double(height_-1));
    const auto x0=std::size_t(gx),y0=std::size_t(gy);
    const auto x1=std::min(x0+1,width_-1),y1=std::min(y0+1,height_-1);
    const double tx=gx-x0,ty=gy-y0;
    const double v0=std::lerp(clearance_[y0*width_+x0],
                              clearance_[y0*width_+x1],tx);
    const double v1=std::lerp(clearance_[y1*width_+x0],
                              clearance_[y1*width_+x1],tx);
    occupied=std::lerp(v0,v1,ty)*resolution_-
             std::numbers::sqrt2_v<double>*resolution_/2.0;
  }
  const double clearance=std::min(edge,occupied);
  const double deficit=std::max(0.0,radius+0.25-clearance);
  return deficit*deficit;
}

double GoalConnectorObstacleGuide::penalty(
    RearState pose,const Parameters& geometry) const noexcept {
  const double fx=pose.xr+geometry.LrMeters*std::cos(pose.thetaRear)+
                  geometry.LfMeters*std::cos(pose.thetaRear+pose.gamma);
  const double fy=pose.yr+geometry.LrMeters*std::sin(pose.thetaRear)+
                  geometry.LfMeters*std::sin(pose.thetaRear+pose.gamma);
  return at(pose.xr,pose.yr,rearRadius_)+at(fx,fy,frontRadius_);
}

std::vector<GoalConnection> proposeGoalConnections(
    const Model& model,RearState from,RearState goal,
    const SearchOptions& options,std::size_t& evaluations,
    const GoalConnectorObstacleGuide* guide) {
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
  const double amp=std::min(.85,p.maxGammaRadians);
  const double minusAmp=std::max(-.85,p.minGammaRadians);
  const double approachSign=aimed<0.0?-1.0:1.0;
  const std::array<std::array<double,3>,6> seeds{{
      {from.gamma,from.gamma,from.gamma},
      {approachSign*.12,0.0,-approachSign*.12},
      {aimed,aimed,aimed},
      {steerSign*amp,-steerSign*amp,steerSign*amp},
      {-steerSign*amp,steerSign*amp,-steerSign*amp},
      {minusAmp,0.0,amp}
  }};
  const bool useGuide=guide&&guide->hasObstacles();
  // Constant-gamma families first (exact circular arcs). Follow with
  // coupled articulation-rate families, using analytic yaw and quadrature
  // for the front reference, not a rigid-car instantaneous steering step.
  for(const Gears& signs:patterns) {
    for(const bool coupled:{false,true}) {
      for(const auto& seed:seeds) {
        if(evaluations>=budget)break;
        Vector x{seed[0],seed[1],seed[2],
                 distance/3.0+.2,distance/3.0+.2,distance/3.0+.2};
        if(std::abs(turn)<options.goalYawToleranceRadians &&
           std::abs(from.gamma-goal.gamma)<options.goalGammaToleranceRadians &&
           std::abs(seed[0]-from.gamma)<1e-8) {
          x[3]=std::abs(ahead);x[4]=0.0;x[5]=0.0;
        }
        clampVector(x,p,maxArc);
        Eval best=evaluate(from,goal,x,signs,p,B,options,coupled,
                           useGuide?guide:nullptr);
        ++evaluations;
        double lambda=.03;
        for(int iteration=0;iteration<24;++iteration) {
          if(nearTarget(best,options)&&
             (!useGuide||best.obstaclePenalty<.002))break;
          if(evaluations+7>=budget)break;
          std::array<std::array<double,6>,3> J{};
          Vector obstacleGrad{};
          for(int j=0;j<6;++j) {
            auto perturbed=x;
            const double step=j<3?1e-4:1e-3;
            perturbed[j]+=step;
            clampVector(perturbed,p,maxArc);
            if(std::abs(perturbed[j]-x[j])<1e-10) {
              perturbed=x;perturbed[j]-=step;
              clampVector(perturbed,p,maxArc);
            }
            const double d=perturbed[j]-x[j];
            if(std::abs(d)<1e-10)continue;
            const Eval next=evaluate(from,goal,perturbed,signs,p,B,options,
                                     coupled,useGuide?guide:nullptr);
            ++evaluations;
            for(int k=0;k<3;++k)
              J[k][j]=k==2
                  ?2.0*std::remainder((next.error[k]-best.error[k])/2.0,2*pi)/d
                  :(next.error[k]-best.error[k])/d;
            if(useGuide)
              obstacleGrad[j]=(next.obstaclePenalty-best.obstaclePenalty)/d;
          }
          std::array<std::array<double,3>,3> A{};
          for(int row=0;row<3;++row)for(int col=0;col<3;++col) {
            for(int j=0;j<6;++j)A[row][col]+=J[row][j]*J[col][j];
            if(row==col)A[row][col]+=lambda;
          }
          Error direction{};
          if(!solve3(A,best.error,direction))break;
          Vector delta{};
          for(int j=0;j<6;++j)
            for(int k=0;k<3;++k)delta[j]-=J[k][j]*direction[k];
          if(useGuide&&best.obstaclePenalty>.001) {
            // Move in a damped approximate null space of endpoint Jacobian:
            // lower obstacle penalty without unnecessarily losing the
            // terminal four-state fit. Collision is STILL checked exactly.
            Error projected{};
            for(int k=0;k<3;++k)for(int j=0;j<6;++j)
              projected[k]+=J[k][j]*obstacleGrad[j];
            Error correction{};
            if(solve3(A,projected,correction)) {
              for(int j=0;j<6;++j) {
                double nullGrad=obstacleGrad[j];
                for(int k=0;k<3;++k)
                  nullGrad-=J[k][j]*correction[k];
                delta[j]-=1.4*nullGrad;
              }
            }
          }
          for(int j=0;j<6;++j)
            delta[j]=std::clamp(delta[j],j<3?-.5:-3.0,j<3?.5:3.0);
          bool improved=false;
          for(double scale:{1.0,.5,.25,.125}) {
            if(evaluations>=budget)break;
            auto trial=x;
            for(int j=0;j<6;++j)trial[j]+=scale*delta[j];
            clampVector(trial,p,maxArc);
            const Eval next=evaluate(from,goal,trial,signs,p,B,options,
                                     coupled,useGuide?guide:nullptr);
            ++evaluations;
            if(next.score+1e-10<best.score) {
              x=trial;best=next;improved=true;break;
            }
          }
          lambda=std::clamp(lambda*(improved?.7:4.0),1e-5,1e7);
          if(lambda>=1e6)break;
        }
        if(!nearTarget(best,options))continue;
        auto controls=makeControls(x,signs,from,goal,options,coupled);
        if(controls.empty())continue;
        const double cost=costOf(controls,options);
        const bool hasCoupled=std::any_of(controls.begin(),controls.end(),
            [](const GoalControl& control) {
              return control.control.frontSpeedMetersPerSecond!=0.0&&
                     control.control.gammaRateRadiansPerSecond!=0.0;
            });
        if(std::isfinite(cost))
          found.push_back({std::move(controls),best.end,cost,
                           best.obstaclePenalty,hasCoupled});
      }
      if(evaluations>=budget)break;
    }
    if(evaluations>=budget)break;
  }
  // Prefer lower-cost, higher-clearance candidates for expensive exact
  // collision checks. Never label this approximation a clearance proof.
  std::stable_sort(found.begin(),found.end(),[](const auto& a,const auto& b) {
    return a.costMeters+12*a.clearancePenalty<
           b.costMeters+12*b.clearancePenalty;
  });
  if(found.size()>options.maxGoalConnectorCandidatesPerAttempt)
    found.resize(options.maxGoalConnectorCandidatesPerAttempt);
  return found;
}
} // namespace hybrid_astar::articulated_steering
