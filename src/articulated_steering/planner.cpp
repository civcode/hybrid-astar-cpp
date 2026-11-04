#include "hybrid_astar/articulated_steering/planner.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace hybrid_astar::articulated_steering {
namespace {
constexpr double pi=std::numbers::pi_v<double>;
constexpr std::size_t noParent=std::numeric_limits<std::size_t>::max();
bool positive(double x) {return std::isfinite(x)&&x>0.0;}
bool nonnegative(double x) {return std::isfinite(x)&&x>=0.0;}
bool finiteRear(RearState s) {
  return std::isfinite(s.xr)&&std::isfinite(s.yr)&&
         std::isfinite(s.thetaRear)&&std::isfinite(s.gamma);
}
bool validBody(ackermann::Footprint f) {
  return std::isfinite(f.rearX)&&std::isfinite(f.frontX)&&
         std::isfinite(f.halfWidth)&&f.frontX>f.rearX&&f.halfWidth>0.0;
}
bool cancelled(const Request& r) {
  return r.cancelled && r.cancelled->load(std::memory_order_relaxed);
}
double yawError(double a,double b) {return std::abs(std::remainder(a-b,2*pi));}
std::int64_t bin(double value,double width) {
  const double b=std::floor(value/width);
  // Avoid undefined float-to-integer conversion on unbounded inputs.
  if(!std::isfinite(b)||std::abs(b)>9e15)
    throw std::invalid_argument("Articulated search discretization overflow");
  return static_cast<std::int64_t>(b);
}
bool validOptions(const SearchOptions& o) {
  return positive(o.translationSpeedMetersPerSecond)&&
         positive(o.articulationRateRadiansPerSecond)&&
         positive(o.translationPrimitiveMeters)&&
         positive(o.articulationPrimitiveRadians)&&
         positive(o.xyBinMeters)&&positive(o.yawBinRadians)&&
         positive(o.gammaBinRadians)&&positive(o.goalXYToleranceMeters)&&
         positive(o.goalYawToleranceRadians)&&positive(o.goalGammaToleranceRadians)&&
         positive(o.maxRk4StepSeconds)&&
         positive(o.searchMaxBodyPointMotionMeters)&&
         positive(o.finalMaxBodyPointMotionMeters)&&
         positive(o.articulationCostMetersPerRadian)&&
         positive(o.reverseCostMultiplier)&&
         nonnegative(o.heuristicWeight)&&o.heuristicWeight<=3.0&&
         o.maxExpansions>0&&o.maxGenerated>1&&
         o.maxSamplesPerPrimitive>=2&&o.maxFinalSamplesPerPrimitive>=2&&
         o.maxTotalFinalBodyChecks>=2;
}
struct KeyHash {
  std::size_t operator()(const SearchKey& k) const noexcept {
    std::size_t hash=std::hash<std::int64_t>{}(k.x);
    const auto mix=[&](std::size_t h){hash^=h+0x9e3779b97f4a7c15ULL+
                                               (hash<<6)+(hash>>2);};
    mix(std::hash<std::int64_t>{}(k.y));
    mix(std::hash<std::int64_t>{}(k.yaw));
    mix(std::hash<std::int64_t>{}(k.gamma));
    mix(std::hash<int>{}(k.incomingGear));
    return hash;
  }
};
struct Node {
  State state;
  RearState rear;
  SearchKey key;
  std::size_t parent=noParent;
  Control incoming{};
  double duration=0.0,g=0.0;
  std::size_t gammaIndex=0; // exact request-local articulation lattice value
};
struct HeapItem {
  double f=0.0;
  std::size_t node=0;
  bool operator<(const HeapItem& other) const noexcept {
    return f==other.f ? node>other.node : f>other.f;
  }
};
struct Best {double g;std::size_t id;};
// Drive uses a fixed articulation. Changing articulation is a separate
// stationary control edge, never an instantaneous node-state jump.
struct ControlPrimitive {Control u;double duration;int gear;};
double transitionCost(const ControlPrimitive& primitive,
                      const SearchOptions& o) {
  const double length=std::abs(primitive.u.frontSpeedMetersPerSecond)*
                      primitive.duration;
  const double joint=std::abs(primitive.u.gammaRateRadiansPerSecond)*
                     primitive.duration;
  const double cost=length*(primitive.gear<0?o.reverseCostMultiplier:1.0)+
                    joint*o.articulationCostMetersPerRadian;
  return cost;
}
double heuristic(const RearState& state,const RearState& goal,
                 const SearchOptions& o) {
  // Performance ordering only; rear translation may be induced by joint
  // motion and the XY heuristic is NOT claimed admissible.
  return o.heuristicWeight*std::hypot(state.xr-goal.xr,state.yr-goal.yr);
}
bool atGoal(const RearState& a,const RearState& b,const SearchOptions& o) {
  return std::hypot(a.xr-b.xr,a.yr-b.yr)<=o.goalXYToleranceMeters&&
         yawError(a.thetaRear,b.thetaRear)<=o.goalYawToleranceRadians&&
         std::abs(a.gamma-b.gamma)<=o.goalGammaToleranceRadians;
}
void goalErrors(Result& out,const RearState& a,const RearState& b) {
  out.reached=a;
  out.goalXYErrorMeters=std::hypot(a.xr-b.xr,a.yr-b.yr);
  out.goalYawErrorRadians=yawError(a.thetaRear,b.thetaRear);
  out.goalGammaErrorRadians=std::abs(a.gamma-b.gamma);
}
double safeSampleStep(const Model& m,const BodyCollisionChecker& collision,
                      Control c,double duration,double maxStep,double maxMotion) {
  const auto& p=m.parameters();
  const double v=std::abs(c.frontSpeedMetersPerSecond);
  const double u=std::abs(c.gammaRateRadiansPerSecond);
  // Upper bound over the ENTIRE permitted articulation interval.
  const double wf=(v+p.LrMeters*u)/p.minDenominatorMeters;
  const double wr=(v+p.LfMeters*u)/p.minDenominatorMeters;
  const double rearSpeed=(v*(p.LfMeters+p.LrMeters)+p.LfMeters*p.LrMeters*u)/
                          p.minDenominatorMeters;
  const double bodyPointSpeed=std::max(
      v+collision.maxFrontBodyRadius()*wf,
      rearSpeed+collision.maxRearBodyRadius()*wr);
  if(!std::isfinite(bodyPointSpeed))
    throw std::length_error("Articulated body motion bound overflow");
  double h=maxStep;
  if(bodyPointSpeed>0.0)h=std::min(h,maxMotion/bodyPointSpeed);
  if(!positive(h)||!std::isfinite(duration)||duration<=0.0)
    throw std::length_error("Invalid articulated sample schedule");
  return h;
}

// Each template starts in the canonical REAR base_link pose (0,0,0,gammaFrom).
// Drive arcs use closed-form integration. Stationary joint transitions are
// integrated ONCE per request using our own RK4, then rigidly transported
// to each expanded node via SE(2). Collision queries remain map-specific
// at EVERY transported intermediate configuration.
struct PrimitiveTemplate {
  ControlPrimitive command;
  std::size_t targetGammaIndex=0;
  std::vector<Sample> localSamples;
};
struct PrimitiveLibrary {
  std::vector<double> angles;
  std::vector<std::vector<PrimitiveTemplate>> atGamma;
  std::size_t initialIndex=0;
  std::size_t analyticDriveCount=0;
  std::size_t stationaryRk4Count=0;
  std::size_t sampleCount=0;

  PrimitiveLibrary(const Model& model,const BodyCollisionChecker& body,
                   const SearchOptions& o,double startGamma,double goalGamma) {
    const auto& p=model.parameters();
    const double width=p.maxGammaRadians-p.minGammaRadians;
    const double partitions=std::ceil(width/o.articulationPrimitiveRadians);
    // Explicit finite lattice budget: a user-supplied tiny steering step
    // must not allocate millions of angle states/templates.
    if(!std::isfinite(partitions)||partitions>63.0)
      throw std::invalid_argument("Articulation lattice exceeds 64 angular intervals");
    const auto count=static_cast<std::size_t>(partitions);
    angles.reserve(count+4);
    for(std::size_t i=0;i<=count;++i) {
      const double g=std::min(p.maxGammaRadians,
                            p.minGammaRadians+static_cast<double>(i)*
                            o.articulationPrimitiveRadians);
      angles.push_back(g);
    }
    angles.push_back(p.maxGammaRadians);
    // Explicit start/goal gamma are physical configurations, not
    // teleported or silently rounded to a nearby steering bin.
    angles.push_back(startGamma);
    angles.push_back(goalGamma);
    std::sort(angles.begin(),angles.end());
    std::vector<double> unique;
    for(double gamma:angles) {
      if(unique.empty()||gamma-unique.back()>1e-9)unique.push_back(gamma);
    }
    angles.swap(unique);
    const auto findIndex=[&](double gamma) {
      for(std::size_t i=0;i<angles.size();++i)
        if(std::abs(angles[i]-gamma)<1e-8)return i;
      throw std::invalid_argument("Articulation start/goal not in lattice");
    };
    initialIndex=findIndex(startGamma);
    (void)findIndex(goalGamma);
    atGamma.resize(angles.size());
    const double v=o.translationSpeedMetersPerSecond;
    const double u=o.articulationRateRadiansPerSecond;
    const auto addTemplate=[&](std::size_t from,std::size_t to,
                               ControlPrimitive command,bool analytic) {
      const double step=safeSampleStep(model,body,command.u,
                                      command.duration,o.maxRk4StepSeconds,
                                      o.searchMaxBodyPointMotionMeters);
      const double countD=std::ceil(command.duration/step);
      if(!std::isfinite(countD)||
         countD>static_cast<double>(o.maxSamplesPerPrimitive-1))
        throw std::length_error("Precomputed articulated motion exceeds sampling budget");
      const auto steps=std::max<std::size_t>(1,
                                      static_cast<std::size_t>(countD));
      PrimitiveTemplate t;
      t.command=command;
      t.targetGammaIndex=to;
      const RearState origin{0,0,0,angles[from]};
      const State initial=model.fromRear(origin);
      if(analytic) {
        const double gamma=angles[from];
        const double D=p.LfMeters*std::cos(gamma)+p.LrMeters;
        const double omega=command.u.frontSpeedMetersPerSecond*
                            std::sin(gamma)/D;
        const double vr=model.rearSpeed(initial,command.u);
        t.localSamples.reserve(steps+1);
        for(std::size_t k=0;k<=steps;++k) {
          const double time=k==steps?command.duration:
                             command.duration*static_cast<double>(k)/
                             static_cast<double>(steps);
          const double a=omega*time;
          // sinc(a), (1-cos(a))/a expansions avoid cancellation
          // when an articulation bin is near straight.
          const double sinc=std::abs(a)<1e-5?
                   1-a*a/6+a*a*a*a/120:std::sin(a)/a;
          const double cosc=std::abs(a)<1e-5?
                   a/2-a*a*a/24+a*a*a*a*a/720:(1-std::cos(a))/a;
          const RearState rear{vr*time*sinc,vr*time*cosc,a,gamma};
          t.localSamples.push_back({time,model.fromRear(rear),rear});
        }
        ++analyticDriveCount;
      } else {
        // Integrate stationary articulation for this angle interval ONCE.
        // The front reference stays fixed, while both carriage
        // headings and the rear base_link move consistently.
        t.localSamples=model.trajectory(initial,command.u,command.duration,
                                         command.duration/static_cast<double>(steps),
                                         steps);
        // Snap ONLY floating-point endpoint gamma to the exact lattice
        // angle; retain the physically integrated rear pose and reconstruct
        // its dependent front heading to preserve the angle invariant.
        auto& endpoint=t.localSamples.back();
        if(std::abs(endpoint.rear.gamma-angles[to])>1e-9)
          throw std::invalid_argument("Precomputed steering misses target angle");
        endpoint.rear.gamma=angles[to];
        endpoint.state=model.fromRear(endpoint.rear);
        ++stationaryRk4Count;
      }
      sampleCount+=t.localSamples.size();
      atGamma[from].push_back(std::move(t));
    };
    for(std::size_t i=0;i<angles.size();++i) {
      const double driveTime=o.translationPrimitiveMeters/v;
      for(int gear:{-1,1}) {
        addTemplate(i,i,{{static_cast<double>(gear)*v,0},driveTime,gear},
                    true);
      }
      for(const int neighbor:{-1,1}) {
        const auto toSigned=static_cast<std::int64_t>(i)+neighbor;
        if(toSigned<0||toSigned>=static_cast<std::int64_t>(angles.size()))
          continue;
        const auto to=static_cast<std::size_t>(toSigned);
        const double delta=angles[to]-angles[i];
        if(std::abs(delta)<1e-10)continue;
        const double rate=std::copysign(u,delta);
        addTemplate(i,to,{{0,rate},std::abs(delta)/u,0},false);
      }
    }
  }
};
RearState transport(const RearState& anchor,const RearState& local) {
  const double c=std::cos(anchor.thetaRear),s=std::sin(anchor.thetaRear);
  return {anchor.xr+c*local.xr-s*local.yr,
          anchor.yr+s*local.xr+c*local.yr,
          anchor.thetaRear+local.thetaRear,
          local.gamma};
}

enum class StrictResult { Accept, Reject, Incomplete, Cancelled };
StrictResult strictValidate(const Request& r,const Model& model,
                            const BodyCollisionChecker& body,
                            const std::vector<Node>& nodes,std::size_t goalId,
                            Result& out) {
  std::vector<std::size_t> chain;
  std::size_t index=goalId;
  while(nodes[index].parent!=noParent) {
    chain.push_back(index);
    index=nodes[index].parent;
  }
  std::reverse(chain.begin(),chain.end());
  std::vector<Motion> motions;
  motions.reserve(chain.size());
  State state=nodes.front().state;
  std::size_t bodyChecks=0;
  for(const auto id:chain) {
    if(cancelled(r))return StrictResult::Cancelled;
    const auto& node=nodes[id];
    try {
      const double dt=safeSampleStep(model,body,node.incoming,node.duration,
               r.options.maxRk4StepSeconds,r.options.finalMaxBodyPointMotionMeters);
      auto samples=model.trajectory(state,node.incoming,node.duration,dt,
                                    r.options.maxFinalSamplesPerPrimitive-1);
      for(const auto& sample:samples) {
        if(cancelled(r))return StrictResult::Cancelled;
        if(bodyChecks>r.options.maxTotalFinalBodyChecks-2)
          return StrictResult::Incomplete;
        bodyChecks+=2;
        out.diagnostics.finalBodyChecks+=2;
        if(!body.clear(model,sample.state))return StrictResult::Reject;
      }
      const auto end=samples.back().state;
      // Numerically independent, denser RK4 must agree with the sampled
      // search successor. Drift outside the tolerance rejects the route.
      if(std::hypot(end.xf-node.state.xf,end.yf-node.state.yf)>1e-5||
         yawError(end.thetaFront,node.state.thetaFront)>1e-5||
         yawError(end.thetaRear,node.state.thetaRear)>1e-5||
         std::abs(end.gamma-node.state.gamma)>1e-6)return StrictResult::Reject;
      motions.push_back({node.incoming,node.duration,state,end,std::move(samples)});
      state=end;
    } catch(const std::length_error&) {
      return StrictResult::Incomplete;
    } catch(const std::invalid_argument&) {
      return StrictResult::Reject;
    } catch(const std::out_of_range&) {
      return StrictResult::Reject;
    }
  }
  const auto rear=model.toRear(state);
  if(!atGoal(rear,r.goal,r.options))return StrictResult::Reject;
  out.motions=std::move(motions);
  goalErrors(out,rear,r.goal);
  out.independentlySampledBodyValidated=true;
  out.costMeters=nodes[goalId].g;
  out.status=Status::Success;
  return StrictResult::Accept;
}
} // namespace

SearchKey makeSearchKey(const RearState& rear,int gear,
                        const SearchOptions& o,const Parameters& p) {
  if(!finiteRear(rear)||!validOptions(o)||
     !std::isfinite(p.minGammaRadians)||!std::isfinite(p.maxGammaRadians)||
     p.minGammaRadians>=p.maxGammaRadians||
     rear.gamma<p.minGammaRadians||rear.gamma>p.maxGammaRadians||
     gear<-1||gear>1)
    throw std::invalid_argument("Invalid articulated search key request");
  // Canonical [0, 2pi) yaw: -pi and +pi must map to the SAME key.
  const double angle=std::fmod(std::remainder(rear.thetaRear,2*pi)+2*pi,2*pi);
  return {bin(rear.xr,o.xyBinMeters),bin(rear.yr,o.xyBinMeters),
          bin(angle,o.yawBinRadians),
          bin(rear.gamma-p.minGammaRadians,o.gammaBinRadians),gear};
}
BodyCollisionChecker::BodyCollisionChecker(const ackermann::MapSnapshot& map,
                  ackermann::OccupancyPolicy occupancy,BodyGeometry g)
  :front_(map,occupancy,g.front,g.inflationMeters,true),
   rear_(map,occupancy,g.rear,g.inflationMeters,true) {}
bool BodyCollisionChecker::clear(const Model& m,const State& s) const {
  if(!m.validState(s))return false;
  // Check BOTH independently, including when the front body is blocked.
  // Each accepted sample therefore performs exactly two body checks.
  const bool frontClear=front_.check(m.frontPose(s))==ackermann::PoseCollision::Clear;
  const bool rearClear=rear_.check(m.rearPose(s))==ackermann::PoseCollision::Clear;
  return frontClear&&rearClear;
}
double BodyCollisionChecker::maxFrontBodyRadius() const noexcept {
  return front_.maxBodyRadius();
}
double BodyCollisionChecker::maxRearBodyRadius() const noexcept {
  return rear_.maxBodyRadius();
}
Result plan(const Request& r) {
  Result out;
  if(r.map)out.mapRevision=r.map->revision;
  if(!r.map||!validOptions(r.options)||!finiteRear(r.start)||
     !finiteRear(r.goal)||!nonnegative(r.bodies.inflationMeters)||
     !validBody(r.bodies.front)||!validBody(r.bodies.rear))
    return out;
  if(cancelled(r)){out.status=Status::Cancelled;return out;}
  try {
    const Model model(r.parameters);
    if(r.options.translationSpeedMetersPerSecond>
         r.parameters.maxAbsFrontSpeedMetersPerSecond||
       r.options.articulationRateRadiansPerSecond>
         r.parameters.maxAbsGammaRateRadiansPerSecond)
      return out;
    const State start=model.fromRear(r.start);
    const State goal=model.fromRear(r.goal);
    // Validate bin ranges before creating an unbounded search.
    (void)makeSearchKey(r.start,0,r.options,r.parameters);
    (void)makeSearchKey(r.goal,0,r.options,r.parameters);
    // Validate the full immutable map and both physical rectangles using
    // the same exact SAT+boundary+inflation policy as existing robot models.
    std::unique_ptr<BodyCollisionChecker> body;
    try {
      body=std::make_unique<BodyCollisionChecker>(*r.map,r.occupancy,r.bodies);
    } catch(const std::domain_error&) {
      out.status=Status::RejectedUnknownMap;return out;
    } catch(const std::invalid_argument&) {
      out.status=Status::InvalidMap;return out;
    }
    out.diagnostics.searchBodyChecks+=2;
    if(!body->clear(model,start)){out.status=Status::StartInCollision;return out;}
    out.diagnostics.searchBodyChecks+=2;
    if(!body->clear(model,goal)){out.status=Status::GoalInCollision;return out;}
    std::vector<Node> nodes;
    nodes.reserve(std::min<std::size_t>(r.options.maxGenerated,16384));
    const auto rootKey=makeSearchKey(r.start,0,r.options,r.parameters);
    nodes.push_back({start,r.start,rootKey,noParent,{},0.0,0.0,
                     primitives.initialIndex});
    out.diagnostics.generated=1;
    std::unordered_map<SearchKey,Best,KeyHash> best;
    best.reserve(std::min<std::size_t>(r.options.maxGenerated,16384));
    best.emplace(rootKey,Best{0.0,0});
    std::priority_queue<HeapItem> open;
    open.push({heuristic(r.start,r.goal,r.options),0});
    PrimitiveLibrary primitives(model,*body,r.options,r.start.gamma,r.goal.gamma);
    out.diagnostics.precomputedAnalyticDriveTemplates=primitives.analyticDriveCount;
    out.diagnostics.precomputedStationaryRk4Templates=primitives.stationaryRk4Count;
    out.diagnostics.precomputedSampleCount=primitives.sampleCount;
    bool sampledIncomplete=false;
    while(!open.empty()) {
      if(cancelled(r)){out.status=Status::Cancelled;return out;}
      const auto top=open.top();open.pop();
      const auto current=nodes[top.node];
      const auto it=best.find(current.key);
      if(it==best.end()||it->second.id!=top.node) {
        ++out.diagnostics.stalePops;continue;
      }
      if(out.diagnostics.expanded>=r.options.maxExpansions) {
        out.status=Status::BudgetExceeded;return out;
      }
      ++out.diagnostics.expanded;
      if(atGoal(current.rear,r.goal,r.options)) {
        ++out.diagnostics.strictGoalCandidates;
        const auto verdict=strictValidate(r,model,*body,nodes,top.node,out);
        if(verdict==StrictResult::Accept)return out;
        if(verdict==StrictResult::Cancelled){out.status=Status::Cancelled;return out;}
        if(verdict==StrictResult::Incomplete) {
          // A feasible-looking candidate must never be reported as a safe
          // path when the independent final validation budget is exhausted.
          out.status=Status::ValidationIncomplete;
          return out;
        }
        ++out.diagnostics.strictRejected;
      }
      for(const auto& templateMotion:primitives.atGamma[current.gammaIndex]) {
        if(cancelled(r)){out.status=Status::Cancelled;return out;}
        ++out.diagnostics.appliedPrecomputedTemplates;
        const auto& command=templateMotion.command;
        try {
          // No RK4 in successor generation. Transport each precomputed
          // canonical template from rear (0,0,0) to current world pose.
          // The complete map-specific, two-body collision validation
          // still runs on EVERY transported sample.
          bool clear=true;
          for(const auto& sample:templateMotion.localSamples) {
            const auto world=transport(current.rear,sample.rear);
            const State q=model.fromRear(world);
            out.diagnostics.searchBodyChecks+=2;
            if(!body->clear(model,q)) {
              clear=false;++out.diagnostics.rejectedSearchCollision;break;
            }
          }
          if(!clear)continue;
          const auto rear=transport(current.rear,
                                   templateMotion.localSamples.back().rear);
          const State state=model.fromRear(rear);
          const auto key=makeSearchKey(rear,command.gear,r.options,r.parameters);
          const double newG=current.g+transitionCost(command,r.options);
          if(!std::isfinite(newG))continue;
          const auto prev=best.find(key);
          if(prev!=best.end()&&newG>=prev->second.g-1e-12) {
            ++out.diagnostics.pruned;continue;
          }
          if(nodes.size()>=r.options.maxGenerated) {
            out.status=Status::BudgetExceeded;return out;
          }
          const auto id=nodes.size();
          nodes.push_back({state,rear,key,top.node,command.u,command.duration,
                           newG,templateMotion.targetGammaIndex});
          best[key]={newG,id};
          ++out.diagnostics.generated;
          open.push({newG+heuristic(rear,r.goal,r.options),id});
        } catch(const std::length_error&) {
          sampledIncomplete=true;
          ++out.diagnostics.rejectedInvalidMotion;
        } catch(const std::invalid_argument&) {
          ++out.diagnostics.rejectedInvalidMotion;
        } catch(const std::out_of_range&) {
          ++out.diagnostics.rejectedInvalidMotion;
        }
      }
    }
    out.status=sampledIncomplete ? Status::ValidationIncomplete :
               out.diagnostics.strictRejected ? Status::StrictValidationFailed :
               Status::NoPath;
    return out;
  } catch(const std::invalid_argument&) {
    out.status=Status::InvalidRequest;
    return out;
  } catch(const std::length_error&) {
    out.status=Status::ValidationIncomplete;
    return out;
  } catch(const std::out_of_range&) {
    out.status=Status::ValidationIncomplete;
    return out;
  }
}
} // namespace hybrid_astar::articulated_steering
