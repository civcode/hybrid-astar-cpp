#include "hybrid_astar/articulated_steering/planner.hpp"
#include <algorithm>
#include <array>
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
};
struct HeapItem {
  double f=0.0;
  std::size_t node=0;
  bool operator<(const HeapItem& other) const noexcept {
    return f==other.f ? node>other.node : f>other.f;
  }
};
struct Best {double g;std::size_t id;};
struct ControlPrimitive {Control u;double duration;int gear;};
std::array<ControlPrimitive,8> catalog(const SearchOptions& o) {
  const double v=o.translationSpeedMetersPerSecond;
  const double u=o.articulationRateRadiansPerSecond;
  const double movingTime=o.translationPrimitiveMeters/v;
  const double jointTime=o.articulationPrimitiveRadians/u;
  return {{{{ v,-u},movingTime,1},{{v,0},movingTime,1},
           {{v,u},movingTime,1},{{-v,-u},movingTime,-1},
           {{-v,0},movingTime,-1},{{-v,u},movingTime,-1},
           {{0,-u},jointTime,0},{{0,u},jointTime,0}}};
}
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
    nodes.push_back({start,r.start,rootKey,noParent,{},0.0,0.0});
    out.diagnostics.generated=1;
    std::unordered_map<SearchKey,Best,KeyHash> best;
    best.reserve(std::min<std::size_t>(r.options.maxGenerated,16384));
    best.emplace(rootKey,Best{0.0,0});
    std::priority_queue<HeapItem> open;
    open.push({heuristic(r.start,r.goal,r.options),0});
    const auto primitives=catalog(r.options);
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
      for(const auto& primitive:primitives) {
        if(cancelled(r)){out.status=Status::Cancelled;return out;}
        try {
          const double dt=safeSampleStep(model,*body,primitive.u,
                         primitive.duration,r.options.maxRk4StepSeconds,
                         r.options.searchMaxBodyPointMotionMeters);
          auto samples=model.trajectory(current.state,primitive.u,
                             primitive.duration,dt,r.options.maxSamplesPerPrimitive-1);
          bool valid=true;
          for(const auto& sample:samples) {
            out.diagnostics.searchBodyChecks+=2;
            if(!body->clear(model,sample.state)) {
              valid=false; ++out.diagnostics.rejectedSearchCollision;break;
            }
          }
          if(!valid)continue;
          const auto state=samples.back().state;
          const auto rear=samples.back().rear;
          const auto key=makeSearchKey(rear,primitive.gear,r.options,r.parameters);
          const double newG=current.g+transitionCost(primitive,r.options);
          if(!std::isfinite(newG))continue;
          const auto prev=best.find(key);
          if(prev!=best.end()&&newG>=prev->second.g-1e-12) {
            ++out.diagnostics.pruned;continue;
          }
          if(nodes.size()>=r.options.maxGenerated) {
            out.status=Status::BudgetExceeded;return out;
          }
          const auto id=nodes.size();
          nodes.push_back({state,rear,key,top.node,primitive.u,primitive.duration,newG});
          best[key]={newG,id};
          ++out.diagnostics.generated;
          open.push({newG+heuristic(rear,r.goal,r.options),id});
        } catch(const std::length_error&) {
          sampledIncomplete=true;
          ++out.diagnostics.rejectedInvalidMotion;
        } catch(const std::invalid_argument&) {
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
  }
}
} // namespace hybrid_astar::articulated_steering
