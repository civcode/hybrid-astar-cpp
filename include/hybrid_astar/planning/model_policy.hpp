#pragma once

#include "hybrid_astar/ackermann/planner.hpp"
#include "hybrid_astar/ackermann/primitives.hpp"
#include "hybrid_astar/ackermann/hotpath_profiler.hpp"
#include <cmath>
#include <concepts>
#include <cstdint>
#include <functional>
#include <algorithm>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

namespace hybrid_astar::planning {

// A model is responsible for its local primitive catalog, state discretization,
// successor materialization, incoming drive mode and nonnegative edge cost.
// The existing native search works on SE(2) rigid bodies and rectangle
// footprints. State dimensions and multibody geometry are separate future
// generalizations; no Ackermann-specific steering belongs in the search loop.
struct PlanarKeyHash {
  ackermann::PlanDiagnostics* diagnostics = nullptr;
  std::size_t operator()(const ackermann::SearchKey& key) const noexcept {
#if defined(HYBRID_ASTAR_PROFILE_HOTPATHS)
    if(diagnostics) {
      ++diagnostics->hashCalls;
      ackermann::HotPathTimer timer(diagnostics->hashNanoseconds);
      return hash(key);
    }
#endif
    return hash(key);
  }
private:
  static std::size_t hash(const ackermann::SearchKey& k) noexcept {
    std::size_t h=std::hash<std::int64_t>{}(k.xBin);
    const auto mix=[&](std::size_t v) {
      h^=v+0x9e3779b97f4a7c15ULL+(h<<6)+(h>>2);
    };
    mix(std::hash<std::int64_t>{}(k.yBin));
    mix(k.yawBin);mix(static_cast<unsigned>(k.incomingGear));
    return h;
  }
};

template <class Policy>
concept PlanarSearchModel = requires(
    const Policy& policy,
    const typename Policy::State& state,
    const typename Policy::Mode& previous,
    const typename Policy::Primitive& primitive,
    const typename Policy::Section& section,
    const motion::SampleLimits& limits) {
  typename Policy::Key;
  typename Policy::Hash;
  { policy.keyHash(nullptr) } -> std::same_as<typename Policy::Hash>;
  { std::declval<typename Policy::Hash>()(std::declval<typename Policy::Key>()) }
    -> std::convertible_to<std::size_t>;
  requires std::equality_comparable<typename Policy::Key>;
  { policy.validate() } -> std::same_as<void>;
  { policy.footprint() } -> std::same_as<ackermann::Footprint>;
  { policy.catalog(limits) } -> std::same_as<std::vector<typename Policy::Primitive>>;
  { policy.key(state,previous) } -> std::same_as<typename Policy::Key>;
  { policy.materialize(state,primitive) } -> std::same_as<typename Policy::Section>;
  { policy.nextMode(previous,primitive) } -> std::same_as<typename Policy::Mode>;
  { policy.edgeCost(previous,primitive,section) } -> std::convertible_to<double>;
};

template<class StateT,class KeyT,class ModeT,class SectionT>
struct SearchNode {
  StateT pose;
  KeyT key;
  ModeT gear;
  double g=0,h=0,clearanceCost=0;
  std::size_t parent=0;
  SectionT via{};
};

struct PlanarPrimitive {
  ackermann::Gear gear=ackermann::Gear::None;
  double curvature=0;
  motion::LocalPrimitive local;
};

// Optional public extension point for another rigid planar chassis.
// A custom model can implement PlanarPolicy and call planWithPolicy()
// without cloning the Hybrid A* search. It must still return evaluable
// planar MotionSections, a complete rectangle footprint and finite costs.
class PlanarPolicy {
public:
  using State=Pose2D;
  using Key=ackermann::SearchKey;
  using Hash=PlanarKeyHash;
  using Mode=ackermann::Gear;
  using Section=ackermann::MotionSection;
  using Primitive=PlanarPrimitive;
  virtual ~PlanarPolicy()=default;
  virtual void validate() const=0;
  virtual ackermann::Footprint footprint() const=0;
  virtual std::vector<Primitive> catalog(const motion::SampleLimits&) const=0;
  virtual Key key(State,Mode) const=0;
  virtual Section materialize(State,const Primitive&) const=0;
  virtual Mode nextMode(Mode,const Primitive&) const=0;
  virtual double edgeCost(Mode,const Primitive&,const Section&) const=0;
  // Optional holonomic capabilities. Defaults preserve W11 custom policies.
  virtual bool strictSearchSampling() const noexcept {return false;}
  virtual double additionalGoalLowerBound(State,State,double) const noexcept {return 0;}
  virtual std::vector<std::vector<Section>> terminalConnections(State,State) const {return {};}
  virtual double terminalEdgeCost(const Section&) const noexcept {
    return std::numeric_limits<double>::quiet_NaN();
  }
  Hash keyHash(ackermann::PlanDiagnostics* diagnostics) const noexcept {
    return {diagnostics};
  }
};

template<PlanarSearchModel Policy>
class PlanarPolicyAdapter final : public PlanarPolicy {
public:
  explicit PlanarPolicyAdapter(Policy policy):policy_(std::move(policy)) {
    static_assert(std::same_as<typename Policy::State,State> &&
                  std::same_as<typename Policy::Key,Key> &&
                  std::same_as<typename Policy::Hash,Hash> &&
                  std::same_as<typename Policy::Mode,Mode> &&
                  std::same_as<typename Policy::Section,Section> &&
                  std::same_as<typename Policy::Primitive,Primitive>,
                  "The current runtime adapter accepts rigid SE2 search policies");
  }
  void validate() const override {policy_.validate();}
  ackermann::Footprint footprint() const override {return policy_.footprint();}
  std::vector<Primitive> catalog(const motion::SampleLimits& s) const override {
    return policy_.catalog(s);
  }
  Key key(State s,Mode m) const override {return policy_.key(s,m);}
  Section materialize(State s,const Primitive& p) const override {
    return policy_.materialize(s,p);
  }
  Mode nextMode(Mode m,const Primitive& p) const override {
    return policy_.nextMode(m,p);
  }
  double edgeCost(Mode m,const Primitive& p,const Section& s) const override {
    return policy_.edgeCost(m,p,s);
  }
  bool strictSearchSampling() const noexcept override {
    if constexpr(requires {policy_.strictSearchSampling();})
      return policy_.strictSearchSampling();
    return false;
  }
  double additionalGoalLowerBound(State from,State goal,double yawTolerance) const noexcept override {
    if constexpr(requires {policy_.additionalGoalLowerBound(from,goal,yawTolerance);})
      return policy_.additionalGoalLowerBound(from,goal,yawTolerance);
    return 0;
  }
  std::vector<std::vector<Section>> terminalConnections(State from,State goal) const override {
    if constexpr(requires {policy_.terminalConnections(from,goal);})
      return policy_.terminalConnections(from,goal);
    return {};
  }
  double terminalEdgeCost(const Section& section) const noexcept override {
    if constexpr(requires {policy_.terminalEdgeCost(section);})
      return policy_.terminalEdgeCost(section);
    return std::numeric_limits<double>::quiet_NaN();
  }
private:
  Policy policy_;
};

class AckermannPolicy {
public:
  using State=Pose2D;
  using Key=ackermann::SearchKey;
  using Hash=PlanarKeyHash;
  Hash keyHash(ackermann::PlanDiagnostics* diagnostics) const noexcept {
    return {diagnostics};
  }
  using Mode=ackermann::Gear;
  using Section=ackermann::MotionSection;
  using Primitive=PlanarPrimitive;
  explicit AckermannPolicy(const ackermann::PlanRequest& r):request_(r) {}
  void validate() const noexcept {}
  ackermann::Footprint footprint() const {return request_.footprint;}
  std::vector<Primitive> catalog(const motion::SampleLimits& limits) const {
    const auto built=ackermann::buildPrimitiveCatalog(
        request_.options.search,request_.options.minimumTurningRadiusMeters,limits);
    std::vector<Primitive> out;
    out.reserve(built.size());
    for(const auto& p:built)out.push_back({p.gear,p.curvature,p.local});
    return out;
  }
  Key key(State pose,Mode incoming) const {
    return ackermann::quantize(pose,incoming,request_.options.search);
  }
  Section materialize(State pose,const Primitive& p) const {
    if(!p.local.samples.empty())return {pose,p.local.endpoint(pose),p.gear,
                                        p.curvature,p.local.extent};
    return ackermann::makeSection(pose,p.gear,p.curvature,p.local.extent);
  }
  Mode nextMode(Mode, const Primitive& p) const noexcept {return p.gear;}
  double edgeCost(Mode incoming,const Primitive& p,const Section& section) const {
    return ackermann::edgeCost(section.lengthMeters,p.gear,incoming,
                                request_.options.search);
  }
private:
  const ackermann::PlanRequest& request_;
};

class DifferentialDrivePolicy {
public:
  using State=Pose2D;
  using Key=ackermann::SearchKey;
  using Hash=PlanarKeyHash;
  Hash keyHash(ackermann::PlanDiagnostics* diagnostics) const noexcept {
    return {diagnostics};
  }
  using Mode=ackermann::Gear;
  using Section=ackermann::MotionSection;
  using Primitive=PlanarPrimitive;
  explicit DifferentialDrivePolicy(const ackermann::PlanRequest& r):request_(r) {}

  void validate() const {
    const auto& options=request_.options;
    if(!std::isfinite(options.rotationCostMetersPerRadian) ||
       options.rotationCostMetersPerRadian<=0 ||
       request_.analyticConnector || request_.analyticDistance ||
       options.clearanceWeight>0)
      throw std::invalid_argument("Unsupported differential-drive planning option");
    const differential_drive::Model model(options.diffParameters);
    auto config=options.diffPrimitives;
    const auto& c=config;
    if(!std::isfinite(c.translationMeters)||c.translationMeters<=0 ||
       !std::isfinite(c.arcTurnRadians)||c.arcTurnRadians<=0 ||
       !std::isfinite(c.spinRadians)||c.spinRadians<=0 ||
       !std::isfinite(c.translationSpeed)||c.translationSpeed<=0 ||
       !std::isfinite(c.rotationSpeed)||c.rotationSpeed<=0)
      throw std::invalid_argument("Invalid differential-drive control catalog");
    const double travelTime=c.translationMeters/c.translationSpeed;
    if(!std::isfinite(travelTime)||travelTime<=0||
       !model.valid({c.translationSpeed,c.arcTurnRadians/travelTime})||
       !model.valid({0,c.rotationSpeed}))
      throw std::invalid_argument("Differential-drive wheel speed bounds exceeded");
    // Source controls validated without generating a redundant catalog.
    (void)motion::sampleIntervals(c.translationMeters,c.arcTurnRadians,
                                 effectiveSamples());
    (void)motion::sampleIntervals(0,c.spinRadians,effectiveSamples());
  }
  ackermann::Footprint footprint() const {return request_.footprint;}
  std::vector<Primitive> catalog(const motion::SampleLimits& limits) const {
    auto config=request_.options.diffPrimitives;
    config.sampling=limits;
    const differential_drive::Model model(request_.options.diffParameters);
    const auto built=differential_drive::buildCatalog(model,config);
    std::vector<Primitive> out;
    out.reserve(built.size());
    for(const auto& p:built) {
      const auto& local=p.local;
      const bool rotate=local.extentKind==motion::ExtentKind::RotationRadians;
      const auto gear=rotate?Mode::None:
         (local.signedTranslationMeters>0?Mode::Forward:Mode::Reverse);
      const double curvature=rotate?0:
         local.headingChangeRadians/local.signedTranslationMeters;
      out.push_back({gear,curvature,local});
    }
    return out;
  }
  Key key(State pose,Mode incoming) const {
    return ackermann::quantize(pose,incoming,request_.options.search);
  }
  Section materialize(State pose,const Primitive& p) const {
    if(p.local.extentKind==motion::ExtentKind::RotationRadians)
      return ackermann::makeRotationSection(pose,p.local.headingChangeRadians);
    if(!p.local.samples.empty())return {pose,p.local.endpoint(pose),p.gear,
                                        p.curvature,p.local.extent};
    return ackermann::makeSection(pose,p.gear,p.curvature,p.local.extent);
  }
  Mode nextMode(Mode incoming,const Primitive& p) const noexcept {
    return p.local.extentKind==motion::ExtentKind::RotationRadians
        ? incoming:p.gear;
  }
  double edgeCost(Mode incoming,const Primitive& p,const Section& section) const {
    if(section.kind==ackermann::MotionKind::RotateInPlace)
      return request_.options.rotationCostMetersPerRadian*
             std::abs(section.turnRadians);
    return ackermann::edgeCost(section.lengthMeters,p.gear,incoming,
                                request_.options.search);
  }
  // Fixed 22.5-degree turn primitives need not land inside an arbitrary
  // goal-yaw tolerance. Near the goal's XY region, try an exact bounded-rate
  // stationary turn. The shared planner independently checks the entire
  // prefix and the rotation's swept footprint before accepting a candidate.
  std::vector<std::vector<Section>> terminalConnections(State from,State goal) const {
    const auto& options=request_.options;
    if(std::hypot(from.x-goal.x,from.y-goal.y)>
       options.goalXYToleranceMeters+1e-9)
      return {};
    const double yawDelta=motion::wrapAngle(goal.yaw-from.yaw);
    if(!std::isfinite(yawDelta)||
       std::abs(yawDelta)<=options.goalYawToleranceRadians+1e-9)
      return {};
    return {{ackermann::makeRotationSection(from,yawDelta)}};
  }
  double terminalEdgeCost(const Section& section) const noexcept {
    if(section.kind!=ackermann::MotionKind::RotateInPlace)
      return std::numeric_limits<double>::quiet_NaN();
    return request_.options.rotationCostMetersPerRadian*
           std::abs(section.turnRadians);
  }
private:
  motion::SampleLimits effectiveSamples() const {
    const auto& r=request_;
    const auto& f=r.footprint;
    return {r.options.searchSampleSpacingMeters,r.options.searchMaxBodyPointMotionMeters,
            std::hypot(std::max(std::abs(f.rearX),std::abs(f.frontX)),f.halfWidth),
            std::numbers::pi_v<double>,r.options.maxSamplesPerSection};
  }
  const ackermann::PlanRequest& request_;
};


class MecanumPolicy {
public:
  using State=Pose2D;
  using Key=ackermann::SearchKey;
  using Hash=PlanarKeyHash;
  using Mode=ackermann::Gear;
  using Section=ackermann::MotionSection;
  using Primitive=PlanarPrimitive;
  explicit MecanumPolicy(const ackermann::PlanRequest& request):request_(request) {}
  Hash keyHash(ackermann::PlanDiagnostics* diagnostics) const noexcept {return {diagnostics};}
  bool strictSearchSampling() const noexcept {return true;}
  double additionalGoalLowerBound(State from,State goal,double yawTolerance) const noexcept {
    const double error=std::abs(motion::wrapAngle(goal.yaw-from.yaw));
    return request_.options.rotationCostMetersPerRadian*std::max(0.0,error-yawTolerance);
  }
  // A nearby arbitrary SE(2) goal can be reached by bounded-rate pure strafe
  // and stationary turn, in either order. These are ONLY candidates: the
  // search kernel must strictly revalidate the entire path independently.
  std::vector<std::vector<Section>> terminalConnections(State from,State goal) const {
    const double distance=std::hypot(goal.x-from.x,goal.y-from.y);
    if(!std::isfinite(distance)||distance>std::max(1.0,request_.options.goalXYToleranceMeters))
      return {};
    const double yawDelta=motion::wrapAngle(goal.yaw-from.yaw);
    const mecanum::Model model(request_.options.mecanumParameters);
    std::vector<std::vector<Section>> candidates;
    candidates.reserve(2);
    const auto candidate=[&](bool rotateFirst) {
      std::vector<Section> path;
      State pose=from;
      const auto spin=[&]() {
        if(std::abs(yawDelta)<=1e-12)return;
        const auto section=ackermann::makeRotationSection(pose,yawDelta);
        pose=section.end;path.push_back(section);
      };
      const auto translate=[&]() {
        if(distance<=1e-12)return;
        const double dx=goal.x-pose.x,dy=goal.y-pose.y;
        const double cs=std::cos(pose.yaw),sn=std::sin(pose.yaw);
        const double f=cs*dx+sn*dy,l=-sn*dx+cs*dy;
        const double norm=std::hypot(f,l);
        if(norm<=1e-12)return;
        // Wheel-rate feasibility of an arbitrary diagonal direction is
        // achieved by lowering speed; geometric cost remains unchanged.
        const auto& params=model.parameters();
        const double cap=.99*params.wheelRadiusMeters*
            params.maxAbsWheelAngularRate/(std::abs(f/norm)+std::abs(l/norm));
        const double speed=std::min(request_.options.mecanumPrimitives.translationSpeed,cap);
        if(!std::isfinite(speed)||speed<=0||
           !model.valid({speed*f/norm,speed*l/norm,0}))
          throw std::invalid_argument("Infeasible terminal mecanum translation");
        const auto section=ackermann::makeHolonomicSection(pose,f,l,0);
        pose=section.end;path.push_back(section);
      };
      if(rotateFirst){spin();translate();}else{translate();spin();}
      return path;
    };
    try {
      auto first=candidate(false);
      if(!first.empty())candidates.push_back(std::move(first));
      if(std::abs(yawDelta)>1e-12&&distance>1e-12) {
        auto second=candidate(true);
        if(!second.empty())candidates.push_back(std::move(second));
      }
    }catch(const std::exception&) {return {};}
    return candidates;
  }
  double terminalEdgeCost(const Section& s) const noexcept {
    return s.lengthMeters+
        request_.options.rotationCostMetersPerRadian*std::abs(s.turnRadians)+
        (s.kind==ackermann::MotionKind::RotateInPlace?
         request_.options.mecanumStationarySpinPenaltyMeters:0.0);
  }
  void validate() const {
    const auto& o=request_.options;
    if(!std::isfinite(o.rotationCostMetersPerRadian)||
       o.rotationCostMetersPerRadian<=0||
       !std::isfinite(o.mecanumStationarySpinPenaltyMeters)||
       o.mecanumStationarySpinPenaltyMeters<0||
       request_.analyticConnector||request_.analyticDistance||o.clearanceWeight>0)
      throw std::invalid_argument("Unsupported mecanum planning option");
    const mecanum::Model model(o.mecanumParameters);
    auto config=o.mecanumPrimitives;
    const auto& f=request_.footprint;
    config.sampling={o.searchSampleSpacingMeters,o.searchMaxBodyPointMotionMeters,
        std::hypot(std::max(std::abs(f.rearX),std::abs(f.frontX)),f.halfWidth),
        std::numeric_limits<double>::max(),o.maxSamplesPerSection};
    const auto& c=config;
    if(!std::isfinite(c.translationMeters)||c.translationMeters<=0||
       !std::isfinite(c.translationSpeed)||c.translationSpeed<=0||
       !std::isfinite(c.arcTurnRadians)||c.arcTurnRadians<=0||
       !std::isfinite(c.spinRadians)||c.spinRadians<=0||
       !std::isfinite(c.rotationSpeed)||c.rotationSpeed<=0)
      throw std::invalid_argument("Invalid mecanum primitive options");
    const double duration=c.translationMeters/c.translationSpeed;
    const double spinTime=c.spinRadians/c.rotationSpeed;
    if(!std::isfinite(duration)||duration<=0||
       !std::isfinite(spinTime)||spinTime<=0)
      throw std::invalid_argument("Invalid mecanum primitive duration");
    if(!model.valid({c.translationSpeed,0,c.arcTurnRadians/duration})||
       !model.valid({0,c.translationSpeed,c.arcTurnRadians/duration})||
       !model.valid({c.translationSpeed/std::sqrt(2.0),
                     c.translationSpeed/std::sqrt(2.0),0})||
       !model.valid({0,0,c.rotationSpeed}))
      throw std::invalid_argument("Mecanum wheel speed bounds exceeded");
    (void)motion::sampleIntervals(c.translationMeters,c.arcTurnRadians,
                                  config.sampling);
    (void)motion::sampleIntervals(0,c.spinRadians,config.sampling);
  }
  ackermann::Footprint footprint() const {return request_.footprint;}
  std::vector<Primitive> catalog(const motion::SampleLimits& limits) const {
    auto config=request_.options.mecanumPrimitives;
    config.sampling=limits;
    const mecanum::Model model(request_.options.mecanumParameters);
    const auto built=mecanum::buildCatalog(model,config);
    std::vector<Primitive> out;
    out.reserve(built.size());
    for(const auto& p:built)out.push_back({Mode::None,0.0,p.local});
    return out;
  }
  Key key(State pose,Mode) const {
    return ackermann::quantize(pose,Mode::None,request_.options.search);
  }
  Section materialize(State start,const Primitive& primitive) const {
    const auto& p=primitive.local;
    if(p.extentKind==motion::ExtentKind::RotationRadians)
      return ackermann::makeRotationSection(start,p.headingChangeRadians);
    return ackermann::makeHolonomicSection(
        start,p.signedTranslationMeters,p.lateralTranslationMeters,p.headingChangeRadians);
  }
  Mode nextMode(Mode,const Primitive&) const noexcept {return Mode::None;}
  double edgeCost(Mode,const Primitive&,const Section& s) const {
    return s.lengthMeters+
        request_.options.rotationCostMetersPerRadian*std::abs(s.turnRadians)+
        (s.kind==ackermann::MotionKind::RotateInPlace?
         request_.options.mecanumStationarySpinPenaltyMeters:0.0);
  }
private:
  const ackermann::PlanRequest& request_;
};

static_assert(PlanarSearchModel<PlanarPolicy>);
static_assert(PlanarSearchModel<AckermannPolicy>);
static_assert(PlanarSearchModel<DifferentialDrivePolicy>);
static_assert(PlanarSearchModel<MecanumPolicy>);
} // namespace hybrid_astar::planning
