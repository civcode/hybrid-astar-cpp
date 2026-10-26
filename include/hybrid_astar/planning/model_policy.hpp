#pragma once

#include "hybrid_astar/ackermann/planner.hpp"
#include "hybrid_astar/ackermann/primitives.hpp"
#include <cmath>
#include <concepts>
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
template <class Policy>
concept PlanarSearchModel = requires(
    const Policy& policy,
    const typename Policy::State& state,
    const typename Policy::Mode& previous,
    const typename Policy::Primitive& primitive,
    const typename Policy::Section& section,
    const motion::SampleLimits& limits) {
  typename Policy::Key;
  requires std::equality_comparable<typename Policy::Key>;
  { policy.validate() } -> std::same_as<void>;
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

class AckermannPolicy {
public:
  using State=Pose2D;
  using Key=ackermann::SearchKey;
  using Mode=ackermann::Gear;
  using Section=ackermann::MotionSection;
  using Primitive=PlanarPrimitive;
  explicit AckermannPolicy(const ackermann::PlanRequest& r):request_(r) {}
  void validate() const noexcept {}
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

static_assert(PlanarSearchModel<AckermannPolicy>);
static_assert(PlanarSearchModel<DifferentialDrivePolicy>);
} // namespace hybrid_astar::planning
