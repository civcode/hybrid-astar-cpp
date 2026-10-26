#include <catch2/catch_test_macros.hpp>
#include "hybrid_astar/planning/model_policy.hpp"
#include <cmath>
#include <memory>
#include <numbers>
#include <type_traits>
#include <vector>

using namespace hybrid_astar;
namespace a=hybrid_astar::ackermann;
namespace p=hybrid_astar::planning;
namespace m=hybrid_astar::motion;

static a::PlanRequest openRequest() {
  auto map=std::make_shared<a::MapSnapshot>();
  map->width=160;map->height=120;map->resolutionMeters=.25;
  map->cells.assign(map->width*map->height,0);
  map->frameId="policy-test-map";map->sourceId="w11";map->revision=51;
  a::PlanRequest r;r.map=map;
  r.start={10,15,0};r.goal={12,15,0};
  r.options.maxExpansions=8000;
  return r;
}

// Example third planar model, supplied without modifying the native Hybrid A*
// loop: a stationary pivoting platform with two turn controls. This is NOT a
// claim that the current core can yet represent articulated state or holonomy.
struct PivotOnlyPolicy {
  using State=Pose2D;
  using Key=a::SearchKey;
  using Hash=p::PlanarKeyHash;
  using Mode=a::Gear;
  using Section=a::MotionSection;
  using Primitive=p::PlanarPrimitive;
  const a::PlanRequest& request;

  void validate() const {}
  Hash keyHash(a::PlanDiagnostics* diagnostics) const noexcept {return {diagnostics};}
  std::vector<Primitive> catalog(const m::SampleLimits& limits) const {
    std::vector<Primitive> out;
    for(double turn:{.25*std::numbers::pi_v<double>,
                     -.25*std::numbers::pi_v<double>}) {
      auto spin=m::generateConstantTwist(0,turn,m::ExtentKind::RotationRadians,limits);
      out.push_back({a::Gear::None,0,std::move(spin)});
    }
    return out;
  }
  Key key(State pose,Mode) const {
    // This model deliberately collapses incoming drive-gear keys: it cannot
    // translate and only the pose/heading affect the next motion.
    return a::quantize(pose,a::Gear::None,request.options.search);
  }
  Section materialize(State state,const Primitive& primitive) const {
    return a::makeRotationSection(state,primitive.local.headingChangeRadians);
  }
  Mode nextMode(Mode incoming,const Primitive&) const noexcept {return incoming;}
  double edgeCost(Mode,const Primitive&,const Section& section) const {
    return .3*std::abs(section.turnRadians);
  }
};
struct ZeroCostPivotPolicy : PivotOnlyPolicy {
  explicit ZeroCostPivotPolicy(const a::PlanRequest& r):PivotOnlyPolicy{r} {}
  double edgeCost(Mode,const Primitive&,const Section&) const {return 0;}
};
static_assert(p::PlanarSearchModel<PivotOnlyPolicy>);
static_assert(p::PlanarSearchModel<ZeroCostPivotPolicy>);
static_assert(p::PlanarSearchModel<p::AckermannPolicy>);
static_assert(p::PlanarSearchModel<p::DifferentialDrivePolicy>);
static_assert(p::PlanarSearchModel<p::PlanarPolicy>);

static void sameResult(const a::PlanResult& actual,const a::PlanResult& expected) {
  REQUIRE(actual.status==expected.status);
  REQUIRE(actual.collisionEvidence==expected.collisionEvidence);
  REQUIRE(actual.mapRevision==expected.mapRevision);
  REQUIRE(actual.mapFrameId==expected.mapFrameId);
  REQUIRE(actual.mapSourceId==expected.mapSourceId);
  REQUIRE(actual.diagnostics.expanded==expected.diagnostics.expanded);
  REQUIRE(actual.diagnostics.generated==expected.diagnostics.generated);
  REQUIRE(actual.diagnostics.primitiveTemplatesBuilt==expected.diagnostics.primitiveTemplatesBuilt);
  REQUIRE(actual.diagnostics.finalPoseChecks==expected.diagnostics.finalPoseChecks);
  REQUIRE(actual.sections.size()==expected.sections.size());
  REQUIRE(std::abs(actual.weightedCostMeters-expected.weightedCostMeters)<1e-9);
  for(std::size_t i=0;i<actual.sections.size();++i) {
    REQUIRE(actual.sections[i].kind==expected.sections[i].kind);
    REQUIRE(std::abs(actual.sections[i].end.x-expected.sections[i].end.x)<1e-9);
    REQUIRE(std::abs(actual.sections[i].end.y-expected.sections[i].end.y)<1e-9);
    REQUIRE(std::abs(actual.sections[i].end.yaw-expected.sections[i].end.yaw)<1e-9);
  }
}

TEST_CASE("Built-in Ackermann uses the same kernel as the public policy adapter",
          "[w11][policy][ackermann]") {
  auto r=openRequest();
  const auto original=a::plan(r);
  p::PlanarPolicyAdapter<p::AckermannPolicy> adapter{p::AckermannPolicy{r}};
  const auto viaPolicy=a::planWithPolicy(r,adapter);
  REQUIRE(original.status==a::PlanStatus::Success);
  sameResult(viaPolicy,original);
}

TEST_CASE("Built-in differential drive uses identical policy-driven search",
          "[w11][policy][differential]") {
  auto r=openRequest();
  r.options.model=a::PlanOptions::Model::DifferentialDrive;
  r.footprint={-.35,.35,.25};
  r.goal={10,15,.5*std::numbers::pi_v<double>};
  const auto original=a::plan(r);
  p::PlanarPolicyAdapter<p::DifferentialDrivePolicy> adapter{p::DifferentialDrivePolicy{r}};
  const auto viaPolicy=a::planWithPolicy(r,adapter);
  REQUIRE(original.status==a::PlanStatus::Success);
  REQUIRE(original.weightedCostMeters>0);
  sameResult(viaPolicy,original);
}

TEST_CASE("Third planar model reuses actual Hybrid A* with only a policy",
          "[w11][policy][extension]") {
  auto r=openRequest();
  r.footprint={-.35,.35,.25};
  r.goal={10,15,.5*std::numbers::pi_v<double>};
  p::PlanarPolicyAdapter<PivotOnlyPolicy> custom{PivotOnlyPolicy{r}};
  const auto result=a::planWithPolicy(r,custom);
  REQUIRE(result.status==a::PlanStatus::Success);
  REQUIRE(result.diagnostics.primitiveTemplatesBuilt==2);
  REQUIRE(result.collisionEvidence==a::CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
  REQUIRE(result.mapRevision==51);
  REQUIRE(result.weightedCostMeters>0);
  REQUIRE(result.geometricLengthMeters==0);
  REQUIRE(!result.sections.empty());
  for(const auto& section:result.sections) {
    REQUIRE(section.kind==a::MotionKind::RotateInPlace);
    REQUIRE(section.lengthMeters==0);
    REQUIRE(std::abs(section.turnRadians)>0);
    REQUIRE(std::abs(section.start.x-section.end.x)<1e-10);
    REQUIRE(std::abs(section.start.y-section.end.y)<1e-10);
  }
  const auto repeat=a::planWithPolicy(r,custom);
  sameResult(repeat,result);
}

TEST_CASE("Policy validation failures become explicit invalid requests",
          "[w11][policy][validation]") {
  auto r=openRequest();
  r.options.model=a::PlanOptions::Model::DifferentialDrive;
  r.options.rotationCostMetersPerRadian=0;
  const p::PlanarPolicyAdapter<p::DifferentialDrivePolicy> model{p::DifferentialDrivePolicy{r}};
  REQUIRE(a::planWithPolicy(r,model).status==a::PlanStatus::InvalidRequest);
  REQUIRE(a::plan(r).status==a::PlanStatus::InvalidRequest);
}

TEST_CASE("External policy rejects unsupported analytic and nonpositive edges",
          "[w11][policy][safety]") {
  auto r=openRequest();
  r.goal={10,15,.5*std::numbers::pi_v<double>};
  r.footprint={-.35,.35,.25};
  p::PlanarPolicyAdapter<PivotOnlyPolicy> pivot{PivotOnlyPolicy{r}};
  r.analyticConnector=[](const Pose2D&,const Pose2D&) {
    return std::vector<a::MotionSection>{};
  };
  const auto rejected=a::planWithPolicy(r,pivot);
  REQUIRE(rejected.status==a::PlanStatus::InvalidRequest);
  REQUIRE(rejected.mapRevision==51);
  r.analyticConnector={};
  r.options.clearanceWeight=1.;
  REQUIRE(a::planWithPolicy(r,pivot).status==a::PlanStatus::InvalidRequest);
  r.options.clearanceWeight=0.;
  p::PlanarPolicyAdapter<ZeroCostPivotPolicy> zero{ZeroCostPivotPolicy{r}};
  REQUIRE(a::planWithPolicy(r,zero).status==a::PlanStatus::InvalidRequest);
}
