# Interface contracts — proposed v0.1 (pre-implementation)

**All C++ fragments are illustrative API sketches, not compilable headers.** Exact names, constness, templates, error types and ownership will be reviewed *before coding*. The requirements are semantic; implementations may choose concept-based static dispatch for hot paths, and type erasure at application boundaries.

## Shared semantics and units

- Positions and collision geometry are 2D in **meters**; headings/steering in **radians**, time in seconds; avoid implicit units conversions.
- State includes all variables that influence propagation, collision-body poses, or future reachability. A world pose `Pose2D{x,y,yaw}` is a projection of state, **not** necessarily the full state.
- An integration domain may be signed travel distance or nonnegative time; they are not interchangeable for stationary rotation.
- State controls, gear and maneuver events are **model-specific**. Never require all models to expose turning radius, steering angle, or reverse movement.
- Every path and map has a known coordinate frame and a bounded validity context (map version and model/config identity).
- Prefer status/result values at request boundaries over uncaught exceptions in planning hot paths.

## 1. Configuration and environment

```cpp
struct Pose2D { double x, y, yaw; };
struct GridGeometry {
    double resolution;
    Pose2D origin;
    uint32_t width, height;
    std::string frameId; // public-side metadata, not per-cell
};

enum class UnknownCellPolicy { Free, Occupied, Reject };
struct MapSnapshot {
    GridGeometry geometry;
    std::span<const uint8_t> occupancy; // owning handle specified separately
    uint64_t version;
    UnknownCellPolicy unknownPolicy;
};
```

Contract: snapshot holds its backing storage alive for the whole request (e.g., via an immutable owning handle). Occupancy interpretation and threshold are explicit. World/grid transforms handle rotated origins; out-of-bounds behavior is explicit. Search, heuristic and collision code consume the same snapshot but may apply different **conservative** relaxations for heuristic estimation.

Plan configuration is a validated immutable typed structure. YAML/JSON adapters own parsing and versioning, not search.

## 2. Kinematic model

```cpp
template<class M>
concept KinematicModel = requires(
    const M& m, const typename M::State& q,
    const typename M::Control& u, double step, FrameId id)
{
    typename M::State;
    typename M::Control;
    { m.pose(q) } -> std::same_as<Pose2D>;
    { m.propagate(q, u, step) } -> std::same_as<typename M::State>;
    { m.frameTransform(q, id) } -> std::same_as<Transform2D>;
};
```

Contract:
- `propagate` uses declared integration domain and deterministic numerical policy; it does not consult the map.
- Model validates finite parameters/states, control bounds and any model-specific constraints.
- `frameTransform` evaluates *hypothetical* state for any declared collision-body frame.
- Capability queries advertise optional reverse, rotate-in-place, analytic connector compatibility and extended state; they must not be inferred from model class names.
- Exact integration where available; numerical integration through an independently testable generic integrator otherwise.
- Initial implementations: differential drive and geometric bicycle/Ackermann; articulated state support is a required extension point, **not** a claim of a first-milestone implementation.

## 3. Motion primitive generation and indexing

```cpp
template<class M> struct PrimitiveCandidate {
    typename M::Control control;
    double extent;          // explicit time/distance domain
    uint32_t primitiveId;
};
template<class M> concept PrimitiveProvider = requires(
    const M& model, const typename M::State& q,
    PrimitiveBuffer<typename M::Control>& controls)
{
    // Model-dependent control and extent generation.
    // Candidate evaluation returns intermediate states + final state.
};
template<class M> concept StateIndexer = requires(
    const M& model, const typename M::State& q)
{
    // quantize(q) -> hashable/equatable model-aware search key
};
```

Contracts:
- Candidate trajectories include start, end, and sufficient intermediate state/body motion information for collision validation. Cached local trajectories must still transform against the **continuous** parent state.
- Search node stores continuous state, accumulated `g`, stable parent reference and the exact action/connector necessary to reconstruct its edge.
- The state key must include steering/articulation/other dimensions when their values affect reachability or collision. Do not conflate identical poses with different vehicle configurations.
- A single `g` per discretized key is only valid if the state-abstraction/dominance assumptions are documented. Alternative labels or augmented keys may be required; optimality/completeness is not automatic.
- Precomputation is allowed only if validity assumptions (initial internal state, control limits, etc.) remain true.

## 4. Geometry and collision

```cpp
struct CollisionBody2D {
    FrameId frame;
    ConvexShape2D shape;   // circle or convex polygon; compound as list
    double safetyMargin;
};
template<class M>
concept CollisionValidator = requires(
    const M& model, const typename M::State& state,
    const TrajectoryView<typename M::State>& motion,
    const MapSnapshot& map)
{
    // stateValid(state, map) -> CollisionCheckResult
    // motionValid(motion, map) -> CollisionCheckResult
};
```

Contract: collision geometry is declarative, frame-attached, and immutable inside a request. Each sample uses model-derived transforms for all bodies, including articulation. Motion checking validates the **swept robot geometry**, not endpoints alone. Finite sampling is approximate: the checker must report the assurance level/resolution; strict continuous guarantees require conservative motion bounds or continuous collision detection. Self-collision policy and adjacent-body exclusions are separate configuration.

## 5. Heuristics and analytic steering functions

```cpp
struct HeuristicValue {
    double lowerBoundCost;
    bool certifiedAdmissible;
};

template<class State>
concept SteeringConnector = requires(const State& a, const State& b)
{
    // shortestPath(a,b) -> model-compatible optional path
    // distanceLowerBound(a,b) -> cost lower bound, if available
};
```

Contract:
- Reverse Dijkstra estimates obstacle-aware distance from **static** map; key/cache includes map version/goal/grid and free-space policy. Lazy distance expansion must settle cells before returning final distance values.
- Heuristics combine as `max(h_obstacle, h_model)` **only when both are proven lower bounds on the configured `g` cost**. Otherwise use a documented nonadmissible search mode; do not claim optimality.
- Discrete 2D shortest-path distances and obstacle inflation can *overestimate* the continuous robot path distance; construction of admissible bounds needs a separate proof or conservative correction. Mark as an open design question.
- Native Dubins and Reeds–Shepp connectors are valid only for compatible geometric models with constant turning-radius assumptions. Neither inherently enforces steering-rate constraints or articulated dynamics.
- Analytic paths must be fully collision-checked, model-valid, and costed consistently; multiple equal-optimal path families are acceptable.

## 6. Planning request and result

```cpp
enum class PlanStatus {
    Success, NoPath, Cancelled, ResourceLimit,
    InvalidRequest, MapUnavailable, InternalError
};
template<class State>
struct PlanRequest {
    State start, goal;
    // immutable map handle, model/config identity, limits,
    // optional cancellation token
};
template<class State>
struct PlanResult {
    PlanStatus status;
    // optional GeometricPath<State>, diagnostics and
    // model/map version identities
};
```

Contract: no partial/unverified path is reported as `Success`. Failure carries diagnostic reason and counters; cancellation is checked at bounded intervals. Planner instances should avoid rebuilding immutable caches unnecessarily; concurrency semantics remain proposed in [components.md](components.md).

## 7. Piecewise geometric path and refinement

The path represents a sequence of **model-feasible motions**, not merely a polyline or a spline. Each motion segment has:
- a monotone **segment-local progress** domain, start/end state and direction/maneuver kind;
- an evaluable complete model configuration `q(progress)`, including heading and articulation when present;
- optional derivatives/curvature with an explicit domain of definition and orientation/direction convention;
- boundary metadata for reverse cusps and rotations in place; global differentiability is **not** required;
- a validation record identifying the map/model configuration and guarantee level.

```cpp
template<class State> struct PathEvaluation {
    State state;
    // optional: tangent, curvature, control, derivative validity
};
template<class State>
struct GeometricSegment {
    // progressDomain() -> [0, length] or model-defined progress
    // evaluate(progress) -> PathEvaluation<State>
    // motionDirection(), endConditions(), provenance()
};
template<class State>
struct PiecewiseGeometricPath {
    // segments(); evaluate(globalProgress); totalProgress();
    // map/config identity, validation metadata
};
```

**Progress is not universally translational arc length.** In-place rotations have zero translation but nonzero maneuver progress. For ordinary translations, arc length is preferred. Explicit segment-local parameterization prevents the controller from assuming arbitrary `ds/dt` through a reversing cusp.

Refinement consumes an already validated raw path and returns either:
1. a new path with verified model constraints, endpoint conditions and collision status; or
2. a failure status plus unchanged raw path.

Curve fitting (B-splines, Hermite, clothoids, arcs) is an internal interchangeable strategy, not a public path requirement. Optional Ceres backend does not leak into search or public path types. For articulated models, optimizing controls and integrating the model may be preferable to smoothing only `x,y`.

## Interface decisions to close

Review the high-priority items in [open-questions.md](open-questions.md): control/integration domains, exact state-key/dominance semantics, map ownership, cost units/heuristic admissibility, geometric-path parameterization, validation assurance, and type-erasure boundaries.
