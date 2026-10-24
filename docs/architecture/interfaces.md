# Interface contracts — proposed v0.1 (pre-implementation)

**All C++ fragments are illustrative API sketches, not compilable headers.** Exact names, constness, templates, error types and ownership will be reviewed *before coding*. The requirements are semantic; implementations may choose concept-based static dispatch for hot paths, and type erasure at application boundaries.

## Shared semantics and units

- Positions and collision geometry are 2D in **meters**; headings/steering in **radians**, time in seconds; avoid implicit units conversions. Follow REP-103 right-handed body axes (+X forward, +Y left) and counterclockwise-positive yaw about +Z.
- State includes all variables that influence propagation, collision-body poses, or future reachability. A world pose `Pose2D{x,y,yaw}` is a projection of state, **not** necessarily the full state.
- An integration domain may be signed travel distance or nonnegative time; they are not interchangeable for stationary rotation.
- State controls, gear and maneuver events are **model-specific**. Never require all models to expose turning radius, steering angle, or reverse movement.
- Holonomic chassis motion may translate sideways at constant heading or translate and rotate simultaneously; body yaw and travel tangent are independent. The search cannot infer feasibility from a holonomic/nonholonomic flag.
- Every path and map has a known coordinate frame and a bounded validity context (map version and model/config identity). The default planning frame is `map` under REP-105. Start, goal, snapshot, and output frames must agree; ROS transforms are applied externally.
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

enum class UnknownCellPolicy { Occupied, Free, Reject };
struct PlanningOptions {
    UnknownCellPolicy unknownCells = UnknownCellPolicy::Occupied;
    // Occupancy threshold and other policies: separate settings, TBD.
};
struct MapSnapshot {
    GridGeometry geometry;
    std::span<const int8_t> occupancy;  // candidate ROS occupancy encoding, final internal storage TBD
    uint64_t version;
    // Original occupancy values remain independent of planning policy.
};
```

**Accepted OQ-006 unknown-policy subset (ADR-0017):** the immutable snapshot preserves original cell semantics (`-1` unknown, `0` known free, `1..99` intermediate occupancy, `100` occupied). Each request explicitly selects `Occupied` (the default for frontier exploration), `Free` (optimistic, with provisional map-validity status), or `Reject` (error on relevant unknown occupancy; exact encounter scope remains under review). The policy belongs to planning options, **not the map snapshot**, and must agree across collision/traversability and heuristic interpretation. It never rewrites or duplicates the occupancy data merely to select a policy.

Contract: snapshot holds its backing storage alive for the whole request (e.g., via an immutable owning handle). Signed `int8_t` above is a candidate wire-compatible representation; alternative typed occupancy storage remains open. Occupied-probability threshold and invalid-encoding behavior must be explicit, and Nav2 uint8 costmaps require conversion rather than raw reinterpretation. Grid indices use `index = j*width+i` (X-fastest); grid origin is the lower-left corner of cell `(0,0)` with a `(+0.5,+0.5)` cell-center offset. World/grid conversions must handle rotated origins through full SE(2), floor negative coordinates correctly, and check bounds. Details: [map-coordinates.md](map-coordinates.md).

Plan configuration is a validated immutable typed structure. YAML/JSON adapters own parsing and versioning, not search.

## 2. Kinematic model

The model is defined primarily by the controlled differential equation `dq/dt = f(q,u;p)`, the valid state/control sets, state-to-pose projection and model-defined collision-frame transforms. A full mathematical definition, C++20 concept sketch, and examples for differential drive, bicycle/Ackermann, ideal mecanum and articulation are in [kinematic-models.md](kinematic-models.md).

Contract:
- Model owns state, control, tangent/derivative and validity operations; feasible control bounds can depend on current state.
- **Integration is separate**: a numerical/analytical propagator evaluates the vector field and yields intermediate states for collision and path reconstruction. Exact propagation may be specialized by a model.
- The primary ODE uses time; local primitive progress and final geometric path progress are separate representations, not an MPC execution schedule.
- Capability queries describe possibilities, but do not replace checking admissible controls. In particular do not branch search on `isHolonomic()`.
- State may live on a manifold (e.g. yaw or joint angles), with model-defined tangent/state-update conventions; discrete modes are not differentiated.
- Implementation priority: differential drive and geometric bicycle/Ackermann. Ideal mecanum is the third **reference and integration-test model**. Articulated state support remains a design extension, not a claim of implementation.
- **Accepted OQ-007:** initial Ackermann model is `(x,y,yaw)` with fixed-curvature straight/left/right primitives and optional intermediate curvatures ([ADR-0014](../adr/0014-fixed-curvature-ackermann-baseline.md)). Steering angle is a primitive control, not baseline state. Raw arc concatenations do not guarantee finite steering rate; later refinement or a separate four-state model must validate this requirement.

## 3. Motion primitive generation and indexing

```cpp
template<class M> struct PrimitiveCandidate {
    typename M::Control control;
    typename M::PrimitiveExtent extent;  // geometric; model-defined units
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
- **Accepted OQ-002:** translating primitives use explicit traveled-distance extents; zero-translation rotations/steering/articulation maneuvers use model-specific angular/configuration extents ([ADR-0012](../adr/0012-model-aware-geometric-primitive-extents.md)). Kinematic models retain time-domain ODEs and may use time-domain integration; spatial reparameterization must not divide by zero at stationary motions. The exact typed extent/control-law signature is still under review.
- Search node stores continuous state, accumulated `g`, stable parent reference and the exact action/connector necessary to reconstruct its edge.
- The **model-specific** discretized key includes configuration dimensions necessary to distinguish reachability and collision geometry (e.g. articulation for a four-state model). The initial three-state Ackermann key may use only quantized `(x,y,yaw)`. Quantization may still merge nonequivalent continuous states.
- **Accepted OQ-001:** exactly **one retained continuous node per discretized key** in the initial Hybrid A* ([ADR-0013](../adr/0013-single-label-search-baseline.md), superseding ADR-0010). Keep continuous state, `g`, parent and incoming primitive on the node; do not implement multi-label storage, retention policies or runtime selection in the baseline. Relaxation, reopening and stale-queue handling are required. Incoming maneuver is not part of the key by default.
- One lowest-`g` arrival per discretized key is **approximate pruning** because equal keys do not imply identical future reachability. Incoming-primitive-dependent effort costs also introduce history dependence not represented by the key. We explicitly accept this simplification and make no global optimality or resolution-completeness guarantee. Optional refinement cannot always recover a discarded arrival or alternative homotopy.
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
- **Accepted OQ-005:** default mode evaluates only certified admissible lower bounds for the configured cost and **accepted goal region** ([ADR-0015](../adr/0015-admissible-default-heuristics.md)); optional weighted/experimental modes are explicit and do not inherit unsupported suboptimality claims. A zero heuristic is the safe fallback for nonnegative costs. Reverse Dijkstra estimates obstacle-aware distance from a **static** map; key/cache includes map version/goal/grid and free-space policy. Lazy distance expansion must settle cells before returning final distance values.
- Heuristics combine as `max(h_obstacle, h_model)` **only when both are proven lower bounds on the configured `g` cost**. Otherwise use a documented nonadmissible search mode; do not claim optimality.
- Discrete 2D shortest-path distances and obstacle inflation can *overestimate* continuous robot path distance; a reverse-Dijkstra field may be computed but **cannot be marked certified** until a suitable conservative bound is proven/corrected. A certified heuristic does not undo quantized-key pruning (ADR-0013).
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
    State start;
    PlanningOptions options{};  // explicit per-request unknown handling
    // Goal specification: world pose + independent XY/yaw tolerances;
    // optional model-specific state constraints, not a mandatory full State goal.
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

**Accepted OQ-008:** the default goal accepts the **continuous model-defined reference pose** when XY Euclidean distance to the target is `<= 0.25 m` **and** wrapped yaw error is `<= 10°` (`pi/18 rad`). These tolerances are independently configurable, including zero. Other state components (such as articulation) are unconstrained by default but must remain valid and collision-free; the caller may request explicit additional state/terminal constraints. Validate the incoming swept motion and return the actual reached endpoint, never snap the path to the goal center. Goal-region heuristics must account for tolerance ([ADR-0016](../adr/0016-default-pose-goal-tolerances.md)).

Contract: no partial/unverified path is reported as `Success`. Failure carries diagnostic reason and counters; cancellation is checked at bounded intervals. Planner instances should avoid rebuilding immutable caches unnecessarily; concurrency semantics remain proposed in [components.md](components.md).

**Speed-profile boundary (OQ-024):** core output is a geometric path, not a time-stamped trajectory. Each segment must support evaluating full configuration at its canonical `(segmentIndex, lambda)` position; optional model-appropriate first/second derivatives, manifold operations and control-law metadata permit later time scaling. A separate trajectory planner checks wheel/steering/articulation rates, accelerations/jerks, stops at mode transitions and moving obstacles. Model constraints depending on an assumed primitive duration are **not** guaranteed after arbitrary retiming. A speed-aware feasibility estimate inside search remains an open design choice.

## 7. Piecewise geometric path and refinement

The path represents a sequence of **model-feasible motions**, not merely a polyline or a spline. Each motion segment has:
- a **canonical path address** `(segmentIndex, lambda)` with `lambda in [0,1]`, full start/end states and model-appropriate motion semantics (including lateral translation), as accepted in [ADR-0011](../adr/0011-canonical-segment-path-position.md);
- an evaluable complete model configuration `q(progress)`, including heading and articulation when present;
- optional derivatives/curvature with an explicit domain of definition and orientation/direction convention;
- boundary metadata for reverse cusps, rotations in place, and combined translation/rotation; body yaw need not equal spatial tangent and global differentiability is **not** required;
- a validation record identifying the map/model configuration and guarantee level.

```cpp
struct PathPosition { std::size_t segmentIndex; double lambda; };

template<class State> struct PathEvaluation {
    State state;
    // optional: tangent, curvature, control, derivative validity
};
template<class State>
struct GeometricSegment {
    // evaluate(lambda in [0,1]) -> PathEvaluation<State>
    // motionDirection(), endConditions(), provenance()
};
template<class State>
struct PiecewiseGeometricPath {
    // segments(); evaluate(PathPosition{segmentIndex,lambda});
    // optional translationalArcLength(), cost(), other derived metrics
    // map/config identity, validation metadata
};
```

**The canonical path address is not universally translational arc length.** A segment's normalized `lambda` advances even during in-place rotation or articulation-only movement with zero translation. Holonomic models can translate sideways at fixed yaw, so pose heading and spatial tangent need not coincide. Translational arc length is a useful *derived metric*, not the address. Segment endpoints must agree in complete continuous configuration; controls or derivatives may have one-sided/discontinuous limits at cusps. Boundary canonicalization remains an interface detail.

Refinement consumes an already validated raw path and returns either:
1. a new path with verified model constraints, endpoint conditions and collision status; or
2. a failure status plus unchanged raw path.

Curve fitting (B-splines, Hermite, clothoids, arcs) is an internal interchangeable strategy, not a public path requirement. Optional Ceres backend does not leak into search or public path types. For articulated models, optimizing controls and integrating the model may be preferable to smoothing only `x,y`.

## Interface decisions to close

Review the remaining details in [open-questions.md](open-questions.md): primitive-extent/control-law ABI (OQ-002), path boundary canonicalization (OQ-003), state/tangent conventions (OQ-021), capability metadata (OQ-019), map ownership (OQ-006), effort-cost objective (OQ-004/OQ-024), collision assurance and type-erasure boundaries. Baseline one-key/one-node search, three-state fixed-curvature Ackermann, admissible-default heuristics and pose-tolerant goals are accepted (ADRs 0013–0016).
