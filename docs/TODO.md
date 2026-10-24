# Implementation and architecture TODOs

A prioritized, incremental checklist. **All tasks unchecked as of initial documentation baseline**; this is a proposed plan, not a report of completed code. See [architecture index](architecture/README.md) and [open questions](architecture/open-questions.md).

## M0 — Review and accept architecture baseline

- [ ] Review [requirements](architecture/requirements.md): correct scope, vehicle models, non-goals.
- [ ] Review [components](architecture/components.md): dependency direction, ownership and map snapshots.
- [ ] Review [interfaces](architecture/interfaces.md): state/control types, primitive/domain, indexed labels, path evaluation.
- [ ] Review [controlled-ODE model specification](architecture/kinematic-models.md) for derivative, tangent, constraints, body transforms and integration semantics.
- [ ] Review [ROS-compatible map coordinate specification](architecture/map-coordinates.md) and [ADR-0009](adr/0009-ros-map-coordinate-conventions.md); settle the native occupancy/frame representation (OQ-023).
- [ ] Confirm outdoor world frame uses local ENU (+X east, +Y north, +Z up) with explicit geodetic reference when GNSS is used; indoor frame may be building-aligned.
- [ ] Review [ADR-0008](adr/0008-motion-model-capabilities.md): model control constraints, ideal mecanum as third reference and holonomic travel semantics.
- [ ] Confirm supplied four-state articulated-steering ODE input interpretation, joint/body frame conventions and link lengths (OQ-022).
- [x] Accept OQ-001 **single retained node per model-specific discretized key** (ADR-0013, superseding ADR-0010); [x] accept OQ-003 canonical `(segmentIndex, lambda)` path position (ADR-0011).
- [ ] Benchmark single-key pruning against feasible-path recovery and post-refinement quality in narrow passages; revisit extra arrival labels only if evidence warrants a separate future decision.
- [x] Accept OQ-002 geometric length for translation and angular/configuration extent for stationary motion (ADR-0012).
- [ ] Resolve OQ-002 exact typed extent/control law and OQ-024 downstream speed-profile/feasibility interface; timing/rate limits require explicit treatment.
- [x] Accept OQ-005 certified-admissible heuristic default and explicit opt-in relaxed modes (ADR-0015).
- [x] Accept OQ-007 three-state fixed-curvature Ackermann primitives (ADR-0014).
- [x] Accept OQ-008 pose-tolerance default of 0.25 m / 10 degrees with optional extra-state goal constraints (ADR-0016).
- [ ] Resolve OQ-004 planning effort-cost objective and OQ-006 map ownership/unknown semantics.
- [ ] Agree [quality metrics](architecture/quality.md), reference hardware/scenarios and initial budgets where feasible.
- [ ] Review [verification matrix](architecture/verification.md), Catch2 property and oracle tests.
- [ ] Accept/revise ADRs and mark architecture baseline v0.1 reviewed.

**Exit:** contracts and initial assumptions agreed; no unresolved blocker for M1.

## M1 — Standalone foundation (no search yet)

- [ ] C++20 CMake targets for core/math plus Catch2/CTest; no ROS dependency.
- [ ] Format/lint/test/benchmark commands documented for local use.
- [ ] Common types: poses, transforms, frame ids, angles, units, result/status.
- [ ] Immutable validated native configuration; YAML (yaml-cpp) and JSON (nlohmann/json) adapters.
- [ ] Occupancy grid snapshot, REP-103/105 frame metadata, SE(2) origin/world-grid conversions, row-major X-fastest layout, unknown/out-of-bounds policy and version.
- [ ] Convert ROS OccupancyGrid signed occupancy cells explicitly and disallow raw Nav2 uint8 costmap reinterpretation.
- [ ] Robot collision body definitions (circle/rectangle/convex polygon/compound), frame registration.
- [ ] Catch2 tests for foundational types, serialization and ROS-compatible grid semantics: zero/rotated origins, half-cell offsets, negative/border coordinates, unknown occupancy, frame mismatches and Y-axis direction.
- [ ] Catch2 outdoor ENU axis/yaw fixtures, with independent rotated-grid origin; optional adapter tests for geodetic datum/reference and GNSS-to-local conversion.
- [ ] cxxopts-based CLI fixture runner (optional initial example).

**Exit:** standalone core configure/build/test succeeds locally.

## M2 — Kinematics, primitives and collision

- [ ] Differential-drive model including in-place rotation; compile-time concept checks.
- [ ] Three-state geometric bicycle/Ackermann model with exact fixed-curvature left/straight/right (optional intermediate) forward/reverse arcs; preserve joins and annotate finite steering-rate assumptions (ADR-0014).
- [ ] Ideal mecanum reference model: independent chassis body lateral/forward/yaw velocities, admissible bounds, concept tests and analytic propagation fixtures.
- [ ] Define controlled differential-equation/tangent model interface, and separate generic integrator (native RK4) plus exact constant-curvature propagation.
- [ ] Model-specific control enumeration, primitive templates and cache invalidation.
- [ ] Implement OQ-002 length-parametrized translating primitives plus model-aware angular/configuration extents for stationary maneuvers; retain validated time-domain integration fallback and ensure no division by longitudinal speed at zero.
- [ ] Implement **single retained continuous node per model-defined discrete key** with stable parents, incoming primitive metadata, reopen/stale-queue handling and no multi-label storage (ADR-0013).
- [ ] Deterministic key replacement/pruning tests, including loss of alternative continuous arrivals and history-dependent transition-cost approximation.
- [ ] Geometry frame transforms from hypothetical state; add a fixture for the supplied four-state articulated-steering model after its physical frame conventions are confirmed.
- [ ] Implement articulated steering derivative with safe denominator checks, articulation state indexing, rate constraints and zero-translation articulation sweep tests (extension milestone, not blocking differential/Ackermann).
- [ ] Swept-motion broad/narrow phase, explicit validation assurance.
- [ ] Numerical, collision and maneuver property tests including mecanum lateral/combined motion; baseline microbenchmarks.

**Exit:** differential-drive, Ackermann and ideal mecanum reference models generate repeatable, valid trajectories in fixture maps.

## M3 — Steering and heuristics

- [ ] Native Dubins shortest-path/distance/interpolation (all families).
- [ ] Native Reeds–Shepp shortest-path/distance/interpolation (all relevant families).
- [ ] Segment/direction representation, robust normalization and degenerate handling.
- [ ] Optional OMPL differential test target (test-only, OFF by default), randomized reproducible corpus.
- [ ] Reverse-Dijkstra obstacle-distance field with lazy/eager modes and cache versioning.
- [ ] Implement certified-admissible lower-bound heuristic as default (zero or goal-region Euclidean fallback until obstacle-field bound proven), explicit weighted/experimental modes, and report lack of global optimality under quantized pruning.
- [ ] Profile steering distance-only calls and heuristic cache behavior.

**Exit:** native steering passes analytic/oracle tests and obstacle heuristic passes grid fixtures.

## M4 — Hybrid A* baseline

- [ ] Node pool/open heap, stable indices, stale entry strategy, parent/action records.
- [ ] Model-generic search loop with primitive and optional analytic expansion.
- [ ] Edge costs, tie breaks, reopen/dominance and termination semantics.
- [ ] Cancellation and time/expansion/memory budgets, structured diagnostics.
- [ ] Exact path reconstruction and validated piecewise path output using canonical `(segmentIndex, lambda)` evaluation; complete state and one-sided cusp semantics.
- [ ] Evaluate goal conditions on actual continuous endpoints with configurable XY/yaw tolerances and optional additional full-state constraints; verify final swept motion.
- [ ] Scenario tests (wall/doorway, narrow turns, reverse parking, unreachable goal; mecanum sideways travel and yaw-independent footprint constraints).
- [ ] Compare geometric raw versus refined path planning outcomes, including paths lost to single-label key pruning, without promising refinability of missing homotopy classes.
- [ ] Local latency/memory/quality baseline with and without obstacle-aware heuristic.

**Exit:** model-independent, validated baseline paths with measured performance.

## M5 — Path representation and optional refinement

- [ ] Segment-level continuous evaluation using canonical `(segmentIndex, lambda)`, derivative availability, progress/cusp/rotation semantics, and lateral holonomic motion without heading-as-tangent assumptions.
- [ ] Define external speed-profile input requirements: segment derivatives, rate/acceleration/jerk limits, start/end speeds and mandatory stop conditions (OQ-024).
- [ ] Validate final timed trajectory against model actuation limits and moving-obstacle constraints; reject/adjust/replan if a geometric path cannot be executed safely.
- [ ] Exact start/end full configurations and map/model provenance.
- [ ] Native curve math (B-spline / Hermite etc.) without mandating one family in API.
- [ ] Feasible shortcutting and optional optimization (Ceres isolated behind optional target).
- [ ] Curvature, steering/articulation, endpoint and swept-collision validator.
- [ ] Original-path fallback on failed refinement; record validation level.
- [ ] Controller-facing evaluation/projection benchmarks.

**Exit:** refined path never silently displaces valid raw path.

## M6 — External integration (later)

- [ ] YAML/JSON example configs for differential, Ackermann, ideal mecanum and articulated geometry.
- [ ] Standalone examples and CLI traces using cxxopts.
- [ ] Optional ROS 2 adapter and Nav2 plugin without ROS headers in core.
- [ ] Rich path output preserving segment metadata beyond `nav_msgs/Path`.
- [ ] Integration test with separate MPC/trajectory planner contract (no MPC inside core).
- [ ] Release/package documentation, license and dependency notices.

## Working agreements

- Implement on feature branches with **small commits pushed after meaningful progress**; do not merge without review.
- Run local tests/benchmarks using available tooling; document limitations rather than dispatch GitHub Actions.
- No automatic workflow triggers (`push`, `pull_request`, `schedule` etc.) unless explicitly requested.
- Keep requirement IDs, interface docs, ADRs, verification and code changes synchronized.
