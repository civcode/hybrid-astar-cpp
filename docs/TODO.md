# Implementation and architecture TODOs

A prioritized, incremental checklist. **All tasks unchecked as of initial documentation baseline**; this is a proposed plan, not a report of completed code. See [architecture index](architecture/README.md) and [open questions](architecture/open-questions.md).

## M0 — Review and accept architecture baseline

- [ ] Review [requirements](architecture/requirements.md): correct scope, vehicle models, non-goals.
- [ ] Review [components](architecture/components.md): dependency direction, ownership and map snapshots.
- [ ] Review [interfaces](architecture/interfaces.md): state/control types, primitive/domain, indexed labels, path evaluation.
- [ ] Review [controlled-ODE model specification](architecture/kinematic-models.md) for derivative, tangent, constraints, body transforms and integration semantics.
- [ ] Review [ADR-0008](adr/0008-motion-model-capabilities.md): model control constraints, ideal mecanum as third reference and holonomic travel semantics.
- [ ] Resolve or explicitly defer high-priority [open questions](architecture/open-questions.md) OQ-001–OQ-008.
- [ ] Agree [quality metrics](architecture/quality.md), reference hardware/scenarios and initial budgets where feasible.
- [ ] Review [verification matrix](architecture/verification.md), Catch2 property and oracle tests.
- [ ] Accept/revise ADRs and mark architecture baseline v0.1 reviewed.

**Exit:** contracts and initial assumptions agreed; no unresolved blocker for M1.

## M1 — Standalone foundation (no search yet)

- [ ] C++20 CMake targets for core/math plus Catch2/CTest; no ROS dependency.
- [ ] Format/lint/test/benchmark commands documented for local use.
- [ ] Common types: poses, transforms, frame ids, angles, units, result/status.
- [ ] Immutable validated native configuration; YAML (yaml-cpp) and JSON (nlohmann/json) adapters.
- [ ] Occupancy grid snapshot, world/grid conversions, unknown/out-of-bounds policy and version.
- [ ] Robot collision body definitions (circle/rectangle/convex polygon/compound), frame registration.
- [ ] Catch2 tests for all foundational types and serialization roundtrips.
- [ ] cxxopts-based CLI fixture runner (optional initial example).

**Exit:** standalone core configure/build/test succeeds locally.

## M2 — Kinematics, primitives and collision

- [ ] Differential-drive model including in-place rotation; compile-time concept checks.
- [ ] Geometric bicycle/Ackermann model with stated constraints and reversing capability.
- [ ] Ideal mecanum reference model: independent chassis body lateral/forward/yaw velocities, admissible bounds, concept tests and analytic propagation fixtures.
- [ ] Define controlled differential-equation/tangent model interface, and separate generic integrator (native RK4) plus exact constant-curvature propagation.
- [ ] Model-specific control enumeration, primitive templates and cache invalidation.
- [ ] Model-aware state indexing / dominance labels according to accepted OQ-001.
- [ ] Geometry frame transforms from hypothetical state; articulated extension test model or fixture.
- [ ] Swept-motion broad/narrow phase, explicit validation assurance.
- [ ] Numerical, collision and maneuver property tests including mecanum lateral/combined motion; baseline microbenchmarks.

**Exit:** differential-drive, Ackermann and ideal mecanum reference models generate repeatable, valid trajectories in fixture maps.

## M3 — Steering and heuristics

- [ ] Native Dubins shortest-path/distance/interpolation (all families).
- [ ] Native Reeds–Shepp shortest-path/distance/interpolation (all relevant families).
- [ ] Segment/direction representation, robust normalization and degenerate handling.
- [ ] Optional OMPL differential test target (test-only, OFF by default), randomized reproducible corpus.
- [ ] Reverse-Dijkstra obstacle-distance field with lazy/eager modes and cache versioning.
- [ ] Define, test and clearly report heuristic admissibility/optimality mode.
- [ ] Profile steering distance-only calls and heuristic cache behavior.

**Exit:** native steering passes analytic/oracle tests and obstacle heuristic passes grid fixtures.

## M4 — Hybrid A* baseline

- [ ] Node pool/open heap, stable indices, stale entry strategy, parent/action records.
- [ ] Model-generic search loop with primitive and optional analytic expansion.
- [ ] Edge costs, tie breaks, reopen/dominance and termination semantics.
- [ ] Cancellation and time/expansion/memory budgets, structured diagnostics.
- [ ] Exact path reconstruction and validated piecewise path output.
- [ ] Scenario tests (wall/doorway, narrow turns, reverse parking, unreachable goal; mecanum sideways travel and yaw-independent footprint constraints).
- [ ] Local latency/memory/quality baseline with and without obstacle-aware heuristic.

**Exit:** model-independent, validated baseline paths with measured performance.

## M5 — Path representation and optional refinement

- [ ] Segment-level continuous evaluation, derivative availability, progress/cusp/rotation semantics, and lateral holonomic motion without heading-as-tangent assumptions.
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
