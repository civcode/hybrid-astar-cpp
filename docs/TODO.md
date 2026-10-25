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
- [x] Accept OQ-006 unknown-cell semantics/policy and known-free frontier default (ADR-0017); selectable per request, map data unchanged.
- [x] Accept OQ-006 configurable known-cell max occupancy, default zero (ADR-0018).
- [x] Accept OQ-006 immutable full-grid shared snapshots with atomic publication, revision-keyed on-demand bounded caching and strict full-snapshot `Reject` validation (ADR-0019).
- [x] Accept **lean MVP and evidence-driven complexity**: rigorous contracts/tests, early working vertical slice, defer speculative modules (ADR-0020).
- [x] Clarify intended **search → downstream path optimization** boundary: search is a collision-checked, model-feasible path initializer; evaluate quality after refinement rather than overengineering raw search, with independently validated optimized output and raw-path fallback (ADR-0020).
- [x] Accept **obstacle-aware optimization whenever refinement is enabled**: use map obstacles/clearance during optimization, pinned snapshot and occupancy policy, full transformed collision bodies and independent final validation (ADR-0024).
- [x] Accept initial Ackermann search cost weights, resolution/gear key, RK4 primitive and collision sample defaults (ADR-0025); further model-generalized effort costs remain OQ-004/OQ-024.
- [ ] Finish native occupancy/frame/handle ABI (OQ-023) and mutable cache concurrency (OQ-012) only to the extent necessary for first runnable MVP.
- [ ] Agree [quality metrics](architecture/quality.md), reference hardware/scenarios and initial budgets where feasible.
- [ ] Review [verification matrix](architecture/verification.md), Catch2 property and oracle tests.
- [ ] Accept/revise ADRs and mark architecture baseline v0.1 reviewed.

**Exit:** contracts and initial assumptions agreed; no unresolved blocker for M1.

## MVP integration priority (ADR-0020)

- [ ] Define the smallest useful end-to-end supported robot/map/scenario slice with explicit validation and diagnostic contract; **do not block it on optional advanced designs**.
- [ ] Run the selected model, map snapshot, primitive propagation, collision, search, and exact geometric-path reconstruction together against deterministic fixtures as early as the foundational modules permit.
- [ ] Expand to other reference models and optional refinement only with targeted model/geometry verification; mark unsupported or partially supported features honestly.
- [ ] Defer multi-label search, speed-state search, speculative effort optimization, incremental heuristic repair, map tiling and sophisticated concurrent-cache machinery until real scenario evidence motivates them.
- [ ] Once optional refinement is available, compare search design choices on **final validated output** and total search+refinement latency; preserve feasibility, route/maneuver choice and original-path fallback, not expensive raw-path smoothness.
- [ ] Log integration failures with reproducing fixtures and compare planning latency, memory, validity and final path quality before adopting new mechanisms.

## M1 — Standalone foundation (no search yet)

- [ ] C++20 CMake targets for core/math plus Catch2/CTest; no ROS dependency.
- [ ] Format/lint/test/benchmark commands documented for local use.
- [ ] Common types: poses, transforms, frame ids, angles, units, result/status.
- [ ] Immutable validated native configuration; YAML (yaml-cpp) and JSON (nlohmann/json) adapters.
- [ ] Immutable owning occupancy grid snapshot and atomic publication, REP-103/105 frame metadata, SE(2) origin/world-grid conversions, row-major X-fastest layout, per-request unknown/out-of-bounds policy and source identity/revision (ADRs 0017–0019).
- [ ] Convert ROS OccupancyGrid signed occupancy cells explicitly and disallow raw Nav2 uint8 costmap reinterpretation.
- [x] Accept independent frame-attached multibody collision geometry and model-provided frame transforms, supporting geometric trailer chains (ADR-0021).
- [ ] Implement circle/rectangle/convex-polygon collision shape primitives and immutable multibody sets; validate model frame references and compound shape offsets. Exact registry and self-collision pair rules remain OQ-014.
- [ ] Catch2 tests for foundational types, serialization and ROS-compatible grid semantics: zero/rotated origins, half-cell offsets, negative/border coordinates, unknown occupancy, frame mismatches and Y-axis direction.
- [ ] Validate frontier known-free full-footprint traversal, three request-level unknown policies, default max-traversable-known-occupancy `0`, threshold equality, optimistic-path assurance and cache-policy separation.
- [ ] Implement accepted atomic publication of privately built immutable shared map snapshots, revision-keyed on-demand cache rebuild with bounded eviction, and up-front full-snapshot strict `Reject` validation (ADR-0019); define exact API types and synchronization at implementation review.
- [ ] Catch2 outdoor ENU axis/yaw fixtures, with independent rotated-grid origin; optional adapter tests for geodetic datum/reference and GNSS-to-local conversion.
- [ ] cxxopts-based CLI fixture runner (optional initial example).

**Exit:** standalone core configure/build/test succeeds locally.

## M2 — Kinematics, primitives and collision

- [ ] Differential-drive model including in-place rotation; compile-time concept checks.
- [ ] Three-state geometric bicycle/Ackermann model with exact fixed-curvature left/straight/right (optional intermediate) forward/reverse arcs; preserve joins and annotate finite steering-rate assumptions (ADR-0014).
- [ ] Ideal mecanum reference model: independent chassis body lateral/forward/yaw velocities, admissible bounds, concept tests and analytic propagation fixtures.
- [x] Accept RK4-based primitive generation to prescribed actual traveled length (terminal iterative refinement when required), retained ordered full-state visualization/debug points, and search-facing instantiation of cached evaluable motions (ADR-0023).
- [x] Accept **0.50 m** primitive length, **0.05 s RK4** at reference **1 m/s**, **0.0001 m** arc-length tolerance, **0.10 m** stored-sample spacing, and six forward/reverse left/straight/right controls for Ackermann (ADR-0025).
- [ ] Define the controlled ODE/tangent generator interface and build the native RK4 integration plus accumulated distance, failure/tolerance handling; use analytic constant-curvature formulas as tests and optional later optimized evaluators.
- [ ] Model-specific control enumeration, primitive templates and cache invalidation; distinguish SE(2)-equivariant reusable catalogs from initial articulation-state-dependent generation.
- [ ] Implement OQ-002 RK4-generated traveled-length translating primitives plus model-aware angular/configuration extents for stationary maneuvers; keep full control-law numerical evaluation across generated checkpoints, including reverse/stall guards, and ensure no division by longitudinal speed at zero.
- [ ] Implement **single retained continuous node per model-defined discrete key** with stable parents, incoming primitive metadata, reopen/stale-queue handling and no multi-label storage (ADR-0013).
- [ ] Deterministic key replacement/pruning tests, including loss of alternative continuous arrivals and history-dependent transition-cost approximation.
- [ ] Geometry frame transforms from hypothetical state; add a tractor/two-trailer synthetic state-transform fixture without assuming a production ODE; retain the supplied four-state articulated-steering model fixture only after its physical frame conventions are confirmed.
- [ ] Implement articulated steering derivative with safe denominator checks, articulation state indexing, rate constraints and zero-translation articulation sweep tests (extension milestone, not blocking differential/Ackermann).
- [x] Accept whole-shape-vs-cell collision at the **stored primitive states**, **six 0.10 m-spaced states per initial 0.50 m Ackermann primitive**, no independent default fixed collision scheduler and adaptive subdivision optional/off by default (ADRs 0022,0025).
- [ ] Implement **conservative supercover rasterized footprint edges plus scanline-fill/interior occupancy guard**, including circle/compound geometry, unknown and map-boundary policies (ADR-0029); no mandatory SAT shape-vs-cell narrow phase.
- [ ] Precompute orientation-hull **collision-possibility grid** from pinned occupancy map, robot geometry and continuous heading/position bin margins; lookup at **each stored primitive or RS segment pose**. False means provably clear; true triggers boundary + interior raster test. Add separate 0.40 m clearance-influence mask or bounded clearance query, and safe cache invalidation (ADR-0029).
- [ ] Implement candidate rejection at every **stored full-state primitive sample** using conservative hull lookup and rasterized actual footprint (with interior guard), not a separate collision schedule; optional adaptive additional states only if enabled. Report sampled-only assurance.
- [ ] Numerical, collision and maneuver property tests including mecanum lateral/combined motion; baseline microbenchmarks.

**Exit:** differential-drive, Ackermann and ideal mecanum reference models generate repeatable, valid trajectories in fixture maps.

## M3 — Steering and heuristics

- [ ] **OMPL-backed Dubins adapter:** provide distance/interpolation/segments behind native project API; defer full native solver (ADR-0028).
- [ ] **Required for first Ackermann end-to-end MVP:** OMPL-backed Reeds–Shepp adapter for signed straight/arcs, path length, interpolation and cusps; self-authored solver low priority (ADR-0028).
- [ ] Segment/direction representation, robust normalization and degenerate handling.
- [ ] Validate OMPL steering adapter against analytic endpoints, invariances, signed segments and cusp/degenerate fixtures; optional independent native solver comparison only if later implemented (ADR-0028).
- [ ] **Required for first Ackermann performance-oriented search:** reverse-Dijkstra obstacle-aware 2D goal field, map/policy/goal revision caching (ADR-0027).
- [ ] Combine nonholonomic OMPL Reeds–Shepp distance with holonomic obstacle-grid distance by `max` (ADR-0027); expose unproven-admissibility fast mode and strict certified Euclidean/zero fallback; test nonzero goal tolerance and grid overestimation.
- [ ] Profile steering distance-only calls and heuristic cache behavior.

**Exit:** native steering passes analytic/oracle tests and obstacle heuristic passes grid fixtures.

## M4 — Hybrid A* baseline

- [ ] Node pool/open heap, stable indices, stale entry strategy, parent/action records.
- [ ] Generic search loop with primitive expansion and **required OMPL Reeds–Shepp near-goal analytic expansion** for geometric Ackermann (ADRs 0026–0028).
- [x] Accept Ackermann **0.25 m XY / 10° yaw + incoming gear** search key, reverse multiplier **1.10**, gear-switch **0.50 m**, clearance weight **0.15** inside **0.40 m**, and goal-region Euclidean heuristic (ADR-0025).
- [ ] Implement edge costs in equivalent meters: forward/reverse, gear-switch (no charge for first gear), and full-body clearance sampled at stored primitive states; apply the **same weighted costs to Reeds–Shepp sections/cusps including initial switch**. Tie breaks, reopen/dominance and **feasible-first validated analytic termination** (not an optimality guarantee).
- [ ] Cancellation and time/expansion/memory budgets, structured diagnostics.
- [ ] Exact path reconstruction and validated piecewise path output using canonical `(segmentIndex, lambda)` evaluation; complete state and one-sided cusp semantics.
- [ ] Validate continuous goal tolerance and complete sampled motions. For near-goal **OMPL Reeds–Shepp**, check every arc/cusp at ≤0.10 m, actual path length and weighted cost against configurable guard limits with difficult-maneuver exceptions (ADR-0027); on rejected connector continue ordinary search.
- [ ] Scenario tests (wall/doorway, narrow turns, reverse parking, unreachable goal; mecanum sideways travel and yaw-independent footprint constraints).
- [ ] Compare geometric raw versus **finally validated refined** path planning outcomes and total pipeline latency, including paths lost to single-label key pruning, without promising refinability of missing homotopy classes.
- [ ] Local latency/memory/quality baseline with and without obstacle-aware heuristic.

**Exit:** model-independent, validated baseline paths with measured performance.

## M5 — Path representation and optional refinement

- [ ] Segment-level continuous evaluation using canonical `(segmentIndex, lambda)`, derivative availability, progress/cusp/rotation semantics, and lateral holonomic motion without heading-as-tangent assumptions.
- [ ] Define external speed-profile input requirements: segment derivatives, rate/acceleration/jerk limits, start/end speeds and mandatory stop conditions (OQ-024).
- [ ] Validate final timed trajectory against model actuation limits and moving-obstacle constraints; reject/adjust/replan if a geometric path cannot be executed safely.
- [ ] Exact start/end full configurations and map/model provenance.
- [ ] Native curve math (B-spline / Hermite etc.) without mandating one family in API.
- [ ] Implement **Stanford Junior-style obstacle-aware optimization** (ADR-0028) with independently validated full-body output, preserving raw path fallback. Benchmark against Nav2-inspired simple smoothing in total pipeline latency and final quality; optional Ceres remains private (ADR-0024).
- [ ] Curvature, steering/articulation, endpoint and swept-collision validator.
- [ ] Independently validate model, endpoints and full-body collision of optimizer output under the selected sampled-assurance level; return untouched original path upon failed optimization, exhausted budget or collision (ADR-0024).
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
