# Open questions and architecture review gates

These items are intentionally **not** silently decided by API sketches. Mark a decision accepted in an ADR and update the related requirements/verification before implementing dependent modules.

## High priority — resolve before core interfaces

| ID | Question | Candidate choices / why it matters |
| --- | --- | --- |
| OQ-001 | **Resolved for baseline — accepted** | **One retained continuous node per model-defined discrete key**, retaining incoming primitive on node; no multi-label implementation in initial scope ([ADR-0013](../adr/0013-single-label-search-baseline.md), superseding ADR-0010). This deliberately sacrifices possible alternative arrivals; refine and benchmark end-to-end quality, without claiming optimality. |
| OQ-002 | **Resolved — accepted extent and generation principle** | Translating primitives use geometric traveled length; stationary configuration changes use model-specific angular/configuration extent (ADR-0012). **ADR-0023** selects RK4 generation, integrated traveled-length termination (terminal iterative refinement if needed), and stored full-state trajectory points for debugging/visualization; search reuses compatible precomputed primitives. **Still open:** exact typed extent/control-law interface, RK4/length solver tolerances and timing-dependent rate validation. |
| OQ-003 | **Resolved — accepted** | Canonical position is `(segmentIndex, lambda)` with `lambda in [0,1]`, evaluating complete model state; distance/time remain optional derived metrics ([ADR-0011](../adr/0011-canonical-segment-path-position.md)). **Interface detail open:** boundary canonicalization. |
| OQ-004 | What is the planning cost in consistent units? | With downstream path optimization intended (ADR-0020), choose a **minimal nonnegative search cost** sufficient for useful route/maneuver selection and justified admissible bounds, rather than an elaborate cost that refinement will overwrite. Candidate future extension: normalized kinematic effort plus geometric progress; dimension conversion and stationary costs need explicit rules. Do not label effort proxy as measured energy. |
| OQ-005 | **Resolved — accepted** | Use certified admissible cost-to-goal-region heuristic by default, with optional explicit weighted/experimental modes ([ADR-0015](../adr/0015-admissible-default-heuristics.md)). Grid reverse Dijkstra must be certified before use as a lower bound; a zero heuristic is valid fallback. Single-label state merging prevents unconditional global optimality/weighted bounds. |
| OQ-006 | **Resolved — accepted baseline** | ROS occupancy: `-1` unknown and `0..100` known. Per-request unknown `Occupied` (default for frontier), `Free`, `Reject` ([ADR-0017](../adr/0017-explicit-unknown-occupancy-policy.md)); known traversable iff `c <= maxTraversableOccupancy` (default `0`, [ADR-0018](../adr/0018-known-free-only-occupancy-threshold.md)). Immutable versioned full-grid shared snapshots with atomic publication, map-dependent cache reuse only for matching effective-input keys or rebuild on demand with bounded retention, and strict `Reject` prevalidation of **entire supplied map** accepted ([ADR-0019](../adr/0019-live-map-snapshot-cache-and-reject.md)). Exact native storage/ABI remains OQ-023 and shared mutable cache concurrency remains OQ-012. |
| OQ-007 | **Resolved — accepted** | Three-state geometric bicycle with fixed-curvature straight/left/right forward/reverse primitives, optional intermediate curvatures ([ADR-0014](../adr/0014-fixed-curvature-ackermann-baseline.md)). Raw joins do not guarantee finite steering rate; refiner/trajectory planner must separately validate if required. Four-state steering-rate model remains an optional extension. |
| OQ-008 | **Resolved — accepted** | Default pose-only goal: Euclidean XY <= 0.25 m, wrapped yaw <= 10 degrees, independent configurable tolerances; test continuous endpoint and full motion validity ([ADR-0016](../adr/0016-default-pose-goal-tolerances.md)). Extra state variables unconstrained unless explicitly required, no snapping; heuristic bounds cost to region. |

## Next priority — resolve with implementation milestones

| ID | Question | Candidate choices / why it matters |
| --- | --- | --- |
| OQ-009 | **Partly resolved — MVP sampled collision baseline accepted** | At each evaluated configuration, exact geometric shape-vs-occupancy-cell intersection (not polygon point sampling) for circles, rectangles and convex polygons. Fixed trajectory sampling **by default**; additional adaptive body-motion subdivision is a **request option default `false`** ([ADR-0022](../adr/0022-opt-in-adaptive-collision-subdivision.md)). Both are approximate swept validation. **Open:** precise fixed-step default/units, optional adaptive thresholds/budgets and future conservative continuous-collision assurance. |
| OQ-010 | Public runtime polymorphism | Static concept-backed model solver + thin type erased facade vs runtime virtual model |
| OQ-011 | Steering cost/termination | How to cost analytic connectors and when is it safe to stop search? |
| OQ-012 | Replanning and cache concurrency | Map revisions can be published atomically and read safely by concurrent requests (ADR-0019). Still choose planner-instance concurrency rule, mutable lazy-cache synchronization/per-request scratch ownership, cancellation and safe eviction under load. |
| OQ-013 | **Partly resolved — obstacle awareness required** | Downstream optimization (when enabled) must use static-map obstacle/clearance information **during** the optimization, not just as a final reject test; use the same pinned map/policy and **all** state-dependent collision bodies. Independently validate the final result and retain the original on failure ([ADR-0024](../adr/0024-obstacle-aware-path-optimization.md)). **Open:** numerical solver, clearance representation, objective weights, continuous/sampled validation assumptions, and curve-vs-control parameterization. A missed route cannot be presumed recoverable by refinement. |
| OQ-014 | **Partly resolved — accepted multibody design** | Collision geometry is independent of kinematics: immutable frame-attached circle, rectangle and convex-polygon bodies; compounds as collections, model supplies state-dependent named frame transforms, including arbitrary-length trailer chains ([ADR-0021](../adr/0021-independent-multibody-collision-geometry.md)). **Open:** exact `FrameId` validation/representation, self-collision pair/exclusion policy and detailed collision checker contracts (OQ-009). |
| OQ-015 | Dependency versions and packaging | Eigen, Catch2, cxxopts, nlohmann/json, yaml-cpp, Ceres optional; vendoring vs package manager |
| OQ-016 | Exact performance target | Reference CPUs, maps, latency percentiles, memory budgets, allocation constraints |
| OQ-017 | ROS adapter boundary | Native path-to-message losslessness, metadata and explicit cusp/rotation support |
| OQ-018 | Thread/cancellation API | Cancellation token granularity, planner scratch ownership and safe shutdown |
| OQ-019 | Motion capabilities | How descriptive capability metadata complements (but never substitutes for) model-defined admissible controls ([ADR-0008](../adr/0008-motion-model-capabilities.md)) |
| OQ-020 | Mecanum milestone | When to implement the third reference model and whether it must complete end-to-end search for M4 |
| OQ-021 | **Partly resolved — model/primitive boundary accepted** | Model supplies state, control, tangent/derivative, validity, reference pose and state-dependent rigid-body frame transforms (ADR-0021). A separate RK4 generator computes endpoints, retains sample checkpoints, and exposes evaluable motions to search (ADR-0023). **Open:** exact manifold tangent/update operations inside generator, discrete modes, timing/progress API and concrete C++20 signatures; no search-facing `integrateIncrement()` requirement. |
| OQ-022 | Supplied articulated-steering geometry and sign conventions | **Confirmed:** `(x,y,Theta)` is the rear-chassis reference pose. **Still open:** exact rear-reference-point location, whether inputs are rear-reference speed and articulation rate, positive sign of `gamma`, physical meaning of `l1`/`l2`, front/joint frame transforms and valid joint range ([model spec](kinematic-models.md)) |
| OQ-023 | Native occupancy/frame representation | REP-103/105 and grid layout accepted (ADR-0009), known-cell occupancy threshold/default accepted (ADR-0018); still choose signed occupancy versus typed cell classification, application-specific encodings, malformed-value policy and frame ID API ([map coordinates](map-coordinates.md)) |
| OQ-024 | Speed-profile and trajectory boundary | The global planner returns geometry only; trajectory planning/MPC owns execution timing. **Proposed extension:** precompute model-derived *kinematic effort* catalogs from actuator-coordinate motion demands and primitive transitions at a declared reference speed, avoiding runtime speed/effort equations. This is **not** electrical energy or guaranteed traversal time. Open: reference-speed feasibility, normalized control/rate measures, smoothing/connector assumptions, objective scale, state-dependent lookup bins, arrival-key semantics and catalog validation. |

## Review checklist for architecture baseline v0.1

- [ ] Cross-check requirements against intended supported vehicles and controller boundary.
- [ ] Agree which constraints are enforced by raw search and which by optional refinement/MPC.
- [x] Adopt OQ-001 one-node-per-model-specific-discrete-key baseline (ADR-0013); multi-label feature deferred rather than implemented.
- [x] Accept OQ-002 translating-length and stationary angular/configuration extents (ADR-0012).
- [x] Accept RK4-built geometric primitives, iterative traveled-length termination where needed, stored complete-state debug/visualization points, and search-time reuse (ADR-0023).
- [ ] Detail OQ-002 typed extent/control-law API and numeric tolerances, state-dependent primitive cache assumptions and retiming validity; discuss OQ-024 speed-profile boundary.
- [x] Accept OQ-003 canonical path address `(segmentIndex, lambda)` (ADR-0011).
- [x] Accept OQ-005 certified-admissible heuristic mode as default (ADR-0015); relaxations explicit.
- [x] Accept OQ-007 fixed-curvature 3-state Ackermann baseline (ADR-0014).
- [x] Accept OQ-008 pose-only goal region with default XY/yaw tolerances (ADR-0016).
- [ ] Resolve OQ-004 kinematic effort-cost objective; OQ-006 baseline map ownership/cache/Reject decisions are accepted (ADRs 0017–0019). Final native cell/frame/handle ABI remains OQ-023; mutable cache and planner concurrency remain OQ-012.
- [ ] Confirm primitive and path parameter units, reversing cusp and rotation semantics.
- [ ] Confirm how heuristic certification and completeness/optimality claims are worded.
- [ ] Agree minimal compile-time policy concepts and public result/error types.
- [x] Accept independent frame-attached multibody geometry and minimal model responsibilities, extensible to trailers (ADR-0021).
- [x] Accept OQ-009 complete-shape vs occupied-cell testing, fixed nonadaptive sampled default and optional adaptive subdivision disabled by default (ADR-0022).
- [x] Accept OQ-013 obstacle-aware optimizer requirement on pinned map/policy and entire vehicle geometry, with independent validation and unchanged raw-path fallback (ADR-0024); keep solver/cost/parameterization details open.
- [ ] Validate ODE/tangent/integrator contracts against differential drive, Ackermann and ideal mecanum (OQ-021); settle remaining concrete signatures.
- [ ] Ensure search and path interfaces never assume that body heading equals travel tangent.
- [ ] Confirm the supplied articulated-steering input and body-frame conventions (OQ-022) before implementing collision transforms.
- [x] Accept per-request unknown-cell policy with conservative `Occupied` frontier default (ADR-0017).
- [x] Accept default known-free-only occupancy threshold `0` with explicit boundary comparison (ADR-0018).
- [x] Accept immutable shared full-grid snapshots with atomic publishing, revision-keyed lazy cache building and bounded retention, and full-map deterministic `Reject` validation (ADR-0019).
- [ ] Confirm ROS-compatible map/grid Catch2 contracts (ADR-0009), select native occupancy storage and final frame/handle ABI (OQ-023), and refine concurrent planner-cache synchronization (OQ-012).
- [ ] Approve verification categories and first benchmark fixtures.
- [ ] Record decisions as accepted ADRs before implementation depends on them.
