# Verification strategy — draft v0.1

Verification starts with the contract, not after the feature. Use **Catch2 v3**, CTest and deterministic test inputs. Run tests and benchmarks locally. GitHub Actions must not execute automatically; no workflow is required for the documentation baseline.

## Verification levels

| Level | Objective | Example |
| --- | --- | --- |
| Compile-time contracts | Reject incompatible policy interfaces | `static_assert(KinematicModel<...>)` and deliberate negative compile tests |
| Unit / analytic tests | Verify math and small rules in isolation | Closed-form differential/Ackermann integration, body transforms |
| Property/metamorphic tests | Verify invariants independent of another implementation | Translation/rotation covariance, round-trip, radius scaling |
| Differential tests | Compare native steering against a known oracle | Optional OMPL Dubins/Reeds–Shepp test target |
| Integration tests | Verify cross-module contracts | One-map snapshot, keys, cost, full collision validation, exact reconstructed edge |
| Scenario tests | Verify behavior under representative environments | Dead ends, narrow turns, multi-point turns, rotate in place |
| Performance tests | Detect regressions with reproducible scenarios | Latency percentiles, allocation and peak-memory measurements |
| Contract/architecture checks | Prevent dependency leakage | Standalone CMake build without ROS, OMPL or Ceres |

## Requirements-to-tests traceability

| IDs | Required evidence |
| --- | --- |
| FR-001, FR-003, FR-016 | Model concept compile tests for differential, Ackermann and mecanum; derivative and integrator checks; model-aware key distinction for articulation and other relevant dimensions; **single-node-per-key** replacement, reopen/stale-queue and pruning approximation tests |
| FR-002, FR-017 | RK4 primitive generation with accumulated traveled-length termination (including iterative final-step refinement), ordered complete-state visualization/debug samples and path evaluation between sample knots (ADR-0023); compare analytic exact-circle solutions, reverse travel, zero-motion articulation/rotation and coupled holonomic motion; test limit/step convergence and state-dependent catalog invalidation |
| FR-004, FR-005 | Independent immutable frame-attached circles/rectangles/convex polygons (ADR-0021); geometric **whole-shape/cell** overlap including obstacle fully enclosed by body; fixed nonadaptive samples by default; opt-in adaptive subdivision `false` until enabled (ADR-0022); multi-trailer and in-place-articulation cases; unknown frame/geometry rejection; sampled-only collision assurance; self-collision pair rules still OQ-014 |
| FR-006, FR-007, FR-018 | REP-103/105 frames, rotated origin conversion, cell-center offsets, row-major indexing, unknown/out-of-bounds semantics, Dijkstra known maps, lazy settlement, cache invalidation |
| FR-008 | Dubins/Reeds–Shepp endpoint, length, segment direction and symmetries; randomized OMPL comparison |
| FR-009, FR-015 | Stale queue labels, reopen rules, budgets/cancellation and structured diagnostic status |
| FR-010, FR-012 | Exact primitive reconstruction, analytic tail, cusp and rotation progress; `(segmentIndex, lambda)` evaluations, full state, shared boundary pose and optional one-sided derivatives |
| FR-011 | Obstacle-aware optimization **during** candidate improvement on same pinned map/policy, full-body clearance for rotated and articulated/trailer bodies, obstacle-cutting shortcut rejection or adjustment, independent final swept collision/model/boundary validation, unchanged raw feasible fallback on failure; distinguish non-certified sampled assurance (ADR-0024) |
| FR-013, FR-014 | YAML/JSON roundtrip/validation in adapters, no ROS dependencies in core |
| FR-019 | XY <= 0.25 m and wrapped yaw <= 10 degrees defaults, independent threshold equality, zero-tolerance mode, model-specific optional state constraints, continuous endpoint check and valid swept motion |
| FR-020 | Fixed-curvature three-state Ackermann exact arcs and reversals; steering rate discontinuity explicitly reported, and optional refiner tested for valid smooth connector rather than assumed valid |
| FR-021 | Certified lower bounds for goal region/edge cost, explicitly relaxed heuristic modes, and no unproven optimality guarantee from quantized-key Hybrid A* |
| FR-022 | Three per-request unknown policies over identical immutable snapshots; default `Occupied`, optimistic `Free` diagnostics, strict `Reject` fails upfront on unknown anywhere in complete map, full-footprint frontier traversability; known-cell threshold `c <= max`, default max `0`, boundary tests |
| NFR-001..012 | See [quality protocol](quality.md); scenario baselines and metadata stored with results |

## Single-label search and canonical path contract tests

- Verify exactly **one retained continuous node per model-defined discretized key**. Check replacement, stable ancestry, deterministic tie-breaking, heap staleness and reopening; record compact memory and per-expansion allocations (ADR-0013). Multi-label infrastructure is out of scope for the baseline.
- Test arrivals with different continuous states or incoming primitives that map to the same key. Document lost alternatives and show that cheaper accumulated `g` does **not** establish true dominance when future motion/effort depends on arrival history.
- Compare raw and optional refined paths in narrow passages and maneuver-heavy fixtures. Refinement may improve an existing path but may not recover a pruned homotopy class.
- Test path evaluation at `(i,0)`, `(i,1)`, interior lambda, adjacent segment interfaces, cusp/gear changes, pure stationary rotation, articulation-only movement, and holonomic lateral translation. Distinguish one-sided derivative conventions from continuity of configuration.
- Check accepted length-parametrized translating primitives against time-domain RK4 integration of *accumulated traveled length* with an explicit tolerance and iterative endpoint refinement when speed is variable, including reverse and zero-speed stall/nontermination; stationary angular/configuration motions must never divide by zero or use an arc-length-only solver (ADRs 0012,0023). Compare integration error versus RK4 step and an independent analytic Ackermann arc oracle. Test rate-limited retiming separately (OQ-024 remains open).
- Every primitive stores deterministic ordered full-state trajectory samples (first/last exact generated states) for visualization/debug, with monotone local progress. Evaluate between samples by the retained numerical motion representation, not assumed linearly feasible state interpolation. Check `evaluate(0/1)`, instantiation at rotated/transformed SE(2) starts, collision-checker extra samples independent from stored debug density, and map-independent cache reuse vs articulated initial-state differences.

## Dubins and Reeds–Shepp battle-hardening

- Implement native solvers independently; *no OMPL source in runtime*.
- Optional CMake switch e.g. `HYBRID_ASTAR_TEST_WITH_OMPL=ON` enables only a differential test executable. Default OFF.
- Fixed-seed randomized pairs plus hand-crafted edge cases: same pose, close poses, headings near ±pi, extreme turning radii, collinear/cusp solutions, reverse symmetry.
- Compare **lengths within mixed absolute/relative tolerance**, reconstructed endpoints, signed segments and interpolation. Different equal-cost families are not failures.
- Include random transformations / scaling and independent analytic invariants to detect shared oracle mistakes.
- Persist any failure's seed, poses, radius, native/OMPL outputs in a deterministic regression case.
- Track distance-only evaluation and full path/interpolation separately in benchmarks.

## Map coordinate and frame verification

- Index fixture: `(0,0)` maps to 0, `(1,0)` to 1, `(0,1)` to `width`, with no Y-axis image flip.
- Full SE(2) origin transform (zero and nonzero yaw), lower-left cell origin and half-cell center offset.
- Negative, upper-bound, rotated-grid and non-finite coordinates rejected with mathematically correct floor.
- Conventional signed occupancy `-1` unknown and `0..100` probability encoding; **accepted default threshold `maxTraversableOccupancy = 0`**: `0` free, `1..100` blocked; compare as `c <= max`, including equality at `max=20` and blocked `21`. Unknown `-1` bypasses comparison. Validate malformed values, out-of-bounds and no raw Nav2 costmap reinterpretation.
- Under default `UnknownCellPolicy::Occupied`, reject candidate swept motions touching unknown space, including footprint corners outside known-free cells; allow reachable goal points near the known-free frontier. `Free` permits unknown traversal but must report optimistic environmental assurance; `Reject` fails request validation if any `-1` occurs **anywhere** in the supplied grid, including well outside eventual search branches, before any heuristic or collision expansion; return a dedicated unknown-occupancy error, not `NoPath` (ADR-0019). Switching options must not mutate the snapshot.
- Reject mismatched map/start/goal frames; optional ROS adapter performs transforms first.
- Outdoor ENU checks: x increases east, y increases north, yaw 0 points east, and yaw +pi/2 points north; grid origin orientation tested independently.
- Optional geodetic adapter tests: explicit latitude/longitude/altitude reference and datum, known GNSS-to-ENU fixtures, and missing/mismatched-origin rejection.
- See [map-coordinate contract tests](map-coordinates.md).

## Collision verification

- State-level tests: independent immutable circles, rectangles, convex polygons and multi-shape compounds, local offsets, rotation, articulated state-dependent frames, tractor with two trailers, and invalid frame/degenerate polygon rejection (ADR-0021).
- Whole-shape state tests: occupied cell completely enclosed by polygon with no boundary/vertex contact; polygon-cell corner and edge intersections; rectangle/polygon containment both ways, touching edges, circle-cell tangencies, rotated grid frames, unknown/occupied policies. Do **not** treat vertex-only or perimeter-only occupancy sampling as geometric collision validity.
- Motion tests: intersecting a thin obstacle between safe endpoints; in-place rotation clipping corners; articulation sweeping without reference-point movement; mecanum sideways rectangular-footprint clearance at fixed yaw.
- Default sampling mode uses configured fixed **start/end/intermediate** state schedule and no adaptive recursion. With `adaptiveSubdivision=true`, verify added evaluations for rotating long bodies/trailers and for changing articulation at fixed reference point; enforce subdivision budget and return incomplete-validation rather than success on budget exhaustion. Never infer safe motion from matching endpoint transforms alone.
- Articulated-steering ODE tests: author-supplied four-state model, denominator guard, finite steering range, nonzero heading change at zero reference-point speed, closed joint-only cycles and frame transforms after conventions are confirmed.
- Property tests: static obstacle map immutability across plan; collision results invariant under paired global transforms where map transform is exact.
- Record whether verification is **fixed sampled**, **adaptive sampled**, or (future) **conservatively continuous**, including configured sample resolution, counts and relevant limits. Neither fixed nor adaptive sampling alone may be labeled formally continuously collision-free (ADR-0022).
- For obstacle-aware optimization (ADR-0024), create a path whose naïve smoothing/shortcut crosses a wall or unknown-as-occupied cell. Verify environment information affects optimization-stage candidate selection/constraints, **not just a post-hoc rejection**. Test that reference-point-only clearance cannot admit a colliding rectangle or trailer, and confirm the optimizer pins the same snapshot and unknown/threshold policy despite a concurrently published revision. An infeasible/budget-exhausted refinement must return the **untouched** validated raw path.
- Independently validate final refined path against original map snapshot, full model geometry and constraints, with the declared fixed/adaptive sampled assurance level. Obstacles/clearance costs alone do not certify safety.

## Heuristic correctness and guarantees

- Tiny hand-computed maps for Dijkstra values, lazy vs eager equivalence, unreachable goals and unknown-cell policies.
- Verify heuristic cache invalidates on map identity/revision, goal *region*, inflation/threshold/unknown policy and grid geometry, including changed origin yaw or frame ID. A policy-only change must not reuse policy-dependent cached traversability/heuristic data. A snapshot revision stays fixed during a request even if a new revision becomes available.
- **Accepted live-map contract tests (ADR-0019):** publish R+1 while two requests read R; old occupancy/metadata remain unchanged, latest-handle handoff is atomic; model caches survive map publication; map-derived cache reuses only effective-input-compatible keys, with bounded eviction and lazy-on-miss rebuild; strict `Reject` fails during deterministic full-map prevalidation, independent of search/heuristic order. Record map identity/revision in path results, enforce release of old snapshot lifetimes, and test replay/revision-reset mismatches. See [accepted ADR](../adr/0019-live-map-snapshot-cache-and-reject.md).
- For any declared *admissible* heuristic mode, compare `h` to a proven lower bound or analytically solvable case. Do not infer admissibility solely because reverse Dijkstra runs.
- Test consistency only for modes where consistency is claimed; otherwise ensure reopen behavior handles inconsistent heuristics.
- Compare obstacle-aware heuristic on/off for performance **and** solution quality; no artificial optimality claims.
- In the default mode, verify only certified goal-region/cost-compatible bounds are enabled; use `h=0` or proven Euclidean-to-tolerance-region fallback when reverse-grid field cannot be certified. Weighted and uncertified modes must report explicit guarantees/limitations.
- Validate pose-goal tolerance region including angular wrap, tolerance-border equality, optional joint target, and a colliding nominal center with a valid endpoint elsewhere inside the region.

## Planned test layout

```text
tests/
  contracts/
  math/
  kinematics/
  motion/
  steering/            # native + optional OMPL
  heuristics/
  collision/
  search/
  path/
  refinement/
  adapters/
  scenarios/
benchmarks/
  scenarios/
  fixtures/
```

## Test acceptance gates

- [ ] Define tolerances and numerical precision policy; include NaN/Inf rejection.
- [ ] Define model-state/controls property generators and reproducible seeds.
- [ ] Define map/robot fixture serialization independent of ROS, preserving ROS-compatible origin, yaw, frame ID, row-major storage and signed occupancy unknowns.
- [ ] Implement compile-time model policy checks for **three** reference chassis: differential drive, Ackermann and ideal mecanum.
- [ ] Independently validate derivatives, tangent/angle-state integration, combined translation/rotation and model-admissible control limits.
- [ ] Derive and test the supplied articulated-steering system's reference point, sign conventions, pure articulation motion and state-dependent collision geometry.
- [ ] Run a common search/primitive/path contract test suite with all three models, without drive-specific branches in the search kernel.
- [ ] Measure compact single-label key/node memory, allocations and expansion timing, and success/refinement quality under coarse versus fine state discretization; do not implement multi-label storage.
- [ ] Verify canonical `(segmentIndex, lambda)` path evaluation across cusps, zero-distance maneuvers and model-specific segment types.
- [ ] Exercise fixed-curvature Ackermann primitive families and optional valid steering-continuity refinement, default certified heuristics and continuous pose-tolerant goals.
- [ ] Enable optional OMPL tests without adding an OMPL core dependency.
- [ ] Validate failure results and fallback behavior.
- [ ] Add sanitizer runs (ASan/UBSan, optionally TSan) in local developer instructions.
- [ ] Produce performance baselines on named hardware before setting regression thresholds.
- [ ] Review requirement-to-test matrix at each implementation milestone.
