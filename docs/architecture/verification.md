> **Current implementation precedence (2026-10-26):** This file preserves the baseline design history/proposal. Where defaults, collision assurance, Ackermann integration, heuristics or RS scheduling differ, follow the [current implementation contract](current-implementation.md) and its work-package statuses. The [consolidated external reviews](Hybrid_AStar_Consolidated_Specification.md) are proposed design guidance, **not** an already approved ADR or evidence of completed features.

# Verification strategy — draft v0.1

Verification starts with the contract, not after the feature. Use **Catch2 v3**, CTest and deterministic test inputs. Run tests and benchmarks locally. GitHub Actions must not execute automatically; no workflow is required for the documentation baseline.

## Verification levels

| Level | Objective | Example |
| --- | --- | --- |
| Compile-time contracts | Reject incompatible policy interfaces | `static_assert(KinematicModel<...>)` and deliberate negative compile tests |
| Unit / analytic tests | Verify math and small rules in isolation | Closed-form differential/Ackermann integration, body transforms |
| Property/metamorphic tests | Verify invariants independent of another implementation | Translation/rotation covariance, round-trip, radius scaling |
| Steering adapter tests | Verify **OMPL-backed** Dubins/Reeds–Shepp endpoints, segments, cusps and interpolation; future native replacement can be compared independently (ADR-0028) |
| Integration tests | Verify cross-module contracts | One-map snapshot, keys, cost, full collision validation, exact reconstructed edge |
| Scenario tests | Verify behavior under representative environments | Dead ends, narrow turns, multi-point turns, rotate in place |
| Performance tests | Detect regressions with reproducible scenarios | Latency percentiles, allocation and peak-memory measurements |
| Contract/architecture checks | Standalone CMake build without ROS/Ceres in generic core; OMPL is a **scoped steering runtime dependency** and must not leak through public model/search/path types (ADR-0028) |

## Requirements-to-tests traceability

| IDs | Required evidence |
| --- | --- |
| FR-001, FR-003, FR-016 | Model concept compile tests for differential, Ackermann and mecanum; derivative and integrator checks; initial Ackermann **(0.25 m XY, **5° yaw/72 bins**, gear)** index with **one continuous node per complete key**, distinct forward/reverse arrivals, reopen/stale queue and pruning approximation tests; future articulation-key distinctions (ADR-0025). |
| FR-002, FR-017 | Accepted initial Ackermann primitives: **0.50 m traveled length**, six controls (forward/reverse × left/straight/right), RK4 **0.05 s** step at reference **1 m/s**, endpoint arc-length tolerance **0.0001 m**, full-state samples at **0.10 m** including endpoints (**six samples**); also test iteration/failure/convergence, analytic arc oracle, future stationary/holonomic models and cache validity (ADRs 0023,0025). |
| FR-004, FR-005 | Precomputed conservative all-heading hull collision-possibility mask; safe lookup bypasses detailed checks only when all possible pose/heading/quantization displacements are covered, flagged pose uses **supercover raster polygon boundaries by default**; optional `checkFootprintInterior=true` enables filled-interior blocked-cell checks. Validate contained obstacles/corner grazes, unknown/threshold/out-of-bounds/map revisions; reuse 0.10 m primitive samples and ≤0.10 m RS segment samples, default boundary-only mode can miss enclosed obstacles and must report that limitation; neither mode certifies continuous sweeps (ADR-0029). |
| FR-006, FR-007, FR-018 | REP-103/105 frames, rotated origin conversion, cell-center offsets, row-major indexing, unknown/out-of-bounds semantics, Dijkstra known maps, lazy settlement, cache invalidation |
| FR-008 | OMPL Reeds–Shepp analytic adapter and adaptive scheduler: **first expanded non-stale node always attempts a complete connection irrespective of XY distance**, ratio 3.5 provisional scheduling in map-cell units, decreasing interval toward goal, reset each plan(), one complete signed/cusp path per attempt, failed-connector ordinary-search continuation, ≤0.10 m samples, optional cost cap and no hard length cap (ADRs 0026–0028). |
| FR-009, FR-015 | Stale queue labels, reopen rules, budgets/cancellation and structured diagnostic status |
| FR-010, FR-012 | Exact primitive reconstruction, analytic tail, cusp and rotation progress; `(segmentIndex, lambda)` evaluations, full state, shared boundary pose and optional one-sided derivatives |
| FR-011 | Obstacle-aware optimization **during** candidate improvement on same pinned map/policy, full-body clearance for rotated and articulated/trailer bodies, obstacle-cutting shortcut rejection or adjustment, independent final swept collision/model/boundary validation, unchanged raw feasible fallback on failure; distinguish non-certified sampled assurance (ADR-0024) |
| FR-013, FR-014 | YAML/JSON roundtrip/validation in adapters, no ROS dependencies in core |
| FR-019 | XY <= 0.25 m and wrapped yaw <= 10 degrees defaults, independent threshold equality, zero-tolerance mode, model-specific optional state constraints, continuous endpoint check and valid swept motion |
| FR-020 | Fixed-curvature three-state Ackermann exact arcs and reversals; steering rate discontinuity explicitly reported, and optional refiner tested for valid smooth connector rather than assumed valid |
| FR-021 | **Required** combined obstacle-free RS + obstacle-aware reverse-Dijkstra heuristic; compare expansion counts and output quality with Euclidean, show raw components can overestimate a continuous **goal-region** lower bound, disclose unproven-admissibility performance mode and strict certified fallback (ADRs 0015,0027). |
| FR-022 | Three per-request unknown policies over identical immutable snapshots; default `Occupied`, optimistic `Free` diagnostics, strict `Reject` fails upfront on unknown anywhere in complete map, full-footprint frontier traversability; known-cell threshold `c <= max`, default max `0`, boundary tests |
| NFR-001..012 | See [quality protocol](quality.md); scenario baselines and metadata stored with results |

## Single-label search and canonical path contract tests

- Verify exactly **one retained continuous node per model-defined discretized key**. Check replacement, stable ancestry, deterministic tie-breaking, heap staleness and reopening; record compact memory and per-expansion allocations (ADR-0013). Multi-label infrastructure is out of scope for the baseline.
- Test arrivals with different continuous states or incoming primitives that map to the same **complete** key. Ackermann forward and reverse modes form **different keys**, while initial no-gear state incurs no switch penalty. Document lost continuous-arrival alternatives and show that cheaper accumulated `g` does **not** establish true dominance if important history remains unindexed (ADR-0025).
- Compare raw and optional refined paths in narrow passages and maneuver-heavy fixtures. Refinement may improve an existing path but may not recover a pruned homotopy class.
- Test path evaluation at `(i,0)`, `(i,1)`, interior lambda, adjacent segment interfaces, cusp/gear changes, pure stationary rotation, articulation-only movement, and holonomic lateral translation. Distinguish one-sided derivative conventions from continuity of configuration.
- Check accepted length-parametrized translating primitives against time-domain RK4 integration of *accumulated traveled length* with an explicit tolerance and iterative endpoint refinement when speed is variable, including reverse and zero-speed stall/nontermination; stationary angular/configuration motions must never divide by zero or use an arc-length-only solver (ADRs 0012,0023). Compare integration error versus RK4 step and an independent analytic Ackermann arc oracle. Test rate-limited retiming separately (OQ-024 remains open).
- Every primitive stores deterministic ordered full-state trajectory samples (first/last match generated states) for visualization/debug, with monotone local progress. **Approved Ackermann: six states at arc lengths 0.0, 0.1, 0.2, 0.3, 0.4, 0.5 m; default collision checks use these exact stored states.** Evaluate between samples by the retained numerical motion representation when required by path evaluation or **explicitly enabled** adaptive subdivision, never assume straight-line interpolation of search endpoints is dynamically feasible. Check `evaluate(0/1)`, SE(2) template instantiation and future model-specific cache validity (ADR-0025).

## Dubins and Reeds–Shepp battle-hardening

- Use **OMPL via a scoped steering adapter** for first MVP; validate signed segment extraction, interpolation, path length, endpoints and cusp metadata independently. Native solver is an optional low-priority follow-up (ADR-0028).
- Build steering adapter with OMPL as declared runtime dependency; test solver behavior directly and keep all OMPL types out of generic core public interfaces (ADR-0028).
- Fixed-seed randomized pairs plus hand-crafted edge cases: same pose, close poses, headings near ±pi, extreme turning radii, collinear/cusp solutions, reverse symmetry.
- Compare **lengths within mixed absolute/relative tolerance**, reconstructed endpoints, signed segments and interpolation. Different equal-cost families are not failures.
- Include random transformations / scaling and independent analytic invariants to detect shared oracle mistakes.
- Persist numerical failures with seed, poses, radius, OMPL adapter output and independent expected values in deterministic regression fixtures.
- Track distance-only evaluation and full path/interpolation separately in benchmarks.
- **Adaptive analytic MVP acceptance (ADRs 0026–0028):** verify first-node attempt at any distance (including yaw mismatch), deterministic retry countdown controlled by closest finite heuristic/map resolution, approach-driven increased frequency, invalid ratio/heuristic handling, exactly one complete path per attempt, reset between requests and no gate based on XY separation. Sample every segment at ≤0.10 m incl cusps; validate collision, incoming/internal gear switches, reverse multiplier, exact endpoint and full parent chain. Long collision-free RS paths must not be rejected solely for length. Optional weighted-cost gate rejects only when enabled; any failed attempt continues primitive expansion.

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

- **MVP nearest-obstacle EDT (ADR-0029):** test distance to blocked-cell centers, conservative `R+sqrt(2)*r` collision flags, unknown policy, map revision, rotated origins and out-of-map boundaries. Confirm **no 0.40 m clearance mask, oriented-footprint EDT scoring or clearance integral is required** in first-implementation primitive/RS edge costs. Preserve optional future tests for footprint boundary `d_hat`, 0.15/0.40 m penalty and five-interval trapezoidal quadrature in the low-priority backlog.
- Benchmark nearest-obstacle EDT precompute/cache reuse for the collision mask. Leave clearance-scoring runtime and comparisons with the 0.15 penalty to the optional future feature. **Never** confuse nearest-obstacle EDT with goal-rooted reverse-Dijkstra.

- State-level tests: independent immutable circles, rectangles, convex polygons and multi-shape compounds, local offsets, rotation, articulated state-dependent frames, tractor with two trailers, and invalid frame/degenerate polygon rejection (ADR-0021).
- Raster state tests: blocked cell completely enclosed by polygon even when all boundary cells are free (must be detected with **interior option on**, may be missed with **default boundary-only** and flagged as weaker assurance), conservative grid supercover edges and touching corners, circle/compound coverage, rotated grid, unknown/threshold/out-of-bounds semantics; compare against exact shape–cell geometry as an **independent test oracle only** (ADR-0029).
- Motion tests: intersecting a thin obstacle between safe endpoints; in-place rotation clipping corners; articulation sweeping without reference-point movement; mecanum sideways rectangular-footprint clearance at fixed yaw.
- **Initial default collision behavior (ADR-0029):** for each 0.50 m Ackermann primitive check all six stored poses; at each pose consult precomputed collision-candidate mask (or fall back if missing/stale), then rasterized polygon edges for flagged poses, plus interior cell checks **only if enabled by `checkFootprintInterior`**; no extra search nodes or independent fixed schedule.
- Articulated-steering ODE tests: author-supplied four-state model, denominator guard, finite steering range, nonzero heading change at zero reference-point speed, closed joint-only cycles and frame transforms after conventions are confirmed.
- Property tests: static obstacle map immutability across plan; collision results invariant under paired global transforms where map transform is exact.
- Record independent dimensions of verification: **boundary-only vs full-footprint sampled coverage** and **primitive vs adaptive trajectory sampling**, or a separately proven future continuous mode; including **primitive sample spacing (0.10 m for initial Ackermann)**, counts and relevant limits. Neither primitive-only nor adaptive sampling alone may be labeled formally continuously collision-free (ADRs 0022,0025).
- For obstacle-aware optimization (ADR-0024), create a path whose naïve smoothing/shortcut crosses a wall or unknown-as-occupied cell. Verify environment information affects optimization-stage candidate selection/constraints, **not just a post-hoc rejection**. Test that reference-point-only clearance cannot admit a colliding rectangle or trailer, and confirm the optimizer pins the same snapshot and unknown/threshold policy despite a concurrently published revision. An infeasible/budget-exhausted refinement must return the **untouched** validated raw path.
- Independently validate final refined path under the chosen coverage option against the original map snapshot and model constraints, recording that **default boundary-only coverage cannot establish full-footprint collision validity**, alongside fixed/adaptive trajectory sampling. Obstacles/clearance costs alone do not certify safety.

## Heuristic correctness and guarantees

- Tiny hand-computed **multi-source** reverse-Dijkstra maps (free goal-tolerance region, blocked nominal goal center, empty seed set) and separate kinematic OMPL RS lookup; verify graph edge units, 8-neighbor corner policy, map revision, yaw-bin guards and goal-tolerance effects.
- **One-request-per-instance concurrency (ADR-0019):** verify overlapping `plan()` calls on one planner instance fail fast with Busy/ConcurrentRequest; later sequential requests resume the same compatible lazy goal-rooted field. Two independent planner instances can plan on the same pinned immutable map concurrently without sharing mutable Dijkstra data.
- **MVP lazy cache tests:** query cells incrementally; each must return a shortest finite value **only after being settled**, not from tentative heap entries. Cache and resume the **remaining frontier** across multiple queries and compatible replans to the same effective XY goal seed set, including a new start/goal yaw. On changed goal XY seeds, map revision or occupancy/graph settings, create a fresh field. Enforce finite work budgets with explicit unavailable/fallback status; mark disconnected cells as unreachable only when the queue is exhausted. Ensure concurrent plan calls cannot mutate the same lazy heap unsafely.
- Verify heuristic cache invalidates on map identity/revision, goal *region*, inflation/threshold/unknown policy and grid geometry, including changed origin yaw or frame ID. A policy-only change must not reuse policy-dependent cached traversability/heuristic data. A snapshot revision stays fixed during a request even if a new revision becomes available.
- **Accepted live-map contract tests (ADR-0019):** publish R+1 while two requests read R; old occupancy/metadata remain unchanged, latest-handle handoff is atomic; model caches survive map publication; map-derived cache reuses only effective-input-compatible keys, with bounded eviction and lazy-on-miss rebuild; strict `Reject` fails during deterministic full-map prevalidation, independent of search/heuristic order. Record map identity/revision in path results, enforce release of old snapshot lifetimes, and test replay/revision-reset mismatches. See [accepted ADR](../adr/0019-live-map-snapshot-cache-and-reject.md).
- For any declared *admissible* heuristic mode, compare `h` to a proven lower bound or analytically solvable case. Do not infer admissibility solely because reverse Dijkstra runs.
- Test consistency only for modes where consistency is claimed; otherwise ensure reopen behavior handles inconsistent heuristics.
- Compare obstacle-aware heuristic on/off for performance **and** solution quality; no artificial optimality claims.
- For a blocked goal-center cell with free XY region overlap, verify **multi-source seed** behavior and eventual continuous goal-region acceptance including heading/whole-body collision. Distinguish `Unvisited`, `Unsettled`, and proven grid-disconnected cells, and verify none automatically yield an infinite Hybrid A* `f` score or incorrectly terminate search. If no 2D seed is available, retain safe finite fallback, report diagnostic and rely on normal search/terminal validation.
- Test six primitive control successors with XY 0.25 m and **72 heading bins** at multiple subcell states/turn radii; check aliasing warnings independently from the goal's unchanged **10° tolerance**.
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
