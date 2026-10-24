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
| FR-001, FR-003, FR-016 | Model concept compile tests for differential, Ackermann and mecanum; derivative and integrator checks; state-key distinction for identical poses/different steering or articulation; pruning/dominance tests |
| FR-002, FR-017 | Primitive propagation against exact solutions, forward/reverse, in-place rotation, sideways mecanum translation with fixed yaw, simultaneous translation/rotation, limits and step-size convergence |
| FR-004, FR-005 | Compound-body transforms, articulated hinge configurations, self-collision exclusions, swept-geometry cases between sample endpoints |
| FR-006, FR-007, FR-018 | REP-103/105 frames, rotated origin conversion, cell-center offsets, row-major indexing, unknown/out-of-bounds semantics, Dijkstra known maps, lazy settlement, cache invalidation |
| FR-008 | Dubins/Reeds–Shepp endpoint, length, segment direction and symmetries; randomized OMPL comparison |
| FR-009, FR-015 | Stale queue labels, reopen rules, budgets/cancellation and structured diagnostic status |
| FR-010, FR-012 | Exact primitive reconstruction, analytic tail, cusp and rotation progress, evaluation at segment boundaries |
| FR-011 | Constraint-valid refiner acceptance, invalid spline rejection, untouched feasible fallback |
| FR-013, FR-014 | YAML/JSON roundtrip/validation in adapters, no ROS dependencies in core |
| NFR-001..012 | See [quality protocol](quality.md); scenario baselines and metadata stored with results |

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
- Conventional signed occupancy `-1` unknown and `0..100` probability encoding; threshold policy and no raw Nav2 costmap reinterpretation.
- Reject mismatched map/start/goal frames; optional ROS adapter performs transforms first.
- See [map-coordinate contract tests](map-coordinates.md).

## Collision verification

- State-level tests: rectangle, circle, convex polygon, compounds, rotation and articulated transforms.
- Motion tests: intersecting a thin obstacle between safe endpoints; in-place rotation clipping corners; articulation sweeping without reference-point movement; mecanum sideways rectangular-footprint clearance at fixed yaw.
- Articulated-steering ODE tests: author-supplied four-state model, denominator guard, finite steering range, nonzero heading change at zero reference-point speed, closed joint-only cycles and frame transforms after conventions are confirmed.
- Property tests: static obstacle map immutability across plan; collision results invariant under paired global transforms where map transform is exact.
- Record whether verification is **discrete sampled** or **conservatively continuous**. Never label sampling alone as a formal collision-free guarantee.
- Independently validate final refined path against original map snapshot and model.

## Heuristic correctness and guarantees

- Tiny hand-computed maps for Dijkstra values, lazy vs eager equivalence, unreachable goals and unknown-cell policies.
- Verify heuristic cache invalidates on map version, goal, inflation/unknown policy and grid geometry, including changed origin yaw or frame ID.
- For any declared *admissible* heuristic mode, compare `h` to a proven lower bound or analytically solvable case. Do not infer admissibility solely because reverse Dijkstra runs.
- Test consistency only for modes where consistency is claimed; otherwise ensure reopen behavior handles inconsistent heuristics.
- Compare obstacle-aware heuristic on/off for performance **and** solution quality; no artificial optimality claims.

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
- [ ] Enable optional OMPL tests without adding an OMPL core dependency.
- [ ] Validate failure results and fallback behavior.
- [ ] Add sanitizer runs (ASan/UBSan, optionally TSan) in local developer instructions.
- [ ] Produce performance baselines on named hardware before setting regression thresholds.
- [ ] Review requirement-to-test matrix at each implementation milestone.
