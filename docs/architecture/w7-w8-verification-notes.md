# W7–W8 implementation and verification notes (2026-10-26)

This note reports an **experimental standalone C++20** slice of the
[consolidated specification](Hybrid_AStar_Consolidated_Specification.md),
not a new swept-safety guarantee or a product performance threshold.
Read [current implementation](current-implementation.md) for the normative
API/defaults.

## W7 — conservative covering-disc search-only prefilter

Implemented in `ackermann/cover_prefilter.{hpp,cpp}`. The whole rectangle
`[rearX,frontX] × [-halfWidth,+halfWidth]` is covered by `k` equal-length
longitudinal strips, each enclosed by the circle centered at its strip midpoint
with radius `sqrt(halfWidth² + (length/(2k))²)`. Their union is therefore a
**superset of the complete filled body** under any rigid SE(2) transform,
including continuously varying yaw.

The per-request blocked-center squared Euclidean distance field uses a
two-pass 1D lower-envelope transform. Blocked centers are selected using the
request's **effective occupancy threshold and unknown policy**. At disc
center `p`, choose containing map cell `c` and its center-based EDT distance
`D(c)`. No exact check is skipped unless all discs satisfy:

```text
D(c) > discRadius + sqrt(2) * mapResolution + numericalMargin
```

and each complete disc is strictly inside the map bounds. Derivation: the
query point can be at most a half-cell diagonal from `c`; any point in a
blocked **closed square** can be at most a half-cell diagonal from its blocked
center. Triangle inequality gives the total `sqrt(2)*r` correction. Thus all
blocked squares are separated from all outer discs when this strict inequality
holds, so the body at **that pose** is clear. Any uncertainty, changed-policy
request, edge proximity, cover overlap or missing positive margin means
**exact rectangle-vs-closed-square SAT fallback**; cover overlap is never
a collision classification.

This geometric argument depends on the true immutable pinned map/policy,
complete rectangle cover and numerically conservative computation. Independent
numeric proof review and more edge-case testing remain required. Sampling or
disc-cover proof does **not** certify the swept body between poses. Strict final
path checking always bypasses this accelerator and calls exact collision
directly. Defaults: `discPrefilterCount=0` (off); 4 and 5 are only
candidate benchmark settings.

Verification: `prefilter_test` performs 9,000 seeded differential
comparisons (3 map resolutions × 2 unknown policies × 3 disc counts × 500
continuous poses), including rotated map origins and randomized blocked/
unknown cells. It also asserts boundary fallback, successful planner usage,
and non-upgraded sampled evidence. All those tests passed on a local core
workspace. This is **not** a probabilistic or formal all-input proof.

## W8 — opt-in quality controls

- `SolutionPolicy::FirstStrictlyValidated` stays the default; optionally use
  `ImproveIncumbent` with a finite separate expansion budget (initial
  illustration: 2500). Every newly accepted incumbent first passes the exact,
  stricter **complete reconstructed path** validation. An additional
  candidate only replaces it for strictly lower **configured weighted cost**.
  No nonadmissible heuristic-derived incumbent pruning or global A* bound
  is claimed. Termination reason and solution-quality metadata disclose
  budgets and improvement.
- `clearanceWeight=0` keeps the original objective; a positive weight
  integrates `lambda * max(0,1-d/influenceRadius)^2` with deterministic
  trapezoidal samples along each real section. The measured clearance `d`
  is from the **oriented physical rectangle** to blocked **cell squares** or
  the finite map boundary, capped by influence radius. No protruding
  covering-disc proxy distorts the clearance objective. It is a preference
  score, not a safety clearance claim.
- Identical clearance integration is applied to primitives and signed analytic
  connector sections **before** key-cost dominance. The base edge cost stays
  a safe cheap lower bound because clearance is nonnegative.
- A separate **opt-in bounded post-path shortcut refiner** is now implemented with strict entire-path revalidation, original-path fallback, and native signed connector injection. See [W8 refinement verification](w8-refinement-verification.md) for the 7-suite local evidence. More general smoothing and the one-time strict-validation retry are still pending.

## Reproduction and preliminary result

Local manually run commands:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DHYBRID_ASTAR_ENABLE_OMPL=OFF \
  -DHYBRID_ASTAR_BUILD_TESTS=ON -DHYBRID_ASTAR_BUILD_BENCHMARKS=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
./build/hybrid_astar_planner_bench 3
```

Local tests: four suites passed in GCC 14.2 Release and Clang 17
ASan+UBSan; the OMPL-ON adapter remained **unverified** (OMPL not installed).
Tests were run against the locally reconstructed core workspace, not a
clone of the exact remote repository tree, because external git DNS/network
access was unavailable; this limitation must be resolved before release.

The **exploratory** 3-run median times below are in milliseconds on a
virtualized Intel Xeon Platinum 8573C, Release, 0.2 m/400×400 fixture.
They are not rigorous p95/p99 or general performance guarantees:

| Scenario | Disc count | Clearance lambda | Validated success / 3 | Median ms | Last weighted cost |
| --- | ---: | ---: | ---: | ---: | ---: |
| Straight open field | 0 | 0 | 3 | 4.21 | 6 |
| Straight open field | 4 | 0 | 3 | 4.66 | 6 |
| Straight open field | 5 | 0 | 3 | 3.98 | 6 |
| Distant doorway, performance-grid | 0 | 0 | 3 | 601.66 | 39 |
| Distant doorway, performance-grid | 4 | 0 | 3 | 274.18 | 39 |
| Distant doorway, performance-grid | 0 | 0.15 | **0** (50,000-expansion limit) | 1548.14 | — |

**Disposition:** W7 is implemented but **disabled by default** pending
representative map/geometry test expansion and measured end-to-end benefit.
W8's bounded incumbent improvement is opt-in; objective clearance is opt-in
and the illustrated `lambda=0.15` must **not** be adopted as the default
given the observed doorway regression. No comparison here ratifies the
product's quality/timing or output-safety requirements.

## Remaining gates

OMPL-ON build and signed cusp/gear-cost tests against actual OMPL, corner/
boundary differential geometry tests and sanitizer matrix on an exact checkout,
parking and narrow-corridor scenarios, independent EDT numeric proof review,
repeatable cold/warm multi-seed performance trials, clearance objective tuning,
failure/cancellation contract review, bounded strict retry decision, downstream
refinement, and physical trajectory/actuator checks. These are separate
from W7's conservative **pose** proof.
