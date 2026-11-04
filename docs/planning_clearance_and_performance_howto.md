# How to configure planning: obstacle clearance, safety margins and performance

This is the **operator/developer how-to** for choosing related
`PlanRequest::options` values in the C++ planner. It distinguishes
**ordinary route planning**, **a softer preference for clearance from objects**,
and a **geometric minimum margin**. A Euclidean distance transform (EDT)
is an *optional acceleration for clearance scoring*, **not a mode that
independently produces safer paths**.

All examples assume a populated, valid `hybrid_astar::ackermann::PlanRequest
request` with an immutable map snapshot, an occupancy policy, the real
robot footprint, and valid start/goal poses. The snippets configure
`request.options`; they do not bypass the planner's normal exact
footprint collision tests or independent stricter final-path validation.

## Quick choice: which settings belong together?

| Desired behavior | `clearanceWeight` | `useClearanceDistanceTransform` | `footprintInflationMeters` | Notes |
| --- | ---: | :---: | ---: | --- |
| **Ordinary planning (default)** | **0.0** | **false** | **0.0** | No added obstacle-proximity objective or EDT cost |
| **Prefer wider clearance** | **0.15** | **true** | 0.0 | Measured *starting point*, not a guaranteed separation or globally tuned optimum |
| **Require a geometric margin in checks** | **0.0** | **false** | e.g. **0.05** | 5 cm expansion of the physical footprint for the sampled collision and final-validation checks |
| **Geometric margin + prefer extra clearance** | **0.15** | **true** | e.g. **0.05** | Combines inflated collision geometry with a soft preference measured beyond that inflated body |
| Compare against the exact scorer | **0.15** | **false** | Same as comparison | Same clearance objective as EDT mode; slower in the measured warehouse smoke test |

The values **0.15**, **0.40 m**, and **0.05 m** are example settings,
not universally safe or optimal. Only choose a geometric margin after
accounting for robot dimensions, localization uncertainty, tracking error,
and real-world clearances. Always pin `request.footprint` and
`request.occupancy` to the actual robot and map policy.

### Recipe A — standard route planning (fastest of the measured clearance modes)

```cpp
request.options.clearanceWeight = 0.0;
request.options.clearanceInfluenceRadiusMeters = 0.40; // unused while weight = 0
request.options.useClearanceDistanceTransform = false;
request.options.footprintInflationMeters = 0.0;         // no extra margin
```

This preserves the normal objective and avoids EDT preprocessing.
The default values in `PlanOptions` already provide these settings;
only assign them explicitly when changing between profiles or making
the intent obvious.

### Recipe B — explicitly prefer paths farther from obstacles

```cpp
request.options.clearanceWeight = 0.15;               // optional soft preference
request.options.clearanceInfluenceRadiusMeters = 0.40; // metres
request.options.useClearanceDistanceTransform = true;  // accelerate this preference
request.options.footprintInflationMeters = 0.0;
```

Use this only when the calling application or operator **explicitly
requests clearance-aware route quality** and accepts greater planning time.
`clearanceWeight > 0` activates the proximity objective; enabling the EDT
by itself does **not** select a longer-clearance route. EDT merely avoids
many exact geometric distance scans where it can prove that the penalty
must be zero. Uncertain/near-obstacle samples still use the exact
footprint-to-obstacle distance.

This soft preference can yield longer paths, increased search work,
or no meaningful clearance gain for a constrained route. It does **not**
enforce 40 cm separation.

### Recipe C — enforce an additional geometric margin in collision checking

```cpp
request.options.clearanceWeight = 0.0;
request.options.useClearanceDistanceTransform = false;
request.options.footprintInflationMeters = 0.05; // 5 cm beyond physical footprint
request.options.useInflatedFootprintBroadPhase = true; // default, exact-equivalent
```

`footprintInflationMeters` adds a metric disk-offset to the **physical
rectangle**; obstacles and map edges are tested against that inflated
shape. This affects both the search collision checker and independent
final-path validation, rather than merely penalizing a close route.
A route that cannot fit through the narrower corridor may be rejected.

**Safety qualification:** The planner performs strict adaptive **sampled**
full-footprint validation, not a mathematically certified continuous swept
volume or closed-loop robot tracking guarantee. Inflation is a constraint
in that implemented collision-checking model, **not** a blanket statement
that the real robot stays 5 cm clear at every instant. If using a margin
for safety-critical operation, independently verify the swept body and
control/localization uncertainty.

### Recipe D — a geometric margin **and** a preference for more space

```cpp
request.options.footprintInflationMeters = 0.05;        // geometric constraint
request.options.useInflatedFootprintBroadPhase = true;
request.options.clearanceWeight = 0.15;                // extra soft preference
request.options.clearanceInfluenceRadiusMeters = 0.40;
request.options.useClearanceDistanceTransform = true;
```

These options are **complementary**. Inflation checks the enlarged
footprint against occupied geometry, while the clearance penalty
discourages passing near objects **beyond the inflated footprint**.
The radius is measured from the inflated footprint and map boundary,
not from the robot's reference point.

Keep the footprint, inflation, occupancy policy and clearance radius
identical when comparing exact and EDT-assisted scoring; only
`useClearanceDistanceTransform` should differ.

## How the clearance objective works

For positive `clearanceWeight`, the scorer integrates a nonnegative
obstacle-proximity penalty along moving sections:

```text
extra_cost_per_metre = clearanceWeight *
                       max(0, 1 - clearance / influenceRadius)^2
```

- `clearanceWeight` sets the **strength** of the soft penalty.
  **0.0 disables it** (default). **0.15** is the tested warehouse
  setting and a reasonable *experimental starting point*, not a
  calibrated value for other maps or robots.
- `clearanceInfluenceRadiusMeters` controls the distance beyond which
  the penalty becomes zero. Default **0.40 m**. It must be finite
  and **strictly positive**, even if the weight is zero.
  **It is not a guaranteed minimum clearance.**
- `useClearanceDistanceTransform` toggles a safe, conservative
  zero-penalty shortcut for the *same* positive-weight objective.
  Default **false**. The planner only constructs a `ClearanceScorer`
  (and therefore its EDT) when `clearanceWeight > 0`.
  For positive weight, `false` selects the exact-distance scorer;
  `true` skips provably zero-cost scans and falls back to the
  exact scorer otherwise.
- The penalty integrates over **translational distance**.
  A pure stationary spin has no additional translational clearance
  penalty, although its rotated footprint is still collision checked.
- Map edges are considered proximity obstacles. Occupied and unknown
  grid cells follow the request's `OccupancyPolicy`.
- **EDT does not replace collision checking**, does not itself
  change paths relative to exact clearance scoring, and is not a
  guaranteed-clearance certificate.

If the goal is an actual geometry constraint, change
`footprintInflationMeters`, not merely the weight or radius.

## Other options: recommended combinations and dependencies

| Option | Current default | Guidance and interaction |
| --- | --- | --- |
| `useEmptyRegionPrefixSum` | **true** | Keep enabled for the established exact empty-region collision bypass; independent of clearance/EDT |
| `useFastEmptyAabb` | **false** | Optional pre-polygon empty-AABB shortcut. It **only operates when `useEmptyRegionPrefixSum=true`**. Our warehouse performance benchmarks enabled it identically across variants; opt in after representative correctness/performance checks |
| `useInflatedFootprintBroadPhase` | **true** | Leave enabled when using positive inflation; this is the default exact-equivalent early-rejection path |
| `discPrefilterCount` | **Automatic** | Defaults to 4 discs for Ackermann, none for differential/mecanum. Keep automatic unless benchmarking an explicit override |
| `searchLabelReserve` | **0** | Independent of clearance selection. Set to a **label count**, not the integer tier index, after calibrating the same map, model, footprint, occupancy, objective and budgets |
| `heuristicMode` | **PerformanceReedsSheppAndObstacleGrid** | Default performance-oriented search ordering; it is **not** a certified optimality guarantee |
| `performanceHeuristicWeight` | **1.0** | Leave at 1.0 when comparing options; supported range is 1–3, and changing it changes search ordering and performance/quality trade-offs |
| `solutionPolicy` | **FirstStrictlyValidated** | Retain unless deliberately evaluating the improvement policy |
| `collectCollisionWork` | **false** | Set true only for diagnostics; extra counting changes measurement overhead |
| `maxExpansions`, `maxGeneratedNodes` | **1,000,000 / 4,000,000** | Search/work caps. Use identical caps in A/B comparisons; increasing them can change the required label-reserve capacity |
| Search/final sample spacings and check budgets | See `PlanOptions` | Keep strict validation enabled and do **not** relax precision or check limits merely to obtain a faster clearance-aware run |

### A practical performance-plus-clearance configuration

If the map/robot combination has already been tested with the optional
fast-AABB shortcut, it can be combined with the clearance preference
without changing its objective:

```cpp
request.options.useEmptyRegionPrefixSum = true;
request.options.useFastEmptyAabb = true; // explicit opt-in, not a default

request.options.clearanceWeight = 0.15;
request.options.clearanceInfluenceRadiusMeters = 0.40;
request.options.useClearanceDistanceTransform = true;

// Optional, only if a physical safety margin is required:
request.options.footprintInflationMeters = 0.05;
request.options.useInflatedFootprintBroadPhase = true;

// Reserve labels only after calibrating THIS exact configuration.
// request.options.searchLabelReserve = measuredReservedLabelCount;
```

Do **not** blindly copy the warehouse's **v2 tier 14 = 655,360 labels**
into a different geometry or a positive-clearance configuration.
The completed 32-route tier benchmark used **clearance OFF** on one
specific large differential-drive footprint. Clearance scoring can
change expansions, path selection and peak label consumption.
See [map-specific reservation calibration](map_specific_label_reserve_calibration.md)
for the explicit offline dial-in tool; for custom positive-clearance
plans, pass a corresponding `PlanRequest` directly to its C++ API.
The manual calibration CLI currently has no `--clearance-weight` flag.

## Performance evidence and operational trade-offs

The **corrected** EDT A/B/C benchmark has only **2 warehouse routes ×
2 footprints × 3 variants**, with **one timed repeat per variant**.
The original uncorrected full CSV rejected positive-clearance native
requests and must **not** be used as performance evidence.

| Comparison | Measured result | Meaning |
| --- | --- | --- |
| EDT vs exact clearance scoring (`weight=0.15`, `radius=0.40`) | **30.2% median paired search-time saving** | The EDT eliminates many unnecessary exact distance scans |
| EDT-assisted clearance vs ordinary clearance-OFF planning | **3.57–4.84× search time** | Prefer-clearance objective still has a large performance cost |
| Exact clearance scans skipped by EDT | **79.8%** | These are *clearance scans*, not collision checks |
| EDT build per request | **10.25–13.61 ms** | Included in measured search times |
| EDT resident grid on the warehouse map | **12.85 MiB** | Grid only; excludes temporary build buffers and other planner memory |

In the smoke cases, positive clearance scoring increased A* expansions
by **3.3–4.2%** and exact collision-check calls by **13.9–15.2%**
relative to clearance OFF. Sampled path quality did improve on
some routes; on one tested route the sampled minimum inflated-footprint
clearance improved from **0.001 m to 0.128 m**, while on another
the minimum barely changed. These measurements are **not** general
guarantees of time or path quality.

**Operational recommendation:** Ordinary requests should keep
`clearanceWeight=0` and EDT off. When the user explicitly prioritizes
obstacle clearance over latency, try `weight=0.15`, `radius=0.40`
and EDT on; compare against ordinary planning and adjust parameters
only after inspecting actual path quality, search status and latency.
If a hard geometry margin is required, add appropriately sized
inflation separately.

The full ten-route, multi-repeat clearance benchmark is still **pending**.
Do not infer general deployment speedups from the two-route smoke sample.
Run the [manual EDT comparison](clearance_edt_benchmark.md) with
`PROFILE_HOTPATHS=OFF` for search-time decisions and use
[measured warehouse findings](warehouse_performance_findings_2026-11-04.md)
for the existing route-by-route evidence.

## Diagnostics and validation checklist

For a matched comparison, use the **same** map snapshot, occupancy
policy (including unknown-cell treatment), model, physical footprint,
inflation, start/goal pairs, heuristic, search limits, sampling budgets
and fast-AABB settings. Change **only** the intended experiment option:

- **Ordinary vs clearance-aware:** `clearanceWeight=0` versus
  `clearanceWeight>0` (usually with EDT on). Expect different
  path costs, expansions and potentially different routes.
- **Exact vs EDT:** keep the positive weight and radius **identical**;
  toggle only `useClearanceDistanceTransform`. Expect unchanged
  paths, costs, collision work and search decisions.
- **Inflation:** compare positive margin to zero only when intentionally
  measuring its geometric and feasibility impact.
- **Map-specific label reserve:** compare identical route and search
  configuration. Reservation is an allocation choice, not a new path
  objective. With positive clearance, recalibrate against that objective.

Read the returned `PlanResult::status` before interpreting timing.
Relevant diagnostics include `expanded`, `generated`,
`exactCollisionPoseCalls`, `clearanceSamples`,
`clearanceDistanceQueries`, `clearanceEdtZeroCostBypasses`,
`clearanceExactFallbacks`, `clearanceEdtBuildMilliseconds`,
`clearanceEdtBytes`, `labelPeakSize`, `labelRehashes`
and `labelPeakBuckets`.

`PROFILE_HOTPATHS=ON` enables extra internal hot-path clocks but
changes timing overhead. Do **not** use instrumented search times to
claim a production-speed improvement. EDT and label-reserve microtimings
cannot simply be summed, because some timers count nested work.

For detailed implementation semantics, read
[the clearance/EDT benchmark](clearance_edt_benchmark.md),
[footprint inflation](../README.md#continuous-footprint-collision-inflation),
and [reservation tier calibration](map_specific_label_reserve_calibration.md).
