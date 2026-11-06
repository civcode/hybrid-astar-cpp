# Articulated goal heuristic (RK4-free guidance)

Implemented on `feature/nav2-equivalent-envelope-footprint-benchmark`.
The default search mode, coupled motion templates, swept-AABB collision
prefilter, and independent final RK4 validation are unchanged.

## Modes

`articulated_steering::SearchOptions::heuristicMode` accepts:

- `HeuristicMode::ObstacleGrid` (default): existing reverse-Dijkstra
  grid with rear XY Euclidean fallback.
- `HeuristicMode::AnalyticOnly`: no grid; a tolerance-aware,
  obstacle-free analytical effort lower bound.
- `HeuristicMode::Combined`: max(existing grid/fallback, analytical
  lower bound). This **is not a certified bound**, because the raw
  rear-reference 8-neighbor grid is not a certified bound on continuous
  articulated cost.
- `HeuristicMode::CombinedCoarseGrid` (experimental): maximum of a
  complete, conservatively downsampled obstacle potential, analytical
  effort bound and rear-XY Euclidean distance. The default coarse grid
  resolution is 0.15 m. All existing search modes are unchanged.

The existing `heuristicWeight` multiplies the chosen result. Values
greater than 1 are a performance experiment, **not** an optimality
guarantee. Likewise, the quantized one-label search is not an optimal
continuous-state solver.

## Derivation and cost units

The rear-reference state is `(xr, yr, thetaRear, gamma)`; the front
reference is

```text
F = R + Lr * e(thetaRear) + Lf * e(thetaRear + gamma)
D(gamma) = Lr + Lf * cos(gamma) > 0
```

For the author-supplied no-slip ODE,

```text
thetaRear' = vFront * sin(gamma)/D(gamma)
            - gammaRate * Lf*cos(gamma)/D(gamma)
```

Define `B(0)=0`, `B'(gamma)=Lf*cos(gamma)/D(gamma)`,
`beta = thetaRear + B(gamma)`. Then exactly,

```text
beta' = vFront * sin(gamma)/D(gamma)
```

`B` is evaluated by a closed-form antiderivative (atan, atanh, or
equal-link-length tangent case) once per request-local gamma lattice
angle. The goal value is evaluated once. No RK4 or OMPL work occurs in
heuristic evaluation.

Let `Kmax=max_allowed_gamma abs(sin(gamma)/D(gamma))` (calculated
from the interval endpoints and its stationary points) and
`M=max_allowed_gamma abs(B'(gamma))` (calculated from endpoints
and zero when included). Given accepted goal tolerances
`exy, eyaw, egamma`:

```text
frontTolerance = exy + (Lr+Lf)*eyaw + Lf*egamma
hFront = max(0, norm(F - F_goal) - frontTolerance)

betaTolerance = eyaw + M*egamma
hHeading = max(0, abs(wrap(betaGoal-beta)) - betaTolerance) / Kmax

hGamma = articulationCostMetersPerRadian *
         max(0, abs(gamma-goalGamma) - egamma)

driveCostPerMeter = min(1, reverseCostMultiplier)
hAnalytic = driveCostPerMeter * max(hFront,hHeading) + hGamma
```

`Kmax` is positive for nondegenerate mechanically permitted steering
intervals. The bounds are lower bounds of the **front-reference
travel-length plus joint-angle effort** for any motion ending within
the specified tolerances, provided the geometric model and control
limits hold and the cost parameters are positive. The discount factor
handles user-configured cheaper reverse driving. The nonnegative
stationary articulation penalty and gear-switch penalties are omitted
conservatively. This is a bound for the ideal no-slip kinematic cost,
not an actuator, obstacle, continuous swept-volume, or path-optimality
certificate.

## Experimental complete multiresolution obstacle guidance

The `CombinedCoarseGrid` mode is a separate opt-in. On large, fine-map
warehouses, the original lazy reverse-Dijkstra search can exhaust its
100,000-settlement total work budget and fall back to Euclidean distance
millions of times. This mode constructs a **complete** coarse-grid
reverse-Dijkstra field once per request and queries it in O(1) time.

`coarseGridResolutionMeters` defaults to **0.15 m** (allowed range
0 < resolution ≤ 2 m) and is rounded up to a whole multiple of the map's
native resolution, aligned to its local origin. Each coarse tile is
considered blocked if **any** contained native cell is occupied or
unknown according to the occupancy policy. The eight-connected search
rejects diagonal corner cutting. Goal-region tiles are seeded by
intersection with the goal XY tolerance disk.

Conservative blocking can erase real narrow passages. Queries falling
in unreachable coarse cells or outside the map revert to Euclidean
distance, combined with the analytical articulated lower bound.
**NoPath is never inferred from coarse-grid connectivity.** This
performance heuristic is not an admissibility/optimality certificate.
All actual search collision checks and independent full-path
RK4/two-carriage validation still use the original high-resolution map.

The field is **request-local**, built once per plan, with no cross-request
cache (avoids stale fields after map edits). Oversized auxiliary grids
above two million cells fall back to geometric/analytical guidance.
The UI and JSON export expose requested and effective grid resolution,
coarse cell/blocked/seed/settlement/query/fallback counts, construction
milliseconds and allocation-skip status.

An offline reconstruction from the uploaded **0.03 m** full warehouse
map found 67,670 coarse cells and 55,253 settled reachable cells at
**0.15 m**, with a finite ~32.70 m potential at the start. At 0.12 m
and 0.18 m, the coarse maps held 105,588 and 46,872 cells.
These measurements verify field connectivity on that input, **not C++
planner timing**.

### Docker A/B regression

```bash
git pull --ff-only
BUILD_JOBS=8 docker compose build test
docker compose run --rm test ctest --test-dir build-docker \
  -R '^(articulated_steering_planner_test|demo_articulated_test|demo_articulated_ui_test|demo_http_test)

`SearchOptions::enableGoalConnector` is **off by default**. When enabled,
the articulated planner reuses Ackermann's `AnalyticExpansionScheduler`
(ratio 3.5) to propose and independently validate terminal maneuvers.
An unsuccessful proposal always falls back to ordinary Hybrid A*.

### Work allocation for nonzero goal headings

The first 4 scheduled attempts (configurable
`goalConnectorGlobalAttempts`) remain unrestricted in distance, including
the first expanded node. The remaining attempts are **reserved** for
search states satisfying an adaptive near-goal gate:

- Rear-reference XY distance is within `max(6 m, 20% of start-to-goal
  XY distance)`; the 6 m floor is configurable.
- The score `XY distance + 0.9 * rear yaw error + 0.4 * gamma error`
  improves by at least 0.25 m-equivalent since the last reserved attempt.

This preserves near-goal work instead of spending all 24 attempts early.
The scheduler still controls WHEN an attempt is due; the allocation
policy controls WHICH due attempts are worth using. Neither the first
shot nor the early global shots have an XY proximity requirement.

Diagnostics include global vs reserved shots, deferred opportunities,
first/last attempted expansion number, and closest rear XY goal distance.

### Obstacle-aware terminal shooting

The request-local `GoalConnectorObstacleGuide` builds an approximate
8-neighbor obstacle-distance field once from the occupancy map. It
scores distance-to-obstacle and map-boundary deficits for both carriage
reference points, accounting conservatively for their body radii.
The nonlinear terminal solver tries one-, two-, then three-arc
signed forward/reverse families with finite-rate articulation changes.
It first minimizes terminal error and sampled, **soft clearance**.
A second, feasible-only stage projects operational-effort gradients
into the endpoint Jacobian's approximate nullspace.
The soft field only influences proposal generation and ordering;
it is NEVER accepted as proof of collision freedom.

The connector interleaves stationary-then-arc and **coupled steering +
driving** trajectory families, including reverse gears. Constant-gamma
arcs and stationary articulation are analytic. During coupled steering,
heading is analytic while front XY uses fixed Simpson quadrature;
the final safety verifier independently reintegrates with RK4.
All steering rates, gear speeds, gamma limits and segment durations
remain bounded. This is a limited three-arc terminal family, not a
complete articulated shortest-path method or a substitute for
higher-level obstacle routing.

Before acceptance the candidate is sampled using body-point-motion
limits, both exact carriage footprints are collision checked, and the
**entire search prefix plus the terminal tail** is independently
replayed with finer RK4 samples. Only a fully accepted four-state goal
becomes `Success`. `trajectoryActuatorFeasible` remains false.

Default budgets: 24 attempts, 3,000 solver evaluations/attempt,
8 proposed candidates/attempt, 100,000 search-stage terminal
body checks, and 40 m maximum arc length per segment.
These are limits, not an assurance of success. If a collision-free
trajectory requires more than the chosen maneuver family, normal
Hybrid A* must find it instead.

### Experimental heading-aware search mode

`HeuristicMode::CombinedHeadingAware` retains
`max(obstacle-grid, articulated analytic lower bound)` and **adds**
`goalHeadingGuidanceWeight * 8/(8+dXY) * analyticHeadingDrivingCost`.
This deliberately re-emphasizes rear heading when the obstacle-grid
distance otherwise hides it. The extra component may double-count
driving cost and the mode is explicitly **performance-oriented**,
not an admissible estimate or shortest-path certificate.
The existing ObstacleGrid, AnalyticOnly and Combined modes and their
defaults remain unchanged.

### Operationally simpler terminal fitting

The native terminal generator now attempts **one driving arc**, then
**two driving arcs**, before its general **three-arc** forward/reverse
family. Each family supports stationary or finite-rate coupled steering.
The total 3,000-evaluation-per-shot ceiling is unchanged: approximately
one eighth of the budget is reserved for one-arc fits, one third total
for one-/two-arc fits, and the remainder for unrestricted three-arc
fallback. Shorter families do not make complicated parking cases
unreachable; they simply get evaluated first.

For each candidate the solver first minimizes endpoint error and
sampled soft obstacle clearance. Once feasible, a damped projection into
the endpoint Jacobian's approximate nullspace polishes a separate
**operational quality score**. It includes actual driving cost, reverse
driving, total articulation effort, stationary steering duration,
steering-direction changes, gear switches, steering-limit excursions,
and unnecessary drive-arc fragmentation. A polishing step is accepted
only when the terminal fit and soft-clearance threshold remain feasible.
The complete control sequence is still checked against *both real
carriage footprints* and independently replayed with RK4 before success.
No finite set of maneuver families certifies shortest or simplest paths.

The operational penalty is used for candidate fitting and provisional
ranking, **not hidden inside the planner's path cost**. The existing
strictly validated incumbent is still replaced only by a candidate
with lower actual accumulated `costMeters`; this protects the existing
anytime non-worsening planner-cost contract. A feasible higher-cost
path with fewer steering reversals may therefore *not* replace a
cheaper existing incumbent. Connector effort remains a preference,
not a hard constraint on steering, reverse or joint limits.

The JSON `original` object now exports:
`goalConnectorTailDrivingMeters`,
`goalConnectorTailReverseMeters`,
`goalConnectorTailArticulationRadians`,
`goalConnectorTailStationarySeconds`,
`goalConnectorTailPeakGammaRadians`,
`goalConnectorTailGearChanges`,
`goalConnectorTailArticulationReversals`,
`goalConnectorTailDriveArcs`,
`goalConnectorTailFittedArcs`,
`goalConnectorTailOperationalPenaltyMeters`, and
`goalConnectorTailQualityScoreMeters` for the **actually returned
incumbent**, not an arbitrary rejected or superseded proposal.
Plain search solutions without terminal shooting have zero tail
metrics. Proposal counts by one-/two-/three-arc family are included.

## Optional cost-aware bounded anytime refinement

With `enableGoalConnector=true`, selecting
`enableAnytimeGoalRefinement=true` **retains the first strictly validated
trajectory** instead of immediately ending the search. It explores at
most `anytimeExtraExpansions=2000` further expanded states (per-plan
total expansion and generation caps still apply). At most
`anytimeExtraConnectorAttempts=8` additional terminal shots are
allowed after the first success, and all connector collision/check
budgets remain in effect.

Post-incumbent connector opportunities must be near the goal and pass a
cost-to-come plus analytical obstacle-free effort lower-bound filter.
The earlier XY-improvement-only gate no longer prevents later, cheaper
approach poses from being considered. All terminal candidates whose
prefix cost plus candidate cost cannot beat the incumbent are skipped.
Successors with accumulated cost at or above the incumbent are also
pruned. After the first validated solution, extra attempts are
**paced across the whole anytime expansion budget** rather than
consumed in the first few dozen expansions. The default 2,000 extra
expansions and eight extra shots yield slots near +222, +444, ...,
+1778 expansions; exceptionally promising kinematic lower-bound
states may shoot up to about 44 expansions ahead of their slot.
An unpromising state simply leaves its slot unused. Before the first
solution, the existing adaptive first-node/near-goal scheduler is
unchanged. Diagnostics `anytimePacedDeferrals`,
`anytimeFirstExtraShotExpansion`, and
`anytimeLastExtraShotExpansion` show when shooting happened.
No guarantee exists that any further shot will succeed.

For every promising candidate, the **complete search prefix plus tail**
must pass the independent fine-step RK4, both-body collision and
four-dimensional terminal checks before it can replace the incumbent.
A validated goal reached through ordinary A* also competes on cost.
If the extra budget, global expansion limit, or generated-node limit is
reached, the best retained strictly validated path is returned as
`Success` instead of being replaced by `BudgetExceeded`.
Cancellation remains cancellation and does not bypass safety policy.

This is a **bounded anytime, not globally optimal** approach: heuristic
weighting, state quantization and the limited 3-arc shooting family
preclude an optimality certificate. It may not improve the first path;
it deliberately costs extra planning time. Defaults stay **off** in C++
and the browser. Diagnostics record
`anytimeFirstSolutionExpansion`, `anytimeExtraExpansions`,
`anytimeExtraConnectorAttempts`, `anytimeValidatedCandidates`,
`anytimeImprovements`, `anytimeCostBoundPrunes`,
`anytimeFirstCostMeters` and `anytimeBestCostMeters`.

## Planning Lab web UI

Select **Articulated steering**. The heuristic mode remains Obstacle
grid by default. The **browser** now initializes heuristic weight to
**1.2 for any articulated mode**, and switching modes preserves any
user-entered value. Existing non-browser C++/HTTP requests default
to weight 1.0, preserving API compatibility. Combined and
Combined + goal-heading emphasis remain selectable (the latter
enables a separate optional guidance strength, initially 0.8).

**Native articulated goal connector** remains unchecked by default.
Once enabled, **Improve first validated path** can be selected, exposing
the extra-expansion budget (default 2,000; 1–100,000). Coupled motion
and swept-AABB remain enabled by default. All effective options and
anytime quality diagnostics appear in exported JSON. These controls
do not enable rigid-car RS curves.

## Docker validation and exact uploaded regression fixtures

The manual warehouse benchmark's optional THIRD argument chooses an
exact screenshot/export fixture: `baseline` (historical default),
`warehouse-heading`, `open-heading`, or `uploaded-heading`.
The last scenario reproduces the two most recent uploaded warehouse
plans: rear start (4.70, 30.49, 0°, gamma 0), rear goal
(31.24, 5.85, -90°, gamma 0). The other named scenarios preserve
the earlier warehouse/open-heading fixtures.
No GitHub Actions are needed or triggered.

```bash
git pull --ff-only
BUILD_JOBS=8 docker compose build test
docker compose run --rm test \
  ctest --test-dir build-docker \
  -R '^(articulated_steering_planner_test|demo_articulated_test|demo_articulated_ui_test|demo_http_test)$' \
  --output-on-failure

# Full fixed warehouse/heading regression: compare identical weight 1.2
for mode in weighted connector heading heading-connector; do
  docker compose run --rm test \
    ./build-docker/hybrid_astar_articulated_warehouse_bench \
    "$mode" 1.2 warehouse-heading
done

# First-solution vs anytime on the uploaded warehouse poses.
# Extra-expansion budgets are independent from the overall 1M limit:
for mode in connector anytime heading-connector heading-anytime; do
  docker compose run --rm test \
    ./build-docker/hybrid_astar_articulated_warehouse_bench \
    "$mode" 1.2 uploaded-heading 2000
done

# Try a larger quality budget only when quality justifies the extra work:
docker compose run --rm test \
  ./build-docker/hybrid_astar_articulated_warehouse_bench \
  anytime 1.2 uploaded-heading 5000

# Open-field nonzero-heading regression (should remain fast):
for mode in connector heading-connector; do
  docker compose run --rm test \
    ./build-docker/hybrid_astar_articulated_warehouse_bench \
    "$mode" 1.2 open-heading
done

docker compose up --build demo
```

CSV output reports the existing expansions/time/path cost/goal error
columns plus scenario, collision/terminal rejections, global/near
attempt counts, solver evaluations, coupled candidates, first/last
shot expansion, first/best validated cost and anytime improvement counts. Compare independent validation as well as speed;
a lower expansion count alone does not establish a safe path.

**Success criterion**: independently sampled two-body collision-free
planning within XY, rear-yaw and gamma tolerances, with materially fewer
warehouse expansions and lower runtime, while preserving the fast
open-field case. No such runtime improvement is claimed until the
above manually executed Docker measurements confirm it.
 \
  --output-on-failure

# Identical synthetic warehouse and weight, only heuristic differs:
for mode in anytime coarse-anytime; do
  docker compose run --rm test \
    ./build-docker/hybrid_astar_articulated_warehouse_bench \
    "$mode" 1.2 uploaded-heading 2000
done
```

**Important:** That synthetic benchmark map has 0.25 m pixels, so
requested 0.12/0.15/0.18 m resolutions all round to 0.25 m.
For a meaningful *resolution* sweep, use the uploaded full 0.03 m
warehouse in Planning Lab, choose the coarse mode with 0.12, 0.15
and 0.18 m, and compare with the existing Combined 1.2 mode.
Record total planning time (including field build), expansions,
cost, connector acceptance, fallbacks and independent validation.

## Native adaptive articulated goal connector

`SearchOptions::enableGoalConnector` is **off by default**. When enabled,
the articulated planner reuses Ackermann's `AnalyticExpansionScheduler`
(ratio 3.5) to propose and independently validate terminal maneuvers.
An unsuccessful proposal always falls back to ordinary Hybrid A*.

### Work allocation for nonzero goal headings

The first 4 scheduled attempts (configurable
`goalConnectorGlobalAttempts`) remain unrestricted in distance, including
the first expanded node. The remaining attempts are **reserved** for
search states satisfying an adaptive near-goal gate:

- Rear-reference XY distance is within `max(6 m, 20% of start-to-goal
  XY distance)`; the 6 m floor is configurable.
- The score `XY distance + 0.9 * rear yaw error + 0.4 * gamma error`
  improves by at least 0.25 m-equivalent since the last reserved attempt.

This preserves near-goal work instead of spending all 24 attempts early.
The scheduler still controls WHEN an attempt is due; the allocation
policy controls WHICH due attempts are worth using. Neither the first
shot nor the early global shots have an XY proximity requirement.

Diagnostics include global vs reserved shots, deferred opportunities,
first/last attempted expansion number, and closest rear XY goal distance.

### Obstacle-aware terminal shooting

The request-local `GoalConnectorObstacleGuide` builds an approximate
8-neighbor obstacle-distance field once from the occupancy map. It
scores distance-to-obstacle and map-boundary deficits for both carriage
reference points, accounting conservatively for their body radii.
The nonlinear terminal solver tries one-, two-, then three-arc
signed forward/reverse families with finite-rate articulation changes.
It first minimizes terminal error and sampled, **soft clearance**.
A second, feasible-only stage projects operational-effort gradients
into the endpoint Jacobian's approximate nullspace.
The soft field only influences proposal generation and ordering;
it is NEVER accepted as proof of collision freedom.

The connector interleaves stationary-then-arc and **coupled steering +
driving** trajectory families, including reverse gears. Constant-gamma
arcs and stationary articulation are analytic. During coupled steering,
heading is analytic while front XY uses fixed Simpson quadrature;
the final safety verifier independently reintegrates with RK4.
All steering rates, gear speeds, gamma limits and segment durations
remain bounded. This is a limited three-arc terminal family, not a
complete articulated shortest-path method or a substitute for
higher-level obstacle routing.

Before acceptance the candidate is sampled using body-point-motion
limits, both exact carriage footprints are collision checked, and the
**entire search prefix plus the terminal tail** is independently
replayed with finer RK4 samples. Only a fully accepted four-state goal
becomes `Success`. `trajectoryActuatorFeasible` remains false.

Default budgets: 24 attempts, 3,000 solver evaluations/attempt,
8 proposed candidates/attempt, 100,000 search-stage terminal
body checks, and 40 m maximum arc length per segment.
These are limits, not an assurance of success. If a collision-free
trajectory requires more than the chosen maneuver family, normal
Hybrid A* must find it instead.

### Experimental heading-aware search mode

`HeuristicMode::CombinedHeadingAware` retains
`max(obstacle-grid, articulated analytic lower bound)` and **adds**
`goalHeadingGuidanceWeight * 8/(8+dXY) * analyticHeadingDrivingCost`.
This deliberately re-emphasizes rear heading when the obstacle-grid
distance otherwise hides it. The extra component may double-count
driving cost and the mode is explicitly **performance-oriented**,
not an admissible estimate or shortest-path certificate.
The existing ObstacleGrid, AnalyticOnly and Combined modes and their
defaults remain unchanged.

## Optional cost-aware bounded anytime refinement

With `enableGoalConnector=true`, selecting
`enableAnytimeGoalRefinement=true` **retains the first strictly validated
trajectory** instead of immediately ending the search. It explores at
most `anytimeExtraExpansions=2000` further expanded states (per-plan
total expansion and generation caps still apply). At most
`anytimeExtraConnectorAttempts=8` additional terminal shots are
allowed after the first success, and all connector collision/check
budgets remain in effect.

Post-incumbent connector opportunities must be near the goal and pass a
cost-to-come plus analytical obstacle-free effort lower-bound filter.
The earlier XY-improvement-only gate no longer prevents later, cheaper
approach poses from being considered. All terminal candidates whose
prefix cost plus candidate cost cannot beat the incumbent are skipped.
Successors with accumulated cost at or above the incumbent are also
pruned. The extra connector attempts still obey the adaptive scheduling
policy; no guarantee exists that additional shots will succeed.

For every promising candidate, the **complete search prefix plus tail**
must pass the independent fine-step RK4, both-body collision and
four-dimensional terminal checks before it can replace the incumbent.
A validated goal reached through ordinary A* also competes on cost.
If the extra budget, global expansion limit, or generated-node limit is
reached, the best retained strictly validated path is returned as
`Success` instead of being replaced by `BudgetExceeded`.
Cancellation remains cancellation and does not bypass safety policy.

This is a **bounded anytime, not globally optimal** approach: heuristic
weighting, state quantization and the limited 3-arc shooting family
preclude an optimality certificate. It may not improve the first path;
it deliberately costs extra planning time. Defaults stay **off** in C++
and the browser. Diagnostics record
`anytimeFirstSolutionExpansion`, `anytimeExtraExpansions`,
`anytimeExtraConnectorAttempts`, `anytimeValidatedCandidates`,
`anytimeImprovements`, `anytimeCostBoundPrunes`,
`anytimeFirstCostMeters` and `anytimeBestCostMeters`.

## Planning Lab web UI

Select **Articulated steering**. The heuristic mode remains Obstacle
grid by default. The **browser** now initializes heuristic weight to
**1.2 for any articulated mode**, and switching modes preserves any
user-entered value. Existing non-browser C++/HTTP requests default
to weight 1.0, preserving API compatibility. Combined and
Combined + goal-heading emphasis remain selectable (the latter
enables a separate optional guidance strength, initially 0.8).

**Native articulated goal connector** remains unchecked by default.
Once enabled, **Improve first validated path** can be selected, exposing
the extra-expansion budget (default 2,000; 1–100,000). Coupled motion
and swept-AABB remain enabled by default. All effective options and
anytime quality diagnostics appear in exported JSON. These controls
do not enable rigid-car RS curves.

## Docker validation and exact uploaded regression fixtures

The manual warehouse benchmark's optional THIRD argument chooses an
exact screenshot/export fixture: `baseline` (historical default),
`warehouse-heading`, `open-heading`, or `uploaded-heading`.
The last scenario reproduces the two most recent uploaded warehouse
plans: rear start (4.70, 30.49, 0°, gamma 0), rear goal
(31.24, 5.85, -90°, gamma 0). The other named scenarios preserve
the earlier warehouse/open-heading fixtures.
No GitHub Actions are needed or triggered.

```bash
git pull --ff-only
BUILD_JOBS=8 docker compose build test
docker compose run --rm test \
  ctest --test-dir build-docker \
  -R '^(articulated_steering_planner_test|demo_articulated_test|demo_articulated_ui_test|demo_http_test)$' \
  --output-on-failure

# Full fixed warehouse/heading regression: compare identical weight 1.2
for mode in weighted connector heading heading-connector; do
  docker compose run --rm test \
    ./build-docker/hybrid_astar_articulated_warehouse_bench \
    "$mode" 1.2 warehouse-heading
done

# First-solution vs anytime on the uploaded warehouse poses.
# Extra-expansion budgets are independent from the overall 1M limit:
for mode in connector anytime heading-connector heading-anytime; do
  docker compose run --rm test \
    ./build-docker/hybrid_astar_articulated_warehouse_bench \
    "$mode" 1.2 uploaded-heading 2000
done

# Try a larger quality budget only when quality justifies the extra work:
docker compose run --rm test \
  ./build-docker/hybrid_astar_articulated_warehouse_bench \
  anytime 1.2 uploaded-heading 5000

# Open-field nonzero-heading regression (should remain fast):
for mode in connector heading-connector; do
  docker compose run --rm test \
    ./build-docker/hybrid_astar_articulated_warehouse_bench \
    "$mode" 1.2 open-heading
done

docker compose up --build demo
```

CSV output reports the existing expansions/time/path cost/goal error
columns plus scenario, collision/terminal rejections, global/near
attempt counts, solver evaluations, coupled candidates, first/last
shot expansion, first/best validated cost and anytime improvement counts. Compare independent validation as well as speed;
a lower expansion count alone does not establish a safe path.

**Success criterion**: independently sampled two-body collision-free
planning within XY, rear-yaw and gamma tolerances, with materially fewer
warehouse expansions and lower runtime, while preserving the fast
open-field case. No such runtime improvement is claimed until the
above manually executed Docker measurements confirm it.
