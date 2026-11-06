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
The nonlinear terminal solver fits three signed arcs with finite-rate
articulation changes and minimizes both terminal error and this
sampled, **soft clearance cost**. It uses a damped projection of
clearance gradients into the endpoint Jacobian's approximate nullspace.
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

## Planning Lab web UI

Select **Articulated steering**. In the **Articulated Search
Optimization** section, choose one of the following heuristic modes:
Obstacle grid (default, weight 1.0), Analytical, Combined (weight 1.2
preset), or **Combined + goal-heading emphasis** (weight 1.2 preset).
Only the last mode activates the additional **Extra goal-heading
guidance** setting (default 0.8, accepted range 0 through 3).

Both coupled motion and swept-AABB optimization are checked by default.
**Native articulated goal connector** remains unchecked by default.
The browser POST, backend option parser, effective search options, and
exported JSON now all expose the effective settings and connector
diagnostics. Rigid Ackermann Reeds–Shepp controls remain separate.

## Docker validation and exact uploaded regression fixtures

The manual warehouse benchmark's optional THIRD argument chooses an
exact screenshot/export fixture: `baseline` (historical default),
`warehouse-heading`, or `open-heading`. The latter two reproduce
the 2026-11-06 uploaded starts, goals, headings, map and geometry.
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
attempt counts, solver evaluations, coupled candidates, and first/last
shot expansion. Compare independent validation as well as speed;
a lower expansion count alone does not establish a safe path.

**Success criterion**: independently sampled two-body collision-free
planning within XY, rear-yaw and gamma tolerances, with materially fewer
warehouse expansions and lower runtime, while preserving the fast
open-field case. No such runtime improvement is claimed until the
above manually executed Docker measurements confirm it.
