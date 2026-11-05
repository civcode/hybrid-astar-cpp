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

## Native adaptive articulated goal connector (experimental)

The opt-in `SearchOptions::enableGoalConnector` adds an articulated-
specific goal connection attempt to the normal four-state search. It
reuses the **exact same Nav2-style `AnalyticExpansionScheduler` as
Ackermann**, including an initial attempt at the first non-stale
expanded node, an adaptively shrinking retry interval, and **no
fixed XY proximity gate**. A failed candidate is not fatal and A*
continues. Default-off preserves the historic search baseline.

The request-local native solver in
`src/articulated_steering/goal_connector.cpp` uses bounded nonlinear
terminal shooting over three signed (forward/reverse) constant-gamma
circular arcs with intervening finite-rate **stationary articulation**
transitions. It generates steering movements around the fixed front
reference point, not impossible instantaneous rear chassis steering.
Candidate endpoints are computed from exact geometric formulas
without RK4; numerical least-squares uses a fixed iteration and
evaluation budget (default 3,000), with no OMPL dependency.

These are a **restricted family of native articulated trajectories**:
not a mathematically complete shortest-path solver and not yet a
coupled-drive-plus-gamma shooting family. This may matter in cramped
warehouse aisles. Candidate arc lengths are bounded and divided into
short segments to respect adaptive body-motion sampling budgets.
The connector can exploit reverse driving, including reversing away
from the goal to realign.

Once a candidate enters the three-dimensional tolerance region
`(XY, rear heading, gamma)`, the planner replays its complete
terminal control sequence with model RK4 and adaptive body-point
sampling, checking **both** carriage collision footprints. Then it
independently re-integrates the **whole preceding search chain and
entire candidate tail** using the final fine RK4 schedule. Only this
full-prefix verified result may become `Success`. A rejected
connection never bypasses normal motion/collision checks.

Work limits and diagnostics (request-local):
`goalConnectorAttemptRatio=3.5`,
`maxGoalConnectorAttempts=24`,
`maxGoalConnectorEvaluationsPerAttempt=3000`,
`maxGoalConnectorCandidatesPerAttempt=8`,
`maxGoalConnectorSearchBodyChecks=100000` and
`maxGoalConnectorArcMeters=40`. Counters include attempts,
candidates, solver evaluations, connector body checks, collision
rejections, terminal mismatch, strict rejections and validated shots.
The normal `maxExpansions`, independent final checks and
cancellation mechanism still apply.

### Docker A/B and web UI

```bash
git pull --ff-only
BUILD_JOBS=8 docker compose build test
docker compose run --rm test ctest --test-dir build-docker \
  -R '^(articulated_steering_planner_test|demo_articulated_test|demo_articulated_ui_test|demo_http_test)Choose **Kinematic model → Articulated steering** and use the separate
**ARTICULATED SEARCH OPTIMIZATION** controls. The original rigid-body
`performance`, Reeds–Shepp and refinement widgets remain disabled for
this model. The options are independent:

- **Heuristic mode:** Obstacle grid (`grid`, default), Analytical
  (`analytic`), or Combined (`combined`, recommended for the measured
  warehouse experiment).
- **Heuristic weight:** editable in `(0,3]`. It initially remains `1.0`
  for the historical grid mode. Selecting Combined automatically presets
  `1.2`; users may override the preset. Switching to other modes
  presets `1.0`.
- **Coupled steering while driving:** checked by default.
- **Swept-AABB free-space prefilter:** checked by default; disabling it
  restores sampled collision checks where the optimization would have
  proven empty free space.
- **Native articulated goal connector:** unchecked by default; enables
  adaptively scheduled, bounded forward/reverse terminal shooting and
  independent full-chain validation.

The browser sends these values as `artHeuristicMode`,
`artHeuristicWeight`, `artCoupledMotion`, `artSweptAabb` and
`artGoalConnector` in the
`/api/plan` form request. `apps/demo/main.cpp` validates them,
`runArticulated` maps them to the real `SearchOptions`, and the
result's top-level `articulatedSearch` object records the effective
`heuristicMode`, `heuristicWeight`, `enableCoupledMotion` and
`useSweptAabbPrefilter` and `enableGoalConnector` settings in
exported JSON.

The performance preset comes from one warehouse benchmark and does not
guarantee a result on another map. The default C++ and preexisting HTTP
behavior remain grid-only with weight 1.0.

### Docker UI and regression checks

```bash
BUILD_JOBS=8 docker compose build test
docker compose run --rm test ctest --test-dir build-docker \
  -R '^(articulated_steering_planner_test|demo_articulated_test|demo_articulated_ui_test|demo_http_test)The planner test suite now includes finite-difference checks against
`B'` for three asymmetric/symmetric geometries, randomized feasible
RK4 rollouts with displaced goals inside all configured tolerance
windows, reverse cost multipliers below and above one, and strict
end-to-end result checks for both new modes.

```bash
cmake -S . -B build -DHYBRID_ASTAR_BUILD_TESTS=ON -DHYBRID_ASTAR_BUILD_BENCHMARKS=ON
cmake --build build --target hybrid_astar_articulated_steering_planner_test hybrid_astar_articulated_warehouse_bench -j
ctest --test-dir build -R '^articulated_steering_planner_test$' --output-on-failure
```

The manual warehouse benchmark accepts the original modes plus the
new grid-only weight control. An optional second argument overrides
`SearchOptions::heuristicWeight` for **any** mode, accepting finite
positive weights up to 3.0 (the planner's validated range).

```bash
./build-docker/hybrid_astar_articulated_warehouse_bench coupled
./build-docker/hybrid_astar_articulated_warehouse_bench analytic-only
./build-docker/hybrid_astar_articulated_warehouse_bench combined
./build-docker/hybrid_astar_articulated_warehouse_bench weighted
./build-docker/hybrid_astar_articulated_warehouse_bench grid-weighted
./build-docker/hybrid_astar_articulated_warehouse_bench grid-weighted 1.2
./build-docker/hybrid_astar_articulated_warehouse_bench weighted 1.2
```

`weighted` (Combined) and `grid-weighted` (ObstacleGrid) **both default
to 1.5**. `coupled` (ObstacleGrid) and `combined` (Combined) both
default to 1.0. All four share exactly the same warehouse, kinematics,
primitive set, costs, search budgets and swept-AABB optimization.
The benchmark appends `heuristic_weight`, `plan_cost_m` (the actual
accumulated planner edge cost, not just front driving distance) and
the three terminal goal errors to its pre-existing CSV columns.
Failed plans report zero `plan_cost_m`; exclude failed plans from
path-cost comparisons.

### Docker weight sweep, manually triggered

Run the sweep from the repository root after pulling this feature branch
and explicitly building the existing Docker Compose test image:

```bash
BUILD_JOBS=8 docker compose build test

docker compose run --rm test python3 -m unittest discover \
  -s tests -p 'test_articulated_heuristic_weight_sweep.py' -v

docker compose run --rm test python3 \
  tools/articulated_heuristic_weight_sweep.py --repeats 3 \
  | tee articulated_weight_sweep_summary.csv
```

The script compares `grid-weighted` and `weighted` at **1.0, 1.1, 1.2,
1.3, 1.4, and 1.5**, with three repeated runs of each configuration.
Its stdout is a 12-row CSV reporting valid plan counts, median expansions,
collision checks, front driving distance, **real accumulated path cost**,
stationary and coupled steering, plus mean/median/min/max planning times.
Progress is printed to stderr and does not contaminate the CSV.
The two variants run at adjacent weights to minimize environmental drift.

For a one-off missing-control run:

```bash
docker compose run --rm test \
  ./build-docker/hybrid_astar_articulated_warehouse_bench grid-weighted 1.5
```

To obtain per-run raw rows rather than an aggregated summary, pass
`--raw` to the sweep script and save its output to a separate CSV.
The raw mode performs fresh benchmark runs; it does not reuse earlier
summary measurements. Neither this script nor its tests are scheduled
or registered for automatic CI execution.

The warehouse scenario is one performance sample, not a general runtime
guarantee. A weighted or grid-based heuristic is not an admissibility or
continuous optimality certificate. Always compare independent geometric
validation and goal errors alongside cost and runtime.
 \
  --output-on-failure

docker compose up --build demo
```

The Planning Lab is then available at `http://127.0.0.1:8765`.
Docker Compose publishes the host port on loopback only. No GitHub
Actions workflow is needed.

## Local verification

The planner test suite now includes finite-difference checks against
`B'` for three asymmetric/symmetric geometries, randomized feasible
RK4 rollouts with displaced goals inside all configured tolerance
windows, reverse cost multipliers below and above one, and strict
end-to-end result checks for both new modes.

```bash
cmake -S . -B build -DHYBRID_ASTAR_BUILD_TESTS=ON -DHYBRID_ASTAR_BUILD_BENCHMARKS=ON
cmake --build build --target hybrid_astar_articulated_steering_planner_test hybrid_astar_articulated_warehouse_bench -j
ctest --test-dir build -R '^articulated_steering_planner_test$' --output-on-failure
```

The manual warehouse benchmark accepts the original modes plus the
new grid-only weight control. An optional second argument overrides
`SearchOptions::heuristicWeight` for **any** mode, accepting finite
positive weights up to 3.0 (the planner's validated range).

```bash
./build-docker/hybrid_astar_articulated_warehouse_bench coupled
./build-docker/hybrid_astar_articulated_warehouse_bench analytic-only
./build-docker/hybrid_astar_articulated_warehouse_bench combined
./build-docker/hybrid_astar_articulated_warehouse_bench weighted
./build-docker/hybrid_astar_articulated_warehouse_bench grid-weighted
./build-docker/hybrid_astar_articulated_warehouse_bench grid-weighted 1.2
./build-docker/hybrid_astar_articulated_warehouse_bench weighted 1.2
```

`weighted` (Combined) and `grid-weighted` (ObstacleGrid) **both default
to 1.5**. `coupled` (ObstacleGrid) and `combined` (Combined) both
default to 1.0. All four share exactly the same warehouse, kinematics,
primitive set, costs, search budgets and swept-AABB optimization.
The benchmark appends `heuristic_weight`, `plan_cost_m` (the actual
accumulated planner edge cost, not just front driving distance) and
the three terminal goal errors to its pre-existing CSV columns.
Failed plans report zero `plan_cost_m`; exclude failed plans from
path-cost comparisons.

### Docker weight sweep, manually triggered

Run the sweep from the repository root after pulling this feature branch
and explicitly building the existing Docker Compose test image:

```bash
BUILD_JOBS=8 docker compose build test

docker compose run --rm test python3 -m unittest discover \
  -s tests -p 'test_articulated_heuristic_weight_sweep.py' -v

docker compose run --rm test python3 \
  tools/articulated_heuristic_weight_sweep.py --repeats 3 \
  | tee articulated_weight_sweep_summary.csv
```

The script compares `grid-weighted` and `weighted` at **1.0, 1.1, 1.2,
1.3, 1.4, and 1.5**, with three repeated runs of each configuration.
Its stdout is a 12-row CSV reporting valid plan counts, median expansions,
collision checks, front driving distance, **real accumulated path cost**,
stationary and coupled steering, plus mean/median/min/max planning times.
Progress is printed to stderr and does not contaminate the CSV.
The two variants run at adjacent weights to minimize environmental drift.

For a one-off missing-control run:

```bash
docker compose run --rm test \
  ./build-docker/hybrid_astar_articulated_warehouse_bench grid-weighted 1.5
```

To obtain per-run raw rows rather than an aggregated summary, pass
`--raw` to the sweep script and save its output to a separate CSV.
The raw mode performs fresh benchmark runs; it does not reuse earlier
summary measurements. Neither this script nor its tests are scheduled
or registered for automatic CI execution.

The warehouse scenario is one performance sample, not a general runtime
guarantee. A weighted or grid-based heuristic is not an admissibility or
continuous optimality certificate. Always compare independent geometric
validation and goal errors alongside cost and runtime.
 \
  --output-on-failure

# Same warehouse, same combined heuristic and weight, connector toggled:
docker compose run --rm test \
  ./build-docker/hybrid_astar_articulated_warehouse_bench weighted 1.2
docker compose run --rm test \
  ./build-docker/hybrid_astar_articulated_warehouse_bench connector 1.2

docker compose up --build demo
```

In the browser select **Articulated steering → Combined → weight 1.2**
and opt into **Native articulated goal connector**. The checkbox is off
by default. `articulatedSearch.enableGoalConnector` and
`original.goalConnectorAttempts`, `goalConnectorCandidates`,
`goalConnectorValidated`, and the other connector diagnostics are
included in exported JSON. An accepted connection still means only
independently sampled kinematic/geometry validation, *not* actuator
feasibility, self-collision certification or continuous swept safety.

## Planning Lab web UI (articulated steering only)

Choose **Kinematic model → Articulated steering** and use the separate
**ARTICULATED SEARCH OPTIMIZATION** controls. The original rigid-body
`performance`, Reeds–Shepp and refinement widgets remain disabled for
this model. The options are independent:

- **Heuristic mode:** Obstacle grid (`grid`, default), Analytical
  (`analytic`), or Combined (`combined`, recommended for the measured
  warehouse experiment).
- **Heuristic weight:** editable in `(0,3]`. It initially remains `1.0`
  for the historical grid mode. Selecting Combined automatically presets
  `1.2`; users may override the preset. Switching to other modes
  presets `1.0`.
- **Coupled steering while driving:** checked by default.
- **Swept-AABB free-space prefilter:** checked by default; disabling it
  restores sampled collision checks where the optimization would have
  proven empty free space.

The browser sends these values as `artHeuristicMode`,
`artHeuristicWeight`, `artCoupledMotion`, and `artSweptAabb` in the
`/api/plan` form request. `apps/demo/main.cpp` validates them,
`runArticulated` maps them to the real `SearchOptions`, and the
result's top-level `articulatedSearch` object records the effective
`heuristicMode`, `heuristicWeight`, `enableCoupledMotion` and
`useSweptAabbPrefilter` settings in exported JSON.

The performance preset comes from one warehouse benchmark and does not
guarantee a result on another map. The default C++ and preexisting HTTP
behavior remain grid-only with weight 1.0.

### Docker UI and regression checks

```bash
BUILD_JOBS=8 docker compose build test
docker compose run --rm test ctest --test-dir build-docker \
  -R '^(articulated_steering_planner_test|demo_articulated_test|demo_articulated_ui_test|demo_http_test)The planner test suite now includes finite-difference checks against
`B'` for three asymmetric/symmetric geometries, randomized feasible
RK4 rollouts with displaced goals inside all configured tolerance
windows, reverse cost multipliers below and above one, and strict
end-to-end result checks for both new modes.

```bash
cmake -S . -B build -DHYBRID_ASTAR_BUILD_TESTS=ON -DHYBRID_ASTAR_BUILD_BENCHMARKS=ON
cmake --build build --target hybrid_astar_articulated_steering_planner_test hybrid_astar_articulated_warehouse_bench -j
ctest --test-dir build -R '^articulated_steering_planner_test$' --output-on-failure
```

The manual warehouse benchmark accepts the original modes plus the
new grid-only weight control. An optional second argument overrides
`SearchOptions::heuristicWeight` for **any** mode, accepting finite
positive weights up to 3.0 (the planner's validated range).

```bash
./build-docker/hybrid_astar_articulated_warehouse_bench coupled
./build-docker/hybrid_astar_articulated_warehouse_bench analytic-only
./build-docker/hybrid_astar_articulated_warehouse_bench combined
./build-docker/hybrid_astar_articulated_warehouse_bench weighted
./build-docker/hybrid_astar_articulated_warehouse_bench grid-weighted
./build-docker/hybrid_astar_articulated_warehouse_bench grid-weighted 1.2
./build-docker/hybrid_astar_articulated_warehouse_bench weighted 1.2
```

`weighted` (Combined) and `grid-weighted` (ObstacleGrid) **both default
to 1.5**. `coupled` (ObstacleGrid) and `combined` (Combined) both
default to 1.0. All four share exactly the same warehouse, kinematics,
primitive set, costs, search budgets and swept-AABB optimization.
The benchmark appends `heuristic_weight`, `plan_cost_m` (the actual
accumulated planner edge cost, not just front driving distance) and
the three terminal goal errors to its pre-existing CSV columns.
Failed plans report zero `plan_cost_m`; exclude failed plans from
path-cost comparisons.

### Docker weight sweep, manually triggered

Run the sweep from the repository root after pulling this feature branch
and explicitly building the existing Docker Compose test image:

```bash
BUILD_JOBS=8 docker compose build test

docker compose run --rm test python3 -m unittest discover \
  -s tests -p 'test_articulated_heuristic_weight_sweep.py' -v

docker compose run --rm test python3 \
  tools/articulated_heuristic_weight_sweep.py --repeats 3 \
  | tee articulated_weight_sweep_summary.csv
```

The script compares `grid-weighted` and `weighted` at **1.0, 1.1, 1.2,
1.3, 1.4, and 1.5**, with three repeated runs of each configuration.
Its stdout is a 12-row CSV reporting valid plan counts, median expansions,
collision checks, front driving distance, **real accumulated path cost**,
stationary and coupled steering, plus mean/median/min/max planning times.
Progress is printed to stderr and does not contaminate the CSV.
The two variants run at adjacent weights to minimize environmental drift.

For a one-off missing-control run:

```bash
docker compose run --rm test \
  ./build-docker/hybrid_astar_articulated_warehouse_bench grid-weighted 1.5
```

To obtain per-run raw rows rather than an aggregated summary, pass
`--raw` to the sweep script and save its output to a separate CSV.
The raw mode performs fresh benchmark runs; it does not reuse earlier
summary measurements. Neither this script nor its tests are scheduled
or registered for automatic CI execution.

The warehouse scenario is one performance sample, not a general runtime
guarantee. A weighted or grid-based heuristic is not an admissibility or
continuous optimality certificate. Always compare independent geometric
validation and goal errors alongside cost and runtime.
 \
  --output-on-failure

docker compose up --build demo
```

The Planning Lab is then available at `http://127.0.0.1:8765`.
Docker Compose publishes the host port on loopback only. No GitHub
Actions workflow is needed.

## Local verification

The planner test suite now includes finite-difference checks against
`B'` for three asymmetric/symmetric geometries, randomized feasible
RK4 rollouts with displaced goals inside all configured tolerance
windows, reverse cost multipliers below and above one, and strict
end-to-end result checks for both new modes.

```bash
cmake -S . -B build -DHYBRID_ASTAR_BUILD_TESTS=ON -DHYBRID_ASTAR_BUILD_BENCHMARKS=ON
cmake --build build --target hybrid_astar_articulated_steering_planner_test hybrid_astar_articulated_warehouse_bench -j
ctest --test-dir build -R '^articulated_steering_planner_test$' --output-on-failure
```

The manual warehouse benchmark accepts the original modes plus the
new grid-only weight control. An optional second argument overrides
`SearchOptions::heuristicWeight` for **any** mode, accepting finite
positive weights up to 3.0 (the planner's validated range).

```bash
./build-docker/hybrid_astar_articulated_warehouse_bench coupled
./build-docker/hybrid_astar_articulated_warehouse_bench analytic-only
./build-docker/hybrid_astar_articulated_warehouse_bench combined
./build-docker/hybrid_astar_articulated_warehouse_bench weighted
./build-docker/hybrid_astar_articulated_warehouse_bench grid-weighted
./build-docker/hybrid_astar_articulated_warehouse_bench grid-weighted 1.2
./build-docker/hybrid_astar_articulated_warehouse_bench weighted 1.2
```

`weighted` (Combined) and `grid-weighted` (ObstacleGrid) **both default
to 1.5**. `coupled` (ObstacleGrid) and `combined` (Combined) both
default to 1.0. All four share exactly the same warehouse, kinematics,
primitive set, costs, search budgets and swept-AABB optimization.
The benchmark appends `heuristic_weight`, `plan_cost_m` (the actual
accumulated planner edge cost, not just front driving distance) and
the three terminal goal errors to its pre-existing CSV columns.
Failed plans report zero `plan_cost_m`; exclude failed plans from
path-cost comparisons.

### Docker weight sweep, manually triggered

Run the sweep from the repository root after pulling this feature branch
and explicitly building the existing Docker Compose test image:

```bash
BUILD_JOBS=8 docker compose build test

docker compose run --rm test python3 -m unittest discover \
  -s tests -p 'test_articulated_heuristic_weight_sweep.py' -v

docker compose run --rm test python3 \
  tools/articulated_heuristic_weight_sweep.py --repeats 3 \
  | tee articulated_weight_sweep_summary.csv
```

The script compares `grid-weighted` and `weighted` at **1.0, 1.1, 1.2,
1.3, 1.4, and 1.5**, with three repeated runs of each configuration.
Its stdout is a 12-row CSV reporting valid plan counts, median expansions,
collision checks, front driving distance, **real accumulated path cost**,
stationary and coupled steering, plus mean/median/min/max planning times.
Progress is printed to stderr and does not contaminate the CSV.
The two variants run at adjacent weights to minimize environmental drift.

For a one-off missing-control run:

```bash
docker compose run --rm test \
  ./build-docker/hybrid_astar_articulated_warehouse_bench grid-weighted 1.5
```

To obtain per-run raw rows rather than an aggregated summary, pass
`--raw` to the sweep script and save its output to a separate CSV.
The raw mode performs fresh benchmark runs; it does not reuse earlier
summary measurements. Neither this script nor its tests are scheduled
or registered for automatic CI execution.

The warehouse scenario is one performance sample, not a general runtime
guarantee. A weighted or grid-based heuristic is not an admissibility or
continuous optimality certificate. Always compare independent geometric
validation and goal errors alongside cost and runtime.
