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
