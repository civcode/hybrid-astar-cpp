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

The warehouse benchmark supports the unchanged historical modes and
three new A/B modes:

```bash
./build/hybrid_astar_articulated_warehouse_bench coupled
./build/hybrid_astar_articulated_warehouse_bench analytic-only
./build/hybrid_astar_articulated_warehouse_bench combined
./build/hybrid_astar_articulated_warehouse_bench weighted
```

`weighted` is the same combined heuristic multiplied by 1.5;
all four of these commands enable coupled steering and swept-AABB
optimization with the same map, cost model and limits. Compare status,
independent sampled validation, expansions, generated labels, collision
checks, elapsed time and physical motion metrics. Do not interpret
the target of fewer than 100k expansions or under 2 seconds as an
established result until the matched benchmark runs.
