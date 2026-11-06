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

`HeuristicMode::CombinedCoarseGrid` is opt-in. The existing lazy
obstacle-grid heuristic can hit its total 100,000-settlement limit
and fall back to Euclidean distance; the new mode fully settles a
conservatively downsampled reverse-Dijkstra field once per request.
Queries are constant time. `coarseGridResolutionMeters` defaults to
0.15 m and rounds up to a multiple of native map resolution.

Any coarse tile containing a blocked or nontraversable-unknown fine
cell becomes blocked. Eight-connected moves forbid diagonal corner
cutting. The goal XY tolerance disk seeds the field. If this
conservative rule disconnects a real narrow passage, the query
falls back to Euclidean plus analytical articulated guidance.
The coarse field NEVER establishes collision freedom or NoPath.
All actual search, strict replay and two-carriage collision checks
use the unmodified high-resolution occupancy map. The field is
request-local (not cached across map edits) and has a two-million
coarse-cell auxiliary allocation guard.

JSON diagnostics include `coarseGridCells`,
`coarseGridBlockedCells`, `coarseGridSeedCells`,
`coarseGridSettlements`, `coarseGridQueries`,
`coarseGridFallbacks`, `coarseGridResolutionMeters` (effective),
`coarseGridBuildMilliseconds` and `coarseGridSkipped`.

## Native articulated goal connector

`SearchOptions::enableGoalConnector` is off by default. When enabled,
the pre-solution scheduler makes up to 24 bounded attempts. The
first four attempts are distance-unrestricted; later attempts
are reserved for improving near-goal four-state approach poses.
Failed proposals fall back to regular Hybrid A* search.

Unlike rigid Reeds–Shepp trajectories, each connector segment
uses the articulated no-slip model with bounded front velocity
and articulation rate. Stationary articulation pivots the rear
body around a fixed front reference (it is NOT an in-place yaw).
Constant-gamma driving has exact circular-arc integration; coupled
steering and driving uses analytic heading with numerical front-XY
quadrature only for proposing candidates. The final complete path
is independently reintegrated with fine RK4 and both exact body
footprints are collision-checked. A proposal is not a proof.

### Simpler maneuver families and quality polishing

The terminal generator now tries one-arc and two-arc forward/reverse
families first, then the full three-arc family. Each stage supports
either stationary-then-drive or finite-rate coupled steering.
The same 3,000 solver evaluations per attempt are divided into
a first tranche (~1/8) for one-arc attempts, up to ~1/3 total for
one-/two-arc attempts, and the remainder for unrestricted three-arc
fallback. The complex family is not forbidden or removed.

For each candidate, nonlinear fitting first minimizes terminal
XY/rear-yaw error and a sampled *soft* two-body clearance penalty.
Once a feasible fit is found, damped endpoint-nullspace descent
polishes a **separate operational quality score**. That score
includes the existing driving/steering planner cost plus preferences
for lower stationary steering duration, total articulation travel,
articulation-direction reversals, gear changes, steering excursions
near mechanical limits, and excessive drive-arc fragmentation.
Polishing never deliberately sacrifices endpoint feasibility, and
only full RK4/two-carriage replay can accept a candidate.

Operational quality is used to fit and rank proposals, but is
NOT silently added to `costMeters`. The anytime incumbent is still
replaced only when the new independently validated full path has
lower actual planner cost. Consequently, a simpler but slightly
more expensive proposal may be retained as an alternative candidate
but will not replace a cheaper incumbent. This protects the prior
non-worsening cost guarantee; it does not certify global optimality.

Returned-tail JSON diagnostics (zero for a plain search solution):
`goalConnectorTailDrivingMeters`, `goalConnectorTailReverseMeters`,
`goalConnectorTailArticulationRadians`,
`goalConnectorTailStationarySeconds`,
`goalConnectorTailPeakGammaRadians`,
`goalConnectorTailGearChanges`,
`goalConnectorTailArticulationReversals`,
`goalConnectorTailDriveArcs`,
`goalConnectorTailFittedArcs`,
`goalConnectorTailOperationalPenaltyMeters` and
`goalConnectorTailQualityScoreMeters`. Candidate counts by one-,
two- and three-arc families are also exported. Chunking long
controls for validation does not count as additional gear or
articulation reversals.

## Bounded anytime connector refinement

Opting into `enableAnytimeGoalRefinement` retains the first
**strictly validated** path and searches up to 2,000 extra
expansions by default. At most eight additional connector attempts
may be made, also subject to the global attempt, solver evaluation,
collision-check and generation/expansion budgets. Cancellation
remains cancellation; no invalid path becomes a successful result.

The extra shots now use expansion slots across the full refinement
window: with 2,000 expansions and eight shots, slots are around
+222, +444, ..., +1778 expansions after first success. Exceptionally
promising lower-bound states may take a shot up to approximately
44 expansions early. The state must still be near the goal and
have a possible total cost below the incumbent. An unused slot
does not force shooting from an unpromising pose.

Diagnostics include `anytimePacedDeferrals`,
`anytimeFirstExtraShotExpansion`,
`anytimeLastExtraShotExpansion`,
`anytimeFirstSolutionExpansion`, `anytimeExtraExpansions`,
`anytimeExtraConnectorAttempts`, `anytimeValidatedCandidates`,
`anytimeImprovements`, `anytimeCostBoundPrunes`,
`anytimeFirstCostMeters`, and `anytimeBestCostMeters`.
The fastest first path remains available if refinement runs out
of its allowed search work.

## Planning Lab and regression

In the browser the articulated heuristic weight starts at 1.2.
The C++/HTTP defaults retain weight 1.0 and grid-only heuristic
for compatibility. The recommended coarse mode keeps the native
connector optional, and anytime improvement is separately opt-in.
The UI reports selected-tail steering effort and includes the
complete results in exported JSON.

Run manual validation locally (no GitHub Actions):

```bash
git pull --ff-only
BUILD_JOBS=8 docker compose build test
docker compose run --rm test ctest --test-dir build-docker \
  -R '^(articulated_steering_planner_test|demo_articulated_test|demo_articulated_ui_test|demo_http_test)$' \
  --output-on-failure

# Same synthetic warehouse/goal/weight, different heuristics:
for mode in anytime coarse-anytime; do
  docker compose run --rm test \
    ./build-docker/hybrid_astar_articulated_warehouse_bench \
    "$mode" 1.2 uploaded-heading 2000
done
docker compose up --build demo
```

That synthetic map has 0.25 m cells. To test 0.12/0.15/0.18 m
coarse resolutions meaningfully, use the original Nav2 0.03 m
full warehouse map and the same start/goal in Planning Lab.

The previous uploaded full-map run with start (-10.70, 1.63, 0°),
goal (12.97, -23.29, 180°) and the coarse heuristic took 646.7 ms
with 4,875 expansions. Its terminal connector drove ~7.97 m
with ~18 s stationary articulation, ~1.33 m reverse driving
and final path planner cost ~51.46 m. **Those are the old connector
baseline measurements, not results of the present quality fix.**
Compare driving distance, steering time, reversals, goal errors,
validated cost, expansions and total runtime after rebuilding.

All guidance and connector outputs remain performance-oriented
geometric results: no globally optimal, actuator-feasibility or
continuous swept-volume certification is claimed.
