# W6: label hashing, collision, and queue hot-path profiling

The default planner is unchanged in its geometric and sampled validation
contracts. This milestone adds deterministic call counters and optional
steady-clock nanosecond measurements.

## Instrumented stages

- Label map: finds, hits, inserts, updates, rehashes, peak buckets and labels
- Key hashing: call counts and nanoseconds (profiling build only)
- Collision: exact whole-body pose evaluations and nanoseconds, including
  colliding/failed samples
- Motion: search and final-validation section calls and nanoseconds
- Priority queues: main and reverse-grid push/pop nanoseconds, alongside
  existing stale-pop and peak-storage metrics

**Nested measurements are not additive:** hash time is part of label lookup
and write time; exact collision time is part of motion-sampling time.
The probes add overhead, so do not use profiled end-to-end runtime alone to
choose a production configuration.

## Source-level improvements and controlled experiments

1. Replace redundant label lookup on a successful update with a write
   through the already retrieved iterator. Insert with `emplace` only
   for a new discrete search key. Dominance, reopening and tie breaking
   are unchanged.
2. Cache map-origin sine and cosine once per exact checker. Every pose still
   receives its own yaw transformation and full-footprint collision test.
3. Allow explicit initial hash-table reservation using
   `PlanOptions::searchLabelReserve`. Default 0 preserves allocation
   behavior; values must not exceed `maxGeneratedNodes`. Compare
   4096/16384 entries to no reservation before selecting a new default.
4. Compare optional 4-disc conservative prefilter against exact checking.
   A supposedly faster configuration must preserve planner outcomes.

## Run and compare

Profiled build:

```bash
cmake -S . -B build-hotpath -DCMAKE_BUILD_TYPE=Release \
  -DHYBRID_ASTAR_BUILD_TESTS=ON -DHYBRID_ASTAR_BUILD_BENCHMARKS=ON \
  -DHYBRID_ASTAR_PROFILE_HOTPATHS=ON
cmake --build build-hotpath --parallel
ctest --test-dir build-hotpath --output-on-failure
./build-hotpath/hybrid_astar_hotpath_bench 7 > hotpath-profile.csv
```

A second build with `-DHYBRID_ASTAR_PROFILE_HOTPATHS=OFF` is mandatory
for latency comparison without per-operation clock overhead. Its timing
fields are zero by design. The benchmark checks equivalent status,
expansions, generated nodes, sections, weighted cost, strict collision
evidence and map revision. It reports per-stage timing, p50/p95, and counters
for seven hard scenarios over variants: baseline, reserve_4096,
reserve_16384, disc_4 and reserve_4096_disc_4.

Catch2 tests check result invariance, bad-reserve rejection and
translated/rotated-map exact-collision equivalence.

## Opt-in warehouse and heuristic A/B experiments (dev)

`hybrid_astar_hotpath_bench` now has three **manual-only** modes, in
addition to the historical five-variant hard-scenario benchmark:

```bash
# Run mecanum warehouse start=(4.77,5.62,0), goal=(31.01,3.41,-90deg)
# at the 1M expansion ceiling; run each of three prefilter/hash variants
# once (no warm-up to avoid doubling expensive work).
./build-hotpath/hybrid_astar_hotpath_bench --warehouse 1 1000000 > warehouse.csv

# Compare certified Euclidean vs obstacle-grid ordering on each of the
# seven existing hard scenarios. Same map, model, budget and weight=1.0.
./build-hotpath/hybrid_astar_hotpath_bench --heuristics 7 > heuristics.csv

# Same heuristic A/B, but only for the difficult mecanum warehouse case.
./build-hotpath/hybrid_astar_hotpath_bench --warehouse-heuristics 1 1000000 \
    > warehouse-heuristics.csv
```

For quick experiments, reduce the optional warehouse expansion budget, e.g.
`--warehouse 1 10000`; this can end at `BudgetExceeded` rather than a
validated solution. Warehouse runs are never registered as a scheduled
benchmark or as a new CI/CTest job; the existing W12 warehouse correctness
regression remains in CTest.

All benchmark modes share CSV columns. Additional fields are `model`,
`heuristic`, `max_expansions`, `grid_settlements`, `grid_fallbacks`,
`terminal_state_attempts`, `terminal_candidates`,
`terminal_validated` and `terminal_rejected`. A terminal rejection is
an attempted direct connection that did **not** pass the required full
search/final validation; it must not be interpreted as a continuous-space
impossibility proof. CSV `status` is the numeric `PlanStatus` enum
(`0=Success`, `6=BudgetExceeded`, `9=StrictValidationFailed`).

In a heuristic comparison, success/strict-collision evidence and budgets
remain required, but the expanded nodes, first-found path and weighted cost
**may differ** legitimately. For collision prefilter/hash-table A/B, the
benchmark asserts equal status, expansions, generated nodes, cost, section
count and strict evidence. `p50_ms`/`p95_ms` are measured wall-clock
latencies, while individual stage nanosecond fields describe the last run;
timings are nested rather than additive.

### Docker Compose

Docker profiling remains disabled by default. To enable it for a standalone
benchmark image and redirect CSV cleanly from the following run:

```bash
PROFILE_HOTPATHS=ON BUILD_JOBS=8 docker compose build test
PROFILE_HOTPATHS=ON docker compose run --rm test \
    ./build-docker/hybrid_astar_hotpath_bench --warehouse 1 1000000 \
    > warehouse.csv
PROFILE_HOTPATHS=ON docker compose run --rm test \
    ./build-docker/hybrid_astar_hotpath_bench --heuristics 7 \
    > heuristics.csv
```

Repeat with `PROFILE_HOTPATHS=OFF` for overhead-free median/p95
wall-clock comparisons. Do not mix profiled and unprofiled runs when
comparing internal stage counters.

## Default heuristic selection (2026-10-27)

Following the user's warehouse and controlled heuristic A/B measurements,
`PlanOptions` now defaults to `PerformanceReedsSheppAndObstacleGrid` with
`performanceHeuristicWeight=1.0`. The small-path grid overhead measured in
seven fixtures is outweighed by the large obstacle-aware expansion reduction
in the warehouse reproduction. This is a *performance* default, not an
admissibility or optimality claim. The same exact-collision final validator
is used in both modes.

Certified Euclidean ordering remains explicitly available:

```cpp
request.options.heuristicMode =
    hybrid_astar::ackermann::HeuristicMode::CertifiedRegionEuclidean;
```

Planning Lab starts with its grid checkbox checked, but allows it to be
disabled. The CLI accepts `--certified`, and the HTTP API accepts
`performance=0`. The default-setting change has **not yet been verified**
by a full Docker/CTest regression run; do not claim all tests pass.

## Verification status

The expanded benchmark, warehouse connector diagnostics and targeted
collision-safety tests are committed on `dev`, but the new revisions have
**not** completed a full local build or Docker/CTest run in this environment.
User-supplied earlier hard-scenario timings are historical baseline evidence,
not confirmation of the new warehouse results. Docker and
Catch2 v3 are unavailable here and the entire checkout is not locally
mounted. The next acceptance gate is a profiling-ON build plus a
profiling-OFF latency run on the same host. Do not claim a dominant
hot path, a measured speedup, or memory improvement before those runs.

## Ackermann warehouse hot-path profiling (manual only, 2026-10-27)

Two Planning Lab configurations are preserved in
`bench/warehouse_scenarios.hpp`, sharing the app's 200×160 cell warehouse
layout and the default full Ackermann footprint:

- `ack_across_first_margin_050`: start (4.77, 32.84, 0°), goal
  (44.62, 33.53, −90°), 0.50 m inflated footprint; reported success
  after roughly 428,853 expansions.
- `ack_across_improve_margin_050`: same as above, with bounded incumbent
  improvement enabled for 700 expansions after first strict success.
- `ack_corner_margin_000`, `ack_corner_margin_010`,
  `ack_corner_margin_050`: start (3.25, 3.64, 0°), goal
  (30.93, 38.17, −90°), varying inflation. The original screenshot
  reported a 1M-expansion budget failure but did **not** show its margin.
  These are experiments, not claims of matching the unknown margin.

The Planning Lab "planning time" includes strict post-path refinement if
enabled. In the successful screenshot refinement hit its own
`WorkBudgetExceeded` limit **after a validated search path had been found**.
Therefore the pictured milliseconds divided by search expansions are
only an *all-in* estimate, not the mean cost of a search expansion. The
manual refiner benchmark can separate those phases:

```bash
BUILD_JOBS=28 PROFILE_HOTPATHS=OFF docker compose run --build --rm test \
  ./build-docker/hybrid_astar_refinement_bench \
  --warehouse-ackermann 5 1000000 > ackermann_search_vs_refine.csv
```

The CSV reports independently measured one-time search wall time and
p50/p95 strict-refinement wall time, cost, accepted/rejected windows,
strict collision pose checks and budget status. It uses the exact
`ack_across_improve_margin_050` screenshot request and the Planning Lab
35-attempt/3-accepted-refinement settings. Like the other profile modes,
it is manual only.

These **are not run by CTest or GitHub Actions**. Execute the same
single case with profiling disabled (reliable wall-clock comparisons)
and enabled (diagnosis of collision/queue/hash/search stages):

```bash
# Inside the repo root on your machine; shell redirection saves CSV on host.
BUILD_JOBS=28 PROFILE_HOTPATHS=OFF docker compose run --build --rm test \
  ./build-docker/hybrid_astar_hotpath_bench \
  --ackermann-warehouse-heuristics 3 1000000 \
  ack_across_first_margin_050 > ackermann_off.csv

BUILD_JOBS=28 PROFILE_HOTPATHS=ON docker compose run --build --rm test \
  ./build-docker/hybrid_astar_hotpath_bench \
  --ackermann-warehouse-heuristics 3 1000000 \
  ack_across_first_margin_050 > ackermann_profiled.csv

# Investigate the troublesome far-corner start with varying margins.
BUILD_JOBS=28 PROFILE_HOTPATHS=OFF docker compose run --build --rm test \
  ./build-docker/hybrid_astar_hotpath_bench \
  --ackermann-warehouse-heuristics 3 1000000 > ackermann_matrix.csv
```

`--ackermann-warehouse` instead of
`--ackermann-warehouse-heuristics` compares the exact baseline against
four covering discs, and against discs plus reserved label buckets.
The `[repeats] [max_expansions] [optional_exact_case_name]` arguments
allow isolating expensive cases. Every row retains the previous CSV
fields and appends the actual start/goal, inflation, reopened/pruned
labels, strict goal candidates, open/obstacle-grid queue peaks and storage.
`profile_timers_on` indicates timing instrumentation. Timer groups
are **nested**, not additive. Do not compare profiled total wall time
directly with nonprofiled total time as a configuration A/B.

For each case, compare **number of expansions and generated labels**,
`grid_settlements`/`grid_fallbacks`, collision pose calls,
`search_motion_ns`/`exact_collision_ns`, queue and hash time.
If the long planning time comes from huge expansions rather than
slow individual collision tests, prefer investigating obstacle-grid
guidance and missing nonholonomic goal connections (optional OMPL
Reeds–Shepp) before micro-optimizing hash lookups.
Neither benchmark automatically invokes OMPL; use the separately
supported demo `--ompl` or OMPL hard-case benchmark for analytic
connector comparisons. No measured results have been collected
for these new fixtures yet.


## Ackermann collision broad-phase and factorial A/B (manual, October 2026)

The **same full rectangular** Ackermann collision footprint is used with
or without prefiltering. Four conservative covering discs can only prove
the occupied grid is clear of the entire inflated rectangle; ambiguous
poses fall back to the existing SAT + continuous shape-to-cell distance
checker. No collision decision, path objective, or validation rule changes.

The new opt-in `PlanOptions::collectCollisionWork` (default **false**)
aggregates request-local counts in `PlanResult::diagnostics.collisionWork`:

- `candidateCells`: map cells actually visited in the inflated body AABB;
  excludes grid-distance preprocessing and cells after an early collision
- `occupiedCells`: visited cells that passed the occupancy threshold;
  equals `satTests` because every such cell reaches the rectangle SAT
- `distanceTests`: continuous square-to-rectangle distance evaluations
  after an SAT non-overlap, performed only for a positive inflation margin
- `boundaryRejections`: checker calls rejected at the finite map boundary
  before the occupancy-grid scan

Counters include collision checker calls from the search, start and strict
final validation (including exact fallbacks); prefilter-clear samples
**do not** count as an exact check. These are integer **work counts**, not
nanosecond timings or counts of complete robot motions. Some full-path
mecanum postprocessing or separately invoked refiners are outside the
planner's aggregate. All are zero when `collectCollisionWork=false`.
Recording each visited cell adds overhead, so do **not** use the work
counting mode for unbiased timing A/B.

The manual hot-path benchmark now supports two equivalent, controlled
2×2×2 experiment matrices on the same exact warehouse request:

- heuristic: certified Euclidean vs obstacle-grid (weight 1.0)
- footprint inflation: 0.0 vs 0.50 m
- conservative-disc prefilter: 0 vs 4 discs

Each matrix automatically asserts **identical** status, expansions,
generated nodes, number of path sections and weighted cost for its
prefilter-off/on pair. Results under different heuristics or margins
can legitimately differ. The default fixtures are the photographed
successful side-to-side Ackermann case and difficult corner-to-corner
case. Filter by a fixture name to reduce the number of runs.

```bash
# Wall clock, instrumentation OFF. Output redirects on the host.
BUILD_JOBS=28 PROFILE_HOTPATHS=OFF docker compose run --build --rm test \
  ./build-docker/hybrid_astar_hotpath_bench \
  --ackermann-collision-ab 3 1000000 \
  ack_across_first_margin_050 > ack_collision_timing.csv

# Work counts, separate measurement on identical scenario and variant matrix.
BUILD_JOBS=28 PROFILE_HOTPATHS=OFF docker compose run --build --rm test \
  ./build-docker/hybrid_astar_hotpath_bench \
  --ackermann-collision-work 3 1000000 \
  ack_across_first_margin_050 > ack_collision_work.csv

# Corner-to-corner case: potentially reaches the expansion limit.
BUILD_JOBS=28 PROFILE_HOTPATHS=OFF docker compose run --build --rm test \
  ./build-docker/hybrid_astar_hotpath_bench \
  --ackermann-collision-work 1 1000000 \
  ack_corner_margin_000 > ack_collision_corner.csv
```

For an apples-to-apples **per-pose** cost comparison, the same manual
binary also has a fixed-pose microbenchmark. It creates a deterministic
set of Ackermann poses with identical world XY and yaw on the unchanged
warehouse map, then compares zero versus 0.50 m inflation and zero versus
four conservative discs:

```bash
BUILD_JOBS=28 PROFILE_HOTPATHS=OFF docker compose run --build --rm test \
  ./build-docker/hybrid_astar_hotpath_bench \
  --ackermann-collision-micro 5 > ack_collision_fixed_poses.csv
```

This microbenchmark measures only exact collision checks and optional
prefilter queries. It excludes EDT/disc-prefilter construction, map loading,
the search loop and label/queue operations from timed loops. Grid-cell
counters are collected in a separate untimed verification pass. Each
prefilter-clear verdict is checked against a full exact oracle; every
other checker verdict must match that oracle. The fixed poses remain
identical across margins, while collision outcomes legitimately change.
`exact_calls`, `candidate_cells`, `sat_tests` and
`distance_tests` reveal the per-pose effect without changes in
expansions or successor distribution. The benchmark also reports the
timed and verification collision counts as an internal consistency check.

With no case-name filter, each mode executes **both distinct route
configurations** (16 matrix rows; repeat count applies to every row).
The printed `scenario` is the original pose pair; read
`footprint_margin_m` and `heuristic` and `variant` to know the
**actually applied** experimental settings. The original far-corner
screenshot's inflation was unknown, which is why both zero and 0.50 m
are tested. The CSV schema appends
`collision_work_enabled,collision_candidate_cells,collision_occupied_cells,
collision_sat_tests,collision_distance_tests,collision_boundary_rejections`
to all pre-existing columns.

Interpretation: `candidate_cells / exact_collision_pose_calls` estimates
the **observed** grid broad-phase scan per exact checker invocation. The
mean includes rejected poses that exit early and checker calls that exit
at the map boundary. `distance_tests / exact_collision_pose_calls`
quantifies additional exact distance work introduced by inflation. Compare
`disc_clear` with `disc_exact_fallback` and count reductions between
disc 0 and 4. Compare `expanded`, `generated`, and grid heuristic
settlements between Euclidean and grid-guided modes to identify search
effort separately from per-collision cost.

`profile_timers_on` remains a separate flag for the existing nested
hotpath clock timers, which are enabled by the CMake profile build
option. This experiment needs neither profiled clocks nor a CTest
registration. No timing or speedup conclusions are asserted until
real CSVs are collected.
 
## Ackermann default cover and exact empty-AABB bypass (October 2026)

Four-disc filtering is now the automatic Ackermann default while remaining
disabled by default for differential-drive/mecanum; the planner's explicit
`discPrefilterCount=0` disables it for controlled experiments.
`discPrefilterCount=PlanOptions::AutomaticDiscPrefilter` resolves the
model-specific default. The obstacle-grid heuristic remains the default.

The optional but Ackermann-default **occupancy prefix sum** is an
`(width+1) × (height+1)` summed-area table built once per request's
checker from the immutable map and effective occupancy policy. Unknown
cells are blocked iff the policy says so; threshold equality is free.
A rotated rectangular robot's *inflated* axis-aligned map-grid bounding
box is queried in O(1) after checking its finite-map boundary. A box
with **zero** blocked cells guarantees that exact rectangle-vs-cell
collision is impossible. Otherwise the original exact SAT and
positive-margin Euclidean cell-distance checks are used, unchanged.
No false negative is allowed; collision checking stays sampled at the
same spatial/body-point density as before.

The option `useEmptyRegionPrefixSum` defaults on for Ackermann only;
disable it explicitly to benchmark the old full scanner. Direct standalone
`ExactCollisionChecker` instances default to no table, retaining the
original lightweight construction unless explicitly requested.
The table takes O(width×height) memory and build time. Unlike the
precomputed conservative distance transform, its construction is simple
but can be wasteful on very short queries. For that reason a dedicated
**easy-map** A/B was added rather than assuming a universal speedup.

Experiment 1 — easy, open map, includes *whole planner* and precomputation
cost for both the prefilter and the prefix sum:

```bash
BUILD_JOBS=28 PROFILE_HOTPATHS=OFF docker compose run --build --rm test \
  ./build-docker/hybrid_astar_hotpath_bench \
  --ackermann-easy-ab 7 > ack_easy_ab.csv
```

Experiment 2 — same Ackermann warehouse pose and margin, prefix on/off,
both with and without four discs. Work counters are enabled in both arms,
so use this mainly for exact candidate-cell work counts and within-mode
timing comparisons, not timing versus noninstrumented earlier CSV files:

```bash
BUILD_JOBS=28 PROFILE_HOTPATHS=OFF docker compose run --build --rm test \
  ./build-docker/hybrid_astar_hotpath_bench \
  --ackermann-empty-region-ab 3 1000000 \
  ack_across_first_margin_050 > ack_prefix_ab.csv
```

Experiment 3 — identical fixed poses independent of heuristic/search.
Now includes **prefix enabled** as a fourth matrix dimension;
its timing loops exclude checker construction, while whole-plan
benchmarks include it:

```bash
BUILD_JOBS=28 PROFILE_HOTPATHS=OFF docker compose run --build --rm test \
  ./build-docker/hybrid_astar_hotpath_bench \
  --ackermann-collision-micro 7 > ack_prefix_micro.csv
```

The legacy `--ackermann-collision-ab` /
`--ackermann-collision-work` modes still explicitly turn the prefix
optimization OFF to keep historical CSVs comparable, while varying
heuristics, inflation, and disc covers as before.
All benchmark CSV columns are preserved, with added
`effective_disc_count`, `empty_region_prefix_enabled`,
`empty_region_queries` and `empty_region_bypasses`.
The prefix mode asserts the same status, expansions, generated nodes,
sections and weighted path cost as the original scan, and refuses
any prefix run that increases the number of actual grid-cell visits.

Results for the *new* prefix-sum optimization have not yet been
measured in Docker; retaining the toggle makes it easy to disable
if the overhead outweighs the saved scans.

