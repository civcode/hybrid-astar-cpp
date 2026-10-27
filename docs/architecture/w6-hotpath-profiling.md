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

