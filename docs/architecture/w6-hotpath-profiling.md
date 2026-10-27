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
