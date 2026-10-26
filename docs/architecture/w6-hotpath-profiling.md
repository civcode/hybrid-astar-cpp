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

## Verification status

The source changes, benchmark and tests are committed, but **actual hotpath
timing results have not been collected in this environment**. Docker and
Catch2 v3 are unavailable here and the entire checkout is not locally
mounted. The next acceptance gate is a profiling-ON build plus a
profiling-OFF latency run on the same host. Do not claim a dominant
hot path, a measured speedup, or memory improvement before those runs.
