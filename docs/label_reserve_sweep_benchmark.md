# Nav2 A* state-label reservation sweep (manual)

## Motivation

The earlier paired benchmark showed that
`PlanOptions.searchLabelReserve=262144` reduced search latency **4.85%
median** across 20/20 warehouse comparisons while preserving search
and refinement results. Median rehashes dropped from **14 to zero**,
at an estimated median additional **0.80 MiB** of hash bucket arrays.
See [measured Nav2 findings](warehouse_performance_findings_2026-11-04.md)
for the original CSV sources and methodology.

That observation does **not** prove that reserving 262,144 labels is
optimal, particularly for small maps/short searches. The next controlled
experiment compares **four** settings without changing the planner default:

| Configuration | `searchLabelReserve` | Purpose |
| --- | ---: | --- |
| Baseline | 0 | Natural table growth |
| Small reserve | 65,536 | Lower up-front bucket allocation |
| Medium reserve | 131,072 | Balance bucket allocation and rehash risk |
| Large reserve | 262,144 | Previously measured 20/20 winner |

The planner still uses `std::unordered_map`. This experiment does
**not** change the key hash, collision checking, cost objective,
search budget, or data structure.

## Measured result: full sweep completed on 2026-11-04

The full unprofiled CSV (`nav2-label-reserve-sweep.csv`) has **80
successful rows**: 10 warehouse routes x 2 footprints x 4 reservation
settings, five timed repeats each. All 20 route/footprint quadruples
preserved search-work counters, full paths and refinement outcomes.
The separate 16-row smoke CSV also passed.

| Reserve | Median search | Median paired saving vs 0 | Wins vs baseline | Median rehashes | Median bucket-array size |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | 231.995 ms | — | — | 14.0 | 1.32 MiB |
| 65,536 | 228.479 ms | **2.06%** | 20/20 | 1.5 | 1.58 MiB |
| 131,072 | 226.000 ms | **3.54%** | 20/20 | 0.5 | 1.58 MiB |
| **262,144** | **221.622 ms** | **4.86%** | **20/20** | **0** | **2.12 MiB** |

262,144 beat 131,072 in **20/20** pairs, saving an additional
**1.32% median paired** (approximately **2.99 ms/search**).
Compared with 131,072, the median additional bucket-array allocation
was **0.54 MiB**. The final bucket count matched in 10/20 comparisons
because 131,072 already rehashed to the larger table; the up-front
262,144 reserve prevented that rehash. In the 10 lower-label-count
cases, where the 131,072 table did not need to grow, the extra benefit
was only **0.64 ms median**, consistent with a smaller payoff on light
searches. Peak unique labels across the full sweep ranged from
42,700 to 179,519.

**Decision for the warehouse workload:** The existing opt-in
`searchLabelReserve=262144` remains the fastest measured choice.
Do not enable it as a universal default or claim an adaptive rule
until it is benchmarked on smaller maps and shorter searches.
Results are from one workload and five-repeat per-variant medians,
not a multi-host confidence interval.

Full route-level evidence and EDT context:
[warehouse performance findings](warehouse_performance_findings_2026-11-04.md).

## Manual Docker run (28 build jobs)

From the repository root on
`feature/nav2-equivalent-envelope-footprint-benchmark`:

```bash
git switch feature/nav2-equivalent-envelope-footprint-benchmark
git pull --ff-only
export BUILD_JOBS=28 TEST_JOBS=4 PROFILE_HOTPATHS=OFF

docker compose build test

docker compose --profile test run --rm test \
  ctest --test-dir build-docker --output-on-failure -j 4 \
  -R '^(hotpath_test|w10_differential_search_test|w12_mecanum_test|clearance_test)$'
```

The four-way sweep is a new explicit/manual executable named
`hybrid_astar_nav2_label_reserve_sweep_benchmark_test`. It is **not**
registered in CTest and does not run automatically.

### 2-route smoke test

The pinned fixture contains **10 routes**. Always pass `--cases 10` so
the whole fixture is checked; `--replay-limit 2` then runs just its
first two routes. This avoids the earlier `--cases 2` mismatch.

```bash
docker compose --profile test run --rm test \
  ./build-docker/hybrid_astar_nav2_label_reserve_sweep_benchmark_test \
  --model differential --connector native --fast-empty-aabb \
  --map apps/demo/maps/nav2_warehouse_full.yaml \
  --replay-csv bench/fixtures/nav2_full_10_seed20261029.csv \
  --cases 10 --replay-limit 2 --repeats 1 \
  --grid-settlements 100000 \
  > nav2-label-reserve-sweep-smoke.csv

python3 tools/analyze_nav2_label_reserve_sweep.py \
  nav2-label-reserve-sweep-smoke.csv --expected-routes 2
```

Expected smoke CSV: **16 data rows** =
2 routes x 2 geometries x 4 reservation sizes.

### Full 10-route performance run

```bash
docker compose --profile test run --rm test \
  ./build-docker/hybrid_astar_nav2_label_reserve_sweep_benchmark_test \
  --model differential --connector native --fast-empty-aabb \
  --map apps/demo/maps/nav2_warehouse_full.yaml \
  --replay-csv bench/fixtures/nav2_full_10_seed20261029.csv \
  --cases 10 --repeats 5 \
  --grid-settlements 100000 \
  > nav2-label-reserve-sweep.csv

python3 tools/analyze_nav2_label_reserve_sweep.py \
  nav2-label-reserve-sweep.csv
```

Expected full CSV: **80 data rows** =
10 routes x 2 geometries x 4 reservation sizes. Each
variant is warmed up outside timing; timings are per-variant medians
across five runs, with the order of the four settings rotated/reversed
between route and geometry groups to reduce ordering bias.

## What the benchmark verifies

For each pinned start/goal and footprint, the benchmark requires
`Success` at all four reservation sizes and calls the shared
full-plan/refinement equivalence check. It additionally checks
labels found/hit/inserted/updated, pruning, reopenings, stale queue pops,
queue push/pop/replacement counts and search/final motion calls.
Collision checking, occupancy prefix, fast-AABB setting and clearance
cost (disabled) are identical on all four variants.

The CSV reports:

- `label_reserve`, geometry, case ID, status and repeats.
- p50/p95 search time, refinement time and total pipeline time.
- Expansions, generated nodes, collision and search-motion calls.
- Label lookups/hits/insertions/updates and rehash count.
- Peak labels, peak unordered-map buckets, and
  `bucket_bytes_estimate` (peak bucket count x `sizeof(void*)`).
- Peak ratio `label_peak_size / label_peak_buckets`, for comparison.
- Hotpath lookup/write/hash nanoseconds when profiling is explicitly
  enabled. Those times contain overlapping nested work.

The analyzer validates four complete variants per matched route,
identical search-work columns, successful statuses and unchanged
prefix/fast-AABB settings, then reports **median paired saving versus
reserve 0**, winning-case counts, rehashes and bucket-array memory.
It also compares each smaller reserve's wins against 262,144.

`bucket_bytes_estimate` is **only a bucket-array estimate**, not total
RSS or total memory allocated. Smaller reservations may cause more
rehashes, while larger reservations can waste buckets on light searches.
A size that improves median runtime but loses badly on the smallest
routes should be treated with caution.

## Interpretation and next step

- Use **`PROFILE_HOTPATHS=OFF`** to select reserve sizes. The analyzer
  refuses profiled timings by default.
- The full four-way run now **confirms 262,144 as the fastest tested
  warehouse setting**, but this does not establish a universal optimum.
- Compare 65,536, 131,072 and 262,144 against the same no-reserve
  baseline and against each other. Pay attention to paired outliers
  and memory overhead, not only global median speed.
- Before adopting an **adaptive** reservation heuristic, repeat with
  smaller maps and short paths. The map's occupancy-cell count is
  not necessarily a good predictor of the number of distinct A* labels;
  choose a reserve rule only after measuring that relationship.
- Later options include alternative cache-friendly hash tables or
  different key hashes, but those are **separate benchmark proposals**,
  not changes introduced by this work.

No GitHub Actions are enabled, dispatched or required by this experiment.
