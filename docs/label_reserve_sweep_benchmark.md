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
- The known 262,144 result is a **warehouse-specific prior**, not the
  winning outcome of this as-yet-unrun four-size experiment.
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
