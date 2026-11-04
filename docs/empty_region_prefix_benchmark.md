# Exact empty-region prefix-sum benchmark

## Confirmed state-label reservation result and next sweep

The unprofiled warehouse replay yielded **20/20 faster searches** with
`PlanOptions.searchLabelReserve=262144`, **4.85% median paired savings**,
**14 to 0 median label rehashes**, and **+0.80 MiB** estimated median
bucket-array memory, without changing search decisions or paths.
This remains **opt-in**, not a universal default. The next manual comparison
checks **0 / 65,536 / 131,072 / 262,144** labels with matched routes:
[Four-size reservation sweep](label_reserve_sweep_benchmark.md).
See [measured performance findings](warehouse_performance_findings_2026-11-04.md).


The collision checker uses an occupancy integral image to prove that the
axis-aligned grid box enclosing the complete rotated and inflated footprint
contains zero blocked cells. Such a box is collision-free without scanning
individual cells. Nonempty boxes still run the original SAT and exact
polygon-distance tests. The occupancy threshold and unknown-cell policy are
unchanged.

PlanOptions.useEmptyRegionPrefixSum is now enabled by default for Ackermann,
differential drive, and mecanum, with an opt-out for A/B comparisons.
Ackermann's automatic covering-disc prefilter remains Ackermann-only.
Refinement still uses its original no-prefix checker: constructing a second
table for a short refinement might outweigh the savings.

## Manually benchmark identical routes with the prefix ON versus OFF

The pinned full-resolution Nav2 map is generated **inside Docker** by the
repository's Dockerfile. The test image also builds the manual benchmark
executable; no host-side CMake build or map preparation is required.

From the repository root:

```bash
git switch feature/nav2-equivalent-envelope-footprint-benchmark
git pull --ff-only

# Limit BOTH OMPL and Hybrid A* compilation to 28 parallel jobs.
export BUILD_JOBS=28
export TEST_JOBS=4
export PROFILE_HOTPATHS=OFF

docker compose build test

# Manually run the A/B benchmark; CSV lands on the host.
docker compose --profile test run --rm test \
  ./build-docker/hybrid_astar_nav2_prefix_sum_benchmark_test \
  --model differential \
  --connector native \
  --map apps/demo/maps/nav2_warehouse_full.yaml \
  --replay-csv bench/fixtures/nav2_full_10_seed20261029.csv \
  --cases 10 \
  --repeats 5 \
  --micro-poses 20000 \
  --micro-repeats 5 \
  --grid-settlements 100000 \
  > nav2-prefix-sum.csv
```

`BUILD_JOBS=28` affects Docker **build-time** parallelism, not the number of
concurrent benchmark runs. Run timed measurements serially for stable results.
`TEST_JOBS=4` would apply only to an explicit CTest invocation; the manual
benchmark above does not use it. Building with 28 concurrent compiler jobs can
require substantial RAM, especially while compiling OMPL.

Optional CTest verification before timing (not run automatically):

```bash
docker compose --profile test run --rm test \
  ctest --test-dir build-docker --output-on-failure -j 4 \
  -R '^(hotpath_test|footprint_inflation_test|w10_differential_search_test|w12_mecanum_test)$'
```

The 10 matched warehouse cases are tested for both the 70x50 cm rectangle
and the 50x30 cm rectangle with 10 cm metric inflation. For each geometry,
all poses, routes, occupancy policies, model settings, and refinement options
remain identical between prefix OFF and ON.

The search/total pipeline timings include map validation and prefix
construction. Separate checker-construction timings include validation in
both modes; the ON/OFF difference estimates the prefix build cost. The
per-pose collision microbenchmark excludes construction. Work counters
are collected separately from timed runs. Differences in collision decisions,
search effort, resulting motion sections, or refinement outcomes fail the
benchmark. Variant order alternates between cases to reduce order effects.

Mecanum is supported using --model mecanum without --replay-csv (it must
discover its own model-specific routes instead of replaying the differential
discovery fixture). The benchmark executable is opt-in, not registered as an
ordinary CTest test, and does not run GitHub Actions.


## Profile the optimized checker: empty bypass versus nonempty fallback

This is the **first stage** before deciding whether an occupied-cell row
index is worth its construction cost. The collision checker now collects
three disjoint full-check categories when `PROFILE_HOTPATHS=ON`:
`collision_empty_bypass_ms` (empty prefix AABB),
`collision_nonempty_fallback_ms` (nonempty AABB, with original exact checks),
and `collision_boundary_ms` (map-boundary early rejection). Each category
includes transform and box computation for its own calls; the sum is a
subset of existing `exact_collision_hotpath_ms`. Calls and durations are
zero in regular nonprofiling builds. Counters are accumulated for one
complete representative search, not the median of repeated searches.

```bash
export BUILD_JOBS=28 TEST_JOBS=4 PROFILE_HOTPATHS=ON
docker compose build test

# Verify instrumentation without invoking any GitHub Actions.
docker compose --profile test run --rm test \
  ctest --test-dir build-docker --output-on-failure -j 4 \
  -R '^(hotpath_test|footprint_inflation_test|w10_differential_search_test|w12_mecanum_test)$'

# Reuse the original 10 routes and BOTH original footprint geometries.
# Keep fixed-pose microbenchmark short: this pass is for search profiling.
docker compose --profile test run --rm test \
  ./build-docker/hybrid_astar_nav2_prefix_sum_benchmark_test \
  --model differential --connector native \
  --map apps/demo/maps/nav2_warehouse_full.yaml \
  --replay-csv bench/fixtures/nav2_full_10_seed20261029.csv \
  --cases 10 --repeats 3 --micro-poses 100 --micro-repeats 1 \
  --grid-settlements 100000 > nav2-prefix-collision-profile.csv
```

Summarize the **prefix ON** rows with Python on the host (no external package):

```bash
python3 - <<'PY'
import csv
from statistics import median
with open("nav2-prefix-collision-profile.csv", newline="") as f:
    rows = list(csv.DictReader(f))
assert len(rows) == 40 and all(r["profile_enabled"] == "1" for r in rows)
for geometry in sorted({r["geometry"] for r in rows}):
    selected = [r for r in rows
                if r["geometry"] == geometry and r["prefix_enabled"] == "1"]
    assert len(selected) == 10
    parts = ("empty_bypass", "nonempty_fallback", "boundary")
    shares = {part: [] for part in parts}
    times = {part: [] for part in parts}
    for r in selected:
        assert sum(int(r[f"collision_{p}_calls"]) for p in parts) == int(r["exact_pose_calls"])
        total = sum(float(r[f"collision_{p}_ms"]) for p in parts)
        assert total > 0
        for part in parts:
            t = float(r[f"collision_{part}_ms"])
            times[part].append(t)
            shares[part].append(100 * t / total)
    print("\n", geometry)
    for part in parts:
        print(f"  {part:18s}: {median(times[part]):8.2f} ms, "
              f"{median(shares[part]):5.1f}% of categorized collision time")
PY
```

Interpret the percentages as profiling-build attribution, **not**
un-instrumented production timings. Profiling creates per-check clock
overhead; the existing outer `exact_collision_hotpath_ms` timer overlaps
with all three category timers, so do **not** sum them with that outer
metric. Compare the empty-bypass and fallback shares before investing
in occupied-cell indexing. If fallback time dominates, build an opt-in
row-index A/B mode and verify exact route equivalence; otherwise target
the high-frequency prefix lookup or state-label overhead.

For final end-to-end latency comparisons, rebuild with
`PROFILE_HOTPATHS=OFF` and keep build parallelism at `BUILD_JOBS=28`.


## Experimental fast pre-polygon AABB bypass (opt-in)

`PlanOptions.useFastEmptyAabb` defaults to **false**. When enabled alongside
the default `useEmptyRegionPrefixSum=true`, the exact collision checker:

1. Computes a conservative, outward-padded AABB from the rotated rectangle's
   projected rear/front and half-width extents. It does **not** transform
   the four polygon corners on this path.
2. Requires the *entire* padded/inflated AABB to be strictly inside the map
   boundary, so original edge-contact rules remain authoritative.
3. Checks its complete cell range with the same policy-aware occupancy prefix.
   If empty, return `Clear` without constructing polygon corners.
4. Otherwise run the **original full** corner-transform, boundary, prefix,
   SAT and exact metric-distance checks, unchanged.

There is no map-data-format change and no new construction-time index. The
regular prefix-sum and standard collision paths stay the default until
benchmarks show a consistent speedup with identical results. If prefix sums
are disabled, the fast-AABB option has no effect.

### Docker A/B experiment on the original 10 warehouse routes

The existing manual `hybrid_astar_nav2_prefix_sum_benchmark_test` now supports
`--fast-empty-aabb-ab`. That mode tests *prefix ON in both variants*, with
the optional pre-polygon AABB optimization OFF and ON, alternating run order
across routes. It covers **both** the 70 x 50 cm rectangle and the 50 x
30 cm rectangle with a 10 cm geometric inflation margin, using the same
10 replayed start/goal pairs.

```bash
git switch feature/nav2-equivalent-envelope-footprint-benchmark
git pull --ff-only
export BUILD_JOBS=28 TEST_JOBS=4 PROFILE_HOTPATHS=OFF
docker compose build test

docker compose --profile test run --rm test \
  ctest --test-dir build-docker --output-on-failure -j 4 \
  -R '^(hotpath_test|footprint_inflation_test|w10_differential_search_test|w12_mecanum_test)$'

docker compose --profile test run --rm test \
  ./build-docker/hybrid_astar_nav2_prefix_sum_benchmark_test \
  --fast-empty-aabb-ab \
  --model differential --connector native \
  --map apps/demo/maps/nav2_warehouse_full.yaml \
  --replay-csv bench/fixtures/nav2_full_10_seed20261029.csv \
  --cases 10 --repeats 5 \
  --micro-poses 20000 --micro-repeats 5 \
  --grid-settlements 100000 > nav2-fast-empty-aabb-ab.csv
```

The CSV appends `fast_empty_aabb_enabled`,
`search_fast_aabb_queries`, `search_fast_aabb_bypasses` and corresponding
fixed-pose microbenchmark counters. `prefix_enabled` remains 1 for **both**
variants in this mode. The benchmark fails on any change to the full
search/refinement outcomes, exact-pose call counts, fixed-pose collision
results, or candidate/occupied cell, SAT, or distance-check work.

Use `PROFILE_HOTPATHS=OFF` for a trustworthy end-to-end latency A/B.
Build with `PROFILE_HOTPATHS=ON` only to compare instrumented empty-AABB
timings, not for production speed estimates. Benchmark execution is manual
only; it is not registered in CTest and no GitHub Actions are involved.

Mecanum may also be tested, but **omit** `--replay-csv` when selecting
`--model mecanum`: the 10-route fixture was discovered for differential
drive, not mecanum.


## Label-table profiling and pre-reservation A/B

The next potential bottleneck is the **A* state-label hash table**, not the
collision prefix query itself. On the original 10-route warehouse sample,
a typical route performs ~1.15 million label finds and inserts ~130,000
unique labels; the largest observed routes approach 180,000 inserts.
The planner already has `PlanOptions.searchLabelReserve` (default **0**),
so this experiment compares the existing behavior with a pre-reserve of
262,144 slots, without replacing the data structure or changing search
decisions.

In a `PROFILE_HOTPATHS=ON` build, `labelLookupNanoseconds` is now
partitioned **without overlap** into:

- `labelPopHitNanoseconds` / `labelPopMissNanoseconds`: open-queue
  candidate's state-label lookup
- `labelSuccessorHitNanoseconds` / `labelSuccessorMissNanoseconds`:
  each newly generated successor's state-label lookup

Each category also records its call count. The sum of the four category
times equals the existing `labelLookupNanoseconds`, and the sum of call
counts equals `labelFinds`. `hashNanoseconds` is **nested within** label
lookups and some insertions, so do not add it to the label lookup total.

The manually invoked
`hybrid_astar_nav2_label_lookup_benchmark_test` replays the same 10
start/goal pairs with both warehouse footprint geometries. Each variant
keeps the occupancy prefix-sum ON, and can optionally enable the
previously measured pre-polygon AABB optimization **identically** in both
variants. The benchmark rejects any difference in status, route geometry,
refinement, search effort, label decisions or queue work.

### Profiled build: diagnose hits, misses and rehashes

```bash
git switch feature/nav2-equivalent-envelope-footprint-benchmark
git pull --ff-only
export BUILD_JOBS=28 TEST_JOBS=4 PROFILE_HOTPATHS=ON
docker compose build test

docker compose --profile test run --rm test \
  ctest --test-dir build-docker --output-on-failure -j 4 \
  -R '^(hotpath_test|footprint_inflation_test|w10_differential_search_test|w12_mecanum_test)$'

docker compose --profile test run --rm test \
  ./build-docker/hybrid_astar_nav2_label_lookup_benchmark_test \
  --model differential --connector native --fast-empty-aabb \
  --label-reserve 262144 \
  --map apps/demo/maps/nav2_warehouse_full.yaml \
  --replay-csv bench/fixtures/nav2_full_10_seed20261029.csv \
  --cases 10 --repeats 3 --grid-settlements 100000 \
  > nav2-label-lookup-profile.csv
```

This produces 40 rows: 10 routes × 2 footprints × 2 reservation settings.
The CSV includes overall search/refinement timings, per-lookup hit/miss
counts and times, table rehashes, peak labels, peak hash buckets,
approximate bucket-array bytes, and nested hash timing.
`label_bucket_bytes_estimate` excludes per-label node allocations and
other unordered_map overhead.

### Production-speed A/B: measure whether reserve actually helps

After collecting profile diagnostics, **rebuild** with clock instrumentation
disabled; do not use the profiled build's search medians to judge speed.

```bash
export BUILD_JOBS=28 TEST_JOBS=4 PROFILE_HOTPATHS=OFF
docker compose build test
docker compose --profile test run --rm test \
  ./build-docker/hybrid_astar_nav2_label_lookup_benchmark_test \
  --model differential --connector native --fast-empty-aabb \
  --label-reserve 262144 \
  --map apps/demo/maps/nav2_warehouse_full.yaml \
  --replay-csv bench/fixtures/nav2_full_10_seed20261029.csv \
  --cases 10 --repeats 5 --grid-settlements 100000 \
  > nav2-label-reserve-ab.csv
```

Keep the faster pre-reserve only if it improves the 10 paired routes
consistently and the additional table-memory cost is acceptable. It
remains **opt-in**, and the manual benchmark is not registered in CTest or
scheduled with GitHub Actions. For mecanum, omit the differential-drive
`--replay-csv` fixture and discover its own cases.

### Summarize the baseline-vs-reserved CSV on the host

```bash
python3 - <<'PY'
import csv
from statistics import median
with open("nav2-label-reserve-ab.csv", newline="") as f:
    rows = list(csv.DictReader(f))
assert len(rows) == 40
for geometry in sorted({r["geometry"] for r in rows}):
    matches = {}
    for r in rows:
        if r["geometry"] == geometry:
            matches.setdefault(r["case_id"], {})[r["variant"]] = r
    savings, extra_bucket_bytes = [], []
    for pair in matches.values():
        off, on = pair["baseline"], pair["pre_reserved"]
        before, after = float(off["search_p50_ms"]), float(on["search_p50_ms"])
        savings.append(100*(before-after)/before)
        extra_bucket_bytes.append(int(on["label_bucket_bytes_estimate"]) -
                                  int(off["label_bucket_bytes_estimate"]))
    print(geometry, "routes:", len(savings),
          "median paired saving:", f"{median(savings):.2f}%",
          "wins:", sum(s > 0 for s in savings),
          "median extra bucket memory:",
          f"{median(extra_bucket_bytes)/1024/1024:.2f} MiB")
PY
```
