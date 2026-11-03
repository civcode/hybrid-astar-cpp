# Exact empty-region prefix-sum benchmark

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
