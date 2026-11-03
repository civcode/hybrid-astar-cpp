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
