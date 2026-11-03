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

The full-resolution Nav2 map is generated locally and is not committed. If it
is absent, first run `python3 tools/prepare_nav2_warehouse.py` (downloads the
pinned upstream PGM; alternatively provide `--source /path/to/warehouse.pgm`).

    cmake -S . -B build -DHYBRID_ASTAR_BUILD_TESTS=OFF -DHYBRID_ASTAR_BUILD_APP=ON -DHYBRID_ASTAR_BUILD_BENCHMARKS=ON
    cmake --build build --target hybrid_astar_nav2_prefix_sum_benchmark_test -j 4
    ./build/hybrid_astar_nav2_prefix_sum_benchmark_test \
      --model differential --connector native \
      --map apps/demo/maps/nav2_warehouse_full.yaml \
      --replay-csv bench/fixtures/nav2_full_10_seed20261029.csv \
      --cases 10 --repeats 5 --micro-poses 20000 --micro-repeats 5 \
      --grid-settlements 100000 > nav2-prefix-sum.csv

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
