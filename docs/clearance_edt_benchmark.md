# Opt-in EDT-assisted capped clearance cost: Nav2 warehouse A/B/C

This experiment measures whether a proximity penalty steers Hybrid A* away
from obstacles, and whether a precomputed Euclidean distance transform (EDT)
makes this penalty affordable. It does **not** replace exact collision checks,
and it does not change the default planner objective.

## What changes

The existing clearance objective integrates a nonnegative quadratic penalty
over every accepted motion section:

```text
penalty(d) = clearanceWeight * max(0, 1 - d / radius)^2
```

Here `d` is the **exact oriented robot footprint clearance**, minus the
geometric inflation margin, capped at the chosen radius. The footprint is a
rotated rectangle, not a point; the boundary of the finite map also counts as
an obstacle. This branch makes the scorer inflation-aware in **both** its
exact and EDT-assisted modes, so the 50x30 cm chassis with a 10 cm
inflation margin gets comparable clearance semantics.

The opt-in `PlanOptions.useClearanceDistanceTransform = true` accelerates
the **same objective**:

1. In the scorer's constructor, compute an exact 2D Euclidean distance
   transform from policy-blocked **cell centers** with two linear-time 1D
   passes (unknown-cell policy and occupancy threshold follow the existing
   checker). This is built **once per search request**, not per pose.
2. For each clearance sample, query the nearest anchor cell's EDT value.
   Subtract the anchor offset, cell half-diagonal, maximum body radius,
   inflation margin and numerical guard. Also use a conservative map-boundary
   distance. The minimum is a **lower bound** on real footprint clearance.
3. If this lower bound is at least the influence radius, return the same
   capped exact value (`distance = radius`), yielding zero penalty.
4. Otherwise call the existing exact polygon-to-blocked-square clearance
   scorer. No approximated positive penalties are introduced.

An obstacle-free map needs no EDT grid; border clearance still applies.
The full EDT adds 8 bytes per map cell when blocked cells are present
(about 12.85 MiB on the original 1006x1674 warehouse). The constructor's
map validation and EDT build are INCLUDED in measured search time. Build
time and bytes are separately recorded; map-level caching could be examined
later, but is **not** silently assumed in this benchmark.

The options remain opt-in:
```cpp
request.options.clearanceWeight = 0.15;
request.options.clearanceInfluenceRadiusMeters = 0.40;
request.options.useClearanceDistanceTransform = true;
```

The legacy default is `clearanceWeight=0` (no proximity objective), and
`useClearanceDistanceTransform=false` (no preprocessing). With zero
inflation, the existing exact clearance cost is unchanged.

**The penalty does not prevent collision checks on individual generated
successors.** The current search validates each motion BEFORE it integrates
clearance. The penalty can change which successor states get explored next,
so the total number of collision calls may increase or decrease. The
experiment measures this explicitly.

## Manual Docker benchmark, 28 compilation jobs

From the repository root on the experiment branch:

```bash
git switch feature/nav2-equivalent-envelope-footprint-benchmark
git pull --ff-only
export BUILD_JOBS=28 TEST_JOBS=4 PROFILE_HOTPATHS=OFF

docker compose build test

docker compose --profile test run --rm test \
  ctest --test-dir build-docker --output-on-failure -j 4 \
  -R '^(clearance_test|hotpath_test|footprint_inflation_test|w10_differential_search_test|w12_mecanum_test)$'
```

Try a short manual run first:

```bash
docker compose --profile test run --rm test \
  ./build-docker/hybrid_astar_nav2_clearance_edt_benchmark_test \
  --model differential --connector native --fast-empty-aabb \
  --clearance-weight 0.15 --clearance-radius 0.40 \
  --map apps/demo/maps/nav2_warehouse_full.yaml \
  --replay-csv bench/fixtures/nav2_full_10_seed20261029.csv \
  --cases 2 --repeats 1 --grid-settlements 100000 \
  > nav2-clearance-edt-smoke.csv
```

Then perform the full 10-route A/B/C test, serially, keeping the exact
same map, start/goal, model, footprint and search budget across variants:

```bash
docker compose --profile test run --rm test \
  ./build-docker/hybrid_astar_nav2_clearance_edt_benchmark_test \
  --model differential --connector native --fast-empty-aabb \
  --clearance-weight 0.15 --clearance-radius 0.40 \
  --map apps/demo/maps/nav2_warehouse_full.yaml \
  --replay-csv bench/fixtures/nav2_full_10_seed20261029.csv \
  --cases 10 --repeats 3 --grid-settlements 100000 \
  > nav2-clearance-edt.csv

python3 tools/analyze_nav2_clearance_edt.py nav2-clearance-edt.csv
```

The executable is manually invoked; it is **not registered in CTest**.
No GitHub Actions are triggered, dispatched or required.

## Interpretation and correctness criteria

The CSV has **60 rows**: 10 routes x 2 geometries x 3 variants, with
variant order rotated to reduce temperature/cache bias. Each variant is
warmed up, then timed multiple times. The variants are:

- `off`: standard shortest-cost objective; clearance weight zero.
- `exact`: existing exact geometry clearance with finite-radius cutoff.
- `edt`: **identical** clearance objective, with certified zero-penalty
  bypasses from EDT.

The benchmark asserts **exact vs EDT equivalence** for status, generated
nodes, expansions, queue/label work, motion sections, path costs,
refinement and sample counts. The independent route-quality pass checks
the raw path's *actual inflated-footprint* clearance at 0.2 m intervals,
saturating its displayed metric at 1.0 m. The minimum and 10th-percentile
are not global continuous-path certificates; they are sampling-based
quality indicators and are collected **outside** timed runs.

The OFF variant is expected to take a potentially different route.
Compare its search expansions, exact collision calls, path length, and
clearance metrics against the EDT route. Report the number of success
pairs, not just medians of successful cases.

`edt_zero_cost_bypasses / clearance_queries` measures how many *clearance
scans* are removed, **not** how many collision checks are removed.
`edt_build_ms` captures the per-request EDT preprocessing cost, included
within `search_p50_ms`. The results may show that the EDT is slower for
short or already obstacle-rich searches; do not enable it by default
unless paired runtime and path-quality outcomes justify the trade-off.

The benchmark's defaults cover the current 10 seed routes. For
mecanum, omit `--replay-csv` because the fixture originated from
differential-drive discovery. To test different penalty strengths,
rerun with e.g. `--clearance-weight 0.05` or
`--clearance-radius 0.60`, keeping each three-way comparison matched.
