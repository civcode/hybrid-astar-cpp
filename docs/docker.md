# Docker development, testing and Planning Lab demo

The root [Dockerfile](../Dockerfile) provides a reproducible Ubuntu 24.04
environment for **building, testing and running** the real C++20 Hybrid A*
planner. Dependencies include GCC, Git, CMake, Ninja, Python 3,
Eigen3, Boost, FLANN, ODE and yaml-cpp. **Catch2 v3.16.1 is built by
CMake FetchContent by default**, not installed from Ubuntu's packages.
It builds **OMPL 2.0.2 from its
official tagged source**, with VAMP and Python bindings disabled; this avoids
accidentally linking an older distro OMPL version.

Docker and network access to the upstream source tarball are required on the
host for the initial build. The OMPL archive previously supplied for local
debugging is **not** copied into the repository or Docker context.

## Start the demo (local browser)

```bash
docker compose up --build demo
```

Open **http://127.0.0.1:8765**. Docker Compose explicitly publishes the host
port only on **127.0.0.1**. The executable inside the container must listen
on its container interface, so the Docker image uses
`hybrid_astar_demo --bind 0.0.0.0` (an explicit opt-in to binding beyond
loopback). On bare metal, the default remains 127.0.0.1.

Stop with Ctrl+C, then `docker compose down` if desired.

Without Compose:

```bash
docker build --target demo -t hybrid-astar:demo .
docker run --rm -p 127.0.0.1:8765:8765 hybrid-astar:demo
```

**Do not** publish `-p 8765:8765` on a public interface. The demo server
is a single-user development tool with no authentication. It is not a
production web service or robot motion controller.

## Demo with an imported PGM map

The Docker image includes a [sample PGM occupancy map](../apps/demo/maps/sample.pgm),
and the browser can display any P2/P5 map loaded at server startup:

```bash
docker build --target demo -t hybrid-astar:demo .
docker run --rm -p 127.0.0.1:8765:8765 hybrid-astar:demo \
  ./build-docker/hybrid_astar_demo --bind 0.0.0.0 \
  --map apps/demo/maps/sample.pgm --resolution 0.5
```

To use your own file without rebuilding the image, mount a trusted directory
read-only and pass its **container path**:

```bash
docker run --rm -p 127.0.0.1:8765:8765 \
  -v "$PWD/maps:/imported:ro" hybrid-astar:demo \
  ./build-docker/hybrid_astar_demo --bind 0.0.0.0 \
  --map /imported/my-map.pgm --resolution 0.25
```

The browser automatically selects **Imported map** when available. Its
**Export path JSON** button downloads the actual core planner results to your
browser, not a file written inside the container. CLI JSON export remains
available with `--once --output FILE.json`, where FILE refers to the
container filesystem unless a writeable host volume is mounted.

PGM maps use top-row-first raster order, 0=obstacle, white=free and the
grayscale value 205 as unknown for maxval=255. The default occupancy policy
treats unknown as occupied. No remote map-upload endpoint is enabled.

### Load a ROS map YAML descriptor

The image includes [`sample.yaml`](../apps/demo/maps/sample.yaml), which
references the sibling `sample.pgm` file and uses a 90° rotated world origin.

```bash
docker run --rm -p 127.0.0.1:8765:8765 hybrid-astar:demo \
  ./build-docker/hybrid_astar_demo --bind 0.0.0.0 \
  --map-yaml apps/demo/maps/sample.yaml
```

When mounting your own map files, mount the **YAML and PGM together** so
relative image paths resolve within the container. The browser map, vehicle
headings and click placement use the YAML translation/yaw. The Docker image
also includes Node.js for an optional coordinate round-trip CTest.
This restricted map YAML reader supports P2/P5 and `mode: trinary` only.

## Run Catch2 + CTest (explicit manual action)

```bash
docker compose run --build --rm test
```

### Control build and test parallelism

The Dockerfile's two `cmake --build` invocations (OMPL 2.0.2 and Hybrid A*)
use the same **`BUILD_JOBS`** build argument. The Compose `test` service
also passes **`TEST_JOBS`** as `CTEST_PARALLEL_LEVEL`, independently
controlling the number of simultaneously running CTest executables.

Both variables are optional. Defaults remain **2 compiler jobs** and
**1 test job** (sequential tests). For example:

```bash
# Build OMPL and Hybrid A* with up to 8 compiler processes;
# after the build, execute up to 2 CTest tests concurrently.
BUILD_JOBS=8 TEST_JOBS=2 docker compose run --build --rm test

# Only adjust the compiler count when starting the demo.
BUILD_JOBS=8 docker compose up --build demo
```

With `docker build` and `docker run` directly:

```bash
docker build --build-arg BUILD_JOBS=8 --target test -t hybrid-astar:test .
docker run --rm -e CTEST_PARALLEL_LEVEL=2 hybrid-astar:test
```

Increasing `BUILD_JOBS` increases peak compiler memory use. When in doubt,
use 2–4 jobs, and reduce the count if the OS or container runs out of RAM.
Keep `TEST_JOBS=1` for the most reproducible timing/diagnostics; higher
test parallelism may compete for CPU and memory. These settings do **not**
change GitHub Actions runner limits or start a workflow automatically.

Or:

```bash
docker build --target test -t hybrid-astar:test .
docker run --rm hybrid-astar:test
```

This executes the C++ test suites under **Catch2 v3** (including actual
`TEST_CASE`, `SECTION`, `REQUIRE` assertions) via CTest, the native HTTP
demo smoke (Python 3), and OMPL-enabled signed-path/integration tests.
Tests are **not automatically run on Docker image build**; the `test`
service is under a Compose profile and has to be requested explicitly.
The optional [GitHub Actions verification workflow](ci/manual-verification.md)
uses `workflow_dispatch` only; no workflow run is dispatched automatically.

To run the planner directly in command-line JSON mode:

```bash
docker run --rm hybrid-astar:demo \
  ./build-docker/hybrid_astar_demo --once --scenario reverse --ompl
```

Optional native benchmark executables are also built in the image:

```bash
docker run --rm hybrid-astar:demo \
  ./build-docker/hybrid_astar_planner_bench 3
docker run --rm hybrid-astar:demo \
  ./build-docker/hybrid_astar_refinement_bench 3
```

Benchmarks are manual; they are not stable product latency promises.

## Manual hotpath profiling experiments

To profile the difficult mecanum warehouse geometry or compare the
certified-Euclidean and obstacle-grid heuristics, enable the new opt-in
`PROFILE_HOTPATHS` build argument. Default is `OFF`; `BUILD_JOBS`
and `TEST_JOBS` continue to work independently.

```bash
PROFILE_HOTPATHS=ON BUILD_JOBS=8 docker compose build test

# No rebuild between manual benchmark executions.
PROFILE_HOTPATHS=ON docker compose run --rm test \
  ./build-docker/hybrid_astar_hotpath_bench --warehouse 1 1000000 > warehouse.csv

PROFILE_HOTPATHS=ON docker compose run --rm test \
  ./build-docker/hybrid_astar_hotpath_bench --heuristics 7 > heuristics.csv

# Optional A/B on the difficult warehouse configuration itself.
PROFILE_HOTPATHS=ON docker compose run --rm test \
  ./build-docker/hybrid_astar_hotpath_bench --warehouse-heuristics 1 1000000 \
  > warehouse-heuristics.csv
```

The warehouse modes are explicitly manual and can consume substantial CPU
and RAM near 1 million expansions. Rebuild with `PROFILE_HOTPATHS=OFF`
when comparing unbiased end-to-end runtimes. The 4-disc collision prefilter
is **not** enabled globally; safety equivalence remains a test gate.

## Build without Docker

Catch2 **v3.16.1 is fetched automatically by CMake** when tests are enabled
(the default). The initial configuration requires Git and HTTPS access to
`github.com/catchorg/Catch2`; later configurations reuse the downloaded
FetchContent source tree. An installed Catch2 package is not required.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DHYBRID_ASTAR_BUILD_TESTS=ON \
  -DHYBRID_ASTAR_BUILD_APP=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

To use an already installed Catch2 v3 CMake package instead (for offline
or distribution builds):

```bash
cmake -S . -B build-system-catch2 \
  -DHYBRID_ASTAR_USE_SYSTEM_CATCH2=ON
cmake --build build-system-catch2 --parallel
ctest --test-dir build-system-catch2 --output-on-failure
```

Alternatively, keep FetchContent enabled and supply an **existing** unpacked
Catch2 v3.16.1 source checkout with
`-DFETCHCONTENT_SOURCE_DIR_CATCH2=/absolute/path/to/Catch2`. This is
useful for offline builds and does not change the default behavior.

For a dependency-minimal **core + app only** build, disable tests explicitly
(`-DHYBRID_ASTAR_BUILD_TESTS=OFF`); this avoids requiring Catch2. Likewise,
`-DHYBRID_ASTAR_ENABLE_OMPL=OFF` omits OMPL integration. Those two options
do not alter the project's geometric safety/verification semantics.

## Catch2 migration notes

Earlier executable regression tests used C++ `main()` with `throw` or
numeric error codes. That was useful for rapidly bootstrapping a
no-third-party-dependency vertical slice, **but it deviated from the documented
Catch2 v3 contract**, and the documentation had not clearly labeled this
as a temporary compromise.

All existing C++ tests now run under `Catch2::Catch2WithMain`; their
pre-existing assertions and coverage are retained in registered test cases.
New `tests/catch2_contract_test.cpp` directly uses
`TEST_CASE`/`SECTION`/`REQUIRE` to exercise exact arc and strict validation
properties. Migrating individual legacy assertions to more granular
expression-level Catch2 messages is worthwhile ongoing test cleanup, not a
precondition for using the real Catch2 runner.

## Verification limitations

These Docker paths have not been executed from this assistant environment
because the Docker daemon and HTTPS access needed for the default Catch2
FetchContent download are unavailable here. They require a first on-machine
Docker build/test to confirm OMPL's
full package installation and all imported CMake targets. The earlier local
OMPL 2.0.2 source-subset integration and sanitizer test evidence remain
separate and do not substitute for that Docker verification.

Even a passing Docker build does not certify continuous swept collision
freedom, actuator feasibility, optimality or safety for a real vehicle.
\n## Manual warehouse performance experiments\n\nFor the opt-in Ackermann and diagonal differential-drive warehouse profiling\ncommands, see [W6](architecture/w6-hotpath-profiling.md#ackermann-warehouse-hot-path-profiling-manual-only-2026-10-27)\nand [W10](architecture/w10-differential-search.md#diagonal-smoothing-and-radius-ab-manual-only-2026-10-27).\nBoth benchmarks are built into the Docker test image, but are **not**\nregistered in CTest or scheduled in GitHub Actions. Set\n`PROFILE_HOTPATHS=OFF` for unbiased wall-clock comparisons and\n`PROFILE_HOTPATHS=ON` separately for nested per-hotpath timings.\n
### Ackermann footprint broad-phase profiling

The manual `--ackermann-collision-ab` benchmark times a factorial
Euclidean/obstacle-grid × 0/0.50 m margin × 0/4 disc matrix with
instrumentation OFF. `--ackermann-collision-work` runs the identical
matrix but collects exact per-cell work counts (adds measurement overhead).
Neither runs automatically. See
[W6 experiment details](architecture/w6-hotpath-profiling.md#ackermann-collision-broad-phase-and-factorial-ab-manual-october-2026)
for the exact Docker Compose commands.

For isolated collision-check cost without search-expansion confounding,
run `--ackermann-collision-micro 5` instead. This uses identical poses
for 0/0.50 m inflation with zero/four discs, measures lookup/check time
without counter overhead, and collects SAT/distance work separately.

### Ackermann new defaults and prefix-sum comparison

Ackermann now uses four discs and the exact empty-region prefix-sum
broad phase by default; other models preserve their old prefilter defaults.
To test overhead and collision-work reductions, run
`--ackermann-easy-ab`, `--ackermann-empty-region-ab` or
`--ackermann-collision-micro` as documented in
[W6](architecture/w6-hotpath-profiling.md#ackermann-default-cover-and-exact-empty-aabb-bypass-october-2026).
All are manual opt-in benchmarks, never scheduled in CTest.

## Ackermann RS performance default and independent CSC refinement

The OMPL-enabled Planning Lab preselects adaptive RS connection with grid
heuristic. Extra per-state RS distance is **off** until the dedicated checkbox
or `--rs-distance` CLI flag is enabled; `--rs-distance` requires `--ompl`
and grid guidance (the default). Native CSC and external RS refinement can be
selected together, using identical strict full-footprint validation and
weighted cost gates. Ackermann refiner windows now cover intermediate route
sections within the original attempt and pose-check budgets.

```bash
BUILD_JOBS=28 TEST_JOBS=4 docker compose build test
BUILD_JOBS=28 TEST_JOBS=4 docker compose run --rm test \
  ctest --test-dir build-docker -R 'curved_shortcuts_test|demo_test|ompl_smoke' --output-on-failure

# Ackermann grid + adaptive RS connector (without RS distance)
BUILD_JOBS=28 TEST_JOBS=4 docker compose run --rm test \
  ./build-docker/hybrid_astar_demo --once --scenario parking --ompl --refine --curved

# Explicit extra Reeds-Shepp distance heuristic
BUILD_JOBS=28 TEST_JOBS=4 docker compose run --rm test \
  ./build-docker/hybrid_astar_demo --once --scenario parking --ompl --rs-distance
```


## Full-resolution Nav2 warehouse: margin and pipeline profiler (manual)

The executable `hybrid_astar_nav2_warehouse_pipeline_benchmark_test` loads the
**original 1006 × 1674 = 1,684,044-cell Nav2 PGM at 0.03 m/cell**, with its
original map origin and occupancy/unknown semantics. It **refuses** the reduced
344,128-cell version. Docker prepares both Nav2 maps automatically using
`tools/prepare_nav2_warehouse.py`. It does not change the source map.

### Experiment and selection rules

The benchmark generates **independent seeded random start and goal poses**
(including both yaw angles), with a 15 m minimum direct displacement and
different 3×3 map sectors. The actual polygon collision checker pre-filters
both endpoint footprints using the **more conservative 0.10 m margin** so that
the same start/goal poses are feasible in both treatments. Discovery plans each
candidate with **margin OFF** and accepts it only if it returns a strictly
validated complete path with **at least 30,000 search expansions**. Duplicate
quantized pose pairs are excluded. The baseline qualification search is never
included in measured runtimes.

Each accepted pose pair is replanned **from scratch** under BOTH settings:
`footprintInflationMeters=0` and `0.10`, with the same seed, map, footprint,
heading constraints, heuristic, search budget, refinement limits and pose
endpoints. No variant gets a different route just because its margin increased.
The inflated plan is **not required to succeed or reach 30,000 expansions**:
failures are an important result and remain in the CSV. Conditioning selection
on both variants succeeding would artificially hide margin regressions.

The default model is **differential drive**, matching our long successful
warehouse routes (0.70 × 0.50 m footprint). Pass `--model ackermann` for the
4.60 × 1.80 m car (4.0 m turning radius) or `--model mecanum`.
The default connector is `native`, with `--connector ompl` available for
Ackermann only in OMPL-enabled builds. Refinement uses the Planning Lab's
native CSC settings by default (toggle `--no-curved`), with
**160 attempts / 16 accepted / 200,000 strict pose checks** for differential
and Ackermann, and **220 / 16 / 240,000** for mecanum. As in the UI, the
optional Ackermann steering-variation penalty is *off*.

**Per-route timing** includes the complete **search**, complete
**post-path refinement** (only when search succeeds), and **combined** pipeline.
There is a warmup of each margin profile, then independent whole-pipeline
repetitions; for each case and treatment CSV records p50 and p95 elapsed times.
Discovery and map loading are outside the timed runs. The one-time map-load
time is reported separately on stderr; JSON serialization and browser
rendering are not timed. Order of the two variants alternates between cases.
The benchmark reports successful and failed searches, exact-footprint
validation, search expansions/generation/pruning, collision pose checks,
full-path validation, heuristic grid settlements/fallbacks, label/hash/queue
activity, and refinement attempts/acceptances/checks.

### Commands

```bash
git fetch origin
git switch feature/nav2-full-warehouse-margin-pipeline-benchmark \
  || git switch --track origin/feature/nav2-full-warehouse-margin-pipeline-benchmark

export BUILD_JOBS=28 TEST_JOBS=4
docker compose build test
docker compose --profile test run --rm test \
  ctest --test-dir build-docker --output-on-failure -j "$TEST_JOBS"

# Default: 10 seeded 30k+ routes, 3 timed runs per treatment, finite
# 2500-draw discovery cap, differential drive, native CSC refinement.
docker compose --profile test run --rm test \
  ./build-docker/hybrid_astar_nav2_warehouse_pipeline_benchmark_test \
  > nav2-full-margin-pipeline.csv

# Larger repeatable sample (may be VERY expensive to discover):
docker compose --profile test run --rm test \
  ./build-docker/hybrid_astar_nav2_warehouse_pipeline_benchmark_test \
  --model differential --cases 30 --repeats 5 \
  --min-expansions 30000 --search-budget 500000 \
  --max-candidates 10000 --seed 20261029 \
  > nav2-full-margin-pipeline-30.csv

# Switch to Ackermann with the Reeds-Shepp/OMPL connector:
docker compose --profile test run --rm test \
  ./build-docker/hybrid_astar_nav2_warehouse_pipeline_benchmark_test \
  --model ackermann --connector ompl --cases 5 \
  > nav2-full-ackermann-margin.csv

python3 tools/analyze_nav2_warehouse_pipeline.py nav2-full-margin-pipeline.csv
```

`--max-candidates 0` explicitly allows unbounded discovery; the default
`2500` avoids accidentally running an unbounded experiment. If fewer than
the requested number of valid 30k-expansion routes are discovered, the
benchmark exits with a clear error and **no CSV is written**, rather than
misrepresenting a short sample as a completed experiment. Re-run with a
higher discovery cap, more search expansions, or fewer requested cases.
Discovered route coordinates, discovery expansions, seed and draw index are
written in each row for later same-pose comparisons with Nav2.

### Bottleneck profiling: run separately

`HYBRID_ASTAR_PROFILE_HOTPATHS=ON` enables the existing internal timers
for queue operations, label/hash bookkeeping, search-motion sampling,
exact collision and heuristic-grid queue activity. These timers add
per-operation clock overhead: **profile-enabled absolute runtimes must not
be directly compared with low-overhead OFF runs**. Some subphases are nested:
search-motion time **already includes** exact collision checks. Never add
nested counters together or interpret their sum as total search time.

```bash
export PROFILE_HOTPATHS=ON
docker compose build test
docker compose --profile test run --rm test \
  ./build-docker/hybrid_astar_nav2_warehouse_pipeline_benchmark_test \
  --model differential --cases 10 --repeats 3 \
  > nav2-full-margin-profiled.csv
python3 tools/analyze_nav2_warehouse_pipeline.py nav2-full-margin-profiled.csv

# Restore fast timing for publication:
export PROFILE_HOTPATHS=OFF
docker compose build test
```

The analyzer validates exact paired start/goal poses and selection thresholds,
then reports status/success rate, per-path median/p95 latency, paired
inflated-minus-off differences, path lengths and candidate bottleneck signals.
The most actionable investigations are **grid saturation**
(`grid_settlements==100000`, frequent fallback), **collision cost**
(exact pose checks and search-motion time), **queue/hash overhead**, and
**refinement budgets** (attempts/accepted/strict pose checks). These are
hypotheses, not optimization claims, until verified by controlled A/B runs
on the same cases and same CPU.

This is **our** pipeline on Nav2's published map asset. A direct Nav2 Smac
benchmark still requires running Nav2 Smac State Lattice/Hybrid A* on
the exact same map, footprint, motion constraints and endpoints, using
the same hardware and compatible smoothing/validation behavior.

## Ackermann steering-aware refinement: 100-path A/B (manual)

The **default** `RefinementOptions` budget is now **160 attempts,
8 accepted replacements and 200000 independent strict pose checks**;
the Planning Lab uses these values for both native and OMPL-assisted
Ackermann refinement. The old Ackermann demo profile was 35 / 3 / 100000.
Model-specific Planning Lab settings for differential-drive and mecanum
remain independently configured.

The optional `RefinementOptions::curvatureVariationWeightMetersSquared`
(default **0.0**) adds
`weight * sum(abs(curvature[i] - curvature[i-1]))` to the evaluated
whole-path objective. The refiner now computes exactly the same nonnegative
curvature term in the cheap **candidate cost lower bound**, including splice
boundaries, so candidates that cannot improve the objective are pruned before
expensive collision sampling. This is a selection penalty, **not** a
curvature-continuity, steering-rate or actuator-feasibility guarantee.

### Official Nav2 warehouse map choices

The Docker demo build runs `python3 tools/prepare_nav2_warehouse.py`
automatically after copying the source tree. The script downloads a
**pinned** official Nav2 warehouse raster, verifies its Git blob digest,
and writes ROS-compatible raster/YAML pairs in `apps/demo/maps/`:

- `nav2_warehouse_full`: original raster dimensions and bytes, 0.03 m/cell
- `nav2_warehouse_344128`: **608 × 566 = 344,128 cells**,
  aspect-preserving conservative occupancy resample, also 0.03 m/cell

Both appear under the Planning Lab **Environment** selector once assets
are available. If running without Docker, execute
`python3 tools/prepare_nav2_warehouse.py` before starting the demo.
The full PGM is loaded at startup, with bounded importer support up to
5 million cells. The smaller map's **physical world footprint shrinks**
at the retained 0.03 m/cell; it does not reproduce Nav2's published
344,128-cell benchmark map or preserve the original world obstacle positions.
Details and CLI examples: [Planning Lab map preparation](../apps/demo/README.md#nav2-warehouse-benchmark-maps-003-mcell).

```bash
git fetch origin
git switch feature/nav2-warehouse-benchmark-maps
export BUILD_JOBS=28
docker compose up --build --force-recreate demo
# Open http://127.0.0.1:8765 and select a Nav2 warehouse environment.
```

**Planning Lab UI:** choose **Ackermann, differential drive or mecanum**,
enable **Strict post-path refinement**, and select **Accepted refinements**:
**8 (default), 12 or 16**. This changes only the acceptance cap:
Ackermann and differential drive keep **160 attempts / 200000 strict pose
checks**; mecanum keeps **220 attempts / 240000 strict checks**.
The selector is disabled only when refinement is off. Click **Plan path**
after each change to compare the same poses and map, using the reported
accepted-count, curvature-variation, steering-reversal, length and timing
metrics. The exported `refinementOptions.maxAccepted` records the selected
limit even if search does not return a path.

Optionally check **Steering-aware smoothness** (unchecked by default).
Set **Steering variation weight** to `1.0` m² initially; valid enabled
weights are greater than zero and at most 100. Both native CSC and
OMPL/Reeds–Shepp paths honor this option. Click **Plan path** and compare
the existing original-path overlay plus the new **Curvature variation**
and **Steering reversals** readouts (original → refined).
The exported JSON contains
`refinementOptions.maxAttempts`, `maxAccepted`,
`maxTotalPoseChecks`, `steeringSmoothness`, and
`steeringWeightMetersSquared`. The weighted path cost changes meaning when
the objective is enabled; compare geometric length and steering metrics
rather than directly comparing differently weighted path costs.

```bash
# Planning Lab's web UI is published to localhost after Docker starts.
BUILD_JOBS=28 docker compose up --build demo

# Optional CLI A/B with the same map and path options.
BUILD_JOBS=28 TEST_JOBS=4 docker compose --profile test run --rm test \
  ./build-docker/hybrid_astar_demo --once --scenario warehouse --refine --curved
BUILD_JOBS=28 TEST_JOBS=4 docker compose --profile test run --rm test \
  ./build-docker/hybrid_astar_demo --once --scenario warehouse --refine --curved \
  --steering-weight 1.0

# Compare the accepted-replacement limits without changing other budgets:
BUILD_JOBS=28 TEST_JOBS=4 docker compose --profile test run --rm test \
  ./build-docker/hybrid_astar_demo --once --scenario warehouse --refine --curved \
  --max-accepted 16
```

### Random warehouse map coverage and output

Both C++ benchmark executables now use **randomized start and goal poses**
(including independently randomized orientations) on the **unchanged** 200 x
160-cell / 0.25 m warehouse map. The Ackermann footprint, 0.5 m inflation,
search mode and obstacle geometry are not modified.

The sampler is reproducible: a small fixed PRNG with default seed
`20261028` generates independent poses across a **3 x 3 map grid**. Accepted
routes must have two different endpoint regions and at least 18 m direct
start/goal separation. A whole-body exact collision check filters candidate
poses before planning. **Every accepted route** requires a real successful,
strictly validated Ackermann plan with at least **10,000 expansions**, and
a distinct quantized path-geometry signature. The selection actively favors
underrepresented regions and requires **at least five accepted starts and five
accepted goals in every one of the nine regions** among the 100 paths.
Furthermore no region can account for over 20% of starts or goals, and no
ordered start/goal region pair can account for over 10% of the paths.
This improves spatial coverage without fabricating any trajectories or
changing the map. Note: geographic coverage does not guarantee 100
distinct topological routes.

Defaults:
`repeats=5 minimum_expansions=10000 paths=100 search_budget=250000
max_candidates=0 steering_weight_m2=1.0 seed=20261028`.
**0 candidates limit means unbounded discovery:** the executable keeps
trying reproducible new random poses until all 100 qualifying routes
satisfy the coverage constraints. Depending on map difficulty, this can be
a long-running CPU-intensive operation. You can interrupt it with Ctrl+C,
or provide a positive `max_candidates` safety limit (for example 20000).
If a finite cap is reached without 100 accepted routes, the executable
fails with a diagnostic instead of presenting an incomplete comparison.

Search and selection are **outside** timed optimization. Each variant sees
the same independently verified original route; both benchmark executables
use the same seeded selection. With Docker's OMPL-enabled build they use
Planning Lab's Reeds–Shepp connector plus native CSC; non-OMPL builds use
native CSC only. CSV includes start/goal poses, start/goal grid regions,
random seed, full-path fingerprint, expansions, curvature metrics and
optimizer p50/p95 runtime. Stderr reports progress, rejected candidate
counts, **per-region start/goal coverage**, and across-path runtime and
smoothness summaries. The summaries also report mean accepted replacements,
**paths hitting the accepted-replacement limit**, and mean steering reversals
removed. For comparable results use the same seed, budget, map configuration
and connector.

### Docker commands (BUILD_JOBS=28 / TEST_JOBS=4)

```bash
git fetch origin feature/ackermann-refinement-smoothness-benchmarks
git switch feature/ackermann-refinement-smoothness-benchmarks \
  || git switch --track origin/feature/ackermann-refinement-smoothness-benchmarks

export BUILD_JOBS=28 TEST_JOBS=4

docker compose build test
docker compose --profile test run --rm test \
  ctest --test-dir build-docker --output-on-failure -j "$TEST_JOBS"

# Test 1: legacy 35/3/100k versus expanded 160/8/200k, existing objective.
docker compose --profile test run --rm test \
  ./build-docker/hybrid_astar_ackermann_refinement_budget_benchmark_test \
  5 10000 100 250000 0 1.0 20261028 > budget-results-100.csv

# Test 2: compare 160/8, 160/12 and 160/16, with steering off/on.
# Also includes the original 35/3 reference pair (8 variants total).
docker compose --profile test run --rm test \
  ./build-docker/hybrid_astar_ackermann_refinement_smoothness_benchmark_test \
  5 10000 100 250000 0 1.0 20261028 > smoothness-budget-comparison.csv

# Fast contracts, including steering-aware pruning and discovery uniqueness.
docker compose --profile test run --rm test \
  ctest --test-dir build-docker \
    -R 'steering_objective_test|refinement_scenario_discovery_test' \
    --output-on-failure
```

### Accepted-replacement limit: 8 versus 12 versus 16

The steering/smoothness benchmark now runs **eight variants** on each of the
same 100 strictly validated original warehouse paths:

| Objective | Legacy reference | Expanded acceptance-budget comparison |
|---|---|---|
| Distance/clearance only | 35 attempts / 3 accepted / 100k checks | **160/8**, **160/12**, **160/16** (200k strict checks for all three) |
| Steering-aware penalty | 35 attempts / 3 accepted / 100k checks | **160/8**, **160/12**, **160/16** (200k strict checks for all three) |

The **normal application default stays 160/8**; the 160/12 and 160/16
settings are *benchmark-only*. The original four CSV variant names remain
unchanged for compatibility; the four additional rows per path are named
`expanded_160_12_length`, `expanded_160_16_length`,
`expanded_160_12_steering` and `expanded_160_16_steering`.
The output contains **800 CSV rows** (8 profiles x 100 distinct paths).
Each profile runs five timed repetitions per path, plus one untimed warmup;
scenario discovery is performed once per executable and is excluded from
refinement timing.

Compare `accepted` with `max_accepted` (including
`paths_hitting_accepted_limit` in the stderr summaries) to determine whether
larger acceptance limits are actually exercised. Then compare
`refined_curvature_tv_inv_m`, `refined_steering_reversals`,
`refined_length_m`, `strict_checks` and `refine_p50_ms`
**per matching `path_fingerprint`**. If runs regularly stop at the 160
attempt cap before accepting 16 improvements, a *separate* experiment can
increase the attempt budget; this test deliberately changes *only* the
acceptance limit.

To explore other steering penalties, change the **sixth** numeric argument
of the smoothness executable (for example `0.25`, `0.5` or `2.0`);
the seventh is the random seed. All four length-only profiles always use
weight zero. Run with the same seed/connector and an idle host for comparable
timings; do not assume larger accepted budgets always improve curvature
variation.

The benchmarks are **manual by default** and are not run by ordinary CTest or
GitHub Actions. To register both as **explicitly opted-in, long-running CTest**
cases with the full 100-path/10k-expansion defaults:

```bash
REGISTER_REFINEMENT_BENCHMARK_TESTS=ON BUILD_JOBS=28 TEST_JOBS=4 \
  docker compose build test

# Run the benchmarks serially for fair timing (normal tests can use -j 4).
REGISTER_REFINEMENT_BENCHMARK_TESTS=ON BUILD_JOBS=28 TEST_JOBS=4 \
  docker compose --profile test run --rm test \
  ctest --test-dir build-docker \
    -R 'ackermann_refinement_(budget|smoothness)_benchmark' \
    --output-on-failure -j 1
```

The equivalent native CMake option is
`-DHYBRID_ASTAR_REGISTER_REFINEMENT_BENCHMARK_TESTS=ON` alongside
`-DHYBRID_ASTAR_BUILD_BENCHMARKS=ON`. With registration disabled (the
default), ordinary CTest remains fast and the two benchmark executables are
still available for manual use. Timing comparisons should use an otherwise
idle host.

The shortcut candidates remain fixed-curvature CSC and Reeds-Shepp arcs;
genuine curvature-continuous candidate generation should be evaluated only
after inspecting the new 100-path steering-weight results.
