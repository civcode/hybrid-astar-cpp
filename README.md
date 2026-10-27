# hybrid-astar-cpp

A standalone, high-performance **C++20 Hybrid A*** path-planning library. **Status: initial complete-primitives Ackermann planning vertical slice, with exact filled-footprint pose checks and strict adaptive-sampled final validation. The lazy obstacle-grid performance heuristic is enabled by default (unit weight), with certified Euclidean guidance still selectable; the OMPL signed analytic connector remains optional; A focused real OMPL 2.0.2 state-space source integration and its randomized tests now pass; a complete official OMPL package build and continuous swept certification remain unverified/unimplemented.**

The core is intended to be independent of ROS 2, any single vehicle model, and any single geometric curve family. Initial implementation priorities are differential drive and Ackermann; an ideal mecanum/omnidirectional chassis is the third reference model for testing generic interfaces. A separate trajectory planner/MPC will handle time-parameterization and moving-obstacle prediction.

## Architecture baseline

Start at [Architecture documentation](docs/architecture/README.md), then review:

- [Requirements and constraints](docs/architecture/requirements.md)
- [Components and dependencies](docs/architecture/components.md)
- [Interface contracts](docs/architecture/interfaces.md)
- [ROS-compatible map coordinates](docs/architecture/map-coordinates.md)
- [Planning algorithms](docs/architecture/algorithms.md)
- [Nonfunctional requirements](docs/architecture/quality.md)
- [Verification plan](docs/architecture/verification.md)
- [Open decisions](docs/architecture/open-questions.md)
- [Implementation checklist](docs/TODO.md)
- [Architecture Decision Records](docs/adr/README.md)

The [consolidated external-review specification](docs/architecture/Hybrid_AStar_Consolidated_Specification.md) is versioned as a **proposal**, not an approved ADR. The [current implementation contract and work-package traceability](docs/architecture/current-implementation.md) describes which recommendations have shipped and what remains unverified. See [W3–W5 verification notes](docs/architecture/w3-w5-verification-notes.md) for signed RS decoding, parking fixtures and strict sample-budget evidence, and [W7–W8 verification notes](docs/architecture/w7-w8-verification-notes.md) for conservative cover and quality experiments.

The core now implements a first standalone Ackermann search: six exact analytic primitives, continuous states with gear-aware one-label keys, append-only parent records/reopening, full-body rectangle-versus-closed-grid-square collision, request-pinned occupancy snapshots, and an independent stricter full-path sampled validation pass. This is a geometric initializer; sampling does not prove full continuous swept safety or timed actuation feasibility.

## Continuous footprint collision inflation

`PlanOptions::footprintInflationMeters` adds a nonnegative **metric
Minkowski-disk offset** around the current physical robot rectangle without
discrete grid dilation (default `0.0`). Example:

```cpp
request.options.footprintInflationMeters = 0.05; // 5 cm margin
```

The inflated collision checker checks geometric rectangle-to-occupied-cell
distance, rounded corners and map boundaries, retaining the legacy SAT-only
fast path at zero margin. The margin propagates to search and independent
final validation, conservative-disc cover and strict post-path refinement.
It is independent of soft clearance scoring. The Planning Lab exposes a
checkbox plus arbitrary finite meter input (e.g. 0.05 or 0.1, up to 10 m in
the demo); its JSON exports carry the effective value. The currently
supported collision primitive is a rectangle; a shared convex-polygon versus
square distance utility is ready for other convex geometries later.
This **does not** establish full swept-volume or tracking guarantees.
See [Planning Lab usage](apps/demo/README.md#continuous-metric-footprint-inflation)
and the focused `footprint_inflation_test` regressions.

## Interactive path planning demo

A new [`apps/demo/` Planning Lab](apps/demo/README.md) provides a **browser-based
interactive demo backed by the actual C++ Hybrid A* planner**. It runs locally
on 127.0.0.1, does not depend on ROS and works without OMPL:

```bash
cmake -S . -B build-demo -DCMAKE_BUILD_TYPE=Release -DHYBRID_ASTAR_BUILD_APP=ON
cmake --build build-demo --parallel --target hybrid_astar_demo
./build-demo/hybrid_astar_demo
```

Open **http://127.0.0.1:8765**. The demo now also offers **interactive occupancy-grid editing**: drag obstacle/free/unknown/restore brushes directly on the map, with bounded sparse edits, fresh immutable map revisions and automatic strict replanning on stroke completion. Changing the map immediately hides the earlier validated route. See [brush editing semantics](apps/demo/README.md#interactive-obstacle-editing-immutable-map-revisions). To explore a custom occupancy map,
start the app with `--map apps/demo/maps/sample.pgm --resolution 0.5`,
or use **`--map-yaml apps/demo/maps/sample.yaml`** for ROS-style resolution,
occupancy thresholds and a translated/rotated origin.
The app accepts bounded PGM P2/P5 files and restricted map-server YAML and uses the actual
C++ planner to validate every accepted route. The browser now provides
**Export path JSON**, and CLI mode supports `--once --output plan.json`.
See [map loading and export semantics](apps/demo/README.md#import-a-pgm-map-and-export-a-validated-planning-result).

 Change maps, click to set start
(Shift-click sets goal), adjust headings, replan, compare strict validated
original/refined routes, and animate the full Ackermann footprint.
Forward/reverse sections and cost/expansion/time metrics are displayed.
When the optional OMPL adapter is built, Reeds–Shepp connector controls
are enabled automatically. The executable also supports
`--once --scenario reverse` for machine-readable CLI JSON.
See the [app README](apps/demo/README.md) for full usage, API and limitations.
GitHub Actions is optional and **manual only**, with a limited set of test suites.
See the [manual verification guide](docs/ci/manual-verification.md).

## Docker: dependencies, tests and demo

The root [Dockerfile](Dockerfile) builds a reproducible Ubuntu 24.04 C++20
environment with **Catch2 v3, Boost, Eigen3 and OMPL 2.0.2 from source**.
The optional browser demo lives under [`apps/demo/`](apps/demo/README.md).
Run it locally without first installing toolchains or OMPL:

```bash
docker compose up --build demo
# Browse http://127.0.0.1:8765
```

Explicitly execute the **Catch2/CTest** and real-OMPL integration suites:

```bash
docker compose run --build --rm test
```

The Compose mapping publishes **only 127.0.0.1 on your host**. The Docker
build and test are manual. The optional [manual-only Actions workflow](docs/ci/manual-verification.md)
can invoke the same Docker integration suite by explicit selection; it has
no push, PR or scheduled triggers. See the
[complete Docker guide](docs/docker.md) for CLI planning and benchmarks.
**Docker execution has not yet been validated in this assistant environment**
because no Docker engine is installed here.

## W12 — ideal mecanum/holonomic model

The shared W11 policy-driven kernel now also supports an ideal four-wheel
mecanum chassis, including body-lateral translation, diagonals, simultaneous
translation/rotation and in-place turning. It uses an exact constant-twist
SE(2) evaluator, 14 cached primitive templates, wheel-rate validation,
positive translation/angular effort costs and independent strict sampled
full-rectangle collision validation. Select `--model mecanum` in CLI or
choose **Mecanum** in Planning Lab. This is geometric planning only,
without guaranteed slip-free/actuator-feasible execution.

The new Catch2 and demo HTTP regression tests are **registered, not
verified as executed** in the full GitHub checkout. Complete CMake,
Docker, OMPL and sanitizer gates remain pending; see
[W12 implementation and verification](docs/architecture/w12-mecanum-holonomic.md).
On `dev`, the default work budget is now **1,000,000 expansions** (up to
4,000,000 generated nodes). Mecanum has an optional nearby exact-goal
connection that must pass the full independent strict validator and tighter
search sampling; the new warehouse failure regression remains unverified
until the manual CTest suite has run. Higher budgets may increase memory
and CPU use for unsolvable cases.

## W11 — model-policy Hybrid A* search kernel

The native Hybrid A* expansion loop is now **policy-driven**, not a
switch between hardcoded Ackermann and differential-drive successor
algorithms. Both models provide their local primitive catalog, search-key
discretization and hash, exact section evaluator, incoming-motion mode,
collision footprint, and positive edge costs through a common C++20
`PlanarSearchModel` contract. Search node storage is typed and the
model dispatch happens once per request; priority queues, one-label
dominance, cancellation, immutable map revision, and independent strict
path validation remain in the single shared kernel.

An external rigid-planar policy can use
`planning::PlanarPolicyAdapter<MyPolicy>` with
`ackermann::planWithPolicy(request,adapter)`, without duplicating the
planner or editing the model selection switch. Tests compare both
built-ins to the adapter path and exercise a third **pivot-only** planar
policy, including custom collision geometry and rejection of dishonest
segment endpoints. Unsupported analytic/clearance hooks and nonpositive
edges are rejected for externally supplied policies.

**Current limit:** the reusable runtime policy accepts SE(2) rigid-body
states and a rectangular footprint. Full articulated state, compound
multibody collision and holonomic lateral section support require
additional model-specific state/geometry contracts. See
[W11 model-policy kernel](docs/architecture/w11-model-policy-kernel.md).

## W10 — cached Ackermann primitives and differential-drive kinematics

The C++ core now includes **two exact constant-control planar models**:
the existing Ackermann geometric bicycle and the new ideal
differential-drive (unicycle) model with validated wheel-rate limits.
Both have **precomputed local SE(2) motion-primitive catalogs**, storing
normalized progress and full pose samples.

For Ackermann, six local forward/reverse left/straight/right templates
are built **once per plan** and transformed at every Hybrid A* expansion.
Search samples are taken from the precomputed template; independent final
validation still evaluates exact continuous arc sections at stricter
sampling settings. For differential drive, eight local templates include
forward/reverse straight and curves, plus **in-place rotation** with
angular extent and nonzero intermediate-body movement. An analytic
constant-twist evaluator handles straight, curved and stationary turning
without RK4 in the hot path; tests compare an independent RK4 oracle.

```bash
cmake -S . -B build-w10 -DCMAKE_BUILD_TYPE=Release \
  -DHYBRID_ASTAR_BUILD_TESTS=OFF
cmake --build build-w10 --parallel
```

The differential-drive **model, primitive provider, Hybrid A* successor
expansion, angular effort costs, strict sampled body-motion validation and
Planning Lab selector are implemented**. This is a second end-to-end
geometric model, not yet a fully generic state-space abstraction.
Positive-cost spins preserve the incoming drive mode and independently
sample the whole moving footprint; the existing Ackermann Reeds–Shepp
connector and refinement are explicitly unsupported for differential mode. See
[W10 primitives contract](docs/architecture/w10-differential-primitives.md) and
[search/demo integration](docs/architecture/w10-differential-search.md)
for the API, limitations and Catch2/RK4 tests.

## W6 label hashing, exact collision and CPU hotpath profiling

The planner now counts **search-label hash-table lookups/inserts/rehashes,
exact whole-footprint collision calls, search/final motion checks**, and
queue work. An optional build flag
`-DHYBRID_ASTAR_PROFILE_HOTPATHS=ON` adds per-stage nanosecond timings
without clock-read overhead in ordinary builds. Hash and collision timings
are nested within label operations and motion checks, respectively.

Two conservative source changes remove a redundant hash lookup for
improved search labels and cache the immutable occupancy map's origin
rotation. Neither changes collision semantics or path priority. The new
`PlanOptions::searchLabelReserve` defaults to 0; label-table reservation
is an **opt-in** allocation A/B knob, not a global performance claim.

```bash
cmake -S . -B build-hotpath -DCMAKE_BUILD_TYPE=Release \
  -DHYBRID_ASTAR_BUILD_TESTS=ON -DHYBRID_ASTAR_BUILD_BENCHMARKS=ON \
  -DHYBRID_ASTAR_PROFILE_HOTPATHS=ON
cmake --build build-hotpath --parallel
ctest --test-dir build-hotpath --output-on-failure
./build-hotpath/hybrid_astar_hotpath_bench 7 > hotpath.csv
```

On `dev`, the benchmark also accepts `--warehouse [repeats] [budget]`,
`--heuristics [repeats]` and `--warehouse-heuristics [repeats] [budget]`.
Warehouse runs share the off-lattice mecanum CTest fixture and are manual
because a 1M-expansion run may be expensive. The CSV now exposes exact-goal
connector attempts/strict rejections and grid settlements/fallbacks.
Four-disc direct-checker implication tests cover rotated maps, contacts,
narrow gaps and mecanum. The default prefilter stays OFF pending validation.

### Avoiding unnecessary in-place rotations

Mecanum search now applies a configurable
`PlanOptions::mecanumStationarySpinPenaltyMeters` (default **0.35 m**)
to stationary rotations, in addition to angular motion cost. It favors
simultaneous translation/turns without forbidding necessary pivots.
`mecanumMergeIntermediateSpins` (default `true`) performs a bounded,
independently collision-checked whole-path cleanup before returning a
validated mecanum plan. It uses a constant-body-twist SE(2) connector
with checked wheel-rate limits and accepts only weighted-cost reductions.
It retains the original route if no connector is feasible or safe.
The feature does not provide actuator acceleration or continuous
curvature guarantees, and can increase search work.

Differential-drive post-path refinement no longer adds stop-and-spin
shortcuts. Straight chords are always available, and optional curved
CSC candidates can replace existing zigzags without inserting new
stationary rotations. Existing spins may remain when no safe improvement
is found.

### Strict mecanum post-path refinement (manual opt-in)

With **Mecanum** selected, Planning Lab now permits **Strict post-path
refinement**, using bounded multi-scale windows and direct body-twist
connectors that translate sideways/forward while rotating if needed. These
shortcuts do not generate a stationary spin; every full replacement route
is rechecked against occupied map cells and the configured geometric
inflation. It retains the original valid path unless weighted cost decreases.
Native CSC is deliberately disabled for mecanum.

### Differential-drive path smoothing (manual opt-in)

Diagonal-route CSC smoothing can now try 0.375, 0.75, 1.5 and 3.0 m
radii (the four multipliers around the configurable 0.75 m base).
Use `RefinementOptions::differentialRadiusVariants=1` to reproduce
the historical fixed-radius geometry or `4` for all radii.
The manual `hybrid_astar_refinement_bench --warehouse-differential`
compares both options on the reported diagonal warehouse route.
See [W10 experiment instructions](docs/architecture/w10-differential-search.md#diagonal-smoothing-and-radius-ab-manual-only-2026-10-27).



The strict post-path refiner now also accepts `Model::DifferentialDrive`
and preserves the original path unless independent full-body verification
confirms an actual weighted-cost reduction. Native candidate connections
include straight-line chords and optional CSC curved shortcuts with a
configurable 0.75 m radius. Adjacent moving segments cannot acquire
a sharper curvature discontinuity or a new gear reversal at a splice.
The smoother never generates new in-place
rotations (which require a full stop); it may retain spins already in the
original path if they cannot be safely removed.
Candidate constant-curvature commands are checked against differential
wheel-rate limits and can be traversed at reduced positive speeds.
The finite candidate scheduler samples different spans of long paths.
This is geometric simplification, not a continuous-curvature, acceleration,
or actuator-tracking guarantee.

In Planning Lab choose **Differential drive**, then check **Strict post-path
refinement**; optionally check **Native curved CSC candidates**. Exported
JSON includes both `original` and `refinement.path` when validation succeeds.
The C++ interface remains `ackermann::refinePath(request, original)`, with
`RefinementOptions.useCurvatureBoundedShortcuts` and
`differentialCurveRadiusMeters` providing opt-in CSC. Mecanum remains
now also supported via exact holonomic SE(2) twist shortcuts and independent
full-footprint verification. Tests were added but must be executed locally.
See [the expanded experiment guide](docs/architecture/w6-hotpath-profiling.md).

A second profiling-**OFF** build is required to compare end-to-end
latency without clock overhead. The [profiling and acceptance notes](
docs/architecture/w6-hotpath-profiling.md) distinguish measurements
and unverified hypotheses; no new default is selected until those
hardware runs are completed.

## W6 priority-queue profiling and A/B tuning

Both search queues now expose explicit queue push/pop/stale/peak/memory
diagnostics. The original `std::priority_queue` binary lazy heap is
retained as the **default** for the Hybrid A* open set and the optional
reverse-Dijkstra grid frontier. Four-child and indexed binary/four-child
heaps can be selected independently to compare realistic CPU and memory
tradeoffs, without changing certified search semantics. Expansion-state
copies were also reduced to fields used during expansion rather than
copying full motion sections.

```cpp
request.options.openQueueMode =
    hybrid_astar::ackermann::OpenQueueMode::FourAryLazy;
request.options.gridQueueMode =
    hybrid_astar::ackermann::OpenQueueMode::BinaryLazy;
```

Run the **manual** deterministic p50/p95/memory comparison:

```bash
cmake -S . -B build-queues -DCMAKE_BUILD_TYPE=Release \
  -DHYBRID_ASTAR_BUILD_TESTS=ON -DHYBRID_ASTAR_BUILD_BENCHMARKS=ON
cmake --build build-queues --parallel
ctest --test-dir build-queues --output-on-failure
./build-queues/hybrid_astar_priority_queue_bench 7 > queues.csv
```

The indexed frontiers eliminate stale heap pops but can increase storage
and latency, especially for sparse-index reverse-grid cells. See
[measurements and verification](docs/architecture/w6-priority-queue-verification.md).
No alternative has yet shown sufficiently consistent end-to-end improvements
to replace the original default.

## Hard-scenario verification and W6 search benchmarking

Seven deterministic planner fixtures now cover reverse approach, three-point
turns, parallel parking, narrow corridors, U-shaped dead ends, long obstacle
detours and an impassable sub-vehicle-width gap. They exercise **strict
independent sampled validation**, result provenance, reproducible search
diagnostics and explicit budget-limited outcomes. They explicitly compare
certified Euclidean search against the now-default noncertified obstacle-grid
heuristic; an OMPL-enabled build additionally checks Reeds–Shepp.

W6 now seeds the goal tolerance disk using a **bounded grid-cell window**
rather than scanning the whole map; optionally prevents diagonal cutting
across blocked corners; and applies a **cumulative reverse-grid settlement
budget** with finite fallback. These mechanisms are *performance ordering*
only, not continuous-space admissibility or NoPath proofs.

Manually compare identical maps and work limits:

```bash
cmake -S . -B build-hard -DCMAKE_BUILD_TYPE=Release \
  -DHYBRID_ASTAR_BUILD_BENCHMARKS=ON
cmake --build build-hard --parallel
./build-hard/hybrid_astar_hard_scenario_bench 5 > hard-scenarios.csv
```

Add `-DHYBRID_ASTAR_ENABLE_OMPL=ON` with an installed OMPL package for
signed analytic steering rows. See [hard-scenario evidence and W6
tradeoffs](docs/architecture/w3-w6-hard-scenarios.md). The [failure investigation](docs/architecture/w3-w6-failure-investigation.md)
identified an invalid dead-end fixture goal and a long-wall heuristic work
budget that was too small. The corrected reverse escape succeeds; an opt-in
`performanceHeuristicWeight=1.2` with 20k grid settlements also recovers
the detour within 8k expansions, with a measurable first-path cost penalty.
Since 2026-10-27 the default uses bounded lazy obstacle-grid ordering
with weight 1.0. Certified Euclidean remains an explicit opt-in, and all
successful paths still pass the independent strict collision validator.
Benchmarks are manual and never automatically dispatched to GitHub Actions.
The full regression matrix for the changed default awaits Docker/CTest verification.

## Build and test the Ackermann vertical slice

When tests are enabled (the default), CMake **automatically fetches and
builds Catch2 v3.16.1** using `FetchContent` (Git + network required on
the first configure). No preinstalled Catch2 package is required.
Use `-DHYBRID_ASTAR_USE_SYSTEM_CATCH2=ON` to select a system-installed
Catch2 v3 package instead, or
`-DFETCHCONTENT_SOURCE_DIR_CATCH2=/path/to/Catch2` for an offline
source checkout. If you only need the core/demo without Catch2, configure
with `-DHYBRID_ASTAR_BUILD_TESTS=OFF`. See [testing setup](docs/docker.md#build-without-docker).

```bash
cmake -S . -B build -DHYBRID_ASTAR_ENABLE_OMPL=OFF
cmake --build build
ctest --test-dir build --output-on-failure
# OMPL-ON only when the OMPL development package is installed:
# cmake -S . -B build-ompl -DHYBRID_ASTAR_ENABLE_OMPL=ON
# cmake --build build-ompl && ctest --test-dir build-ompl --output-on-failure
```

For local manual performance experiments (not run by the Actions verification workflow):

```bash
cmake -S . -B build-bench -DCMAKE_BUILD_TYPE=Release -DHYBRID_ASTAR_BUILD_BENCHMARKS=ON
cmake --build build-bench --parallel
./build-bench/hybrid_astar_planner_bench 10
```

This compares deterministic open-field forward/reverse, distant-doorway,
opt-in 4/5-disc EDT and experimental clearance scenarios. Record hardware, compiler, build flags and repository commit
alongside measurements; a single sample is not a performance guarantee.

When OMPL's CMake package is installed, enable the separate steering adapter and its smoke test with `-DHYBRID_ASTAR_ENABLE_OMPL=ON`. OMPL types are not exposed by the public headers. The default OFF builds the entire current six-primitive Ackermann vertical slice. The optional signed RS adapter requires OMPL. Link `hybrid_astar::ompl` and call `hybrid_astar::steering::planWithReedsShepp(request)` for automatic connector wiring. This respects the request's heuristic setting, which now defaults to obstacle-grid; pass `true` to additionally include RS distance guidance. Set `request.options.heuristicMode=HeuristicMode::CertifiedRegionEuclidean` explicitly for the certified alternative. The provider-neutral signed section decoder is in core and is regression-tested without OMPL. The adapter, real OMPL 2.0.2 Reeds–Shepp source implementation and
planner/refiner have been tested **together** using a focused manual offline
CMake bridge; the complete upstream OMPL package/export build remains
unverified. Use
`steering::planAndRefineWithReedsShepp(request, options, performanceFlag)`
when both planning and independently validated post-search refinement are
desired with one evaluator. The original validated plan is returned separately
even when refiner validation is incomplete.

To reproduce the genuine OMPL source-runtime tests without an OMPL
installation (requires the extracted 2.0.2 archive and Eigen3 headers):

```bash
cmake -S tests/ompl_source_bridge -B build-source-ompl \
  -DCMAKE_BUILD_TYPE=Release \
  -DOMPL_SOURCE_ROOT=/path/to/ompl-2.0.2 \
  -DEIGEN3_INCLUDE_DIR=/path/to/eigen3
cmake --build build-source-ompl --parallel
ctest --test-dir build-source-ompl --output-on-failure
```

The source bridge exercises true OMPL runtime code but **is not** a complete
OMPL installation/build. See
[W5 OMPL source verification](docs/architecture/w5-ompl-source-verification.md)
for the 12,000-case integration matrix and numerical sampling fix.

The initial Ackermann **search key** uses independent 0.25 m XY cells, 5-degree heading bins (72), and incoming gear. The goal accepts independent 0.25 m XY and 10-degree yaw tolerances on the actual reached state. Default costs are 1.00× forward, 1.10× reverse, and +0.50 m for a gear switch. The active planner defaults to bounded, lazy reverse-Dijkstra obstacle-grid guidance at weight 1.0. It remains noncertified as a continuous-space cost lower bound. The finite goal-region Euclidean heuristic is available by explicitly selecting `CertifiedRegionEuclidean`; an optional OMPL RS exact-goal distance can augment grid guidance. The analytic-attempt scheduler uses **search-bin** units and is integrated whenever a native signed analytic connector is supplied. The shipped planner uses exact full-body checks by default; an optional request-local EDT multi-disc prefilter (default **off**) can skip proven-clear search poses, never final validation. Legacy `CheckOptions` defaults to full-footprint, not boundary-only. The optional oriented-rectangle clearance cost is zero by default; bounded validated-incumbent improvement is also opt-in.

## Optional W8 post-search shortcut refinement

`plan()` still returns the first strictly validated geometric path by
default. To try bounded, obstacle-checked section-boundary shortcuts **only
after planning**, use the new standalone core API:

```cpp
#include "hybrid_astar/ackermann/refinement.hpp"
auto original = hybrid_astar::ackermann::plan(request);
if (original.status == hybrid_astar::ackermann::PlanStatus::Success) {
  auto refined = hybrid_astar::ackermann::refinePath(request, original);
  // refined.path is populated only after the original was revalidated.
  // Inspect refined.status: Improved, Unchanged, WorkBudgetExceeded, etc.
}
```

The default built-in proposal is a same-heading straight section shortcut.
To try native **curvature-bounded** forward/reverse CSC arcs without an OMPL
dependency, explicitly pass:

```cpp
hybrid_astar::ackermann::RefinementOptions options;
options.useCurvatureBoundedShortcuts = true;
auto refined = hybrid_astar::ackermann::refinePath(request, original, {}, options);
```

This enumerates fixed-gear LSL/LSR/RSL/RSR candidates and applies the same
strict full-path validator. It is **not** a complete Dubins/Reeds–Shepp solver,
a curvature-continuous smoother, or an actuator validator. A nonnegative
base-cost lower bound avoids exact collision checks for candidates that cannot
improve the incumbent. The optional manual benchmark
`./build-bench/hybrid_astar_refinement_bench 12` compares both modes.
See [CSC design and benchmark evidence](docs/architecture/w8-csc-verification.md).
A third `RefinementConnector` argument may supply exact signed analytic
arcs, including from an externally owned Reeds–Shepp evaluator. Every
accepted replacement is strictly less costly under the configured weights
and independently checked against the pinned map and entire filled vehicle.
Failed candidates cannot discard the revalidated original. Strict
**adaptive-sampled** geometry still does not prove continuous swept-body
clearance or timed actuator feasibility. See
[W8 refinement verification](docs/architecture/w8-refinement-verification.md)
for regression fixtures, budgets and unresolved quality gates.

## Conventions

- C++20, standard CMake; core has **zero ROS 2 dependencies**.
- Catch2 v3 **actually powers the C++ test executables** (rather than standalone `main()` assert runners). Other planned libraries include cxxopts for CLI tools, nlohmann/json and yaml-cpp in serialization/configuration adapters, Eigen3 where useful, and optional Ceres Solver for refinement.
- OMPL is the selected runtime Dubins/Reeds–Shepp provider, isolated behind project-native steering APIs; the in-house solver is low priority.
- Kinematic models are controlled differential equations with admissible controls and body-frame transforms; [mathematical specification](docs/architecture/kinematic-models.md) and [ADR-0008](docs/adr/0008-motion-model-capabilities.md).
- Existing civcode reference repositories inform design but **must not be vendored or copied**.
- Local build/test/benchmarks first. The optional GitHub Actions workflow is `workflow_dispatch`-only, with one selected suite per run and per-job usage caps.
