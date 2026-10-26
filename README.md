# hybrid-astar-cpp

A standalone, high-performance **C++20 Hybrid A*** path-planning library. **Status: initial complete-primitives Ackermann planning vertical slice, with exact filled-footprint pose checks and strict adaptive-sampled final validation. The optional signed OMPL connector and lazy performance heuristic are available but require explicit provider configuration; A focused real OMPL 2.0.2 state-space source integration and its randomized tests now pass; a complete official OMPL package build and continuous swept certification remain unverified/unimplemented.**

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
No GitHub Actions workflow is required.

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
build and test are manual, not GitHub Actions automation. See the
[complete Docker guide](docs/docker.md) for CLI planning and benchmarks.
**Docker execution has not yet been validated in this assistant environment**
because no Docker engine is installed here.

## Build and test the Ackermann vertical slice

Catch2 **v3** is required for test builds. If you only need the core/demo
without Catch2, configure with `-DHYBRID_ASTAR_BUILD_TESTS=OFF`.

```bash
cmake -S . -B build -DHYBRID_ASTAR_ENABLE_OMPL=OFF
cmake --build build
ctest --test-dir build --output-on-failure
# OMPL-ON only when the OMPL development package is installed:
# cmake -S . -B build-ompl -DHYBRID_ASTAR_ENABLE_OMPL=ON
# cmake --build build-ompl && ctest --test-dir build-ompl --output-on-failure
```

For local manual performance experiments (no GitHub Actions workflow):

```bash
cmake -S . -B build-bench -DCMAKE_BUILD_TYPE=Release -DHYBRID_ASTAR_BUILD_BENCHMARKS=ON
cmake --build build-bench --parallel
./build-bench/hybrid_astar_planner_bench 10
```

This compares deterministic open-field forward/reverse, distant-doorway,
opt-in 4/5-disc EDT and experimental clearance scenarios. Record hardware, compiler, build flags and repository commit
alongside measurements; a single sample is not a performance guarantee.

When OMPL's CMake package is installed, enable the separate steering adapter and its smoke test with `-DHYBRID_ASTAR_ENABLE_OMPL=ON`. OMPL types are not exposed by the public headers. The default OFF builds the entire current six-primitive Ackermann vertical slice. The optional signed RS adapter requires OMPL. Link `hybrid_astar::ompl` and call `hybrid_astar::steering::planWithReedsShepp(request)` for explicit automatic connector wiring; pass `true` to opt into noncertified RS-plus-obstacle-grid ordering. The provider-neutral signed section decoder is in core and is regression-tested without OMPL. The adapter, real OMPL 2.0.2 Reeds–Shepp source implementation and
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

The initial Ackermann **search key** uses independent 0.25 m XY cells, 5-degree heading bins (72), and incoming gear. The goal accepts independent 0.25 m XY and 10-degree yaw tolerances on the actual reached state. Default costs are 1.00× forward, 1.10× reverse, and +0.50 m for a gear switch. The active planner uses a finite goal-region Euclidean ordering heuristic; a selectable noncertified performance mode uses lazy reverse-Dijkstra obstacle guidance plus an optional injected RS exact-goal distance. The analytic-attempt scheduler uses **search-bin** units and is integrated whenever a native signed analytic connector is supplied. The shipped planner uses exact full-body checks by default; an optional request-local EDT multi-disc prefilter (default **off**) can skip proven-clear search poses, never final validation. Legacy `CheckOptions` defaults to full-footprint, not boundary-only. The optional oriented-rectangle clearance cost is zero by default; bounded validated-incumbent improvement is also opt-in.

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
- Local build/test/benchmarks first. Any future GitHub Actions workflow must be manual (`workflow_dispatch`) unless explicitly requested.
