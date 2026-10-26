# hybrid-astar-cpp

A standalone, high-performance **C++20 Hybrid A*** path-planning library. **Status: initial complete-primitives Ackermann planning vertical slice, with exact filled-footprint pose checks and strict adaptive-sampled final validation. The optional signed OMPL connector and lazy performance heuristic are available but require explicit provider configuration; OMPL-enabled builds and continuous swept certification remain unverified/unimplemented.**

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

## Build and test the Ackermann vertical slice

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

When OMPL's CMake package is installed, enable the separate steering adapter and its smoke test with `-DHYBRID_ASTAR_ENABLE_OMPL=ON`. OMPL types are not exposed by the public headers. The default OFF builds the entire current six-primitive Ackermann vertical slice. The optional signed RS adapter requires OMPL. Link `hybrid_astar::ompl` and call `hybrid_astar::steering::planWithReedsShepp(request)` for explicit automatic connector wiring; pass `true` to opt into noncertified RS-plus-obstacle-grid ordering. The provider-neutral signed section decoder is in core and is regression-tested without OMPL. The OMPL-ON build has not yet been verified here. **The OMPL-enabled build is not yet verified in the current environment**, which has no OMPL installation.

The initial Ackermann **search key** uses independent 0.25 m XY cells, 5-degree heading bins (72), and incoming gear. The goal accepts independent 0.25 m XY and 10-degree yaw tolerances on the actual reached state. Default costs are 1.00× forward, 1.10× reverse, and +0.50 m for a gear switch. The active planner uses a finite goal-region Euclidean ordering heuristic; a selectable noncertified performance mode uses lazy reverse-Dijkstra obstacle guidance plus an optional injected RS exact-goal distance. The analytic-attempt scheduler uses **search-bin** units and is integrated whenever a native signed analytic connector is supplied. The shipped planner uses exact full-body checks by default; an optional request-local EDT multi-disc prefilter (default **off**) can skip proven-clear search poses, never final validation. Legacy `CheckOptions` defaults to full-footprint, not boundary-only. The optional oriented-rectangle clearance cost is zero by default; bounded validated-incumbent improvement is also opt-in.

## Conventions

- C++20, standard CMake; core has **zero ROS 2 dependencies**.
- Catch2 v3 for tests, cxxopts for CLI tools, nlohmann/json and yaml-cpp in serialization/configuration adapters, Eigen3 where useful, and optional Ceres Solver for refinement.
- OMPL is the selected runtime Dubins/Reeds–Shepp provider, isolated behind project-native steering APIs; the in-house solver is low priority.
- Kinematic models are controlled differential equations with admissible controls and body-frame transforms; [mathematical specification](docs/architecture/kinematic-models.md) and [ADR-0008](docs/adr/0008-motion-model-capabilities.md).
- Existing civcode reference repositories inform design but **must not be vendored or copied**.
- Local build/test/benchmarks first. Any future GitHub Actions workflow must be manual (`workflow_dispatch`) unless explicitly requested.
