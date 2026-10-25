# hybrid-astar-cpp

A standalone, high-performance **C++20 Hybrid A*** path-planning library. **Status: architecture baseline plus a compiling C++20 scaffold. The Hybrid A* search/planner is NOT implemented yet.**

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

The architecture is a proposed first Ackermann baseline. The only executable implementation so far is a minimal core with search-key quantization, distance/gear edge-cost arithmetic, analytic-connection guard decisions, and an optional OMPL Reeds–Shepp distance-only adapter. **Motion primitives, collision checking, goal-seeded 2D heuristic, actual analytic connector path segments and Hybrid A* search are not implemented.**

## Build the minimal scaffold

```bash
cmake -S . -B build -DHYBRID_ASTAR_ENABLE_OMPL=OFF
cmake --build build
ctest --test-dir build --output-on-failure
```

When OMPL's CMake package is installed, enable the separate steering adapter and its smoke test with `-DHYBRID_ASTAR_ENABLE_OMPL=ON`. OMPL types are not exposed by the public headers. The default OFF allows a basic core build without OMPL; the complete Ackermann planner will require OMPL when it is implemented. **The OMPL-enabled build is not yet verified in the current environment**, which has no OMPL installation.

The initial Ackermann **search key** uses 0.25 m XY cells, **5-degree yaw bins (72)** and incoming gear. This is distinct from the goal-heading **10-degree tolerance**. The Reeds–Shepp near-goal **XY trigger is 20 m**; the acceptance gate requires valid finite geometry and sampled collision validation, **not a hard path-length cap**; a weighted-cost cap is optional and disabled by default. Passing these guards is not a substitute for path/collision validity. Collision configuration defaults to `BoundaryOnlyPoseSampled` with optional interior checking; the geometry checker itself is not implemented yet.

## Conventions

- C++20, standard CMake; core has **zero ROS 2 dependencies**.
- Catch2 v3 for tests, cxxopts for CLI tools, nlohmann/json and yaml-cpp in serialization/configuration adapters, Eigen3 where useful, and optional Ceres Solver for refinement.
- OMPL is the selected runtime Dubins/Reeds–Shepp provider, isolated behind project-native steering APIs; the in-house solver is low priority.
- Kinematic models are controlled differential equations with admissible controls and body-frame transforms; [mathematical specification](docs/architecture/kinematic-models.md) and [ADR-0008](docs/adr/0008-motion-model-capabilities.md).
- Existing civcode reference repositories inform design but **must not be vendored or copied**.
- Local build/test/benchmarks first. Any future GitHub Actions workflow must be manual (`workflow_dispatch`) unless explicitly requested.
