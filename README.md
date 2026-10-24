# hybrid-astar-cpp

A standalone, high-performance **C++20 Hybrid A*** path-planning library. **Status: architecture specification in progress; no planner implementation yet.**

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

The architecture is a *proposed initial baseline*, not a claim that the listed interfaces are implemented. Review and accept unresolved contracts before coding.

## Conventions

- C++20, standard CMake; core has **zero ROS 2 dependencies**.
- Catch2 v3 for tests, cxxopts for CLI tools, nlohmann/json and yaml-cpp in serialization/configuration adapters, Eigen3 where useful, and optional Ceres Solver for refinement.
- Native Dubins and Reeds–Shepp implementations; OMPL allowed only as an optional testing oracle.
- Kinematic models are controlled differential equations with admissible controls and body-frame transforms; [mathematical specification](docs/architecture/kinematic-models.md) and [ADR-0008](docs/adr/0008-motion-model-capabilities.md).
- Existing civcode reference repositories inform design but **must not be vendored or copied**.
- Local build/test/benchmarks first. Any future GitHub Actions workflow must be manual (`workflow_dispatch`) unless explicitly requested.
