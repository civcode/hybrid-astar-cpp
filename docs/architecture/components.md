# Components and dependency boundaries — draft v0.1

The library is **not** one object hierarchy. The search kernel is a compile-time, policy-driven orchestrator. Data types at public boundaries are value-oriented; runtime adapters can type-erase implementations where useful. The kinematic model supplies a controlled ODE and admissible controls; motion integration is a separate concern ([kinematic contract](kinematic-models.md)).

## System context

```mermaid
flowchart LR
  A[Native application / CLI] --> P[hybrid_astar core API]
  R[Optional ROS 2 / Nav2 adapter] --> P
  M[Static map provider] --> P
  P --> O[Piecewise geometric path]
  O --> T[Separate trajectory planner / MPC]
  T --> V[Vehicle controller]
  D[Dynamic-object prediction] --> T
  T -- persistent blockage / replan request --> P
```

The core accepts a consistent **static map snapshot**. Dynamic-obstacle predictions are not part of the Hybrid A* request. It is a separate planner/controller responsibility to choose a safe time-dependent trajectory.

## Internal component view

```mermaid
flowchart TD
  API[Planner facade + typed config] --> S[Hybrid A* search]
  API --> ENV[Immutable occupancy map snapshot]
  S --> KEY[State index policy]
  S --> MOT[Primitive generator / propagation]
  S --> H[Obstacle-aware + model-specific heuristics]
  S --> COL[Motion collision checker]
  S --> ANA[Optional analytic connector]
  MOT --> KM[Kinematic model]
  ANA --> STEER[Native Dubins / Reeds–Shepp / future]
  H --> STEER
  H --> ENV
  COL --> ENV
  COL --> GEO[Body geometry + frame transforms]
  GEO --> KM
  S --> REC[Exact path reconstruction]
  ANA --> REC
  REC --> PATH[Piecewise geometric path]
  PATH --> REF[Optional path refiner]
  REF --> COL
  REF --> KM
  REF --> CURVE[Curve math]
  REF --> OPT[Optional Ceres backend]
  API --> OUT[Plan result + diagnostics]
  PATH --> OUT
  REF --> OUT
```

Arrows mean *uses*. Diagram is logical; no requirement that every box maps to a separately linked library.

## Responsibilities and ownership

| Component | Owns / provides | Explicitly does not own |
| --- | --- | --- |
| Planner facade | Request validation, setup, cancellation, result and diagnostics | ROS messages, map streaming, MPC |
| Search engine | Node pool, open queue, g-values, parent links, iteration/budget policy | Vehicle equations, collision geometry, YAML/JSON |
| Kinematic model | State/control/tangent types, controlled vector field, admissible constraints, world pose and named body-frame transforms | Search bookkeeping, numerical integration algorithm |
| Integrator | Numerically or analytically propagate the model's ODE, respecting state topology; expose intermediate complete states | Primitive selection, map queries |
| Primitive generator | Candidate admissible controls, durations/lengths, integrated state trajectories; reusable caches | Global map or cost policy |
| State index policy | Quantized complete state, key hash/equality, dominance scope | Continuous propagation |
| Steering connector | Analytic feasible connections when assumptions apply | Map obstacle testing, global optimization |
| Obstacle heuristic | Static map goal-distance cache, lazy reverse graph expansion | Vehicle shape and dynamics |
| Collision checker | State/motion swept-footprint validation against snapshot | State transition selection or global heuristic |
| Robot geometry | Immutable shapes, local frame attachment and margins | Actual state estimation/TF lookup |
| Reconstruction | Reverse parent chain; piecewise exact primitives and analytic segments | Curve fitting |
| Refinement | Shortcut/fitting/optimization, feasibility certification and rollback | Search node ownership |
| Geometric path | Evaluating validated ordered segments and progress semantics | Time-parameterized robot commands |
| Serialization adapters | yaml-cpp / nlohmann/json typed config and result conversion | Hot-path algorithms |
| Optional ROS adapter | TF2, lifecycle, nav messages and plugin glue | Internal vehicle model propagation |

## Compile-time dependency rules

- `math` and foundational value types depend only on standard C++ (Eigen may be used behind an explicit target boundary).
- `kinematics` depends on shared types and math; never on search or ROS.
- `motion` depends on kinematics/math; model derivative and separate integrator are explicit contracts, and primitive caches are scoped to model configuration.
- `collision` depends on environment snapshots, geometry and model frame transforms; must never query TF2 during search.
- `heuristics` depends on environment and optional steering adapters, not on path refinement.
- `search` depends on model/primitive/index/heuristic/collision/connector *contracts*.
- `path` and `refinement` depend on shared model/geometry/validation; Ceres is a private, optional refinement dependency.
- `adapters` depend inward on public core APIs, never vice versa.

## Lifetime, mutability, and concurrency

- `MapSnapshot` is immutable and versioned during a planning request. The producer may concurrently publish a *different* snapshot.
- Model configuration, collision shape definitions, primitive catalogs, and steering solver parameters are immutable within a request. Capability metadata does not supersede model-specific state-dependent admissible control checks.
- The simplest initial policy is **one active `plan()` per planner instance**; this is proposed, not yet accepted. Concurrent requests can use distinct instances sharing immutable resources.
- Scratch pools and open/closed state belong to an individual request/instance; neither are shared unsafely.
- Obstacle-distance caches are keyed by map identity/version, grid geometry, goal cell and heuristic policy; cache-sharing/synchronization remains open.
- Abort/cancel/budget exhaustion produce an explicit result status, not an invalidated successful path.

## Initial layout (proposed)

```text
include/hybrid_astar/{core,math,kinematics,motion,environment,collision,heuristics,steering,path,refinement}
src/{core,math,kinematics,motion,environment,collision,heuristics,steering,path,refinement}
adapters/{yaml,json,ros2}
tests/{contracts,kinematics,motion,steering,heuristics,collision,search,path}
benchmarks/
examples/
docs/{architecture,adr}
```

This is a module map, not a mandate for one class/file per name. Refer to [interfaces.md](interfaces.md) before concrete header design.
