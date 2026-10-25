# Architecture baseline (draft v0.1)

This is a lightweight, reviewable specification rather than a full arc42 document. Documents describe **intent and proposed contracts**; nothing here implies that implementation or benchmarks already exist.

| Document | Focus |
| --- | --- |
| [Requirements](requirements.md) | Traceable capabilities, constraints, non-goals |
| [Components](components.md) | Ownership, dependency direction, data flow |
| [Interfaces](interfaces.md) | C++20-oriented API sketches and semantic contracts |
| [Kinematic models](kinematic-models.md) | Controlled differential equations, constraints and planar chassis examples |
| [Passenger-car fixture](passenger-car-test-fixture.md) | Representative 4.0 m rear-axle turning radius, 4.6 × 1.8 m body and manufacturer turning-circle references |
| [Map coordinates](map-coordinates.md) | ROS-compatible map frames, grid indices and occupancy semantics |
| [Algorithms](algorithms.md) | Search, obstacle heuristic, collision and reconstruction |
| [Quality](quality.md) | Nonfunctional requirements, measurement protocol |
| [Verification](verification.md) | Contract-driven test and benchmark strategy |
| [Open questions](open-questions.md) | Unresolved decisions, review gates |
| [Work items](../TODO.md) | Phased actionable checklists |
| [ADRs](../adr/README.md) | Decision history and rationale |

## Architecture principles

1. **One search engine, many models:** continuous state propagation and frame transforms are provided by model policies. The search does not hardcode steering, wheelbase, holonomicity, or state dimension. **Three-state fixed-curvature Ackermann** ([ADR-0014](../adr/0014-fixed-curvature-ackermann-baseline.md)) is the **first end-to-end implementation**; differential drive and ideal mecanum are later reference models ([ADR-0008](../adr/0008-motion-model-capabilities.md)).
2. **Versioned exploration-friendly environment:** each search uses one immutable map snapshot, even while SLAM publishes newer revisions; unknown cells remain distinct from known-free space, with an explicit per-request `Occupied`-default policy supporting frontier exploration ([ADR-0017](../adr/0017-explicit-unknown-occupancy-policy.md)). Frontier selection, dynamic tracking, prediction and MPC remain outside the core. Persistent blockages may cause a new planning request.
3. **Hybrid heuristic default (ADR-0027):** first Ackermann search combines OMPL Reeds–Shepp distance with **lazy, cached, multi-source reverse Dijkstra** from a valid XY goal region. That performance mode is **not proven admissible** for our goal tolerance and discrete grid; a separately selected certified Euclidean-to-goal-region/zero mode exists ([ADR-0015](../adr/0015-admissible-default-heuristics.md)).
4. **Feasible primitives and connections:** each accepted transition is validated against model constraints and the complete configuration-dependent collision geometry.
5. **Compact search and exact path reconstruction:** retain one active continuous arrival per model-defined discrete key ([ADR-0013](../adr/0013-single-label-search-baseline.md)); preserve primitive trajectories, direction changes, and full model state. The single-key approximation may discard distinct future continuations. Refinement is optional and cannot replace an accepted path without validation.
6. **Representation-independent path output:** piecewise, continuously evaluable segments; motion direction, in-place rotation, sideways translation, combined motion and articulation are explicit. Body heading need not equal travel tangent; global differentiability is not assumed. The default goal checks the continuous pose within independently configurable XY/yaw tolerances ([ADR-0016](../adr/0016-default-pose-goal-tolerances.md)).
7. **ROS 2 conventions natively; ROS APIs only at edges:** use REP-103/105 axes and frames, and OccupancyGrid layout ([ADR-0009](../adr/0009-ros-map-coordinate-conventions.md)). Adapters handle TF, messages and ROS parameters; no ROS headers in core.
8. **Supporting libraries are scoped:** configuration, testing, optimization and visualization must not leak into hot-path search dependencies.
9. **MVP: rigorous contracts, simple mechanisms:** deliver a small, testable end-to-end search/path slice early. Postpone speculative optimizations or generalized plumbing until reproducible real-world scenarios demonstrate a benefit; do not relax validity, collision assurance reporting or failure diagnostics to achieve simplicity ([ADR-0020](../adr/0020-evidence-driven-mvp-simplicity.md)).

## Status rules

- **Accepted:** decision explicitly agreed in design discussions and recorded in an ADR.
- **Proposed:** concrete API or algorithm detail that still needs design review.
- **Open:** a material tradeoff listed in [open questions](open-questions.md).
- Quantitative performance limits are **TBD** until representative maps, robots and hardware are agreed.

## Review process

Review requirements and architecture first, then interface semantics, algorithm guarantees, quality thresholds and verification. Close or explicitly defer every high-impact open question before implementing the corresponding module. **Don't require closure of all optional future architecture choices before building the first verified vertical slice.** Make significant changes via small ADRs; keep interfaces, tests, and checklists synchronized.
