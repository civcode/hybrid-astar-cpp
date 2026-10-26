# Architecture: baseline and current implementation

The baseline and ADRs are historical architecture proposals; the first C++20 Ackermann search vertical slice now exists. Start with [current implementation contract](current-implementation.md), then the [external-review consolidated specification](Hybrid_AStar_Consolidated_Specification.md). The consolidated review is a proposed synthesis; it is not an approved ADR, completed full planner or safety certification. Older ADR behavior is superseded where it conflicts with the current implementation contract.

| Document | Focus |
| --- | --- |
| [Current implementation](current-implementation.md) | Normative shipped behavior, evidence, open W0–W9 work packages |
| [Interactive Planning Lab app](../../apps/demo/README.md) | Loopback C++20 demo, live map/start/goal editing, exact path visualization and optional OMPL/refinement |
| [W10 differential-drive and cached primitives](w10-differential-primitives.md) | Analytic differential model, stationary angular primitives, Ackermann SE2 template reuse, RK4 oracle and integration scope |
| [W6 label hashing and collision profiling](w6-hotpath-profiling.md) | Opt-in per-stage timing for hashing, label lookup, collision and queues, label reserve/prefilter A/B and acceptance gate |
| [W6 priority queue tuning](w6-priority-queue-verification.md) | Binary/4-ary/indexed open and reverse-grid queues, deterministic equivalence, end-to-end p50/p95 and storage diagnostics |
| [W3/W6 failure root-cause analysis](w3-w6-failure-investigation.md) | Invalid dead-end goal correction, insufficient detour grid budget, 18 alternative probes and opt-in heuristic inflation tradeoffs |
| [Hard scenarios and W6 heuristic experiments](w3-w6-hard-scenarios.md) | Seven deterministic maneuver/obstacle cases; exact path verification, finite grid budgets, CSV benchmark and measured tradeoffs |
| [Real OMPL 2.0.2 verification](w5-ompl-source-verification.md) | Offline genuine RS-source runtime tests, 12k pairs, plan/refine and strict-budget evidence, package gate still pending |
| [W3–W5 verification](w3-w5-verification-notes.md) | Signed section decoder, parking fixtures, strict budget and explicit unverified OMPL-ON gate |
| [W8 CSC verification](w8-csc-verification.md) | Opt-in forward/reverse curvature-bounded connectors, certified cost pruning, local randomized tests and benchmark tradeoffs |
| [W8 refinement verification](w8-refinement-verification.md) | Bounded post-search exact-arc shortcutting, independent whole-path SAT checking, safe fallback and 7-suite local evidence |
| [W7–W8 verification](w7-w8-verification-notes.md) | Conservative EDT pose-bypass argument, optional improvement/clearance, exploratory benchmark regression |
| [Consolidated synthesis](Hybrid_AStar_Consolidated_Specification.md) | Original three-review proposed architecture and decision register |
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
3. **Heuristic selection:** the current default is certified Euclidean distance to the goal tolerance region. The optional noncertified performance mode combines request-local lazy reverse-grid settlement with a supplied signed Reeds–Shepp metric (or finite fallback). `steering::planWithReedsShepp(request, true)` explicitly opts into that mode when OMPL is linked; see [current implementation](current-implementation.md). Historical ADR-0027 may describe a different default.
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
