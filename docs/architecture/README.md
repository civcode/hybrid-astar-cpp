# Architecture baseline (draft v0.1)

This is a lightweight, reviewable specification rather than a full arc42 document. Documents describe **intent and proposed contracts**; nothing here implies that implementation or benchmarks already exist.

| Document | Focus |
| --- | --- |
| [Requirements](requirements.md) | Traceable capabilities, constraints, non-goals |
| [Components](components.md) | Ownership, dependency direction, data flow |
| [Interfaces](interfaces.md) | C++20-oriented API sketches and semantic contracts |
| [Algorithms](algorithms.md) | Search, obstacle heuristic, collision and reconstruction |
| [Quality](quality.md) | Nonfunctional requirements, measurement protocol |
| [Verification](verification.md) | Contract-driven test and benchmark strategy |
| [Open questions](open-questions.md) | Unresolved decisions, review gates |
| [Work items](../TODO.md) | Phased actionable checklists |
| [ADRs](../adr/README.md) | Decision history and rationale |

## Architecture principles

1. **One search engine, many models:** continuous state propagation and frame transforms are provided by model policies. The search does not hardcode steering, wheelbase, or state dimension.
2. **Static, versioned environment:** search uses an immutable map snapshot; dynamic obstacle tracking, prediction, and MPC are out of scope. Persistent blockages may cause a new planning request.
3. **Fast obstacle-aware heuristic:** reverse shortest-distance expansion from goal on a 2D static occupancy grid is mandatory; it is independent of robot kinematics and may be computed lazily and cached safely.
4. **Feasible primitives and connections:** each accepted transition is validated against model constraints and the complete configuration-dependent collision geometry.
5. **Exact path reconstruction:** preserve primitive trajectories, direction changes, and model state. Refinement is optional and cannot replace an accepted path without validating the result.
6. **Representation-independent path output:** piecewise, continuously evaluable segments; motion direction, in-place rotations and articulation configuration are explicit. Global differentiability is not assumed.
7. **ROS 2 only at edges:** adapters handle TF, messages, lifecycle, and ROS parameters. No ROS types in core public headers.
8. **Supporting libraries are scoped:** configuration, testing, optimization and visualization must not leak into hot-path search dependencies.

## Status rules

- **Accepted:** decision explicitly agreed in design discussions and recorded in an ADR.
- **Proposed:** concrete API or algorithm detail that still needs design review.
- **Open:** a material tradeoff listed in [open questions](open-questions.md).
- Quantitative performance limits are **TBD** until representative maps, robots and hardware are agreed.

## Review process

Review requirements and architecture first, then interface semantics, algorithm guarantees, quality thresholds and verification. Close or explicitly defer every high-impact open question before implementing the corresponding module. Make significant changes via small ADRs; keep interfaces, tests, and checklists synchronized.
