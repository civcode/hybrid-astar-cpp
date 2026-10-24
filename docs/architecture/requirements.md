# Requirements and constraints — draft v0.1

IDs are stable references for design discussions and tests. **Status: accepted direction, detailed contract proposed unless otherwise stated.** A requirement is fulfilled only when implemented and verified, not when documented.

## Functional requirements

| ID | Requirement | Initial scope / acceptance evidence |
| --- | --- | --- |
| FR-001 | Generic kinematic models | Search compiles against differential-drive and Ackermann model policies without model-specific branches |
| FR-002 | Configurable motion primitives | Forward/reverse where supported, in-place rotation for differential drive; full primitive trajectory preserved |
| FR-003 | Model-aware state indexing | Index includes all state dimensions needed for correct future reachability and dominance |
| FR-004 | Collision geometry | Multiple frame-attached rigid 2D bodies; articulated poses derived from hypothetical state |
| FR-005 | Swept-motion validation | Every accepted transition validated along complete motion, including rotating/articulating bodies |
| FR-006 | Static occupancy environment | Immutable map snapshot with resolution, frame, origin, version, bounds and unknown-cell policy |
| FR-007 | Obstacle-aware heuristic | 2D reverse shortest-distance field from goal, with defined cache invalidation and optional lazy evaluation |
| FR-008 | Model-specific heuristic/connector | Native Dubins and Reeds–Shepp where model assumptions apply; optional analytic goal connections |
| FR-009 | Configurable search semantics | Open/closed handling, configurable costs, cancellation and bounded resource use |
| FR-010 | Exact path reconstruction | Ordered, evaluable motion segments including gear changes and in-place rotations |
| FR-011 | Optional refinement | Model-aware path improvement; validate and fall back to original feasible path on failure |
| FR-012 | Controller-facing output | Piecewise geometric path; endpoint conditions, progress, directional and configuration information |
| FR-013 | External configuration | Typed native config; optional YAML and JSON adapters with validation; SI units, radians |
| FR-014 | Optional ROS 2 integration | Isolated adapter(s), including Nav2 option; core does not depend on ROS |
| FR-015 | Deterministic diagnostics | Failure reason, map/config versions, counters, timings, and reproducible test inputs |

## Architectural constraints

| ID | Rule |
| --- | --- |
| AC-001 | Core and tests build outside ROS 2; ROS types cannot occur in core public API |
| AC-002 | Generic search hot path avoids mandatory virtual dispatch, avoidable heap allocations and format parsing |
| AC-003 | Kinematic state and search key are separate; no universal `(x,y,heading)` key assumption |
| AC-004 | Collision depends on entire model state, not only current physical robot footprint |
| AC-005 | Raw feasible path must remain accessible if optional refinement fails |
| AC-006 | No OMPL, civcode demo-repository source, or external motion-planning implementation incorporated into runtime core |
| AC-007 | Separate dynamic trajectory planning/MPC handles predicted moving obstacles and timing |
| AC-008 | Any future GitHub Actions workflows are manual-only unless explicitly authorized |

## Non-goals (initial milestones)

- No time-expanded Hybrid A*, multi-agent reservation planning, or dynamic-obstacle prediction in the library.
- No MPC, velocity profile optimizer, ROS 2 node, or Nav2 plugin in the initial core milestone.
- No claim of global optimality, completeness, continuous collision certification, or real-time deadlines until assumptions and tests justify it.
- No requirement to use B-splines (or any particular curve family) at the controller boundary.
- No support for arbitrary 3D mesh collision in v0.1; plan a future extension point.

## Traceability

Acceptance tests and thresholds are defined in [verification.md](verification.md) and [quality.md](quality.md). Use stable requirement IDs in tests / benchmark manifests and PR descriptions. Open questions are tracked in [open-questions.md](open-questions.md).
