# Open questions and architecture review gates

These items are intentionally **not** silently decided by API sketches. Mark a decision accepted in an ADR and update the related requirements/verification before implementing dependent modules.

## High priority — resolve before core interfaces

| ID | Question | Candidate choices / why it matters |
| --- | --- | --- |
| OQ-001 | What is the state/index dominance contract? | Pose bins alone vs model-specific extra bins / multiple labels; impacts reachability and correctness |
| OQ-002 | What does primitive extent measure? | Signed distance for moving segments, time for rotate-in-place, explicit domain wrapper; affects reconstruction and costs |
| OQ-003 | What is the path progress contract? | Segment-local normalized progress vs arc length + rotation angle; impacts MPC interpolation and in-place motions |
| OQ-004 | What is the planning cost in consistent units? | Length plus maneuver penalties, rotation cost, reversals, clearance; controls safe heuristic scaling |
| OQ-005 | Which obstacle heuristic mode and guarantees? | Grid reverse Dijkstra often overestimates true continuous distance; choose correction or explicitly nonadmissible mode |
| OQ-006 | What is map ownership and indexing policy? | Immutable owning snapshot, unknown cells, threshold, rotated origin, map revision; affects collision and caching |
| OQ-007 | What are actual initial Ackermann constraints? | Geometric bicycle with fixed steering primitives vs extended steering state/rate model |
| OQ-008 | How are endpoints specified and checked? | Full state vs pose-only goal tolerance; articulation/steering conditions may be required |

## Next priority — resolve with implementation milestones

| ID | Question | Candidate choices / why it matters |
| --- | --- | --- |
| OQ-009 | Collision assurance | Adaptive sampling only vs conservative swept bounds; hard guarantee requires more than discrete samples |
| OQ-010 | Public runtime polymorphism | Static concept-backed model solver + thin type erased facade vs runtime virtual model |
| OQ-011 | Steering cost/termination | How to cost analytic connectors and when is it safe to stop search? |
| OQ-012 | Replanning and cache concurrency | One active request per instance? Mutable lazy cache locking/copying? |
| OQ-013 | Refinement guarantee | Constraint-enforced spatial spline vs control-parameterized segments, and what `validated` means |
| OQ-014 | Geometry data and frames | FrameId registry and validation, compound convex bodies, self-collision policy |
| OQ-015 | Dependency versions and packaging | Eigen, Catch2, cxxopts, nlohmann/json, yaml-cpp, Ceres optional; vendoring vs package manager |
| OQ-016 | Exact performance target | Reference CPUs, maps, latency percentiles, memory budgets, allocation constraints |
| OQ-017 | ROS adapter boundary | Native path-to-message losslessness, metadata and explicit cusp/rotation support |
| OQ-018 | Thread/cancellation API | Cancellation token granularity, planner scratch ownership and safe shutdown |
| OQ-019 | Motion capabilities | How descriptive capability metadata complements (but never substitutes for) model-defined admissible controls ([ADR-0008](../adr/0008-motion-model-capabilities.md)) |
| OQ-020 | Mecanum milestone | When to implement the third reference model and whether it must complete end-to-end search for M4 |
| OQ-021 | Mathematical kinematic contract | Derivative/tangent/state-manifold operations, exact propagation customization, discrete modes and time vs progress conventions ([model spec](kinematic-models.md)) |
| OQ-022 | Supplied articulated-steering geometry and sign conventions | **Confirmed:** `(x,y,Theta)` is the rear-chassis reference pose. **Still open:** exact rear-reference-point location, whether inputs are rear-reference speed and articulation rate, positive sign of `gamma`, physical meaning of `l1`/`l2`, front/joint frame transforms and valid joint range ([model spec](kinematic-models.md)) |
| OQ-023 | Native occupancy/frame representation | REP-103/105 and grid layout are accepted (ADR-0009); still choose signed occupancy vs typed cell classification, application-specific encodings, threshold defaults and frame ID API ([map coordinates](map-coordinates.md)) |

## Review checklist for architecture baseline v0.1

- [ ] Cross-check requirements against intended supported vehicles and controller boundary.
- [ ] Agree which constraints are enforced by raw search and which by optional refinement/MPC.
- [ ] Choose or explicitly defer OQ-001 through OQ-008 with documented rationale.
- [ ] Confirm primitive and path parameter units, reversing cusp and rotation semantics.
- [ ] Confirm how heuristic certification and completeness/optimality claims are worded.
- [ ] Agree minimal compile-time policy concepts and public result/error types.
- [ ] Validate ODE/tangent/integrator contracts against differential drive, Ackermann and ideal mecanum (OQ-021).
- [ ] Ensure search and path interfaces never assume that body heading equals travel tangent.
- [ ] Confirm the supplied articulated-steering input and body-frame conventions (OQ-022) before implementing collision transforms.
- [ ] Confirm ROS-compatible map/grid Catch2 contracts (ADR-0009) and select native occupancy storage (OQ-023).
- [ ] Approve verification categories and first benchmark fixtures.
- [ ] Record decisions as accepted ADRs before implementation depends on them.
