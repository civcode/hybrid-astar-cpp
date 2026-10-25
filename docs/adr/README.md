# Architecture Decision Records (ADRs)

Short records for consequential decisions. Keep the original rationale, add superseding ADRs when reversing a decision, and synchronize requirements/interfaces when an ADR affects a contract.

| ADR | Status | Decision |
| --- | --- | --- |
| [0001](0001-ros-independent-core.md) | Accepted | Standalone C++20 core and external ROS 2 adapters |
| [0002](0002-generic-kinematic-models.md) | Accepted | Generic kinematics and model-aware collision geometry |
| [0003](0003-obstacle-aware-heuristic.md) | Accepted | Static reverse-Dijkstra obstacle heuristic |
| [0004](0004-native-steering-functions.md) | Accepted | Native Dubins/Reeds–Shepp, OMPL only for tests |
| [0005](0005-piecewise-geometric-path.md) | Accepted | Piecewise continuously evaluable model-feasible output |
| [0006](0006-dynamic-planning-boundary.md) | Accepted | Dynamic obstacle handling and MPC separate |
| [0007](0007-dependencies-verification.md) | Accepted | Supporting libraries scoped; contract-driven tests |
| [0008](0008-motion-model-capabilities.md) | Accepted | Admissible-controls-first holonomic/nonholonomic models and mecanum reference |
| [0009](0009-ros-map-coordinate-conventions.md) | Accepted | ROS coordinate frames and map grids |
| [0010](0010-compile-time-search-label-policy.md) | Superseded by 0013 | Previously proposed optional multi-label policy |
| [0011](0011-canonical-segment-path-position.md) | Accepted | Canonical path addresses use segment index and local normalized parameter |
| [0012](0012-model-aware-geometric-primitive-extents.md) | Accepted | Geometric length for translation; angular/configuration extent for stationary motions |
| [0013](0013-single-label-search-baseline.md) | Accepted | Simple single-node-per-discrete-key search; defer multi-label complexity |
| [0014](0014-fixed-curvature-ackermann-baseline.md) | Accepted | Fixed-curvature 3-state geometric bicycle / Ackermann |
| [0015](0015-admissible-default-heuristics.md) | Accepted | Certified lower-bound heuristic default; explicit opt-in alternatives |
| [0016](0016-default-pose-goal-tolerances.md) | Accepted | Continuous-pose goal with XY and wrapped-yaw tolerances |
| [0017](0017-explicit-unknown-occupancy-policy.md) | Accepted | Per-request unknown-cell policy, conservative frontier default; OQ-006 map details remain open |
| [0018](0018-known-free-only-occupancy-threshold.md) | Accepted | Configurable maximum traversable known-cell occupancy, default zero |
| [0019](0019-live-map-snapshot-cache-and-reject.md) | Accepted | Immutable shared map publication, on-demand versioned caches and strict full-map unknown rejection |
| [0020](0020-evidence-driven-mvp-simplicity.md) | Accepted | Thorough contracts, lean first implementation; add complexity only for measured need |
| [0021](0021-independent-multibody-collision-geometry.md) | Accepted | Independent frame-attached multibody collision geometry, support for trailer chains without a scene graph |
| [0022](0022-opt-in-adaptive-collision-subdivision.md) | Accepted | Whole-shape occupancy-cell intersection; fixed sampling default, adaptive subdivision opt-in |
| [0023](0023-rk4-arc-length-primitives-and-samples.md) | Accepted | RK4-generated length-terminated primitives with full-state trajectory checkpoints for debugging/visualization |
| [0024](0024-obstacle-aware-path-optimization.md) | Accepted | Obstacle awareness during refinement, pinned map/full-body geometry and independent final validation |
| [0025](0025-approved-ackermann-mvp-settings-and-primitive-sample-collision.md) | Accepted | Approved Ackermann numerical/search/cost settings, primitive-sample collision checks and adaptive-off default |

**Note:** Accepted ADRs record architectural direction. The precise ABI, model contracts, heuristic guarantees, numerical methods and performance targets remain *proposed/open* until review (see [open questions](../architecture/open-questions.md)).

## Template

```markdown
# ADR-NNNN: Title
Status: Proposed | Accepted | Superseded
Date: YYYY-MM-DD

## Context
## Decision
## Alternatives considered
## Consequences
## Follow-up / unresolved detail
```
