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
