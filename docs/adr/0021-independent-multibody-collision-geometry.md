# ADR-0021: Independent frame-attached multibody collision geometry

**Status:** Accepted (minimal geometry/kinematics boundary; OQ-009, OQ-014 and OQ-021 retain implementation details)  
**Date:** 2026-10-25  
**Related:** [ADR-0002](0002-generic-kinematic-models.md), [ADR-0020](0020-evidence-driven-mvp-simplicity.md)

## Context

The motion-planning model must support rigid mobile robots now and articulated/multi-trailer chassis later without coupling a vehicle's differential equations to its particular collision footprint. A single footprint attached to the reference pose is insufficient: different trailers or chassis sections can have different orientations, local shape offsets and swept motion even when the reference point does not translate.

## Decision

1. Keep **kinematic behavior** and **collision geometry** as independent immutable descriptions. The model defines `State`, `Control`, tangent/derivative, validity constraints, a reference pose and state-dependent world transforms for model-defined named body frames. The map and collision-shape algorithms do not belong to the model.
2. The geometry is an immutable **collection of collision bodies**. Every body contains a validated model-frame identifier, local shape offset and one primitive shape. Initial 2D shape types: **circle, rectangle, convex polygon**. Nonconvex or compound footprints are represented by multiple convex bodies, not a dedicated concave-polygon algorithm. No meshes or 3D physics in the MVP.
3. World geometry is computed via `T_world_shape(q) = T_world_frame(q) * T_frame_shape`. The model must return valid transforms for **all referenced frames at each queried state**. Geometry is configured once per robot, not reconstructed/copied per search node. Named frames may be model-defined indices rather than runtime strings in hot paths; exact `FrameId` API remains open (OQ-014/OQ-023).
4. This accommodates tractor plus **any finite number of trailers geometrically**, provided the model's state includes the relevant joint configurations, its ODE supplies correct propagation, and it evaluates all trailer frame transforms and admissible joint limits. For a two-trailer example, a model might use `q=(x,y,theta_tractor,alpha_1,alpha_2)`. **No claim of efficient or complete multi-trailer search follows from geometric support.** Multi-trailer path planning and reversing/jackknife handling are beyond initial MVP validation.
5. Model-independent **motion primitive provider**, **numerical/exact propagator**, **state indexer**, and **collision checker** remain separate contracts. The propagator must preserve/evaluate full state along a primitive, not merely interpolate the endpoints, so collision can query every body transform at intermediate configurations. State indexers include necessary model-specific joint dimensions; baseline retains one active arrival per discrete key (ADR-0013).
6. **Self-collision policy** remains an explicit OQ-014 decision: which nonadjacent bodies are checked, exceptions for intentional hitch/adjacent-body overlaps, and the assurance that can be claimed. The MVP must not silently claim self-collision has been validated if it has not. The shape-vs-map swept-motion assurance method itself remains **OQ-009**.
7. The minimal kinematic interface direction is accepted (state, control, tangent/derivative, validity, reference pose, frame transforms); exact state-manifold update, propagation/customization, typed control/extent API and error behavior remain **OQ-021/OQ-002**, not prematurely frozen C++ concepts.

## Illustrative independent shape description (not ABI)

```cpp
using CollisionShape = std::variant<Circle, Rectangle, ConvexPolygon>;
struct CollisionBody {
    FrameId frame;
    Pose2D localOffset;
    CollisionShape shape;
};
struct RobotGeometry {
    std::vector<CollisionBody> bodies;
};
```

The model provides `frameTransform(q, frame)`; the collision checker uses `T_world_frame(q) * T_frame_shape` to evaluate each body.

## Verification and limits

- Differential/Ackermann fixtures: one frame with several shapes; circles, rotated rectangles and convex polygons, including compound/nonconvex unions.
- Multi-frame articulated fixture: validate independently transformed bodies across joint changes with zero reference-point translation; trailers in different poses cause different collisions.
- Validate references to unknown frames, malformed polygons, degenerate shapes and invalid state/control limits.
- Document exactly whether map collision and/or pairwise self-collision has been checked; do not claim continuous swept collision guarantees from discrete samples.
- Keep the initial implementation lean: no public scene graph, runtime physics engine or speculative trailer-specific planner branches.
