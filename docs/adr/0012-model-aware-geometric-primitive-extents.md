# ADR-0012: Model-aware geometric motion-primitive extents

**Status:** Accepted (extent principle; exact API still proposed)  
**Date:** 2026-10-24

## Context

The planner generates geometric paths, not fully timed trajectories. For most translating primitives an explicit traveled length is more natural than an arbitrary fixed integration time: costs, collision sampling and search resolution can operate in meters. However, differential-drive rotation, stationary steering and articulation-only movement can have zero translational length but nonzero change in robot configuration. A universal arc-length parametrization is singular at zero translation speed.

## Decision

1. **Translating primitives** may use explicit traveled geometric length (meters) as their native extent, including forward, reverse and holonomic combined translation/rotation. The model's admissible geometric control law determines how its configuration evolves per unit extent.
2. **Stationary configuration-changing primitives** use an appropriate angular/configuration displacement (radians) or another model-specific geometric extent. Examples include in-place yaw rotation, steering changes, and articulation-only maneuvers. Such maneuvers cannot be treated as zero-cost or zero-progress edges.
3. Kinematic models continue to be defined by the controlled time-domain ODE `dq/dt = f(q,u;p)`, state/control constraints and body transforms. Propagation may use validated **spatial reparameterization**, exact analytic integration, or time-domain integration as needed. No universal division by the longitudinal speed at a stationary point.
4. The search core consumes **evaluable motions, costs and validated endpoints**; it does not interpret the model-dependent primitive extent or assume one unit for every model. Prefer compile-time extent types and model-specific policies over a mandatory runtime variant in every node.
5. Store the primitive's original geometric motion/control-law identity so reconstruction and validation reproduce the accepted motion exactly.
6. The resulting path retains the **canonical** `(segmentIndex, lambda)` address from [ADR-0011](0011-canonical-segment-path-position.md), independent of the primitive's original extent type.
7. Speed profiles and physical execution timing are a separate trajectory-planning responsibility. Length alone cannot uniquely determine duration without a speed profile. If physical joint or steering-rate constraints depend on timing, retiming must validate them, and some paths may prove dynamically infeasible.

## Alternatives considered

- Fixed nominal duration for all primitives: compatible with the original ODE, but arbitrarily ties geometric resolution to a speed setting.
- Signed translational distance for all motions: cannot parameterize a valid stationary rotation or joint change.
- A universal generalized weighted distance on full state: requires arbitrary weights and hides model-dependent motion semantics.
- Forcing every primitive to use one runtime variant: risks per-node hot-path overhead for models with statically known extent types.

## Consequences

A translating Ackermann arc can be parameterized in meters; a differential-drive in-place turn can be parameterized in radians; an articulated steering motion at stationary rear reference position can be parameterized by articulation angle. Motion-specific collision validation must bound the sweep of **every** attached body, including zero-translation maneuvers.

## Open technical details

- Exact `PrimitiveExtent` concept and whether one model contains distinct extent families in separate compile-time specializations.
- Which geometric control-law abstraction is appropriate for spatially parametrized steering/articulation changes.
- Handling transitions through zero translational speed and verifying rate/acceleration limits during retiming.
- Whether the planner exposes a cheap conservative speed-feasibility bound during geometric search (see speed-profile question in [open questions](../architecture/open-questions.md)).
