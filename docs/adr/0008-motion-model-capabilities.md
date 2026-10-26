# ADR-0008: Model capabilities and admissible motion

**Status:** Accepted  
**Date:** 2026-10-24

## Context

Hybrid A* must support robots with different instantaneous mobility and motion constraints without branching its search implementation by vehicle class. A differential-drive robot can rotate in place and reach lateral poses through a sequence of maneuvers, but it cannot command an instantaneous sideways chassis velocity. Under the ideal planar chassis model it is **nonholonomic**. By contrast, an ideal mecanum chassis can command independent planar body-frame velocity components and yaw rate and is treated as **holonomic/omnidirectional** at the chassis level. Wheel-contact details, roller mechanics and slip may require a more detailed model; the chassis abstraction must declare its assumptions.

In-place rotation, holonomicity, global reachability/controllability, and orientation-dependent collision constraints are different properties. A single `isHolonomic()` switch does not determine feasible motions, state dimension or search strategy.

## Decision

1. Support **holonomic and nonholonomic chassis models** using the same generic state, control, propagation, primitive, indexing, collision and path contracts.
2. The model's **admissible control set, state propagation and constraints are authoritative**. No search, collision, path or heuristic branch may infer feasibility from the label “holonomic”, from a model name, or only from a set of booleans.
3. Optional **capability metadata** may describe lateral body translation, in-place rotation, reversible longitudinal translation, independent yaw control, and connector compatibility. Metadata is for selecting candidate primitive families, user-facing validation and optimizations; it is not a substitute for state-dependent validity checking. Capabilities need not be mutually exclusive or independently achievable at every configuration.
4. Model-provided primitive generators and analytic connectors determine feasible transitions. Steering functions (e.g., Dubins/Reeds–Shepp) are selected only when their mathematical assumptions are satisfied. The static obstacle heuristic is shared across models; cost-unit/admissibility requirements still apply.
5. Search indexing and collision checking use the **complete model configuration**, including heading and any steering or articulation states needed for correct dynamics, geometry or dominance. Holonomicity does not imply orientation can be removed from the search state.
6. The geometric path representation must preserve **translation direction independently of body heading**. A holonomic path may translate sideways with constant yaw or translate and rotate simultaneously. The path must not equate pose heading with tangent direction or require all moving segments to be only “forward/reverse”; in-place rotation and reversing cusps remain supported.
7. Maintain **three reference chassis models** for extensibility and contract testing:
   - Differential drive: controls such as longitudinal speed and yaw rate; no instantaneous lateral chassis velocity; rotation in place supported.
   - Geometric bicycle/Ackermann: steering-constrained translational motion; no in-place rotation or independent lateral translation.
   - Ideal mecanum/omnidirectional: body-frame longitudinal and lateral velocity plus yaw rate, subject to model-specific bounds.
   
   Differential drive and Ackermann remain the first implementation priorities. Mecanum is the third **reference/contract-test** model; its implementation milestone is planned but not yet completed.

## Reference kinematic equations (illustrative, not an API)

For differential drive, with world pose `(x, y, theta)`, controls `(v, omega)` and no lateral slip:

`x_dot = v cos(theta)`, `y_dot = v sin(theta)`, `theta_dot = omega`.

This satisfies `-sin(theta) x_dot + cos(theta) y_dot = 0`. Rotation in place (`v = 0`) is feasible but does not remove the nonholonomic no-side-slip constraint.

For an ideal mecanum chassis, with controls `(v_x^body, v_y^body, omega)`:

`x_dot = cos(theta) v_x^body - sin(theta) v_y^body`  
`y_dot = sin(theta) v_x^body + cos(theta) v_y^body`  
`theta_dot = omega`.

The planar chassis has three independently controllable instantaneous velocity components under these idealized assumptions. Wheel speed/acceleration saturation and slip constraints can restrict admissible commands even when the chassis is holonomic.

## Alternatives considered

- Binary `isHolonomic()` branching in search: loses in-place-rotation, steering and articulation distinctions.
- Reuse Ackermann forward/reverse-only segment assumptions for every robot: cannot express sideways mecanum travel correctly.
- A separate search engine per drive type: duplicates collision, cost, index and heuristic logic.
- Omit mecanum until after the search is implemented: risks hardcoding nonholonomic assumptions into first-generation interfaces.

## Consequences

- Path evaluation must expose full configuration and model-appropriate control/motion semantics independently of spatial tangent and yaw.
- Primitive generation and connector selection must be customizable without type-specific search branches.
- A rectangular holonomic robot may still need `SE(2)` search because collision depends on orientation. A simple 2D graph planner may be a better *algorithm choice* for a circular holonomic robot; a generic model abstraction does not force Hybrid A* for every problem.
- Contract and scenario tests should compare three model policies without altering the search kernel.

## Follow-up / unresolved detail

- Specify the precise optional capability descriptor and state-dependent constraints before implementing model selection (OQ-019).
- Decide the control/integration domain, path-progress parameter and differential/lateral travel cost model (OQ-002, OQ-003, OQ-004).
- Decide how the third model enters the build and which milestone requires full search integration (OQ-020).
- Test that sideways translation can have constant body yaw, and that collision checks preserve orientation for holonomic robots.
