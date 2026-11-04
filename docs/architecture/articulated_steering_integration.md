# Articulated-steering integration gate and implementation plan

**Status (2026-11-04): mathematical model previously supplied; incomplete
physical geometry and actuation conventions require confirmation before
production planner integration. This is a plan, not a supported fourth
planner model.** The authoritative existing equation is also documented
in [kinematic models](kinematic-models.md); open assumptions are tracked
as [OQ-022](open-questions.md).

## Existing four-state model

The project author previously supplied the rear-chassis equation

```text
q = (x, y, Theta, gamma)
u = (v, u_gamma)
D(gamma) = l1*cos(gamma) + l2

dx/dt      = v*cos(Theta)
dy/dt      = v*sin(Theta)
dTheta/dt  = (v*sin(gamma) + l2*u_gamma) / D(gamma)
dgamma/dt  = u_gamma
```

**Confirmed from earlier author correspondence:** `(x,y,Theta)` is
the pose of a reference point on the **rear chassis**. The specific
location on that chassis remains unknown. `v` and `u_gamma`
are the current documentation's **assumed** longitudinal rear
reference-point speed and articulation angular rate. Neither the
meaning of `l1`/`l2` nor the sign of `gamma` can be confidently
inferred from the ODE alone.

A model with this ODE can change the rear heading while the rear
reference point is stationary: for `v=0` and `u_gamma!=0`,
`dTheta/dt=l2*u_gamma/D`. This is a **coupled articulation motion**,
not an independently controlled differential-drive spin.

## Required physical confirmation (OQ-022)

Please supply or confirm the following as a labeled top-view diagram
plus numbers/limits, or corrected equations:

1. **Rear reference origin:** rear axle midpoint, center of rear body,
   hinge/joint, or another explicitly located point? Which way is
   positive `Theta`?
2. **Front and rear frames:** joint position relative to the rear
   reference, front body reference/axle position relative to the joint,
   each rigid collision body's length, width and offset.
3. **Articulation definition:** whether `gamma = heading_front -
   heading_rear` or the reverse, its positive rotation direction
   and the zero-angle chassis alignment.
4. **Physical lengths:** precise geometric meaning and measurement
   directions of `l1` and `l2`, their allowed ranges, and the
   singularity-free joint range satisfying
   `abs(l1*cos(gamma)+l2) > epsilon`.
5. **Inputs and limits:** whether `v` is rear axle speed or another
   reference speed; whether `u_gamma` is commanded joint rate;
   maximum forward/reverse speeds, joint angle/rate limits and any
   steering/drive constraints or forbidden combinations.
6. **Start and goal state:** whether the initial `gamma` is sensed
   and compulsory, whether the desired final articulation is free,
   restricted or an exact joint-angle goal, and any permitted
   stationary articulation operation.

Once confirmed, record a small forward/reverse/joint-only ground-truth
motion fixture before enabling the model in the public planning UI.

## Implementation stages and acceptance checks

### Stage A — validated model, no planner integration yet

- Define explicit `ArticulatedState{x,y,theta,gamma}`,
  `ArticulatedControl{v,gammaRate}` and robot parameters with SI
  units. Keep them separate from `ackermann::Pose2D`.
- Implement the time-domain derivative with finite-value tests,
  denominator margin, valid joint range and control limits.
- Provide a deterministic integrator respecting angle topology,
  plus exact/independent oracle checks for `gamma=0`,
  constant joint angle, reverse controls, `v=0`, and near-denominator
  boundary rejection. Angle-only moves must not be divided by `v`.
- Confirm the correct frame transforms of BOTH bodies and the
  articulation pivot before binding to any collision geometry.

### Stage B — model-specific primitives, search and path

- Generate bounded, fully evaluable complete-state primitives with
  stored `(x,y,theta,gamma)` samples; preserve joint-velocity and
  mechanical-limit constraints over each integration interval.
- Extend model-specific index keys with articulation bins, while
  testing how single-label pruning affects alternate joint arrivals.
- Define goal semantics for `gamma` and complete-state path
  reconstruction. Standard rigid Ackermann RS arcs do not enforce
  this joint ODE and must not be silently reused as valid connectors.
- Add finite search budgets, cancellation, provenance, explicit
  unsupported states and diagnostics analogous to current native
  three-model behavior.

### Stage C — complete multi-body collision and validation

- Place the front and rear collision shapes from independent
  `SE(2)` model-frame transforms at each evaluated full state,
  including map boundaries, occupied/unknown-cell policy and
  user-selected geometric inflation.
- Test articulated body sweeps when `v=0`; a rear reference point
  remaining clear/stationary does **not** establish clearance of
  the moving body.
- Independently validate the complete reconstructed path at
  sufficiently dense state samples (or later certified swept
  checks), preserving the current honest distinction between
  sampled collision validation and continuous swept guarantees.
- Retain the original validated-path fallback; only enable
  articulation-aware refinement after the new model passes this
  full-state gate.

### Stage D — web UI and smoothing

- Expose an articulated model selector and `gamma` start/goal
  controls **only when supported by the C++ backend**.
- Draw both frame-attached collision bodies and pivot at the
  start/goal and during path replay, using the real transforms.
  The two-body UI must never reuse the single-rectangle
  Ackermann footprint as if it were an articulated body.
- Expand the later smoothing-quality benchmark with joint angle,
  joint-rate discontinuities, each body's sampled clearance,
  stop/cusp handling and tracking/velocity feasibility.

## Relation to the active roadmap

This is **P0-B** in [the current implementation roadmap](../TODO.md).
It comes **after** the smaller P0-A single-body start/goal footprint
preview and **before** the proposed cross-model smoothing-quality
benchmark. Prioritizing the model does not imply making it
available to real robot deployment before collision and control
assumptions have been verified.

The existing planner default remains the previously supported
Ackermann model; differential and mecanum planning are unchanged.
No GitHub Actions or automatic benchmark execution are needed.
