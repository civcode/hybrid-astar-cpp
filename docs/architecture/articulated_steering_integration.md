# Articulated-steering integration gate and implementation plan

**Status (2026-11-04): the author has supplied an explicit
FIVE-variable front/rear chassis ODE.** The documented earlier four-state
ODE was mathematically equivalent but **misidentified the front reference
pose as a rear reference pose**. This correction is captured in the
[canonical articulated kinematic specification](kinematic-models.md).
Physical axle/joint locations, collision-body geometry and operating
limits still need author confirmation before fourth-model integration.

This is a design/integration plan, **not a claim that articulated
search, full-body collision, or smoothing is implemented**.

## Canonical author-supplied five-variable model

```text
q = (xf, yf, Theta_f, Theta_r, gamma)
u = (v, u_gamma) where u_gamma = dgamma/dt
D(gamma) = L1*cos(gamma) + L2

dxf/dt      = v*cos(Theta_f)
dyf/dt      = v*sin(Theta_f)
dTheta_f/dt = (v*sin(gamma) + L2*u_gamma)/D(gamma)
dTheta_r/dt = (v*sin(gamma) - L1*cos(gamma)*u_gamma)/D(gamma)
dgamma/dt   = u_gamma
```

The heading derivatives imply
`d(Theta_f - Theta_r - gamma)/dt = 0`. **If the chassis
are aligned when `gamma=0` and start consistently**, then
`Theta_r = wrap(Theta_f-gamma)`; hence the system has **four
independent configuration variables**. The old four-state ODE is
the resulting reduced system for `(xf,yf,Theta_f,gamma)` and
`v` is the signed velocity of the **front reference point**.
The previously written "rear reference stationary at `v=0`"
assertion is **incorrect**. At zero speed the front reference
stays fixed, while both chassis can rotate with articulation.

The author has also supplied the **ROS 2 `OdeSimulation` code**.
It confirms the intended frame conversion in the actual simulator:
`x_[0:2]` hold the **front** reference pose with heading
`theta1=x_[2]`; `theta2=x_[3]` is the rear heading.
The transform `odom -> base_link` is published at
`x2=x-L1*cos(theta1)-L2*cos(theta2)`,
`y2=y-L1*sin(theta1)-L2*sin(theta2)`, yaw `theta2`.
Thus `base_link` is **rear-referenced**, whereas the ODE
integrates the **front** reference. The example has
`L1_=L2_=0.5 m`.

The code matches `H=F-L1*heading(Theta_f)` and
`R=H-L2*heading(Theta_r)`. This establishes the
**simulator's intended frame offsets**, not that the front/rear
reference points are necessarily the *physical* axle centers.
See [the full rear-reference kinematic reduction and simulator
limitations](kinematic-models.md#confirmed-by-author-supplied-ros-2-ode-simulator).

## Remaining physical confirmation (OQ-022)

1. The code **uses** a front reference `(xf,yf)`, a
   joint offset `L1` backward along `Theta_f`, and rear
   `base_link` a further `L2` backward along `Theta_r`.
   Please confirm whether these are **front and rear physical axle
   midpoints** and whether the example's `L1=L2=0.5 m`
   are the intended robot's real dimensions.
2. Is `gamma = wrap(Theta_f - Theta_r)` with **zero at aligned
   chassis**, and are positive headings counterclockwise in the
   world frame? Provide the mechanical joint sign and stop angles.
3. Confirm the front and rear body dimensions and offsets
   relative to `F` and `R`, and any wheel, cab, load or articulation
   geometry extending beyond those references. These are essential
   for two-body collision checking, not defined by the ODE.
4. Does input `v` correspond to **signed front-axle longitudinal
   velocity**, and is `u_gamma` directly the commanded articulation
   angular rate? Supply admissible speed/joint-rate limits, including
   allowed articulation-at-zero-speed behavior.
5. Are start `gamma` and goal `gamma` constrained or measured?
   A five-variable input must satisfy the heading-angle
   invariant; invalid inputs need explicit rejection.

Before using the ROS 2 code as a numerical oracle, also fix
`0.01 s` per `/cmd_vel` callback instead of elapsed-time integration,
the always-zero `joint_state.velocity` despite nonzero joint rate,
the overloaded `/ode_pose.position.z` field, and the missing
denominator/joint-limit guards. The planner can implement the
validated kinematic model independently without taking on ROS 2.

Once confirmed, capture the frame diagram and numerical
forward/reverse/joint-only motion test fixtures.

## Implementation stages and acceptance checks

### Stage A — validated model, no planner integration yet

- Define explicit `ArticulatedState{xf,yf,thetaFront,gamma}`,
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
  stored `(xf,yf,thetaFront,gamma)` samples; preserve joint-velocity and
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
- Test articulated body sweeps when `v=0`; a front reference point
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
