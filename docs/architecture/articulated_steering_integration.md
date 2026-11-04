# Articulated-steering integration gate and implementation plan

**Status (2026-11-04): the author has supplied an explicit
FIVE-variable front/rear chassis ODE.** The documented earlier four-state
ODE was mathematically equivalent but **misidentified the front reference
pose as a rear reference pose**. This correction is captured in the
[canonical articulated kinematic specification](kinematic-models.md).
Physical axle/joint locations, collision-body geometry and operating
limits still need author confirmation before fourth-model integration.

**Read and sign off the [dedicated kinematic model and frame-conventions review draft](articulated_steering_kinematic_model_review.md) first.** It is the owner-facing review artifact covering the exact ODE, front/rear frames, (L_f/L_r), stationary steering and explicit undecided conventions. Do not implement this plan before the owner checks those assumptions against their notes.

This is a design/integration plan, **not a claim that articulated
search, full-body collision, or smoothing is implemented**.

## Canonical author-supplied five-variable model

```text
q = (xf, yf, Theta_f, Theta_r, gamma)
u = (v, u_gamma) where u_gamma = dgamma/dt
D(gamma) = L_f*cos(gamma) + L_r

dxf/dt      = v*cos(Theta_f)
dyf/dt      = v*sin(Theta_f)
dTheta_f/dt = (v*sin(gamma) + L_r*u_gamma)/D(gamma)
dTheta_r/dt = (v*sin(gamma) - L_f*cos(gamma)*u_gamma)/D(gamma)
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
The transform `odom -> base_link` is published at the rear
reference pose, expressed in our notation as
`x_r=x_f-L_f*cos(Theta_f)-L_r*cos(Theta_r)`,
`y_r=y_f-L_f*sin(Theta_f)-L_r*sin(Theta_r)`, yaw `Theta_r`.
Thus `base_link` is **rear-referenced**, whereas the ODE
integrates the **front** reference. The supplied ROS 2 simulator uses the original source identifiers
`L1_=L2_=0.5 m`; our geometric names are now `L_f=L_r=0.5 m`.

The code matches `H=F-L_f*heading(Theta_f)` and
`R=H-L_r*heading(Theta_r)`. This establishes the
**simulator's intended frame offsets**, not that the front/rear
reference points are necessarily the *physical* axle centers.
See [the full rear-reference kinematic reduction and simulator
limitations](kinematic-models.md#confirmed-by-author-supplied-ros-2-ode-simulator).

## Remaining physical confirmation (OQ-022)

1. The code **uses** front reference `(xf,yf)`, with
   `L_f` as the scalar distance from front reference to joint
   and `L_r` as the scalar distance from joint to rear reference.
   Minus signs in the frame conversion specify the direction
   of coordinate displacement vectors, not the distances.
   Please confirm whether these are **front and rear physical axle
   midpoints** and whether the example's `L_f=L_r=0.5 m`
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

## Zero-longitudinal-speed articulation: both bodies rotate

The desired behavior from the robot owner is **joint steering at zero
commanded longitudinal speed**, updating the orientation and swept
collision geometry of **both** front and rear carriages. In the ROS 2
simulator, the ODE control `v=0` means the **front reference point**
does not translate; it does not mean the rear `base_link` point or
either body stays motionless.

For `v=0`, the authoritative ODE gives

$$
\dot\Theta_f=\frac{L_r}{D(\gamma)}u_\gamma,\qquad
\dot\Theta_r=-\frac{L_f\cos\gamma}{D(\gamma)}u_\gamma,\qquad
D(\gamma)=L_f\cos\gamma+L_r.
$$

These add up to the required
$\dot\Theta_f-\dot\Theta_r=u_\gamma=\dot\gamma$.
For `L_f=L_r` and $\gamma=0$, the front and rear initially
turn by **equal and opposite half angles**:
$\dot\Theta_f=\frac12u_\gamma$,
$\dot\Theta_r=-\frac12u_\gamma$.

**Recommended design:** Use the equal split as a convenient initial
intuition or small-angle approximation; **generate planner motion
samples with the exact angle-dependent ODE**. At finite articulation
the split is not constant. For the simulator's `L_f=L_r=0.5m`,
the front/rear shares of $\dot\gamma$ are 50%/−50% at
0°, about 53.6%/−46.4% at 30°, and 66.7%/−33.3%
at 60°. A globally fixed 50/50 split would violate the
author-supplied no-slip kinematics for finite joint angles.

To distinguish *orientation* from *physical rotation of the bodies*,
define the mean heading $\psi=(\Theta_f+\Theta_r)/2$. Then
$\Theta_f=\psi+\gamma/2$ and
$\Theta_r=\psi-\gamma/2$ are **exact identities** for consistent
states, but at `v=0` the mean heading itself generally evolves:

$$
\dot\psi=\frac{L_r-L_f\cos\gamma}{2D(\gamma)}u_\gamma.
$$

Holding $\psi$ constant while commanding a large joint change
would be a **different motion model**. It would require new
physical control semantics and validation of front/rear axle
translations rather than silently replacing the existing ODE.

**Regression requirement:** For both positive and negative
`u_gamma` (and after reverse traversal where relevant), the
four-state integrator must reproduce the five-state ODE,
preserve $\Theta_f-\Theta_r-\gamma$ after angle wrap,
reconstruct the rear-base position, and collision-check **both
carriages at every sampled articulation state**, including when
`v=0`. Reject invalid joint ranges or near-zero `D`;
no differential-drive `RotateInPlace` shortcut may be substituted
for the articulated movement.

**Terminology warning:** The simulator uses the name `c` for
the denominator; in our notation `c=L_f*cos(gamma)+L_r`. The steering mode discussed above
is `v=0`, **never** literal `c=0`, which makes
the ODE undefined.

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
