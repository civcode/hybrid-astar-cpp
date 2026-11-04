# Articulated-steering kinematic model and frame conventions — review draft

**Status: KINEMATIC MODEL + EXPERIMENTAL FOUR-DOF SEARCH IMPLEMENTED —
physical robot geometry/limits still require owner sign-off.**

*Rendering note:* Multi-line equations use GitHub-supported fenced `math` blocks.

**Date:** 2026-11-04  
**Scope:** two rigid carriages joined by one articulation joint; planar,
ideal no-side-slip kinematics. This document records the equation and
reference-frame choices for matching against the robot owner's notes.
The independent C++ kinematic model and reusable in-house RK4 are
now available in [`articulated_steering/model.hpp`](../../include/hybrid_astar/articulated_steering/model.hpp),
[`model.cpp`](../../src/articulated_steering/model.cpp) and
[`motion/rk4.hpp`](../../include/hybrid_astar/motion/rk4.hpp).
A separate experimental
[articulated planner API](../../include/hybrid_astar/articulated_steering/planner.hpp)
now provides gamma-aware search with two-body **sampled** collision checking.
It leaves the existing rigid planner's model adapters unchanged.
The local [Planning Lab](../../apps/demo/README.md#articulated-steering--experimental-planning-lab-mode)
now has an **experimental articulated model selector**, start/goal
joint angles, independent link/footprint configuration and two-body
replay. Neither that UI nor the underlying planner assigns
production body dimensions or certifies continuous collision, actuator
or control feasibility.

**Review priority:** Please check the **reference points and geometry**
in §2, the **control meanings and sign conventions** in §3, and the
**stationary steering behavior** in §7. Outstanding items are gathered
in the sign-off checklist in §11. Statements are tagged:

- **[FROM EQUATIONS]** Directly given or algebraically implied by the
  author's five-variable ODE.
- **[FROM ROS SIMULATOR]** Explicitly encoded in the author-supplied
  ROS 2 `OdeSimulation` source, including its stated example values.
- **[PROPOSED FOR PLANNER]** A naming/interface choice for review,
  *not* yet approved.
- **[TO CONFIRM]** A physical-robot, controller or safety detail that
  cannot be established from the equation or simulation code alone.

## 1. Symbols, units and accepted naming

| Symbol | Intended meaning | Unit | Evidence / status |
| --- | --- | --- | --- |
| $x_f,y_f$ | Position of the **front carriage reference point** $F$ in the world frame | m | [FROM EQUATIONS] front point; its exact physical axle location is [TO CONFIRM] |
| $\Theta_f$ | Front carriage heading | rad | [FROM EQUATIONS] |
| $x_r,y_r$ | Rear carriage reference position $R$, published as `base_link` in the supplied simulator | m | [FROM ROS SIMULATOR] |
| $\Theta_r$ | Rear carriage heading and `base_link` yaw | rad | [FROM ROS SIMULATOR] |
| $\gamma$ | Articulation angle; with aligned initial angles, $\gamma=\text{wrap}(\Theta_f-\Theta_r)$ | rad | [FROM EQUATIONS]; physical encoder sign and zero need [TO CONFIRM] |
| $L_f$ | Distance between the front reference point $F$ and the articulation joint $H$ | m | Frame formula [FROM ROS SIMULATOR]; physical endpoints [TO CONFIRM] |
| $L_r$ | Distance between the articulation joint $H$ and the rear reference point $R$ | m | Frame formula [FROM ROS SIMULATOR]; physical endpoints [TO CONFIRM] |
| $v$ | Signed longitudinal velocity **at $F$**, not at rear `base_link` | m/s | [FROM EQUATIONS] / [FROM ROS SIMULATOR] `/cmd_vel.linear.x` |
| $u_\gamma=\dot\gamma$ | Articulation angular-rate input | rad/s | [FROM EQUATIONS] / [FROM ROS SIMULATOR] `/cmd_vel.angular.z` |
| $v_r$ | **Derived**, signed longitudinal velocity of $R$ | m/s | Algebraic reduction, §6; it is **not** the input $v$ |
| $D(\gamma)$ | Kinematic denominator $L_f\cos\gamma+L_r$ | m | [FROM EQUATIONS]; must remain away from zero |

**Naming decision for this draft:** use $\boxed{L_f,L_r}$
throughout. The original sample ROS 2 source declares
`L1_=0.5`, `L2_=0.5`; those **literal source identifiers**
correspond to $L_f=0.5\,\mathrm m$ and $L_r=0.5\,\mathrm m$,
respectively. These numbers are **simulator examples**, not
automatically the robot's production dimensions.

## 2. Geometry and coordinate frames

Define $e(\theta)=(\cos\theta,\sin\theta)^T$, where heading zero
points along world $+x$. A conventional right-handed planar
world frame with $+y$ to the left and positive yaw counterclockwise
is **[PROPOSED FOR PLANNER]**, consistent with the standard ROS
planar convention. Confirm it against the robot's frames.

**Front-to-hinge-to-rear construction [FROM ROS SIMULATOR]:**

```math
\begin{aligned}
F&=(x_f,y_f)^T,\\
H&=F-L_f\,e(\Theta_f),\\
R&=H-L_r\,e(\Theta_r)
  =F-L_f e(\Theta_f)-L_r e(\Theta_r).
\end{aligned}
```

Here $L_f$ and $L_r$ are **nonnegative scalar distances**. The minus signs in the coordinate equations specify the displacement directions relative to the chosen heading axes; they are not signs or directional qualifiers on the lengths themselves.

This construction says that, when both carriages are aligned,
the points are ordered **rear reference $R$ → hinge $H$ →
front reference $F$** along their forward longitudinal axis.
For a nonzero articulation angle, the two link offsets have
**different headings**. The reverse transform is

```math
\begin{aligned}
H&=R+L_r\,e(\Theta_r),\\
F&=R+L_r\,e(\Theta_r)+L_f\,e(\Theta_f).
\end{aligned}
```

The corresponding planar body reference transforms are

```math
T^W_F=SE(2)(x_f,y_f,\Theta_f),\quad
T^W_R=SE(2)(x_r,y_r,\Theta_r).
```

**[FROM ROS SIMULATOR]** The published `odom -> base_link`
transform is $T^W_R$, i.e., the **rear** reference pose, even
though the ODE integrates $F$. The simulator computes:

```cpp
// Original source: L1_ -> L_f; L2_ -> L_r; theta1 -> Theta_f;
//                  theta2 -> Theta_r.
x_r = x_f - L_f * cos(Theta_f) - L_r * cos(Theta_r);
y_r = y_f - L_f * sin(Theta_f) - L_r * sin(Theta_r);
base_link_yaw = Theta_r;
```

**[TO CONFIRM]** Are $F$ and $R$ literally the **midpoints
of the front and rear axles**, and $H$ the physical articulation
pivot? The simulator's relative transforms are clear, but the
mechanical and collision-body origins are not independently
verified. In particular, $L_f,L_r$ alone do **not** describe
the full front/rear body dimensions, overhangs, wheels, or loads.

## 3. Articulation angle and control conventions

**[FROM EQUATIONS]** The coupled headings obey

```math
\dot\Theta_f-\dot\Theta_r=\dot\gamma.
```

Therefore $\Theta_f-\Theta_r-\gamma$ is a constant of motion.
When initialized with aligned chassis and $\gamma=0$, the
consistent convention is

```math
\boxed{\gamma=\text{wrap}(\Theta_f-\Theta_r)},\qquad
\Theta_f=\text{wrap}(\Theta_r+\gamma).
```

Here $\text{wrap}$ means equality of physical angles
modulo $2\pi$; use continuous/unwrapped angles internally where
necessary to avoid numerical jumps. **[TO CONFIRM]** Verify
that the physical `steering_joint` encoder uses this sign and
zero offset; the ODE and zero-initialized simulator support
this convention, but do not alone prove the hardware encoder
calibration.

**[FROM ROS SIMULATOR]** The two controls are mapped as follows:

```text
/cmd_vel.linear.x   -> v         (front-reference longitudinal speed)
/cmd_vel.angular.z  -> u_gamma   (joint articulation rate)
```

Importantly `angular.z` is **not** either carriage's yaw
rate $\dot\Theta_f$ or $\dot\Theta_r$. Those rates are
computed by the ODE. **[TO CONFIRM]** Are the real drive/
articulation actuators controlled using exactly these quantities,
and can they realize positive and negative $v$, positive and
negative $u_\gamma$, and joint motion at $v=0$?

## 4. Author-supplied five-variable ODE (normative draft)

**[FROM EQUATIONS]** With state
$\mathbf q_5=(x_f,y_f,\Theta_f,\Theta_r,\gamma)^T$,
controls $\mathbf u=(v,u_\gamma)^T$, and
$D(\gamma)=L_f\cos\gamma+L_r$:

```math
\frac{d}{dt}
\begin{bmatrix}
x_f \\ y_f \\ \Theta_f \\ \Theta_r \\ \gamma
\end{bmatrix}
=
\begin{bmatrix}
\cos\Theta_f & 0 \\
\sin\Theta_f & 0 \\
\dfrac{\sin\gamma}{L_f\cos\gamma+L_r} &
\dfrac{L_r}{L_f\cos\gamma+L_r} \\
\dfrac{\sin\gamma}{L_f\cos\gamma+L_r} &
-\dfrac{L_f\cos\gamma}{L_f\cos\gamma+L_r} \\
0 & 1
\end{bmatrix}
\begin{bmatrix}
v \\ \dot\gamma
\end{bmatrix}.
```

Scalar form:

```math
\begin{aligned}
\dot x_f&=v\cos\Theta_f,\\
\dot y_f&=v\sin\Theta_f,\\
\dot\Theta_f&=\frac{v\sin\gamma+L_r u_\gamma}{D(\gamma)},\\
\dot\Theta_r&=\frac{v\sin\gamma-L_f\cos\gamma\,u_\gamma}{D(\gamma)},\\
\dot\gamma&=u_\gamma.
\end{aligned}
```

The matrix above reproduces the author-supplied five-variable ODE using the agreed $L_f$ and $L_r$ names consistently. The two `L_1` occurrences in the message's typed matrix were interpreted as $L_f$, consistent with the previously supplied equations and simulator.

The equation is a **planar kinematic, no-side-slip** model;
it is not a dynamic force, wheel-slip, hydraulic steering, or
acceleration model. It does not specify joint stops, body shapes,
track widths, wheel radii, or physical speed/rate limits.

## 5. Four-independent-state reduction

Subtracting the two yaw equations gives exactly

```math
\dot\Theta_f-\dot\Theta_r
=\frac{(L_f\cos\gamma+L_r)u_\gamma}{D(\gamma)}
=u_\gamma=\dot\gamma.
```

Consequently, **when the initial headings are consistent**,
the five reported coordinates contain only four independent
configuration variables. The two equivalent choices are:

- **Front-based state**
  $\mathbf q_F=(x_f,y_f,\Theta_f,\gamma)$,
  with $\Theta_r=\text{wrap}(\Theta_f-\gamma)$.
- **Rear-based state**
  $\mathbf q_R=(x_r,y_r,\Theta_r,\gamma)$,
  with $\Theta_f=\text{wrap}(\Theta_r+\gamma)$.

**[PROPOSED FOR PLANNER]** Prefer the **rear-based**
$\mathbf q_R$ as the public pose/state reference, matching
the simulator's `base_link`; preserve both carriage
transforms and all full-state motion samples internally.
This is a representation decision **not yet approved**.
If the caller provides both chassis headings and articulation,
verify the identity (within an explicit tolerance) rather
than silently discarding inconsistent input values.

## 6. Equivalent rear-reference ODE

Differentiate the frame conversion in §2 and substitute
the original five-variable ODE. The rear reference then obeys

```math
\begin{aligned}
\dot x_r &= v_r\cos\Theta_r,\\
\dot y_r &= v_r\sin\Theta_r,\\
\dot\Theta_r&=
 \frac{v\sin\gamma-L_f\cos\gamma\,u_\gamma}{D(\gamma)},\\
\dot\gamma &= u_\gamma,
\end{aligned}
```

where the **derived rear longitudinal velocity** is

```math
\boxed{
v_r=
\frac{
v\bigl(L_f+L_r\cos\gamma\bigr)
+L_fL_r\sin\gamma\,u_\gamma
}{L_f\cos\gamma+L_r}.
}
```

This is the **same physical motion** as the front-based ODE,
not a new control model. $v$ continues to mean **front
reference speed**, while $v_r$ is computed from state and
controls. For aligned carriages $\gamma=0$, $v_r=v$.
For $\gamma\ne0$, **joint motion at $v=0$ can move
the rear reference point**. The rear velocity is longitudinal
under the no-side-slip geometry.

**Implementation caution:** If a controller instead supplies
the *rear* speed as its input, it requires an explicitly
transformed control law. Do not feed rear speed into the $v$
slot and assume the equations are unchanged.

## 7. Steering when commanded longitudinal speed is zero

The robot owner's desired behavior is that **both front and
rear carriages change orientation when articulating at $v=0$**,
not that only the joint angle changes. The supplied ODE already
encodes that behavior:

```math
\dot x_f=\dot y_f=0,\quad
\dot\Theta_f=\frac{L_r}{D(\gamma)}u_\gamma,\quad
\dot\Theta_r=-\frac{L_f\cos\gamma}{D(\gamma)}u_\gamma,\quad
\dot\gamma=u_\gamma.
```

The **front reference position stays fixed**. The **rear**
`base_link` position need not stay fixed; both rigid bodies
must be placed and collision checked at intermediate joint
configurations.

At **zero articulation**, the instantaneous yaw-rate shares are

```math
\frac{\dot\Theta_f}{u_\gamma}
=\frac{L_r}{L_f+L_r},\qquad
\frac{\dot\Theta_r}{u_\gamma}
=-\frac{L_f}{L_f+L_r}.
```

Thus for **equal lengths** $L_f=L_r$, they are **+50% and −50%**
initially, which matches the owner's intuitive starting split.
At a finite joint angle, however, the exact shares are

```math
\frac{\dot\Theta_f}{u_\gamma}
=\frac{L_r}{L_f\cos\gamma+L_r},\qquad
\frac{\dot\Theta_r}{u_\gamma}
=-\frac{L_f\cos\gamma}{L_f\cos\gamma+L_r}.
```

For the sample equal lengths:

| Articulation $\gamma$ | Front yaw-rate share | Rear yaw-rate share |
| ---: | ---: | ---: |
| 0° | +50.0% | −50.0% |
| 30° | +53.6% | −46.4% |
| 60° | +66.7% | −33.3% |

**[PROPOSED FOR PLANNER]** Use the **exact** rate split when
integrating collision-checked articulated primitives, including
zero-speed articulation. Treat a fixed half-angle split as a
small-angle intuition or optional prototype approximation—not
the model used for strict path validity. A fixed half split at
finite $\gamma$ would be a *different* kinematic assumption.

For clarity, define the mean heading
$\psi=(\Theta_f+\Theta_r)/2$. The representation

```math
\Theta_f=\psi+\gamma/2,\qquad
\Theta_r=\psi-\gamma/2
```

is an **identity**, but $\psi$ is not generally constant
during steering. At $v=0$:

```math
\dot\psi=
\frac{L_r-L_f\cos\gamma}{2D(\gamma)}u_\gamma.
```

For equal links, the 50/50 instantaneous split applies only
at $\gamma=0$. **Literal $D(\gamma)=0$ is a singularity,
not a steering mode.** The zero-speed scenario is $v=0$.

## 8. Geometry, map collision and actuator constraints

**[PROPOSED FOR PLANNER]**

- Kinematic parameters $L_f,L_r>0$ must be finite.
  Configure a physical joint range
  $[\gamma_{\min},\gamma_{\max}]$ and a minimum denominator
  margin $|D(\gamma)|\ge D_{\min}>0$ over **every
  generated/interpolated state**, not only endpoints. Actual
  allowed joint range and $D_{\min}$ remain [TO CONFIRM].
- Model controls need explicit signed driving-speed limits,
  joint-rate limits, and eventually acceleration/steering-rate
  feasibility. The supplied ODE only gives the ideal
  kinematic coupling and cannot validate actuator performance.
- Define separate immutable **front and rear full-body
  collision shapes**, including offsets, widths/overhangs
  and any protrusions; evaluate both world transforms for
  every sampled state and apply the same occupancy policy,
  map boundary treatment and optional geometric inflation.
  Do not reuse the present single-rectangle robot footprint.
- At $v=0$ with $u_\gamma\ne0$, still sample by **joint/body
  angular and body-point motion**, not by front-reference
  traveled distance alone: obstacle contact is possible even
  though $F$ does not translate.
- Do not automatically use three-state Ackermann Reeds–Shepp
  shortcuts as valid full articulated trajectories. Preserve
  complete-state reachability, finite work budgets and a
  separately checked final full-body path.

**[TO CONFIRM]** Physical front/rear collision hulls,
joint/cab/bucket overhang, nonzero joint encoder offset,
maximum reverse speed, joint steering at rest, self-collision
considerations and which part of the chassis is actively driven.

## 9. ROS simulator mapping and limitations

| ROS 2 implementation detail | Mapping or implication |
| --- | --- |
| `x_={0,0,0,0,0}` | Starts aligned with $\gamma=0$, consistent with the yaw invariant |
| `x_[0]`, `x_[1]`, `x_[2]`, `x_[3]`, `x_[4]` | $x_f,y_f,\Theta_f,\Theta_r,\gamma$ |
| `L1_=L2_=0.5` in the original source | Example $L_f=L_r=0.5$ m |
| `/cmd_vel.linear.x`, `/cmd_vel.angular.z` | $v$, $u_\gamma=\dot\gamma$, **not** chassis yaw rate |
| `/ode_pose.position.x/y` | Front reference $x_f,y_f$ |
| `/ode_pose.position.z=gamma` | Nonstandard reuse of the height field for a joint angle |
| `odom -> base_link` yaw `theta2` | Rear reference pose $(x_r,y_r,\Theta_r)$ |
| `integrate(..., 0.0, 0.01, 0.001)` in the command callback | Exactly 10 ms simulated per incoming message; not elapsed-time integration |
| `joint_state.velocity[0]=0` | Reported joint velocity is inconsistent with nonzero `gamma_dot_` |
| No `D` or joint-limit guard | Integration can become singular or leave a physical robot's valid joint range |

These observations are for **matching the notes**, not a request
to modify or depend on the simulator during this review.

## 10. Numerical examples for matching notes

Take the author's demonstration lengths
$L_f=L_r=0.5\,\mathrm m$, a positive commanded articulation
rate $u_\gamma=+0.2\,\mathrm{rad/s}$, and $v=0$.

| Initial $\gamma$ | $D$ (m) | $\dot\Theta_f$ (rad/s) | $\dot\Theta_r$ (rad/s) | $v_r$ (m/s) |
| ---: | ---: | ---: | ---: | ---: |
| 0° | 1.000000 | +0.100000 | −0.100000 | 0.000000 |
| 30° | 0.933013 | +0.107180 | −0.092820 | +0.026795 |
| 60° | 0.750000 | +0.133333 | −0.066667 | +0.057735 |

In each row,
$\dot\Theta_f-\dot\Theta_r=+0.2\;\mathrm{rad/s}$.
At zero articulation the rear reference is momentarily
stationary during the joint change, but at finite
articulation it can move even though the front reference
remains fixed. These are **instantaneous derivatives
at the shown initial states**, not the final angles or
displacements after an extended maneuver.

**Future consistency tests** (to run only after agreement):
compare front-based and rear-based integration; check the
angle invariant and frame transform at each sample;
exercise positive/negative $v$ and $u_\gamma$,
zero-speed joint articulation, joint-range boundaries,
and full-body obstacle contact at nontranslating front poses.

## 11. Owner review checklist — please compare with your notes

The following decisions are deliberately **open** until reviewed.
Mark each **Agree / Correct / Not yet defined** and annotate it:

- [ ] **Reference frames:** $(x_f,y_f)$ is the center of
  the **front axle** and the published rear `base_link`
  is at the center of the **rear axle**. If not, specify the
  actual points and offsets.
- [ ] **Link lengths:** $L_f$ is front reference-to-hinge,
  $L_r$ hinge-to-rear reference, each along its carriage
  longitudinal axis. Give the real robot's values in metres.
- [ ] **Angle signs:** $\Theta_f,\Theta_r$ are positive
  counterclockwise and $\gamma=\Theta_f-\Theta_r$;
  $\gamma=0$ means aligned carriages. Does the physical
  steering encoder have an offset or opposite sign?
- [ ] **Controls:** $v$ is signed **front-reference**
  speed and $u_\gamma$ is signed joint angle rate,
  as in the ROS simulator. Clarify which wheels are driven.
- [ ] **Stationary steering:** At $v=0$, the front
  reference stays fixed and both carriage headings follow
  the exact angle-dependent ODE; 50/50 is an initial
  equal-length zero-angle approximation only.
- [ ] **Planner reference:** Use rear
  $(x_r,y_r,\Theta_r,\gamma)$ as the public planning
  configuration, matching the simulator `base_link`.
- [ ] **Body geometry:** Supply each carriage's physical
  footprint and its offset from its reference frame,
  including joint and any protrusions.
- [ ] **Limits:** Supply permissible joint-angle interval,
  front-speed bounds, articulation-rate limits, and whether
  articulation-only motion is mechanically possible at rest.
- [ ] **Goal semantics:** Must the goal prescribe rear pose
  only, or rear pose **and** target $\gamma$?
  How is the initial $\gamma$ measured/provided?
- [ ] **Validation model:** Are no-slip assumptions acceptable
  for path planning, and are there relevant self-collision,
  steering-actuation, or load constraints to include?

**Implementation state:** The kinematic derivative, four-independent-variable
rear frame conversion, explicit limits and dependency-free RK4 are
implemented. Full-state search and separate front/rear occupancy collision checks are
now implemented experimentally, with independent finer-sampled final
validation. Robot-specific frame/body calibration, potential self-collision,
continuous swept checks, performance benchmarking and actuator feasibility
remain pending; this is not a certified real-robot motion planner.

**Review outcome (not yet decided):** After the owner reconciles
these conventions against their notes, freeze a versioned
model specification and only then implement the kinematic
primitive/search/body collision components. Do not change
the active planner merely to make a provisional convention work.

## Related documents

- [Mathematical kinematic model catalog](kinematic-models.md) —
  broader model context and original equations
- [Articulated-steering integration plan](articulated_steering_integration.md) —
  later implementation milestones, not this review decision
- [Active implementation roadmap](../TODO.md) —
  P0-B model milestone awaiting physical-model confirmation
- [Open questions: OQ-022](open-questions.md) —
  physical and control details requiring sign-off
