# Kinematic model contract — proposed v0.1

The mathematical definition is a **controlled system of differential equations**, not a class hierarchy. See [interfaces](interfaces.md), [ADR-0002](../adr/0002-generic-kinematic-models.md), [ADR-0008](../adr/0008-motion-model-capabilities.md) and [open questions](open-questions.md).

## Formal model

```math
\dot{\mathbf q}(t)=f(\mathbf q(t),\mathbf u(t);\mathbf p),\qquad
\mathbf q\in\mathcal Q_{\mathrm{valid}},\qquad
\mathbf u\in\mathcal U(\mathbf q;\mathbf p).
```

- **State** $\mathbf q$: all variables needed to predict motion and locate collision bodies. Initially mostly configuration variables; extended models may include internal rates.
- **Control** $\mathbf u$: independently commanded inputs under explicit idealizations.
- **Parameters** $\mathbf p$: fixed wheelbase, geometry, limits and model assumptions during a request.
- **Vector field** $f$: state derivative/tangent at a valid state/input.
- **Admissible state and control sets**: state/control constraints including coupled wheel-speed limits or steering bounds.
- **Geometric projections**: reference pose $P(\mathbf q)\in SE(2)$ and each named rigid-body frame transform $T_i(\mathbf q)\in SE(2)$. **Accepted ADR-0021:** immutable collision shapes and local offsets belong to a separate `RobotGeometry` description, not the ODE model; each world collision shape is placed by `T_world_frame(q) * T_frame_shape`.

State is not necessarily a Euclidean array: yaw and articulation angles have circular topology. Specify tangent and local state-update operations for numerical integration rather than assuming ordinary vector addition always works. If a model has discrete modes such as gear, express mode changes explicitly instead of differentiating an enum.

## Three planar reference models

### Differential drive

```math
\mathbf q=(x,y,\theta),\quad \mathbf u=(v,\omega),\quad
\dot{\mathbf q}=(v\cos\theta,\ v\sin\theta,\ \omega).
```

No-side-slip constraint:
```math
-\sin\theta\,\dot x+\cos\theta\,\dot y=0.
```

It is nonholonomic despite in-place rotation ($v=0,\omega\ne0$). Optionally relate chassis commands to wheel angular rates by $v=r(\omega_R+\omega_L)/2$ and $\omega=r(\omega_R-\omega_L)/b$ for wheel radius $r$ and track $b$.

### Geometric bicycle / Ackermann

```math
\mathbf q=(x,y,\theta),\quad \mathbf u=(v,\delta),\quad
\dot{\mathbf q}=(v\cos\theta,\ v\sin\theta,\ v\tan\delta/L).
```

**Accepted Ackermann baseline (ADR-0014):** use three-state $\mathbf q=(x,y,\theta)$ and a finite catalog of constant-curvature forward/reverse arcs, including straight and maximum left/right turns (optional intermediate curvatures). The fixed-curvature geometric model limits turning radius but does not guarantee steering-angle continuity or finite steering rate between primitives. For steering-rate constraints, introduce a distinct model with state $\mathbf q=(x,y,\theta,\delta)$ and control $\mathbf u=(v,\dot\delta)$, or separately validate an actual smoothing/steering connector. Dubins/Reeds–Shepp paths do not automatically respect finite steering rate.

### Ideal mecanum / omnidirectional chassis

```math
\mathbf q=(x,y,\theta),\quad \mathbf u=(v_x^b,v_y^b,\omega),
```
```math
\dot{\mathbf q}=\begin{bmatrix}
\cos\theta\,v_x^b-\sin\theta\,v_y^b\\
\sin\theta\,v_x^b+\cos\theta\,v_y^b\\
\omega
\end{bmatrix}.
```

The ideal chassis controls independent forward, lateral and yaw velocities, but wheel-speed saturation may couple input limits. This is a chassis-level holonomic abstraction, not an assertion about detailed roller-contact or slip behavior. Sideways travel at fixed yaw is allowed: **body heading is not travel tangent**.

### Author-supplied articulated-steering model — five reported states (2026-11-04)

The project author has now supplied the following **five-variable** system
explicitly identifying **front** and **rear** chassis headings. This
supersedes the older description that called the four-state `(x,y,Theta)`
model a **rear** reference pose: that earlier wording was incorrect for
the equations shown. The equations are now used by a **separate experimental articulated
C++ planner** with 4D state indexing and two-body sampled collision checks;
the existing rigid 3D planner/model selector remains unchanged.

Let $\mathbf q=(x_f,y_f,\Theta_f,\Theta_r,\gamma)$,
$u=(v,u_\gamma)$, $u_\gamma=\dot\gamma$,
and $D(\gamma)=L_f\cos\gamma+L_r$.

```math
\frac{d}{dt}
\begin{bmatrix}x_f\\y_f\\\Theta_f\\\Theta_r\\\gamma\end{bmatrix}
=
\begin{bmatrix}
\cos\Theta_f & 0\\
\sin\Theta_f & 0\\
\dfrac{\sin\gamma}{D(\gamma)} & \dfrac{L_r}{D(\gamma)}\\
\dfrac{\sin\gamma}{D(\gamma)} & -\dfrac{L_f\cos\gamma}{D(\gamma)}\\
0 & 1
\end{bmatrix}
\begin{bmatrix}v\\u_\gamma\end{bmatrix}.
```

Equivalently:

```math
\dot x_f=v\cos\Theta_f,\qquad
\dot y_f=v\sin\Theta_f,
```

```math
\dot\Theta_f=\frac{v\sin\gamma+L_ru_\gamma}{D(\gamma)},\qquad
\dot\Theta_r=\frac{v\sin\gamma-L_f\cos\gamma\,u_\gamma}{D(\gamma)},
\qquad
\dot\gamma=u_\gamma.
```

#### Exact relationship to the previous four-state model

Subtracting the two heading derivatives gives

```math
\dot\Theta_f-\dot\Theta_r
=\frac{(L_f\cos\gamma+L_r)u_\gamma}{D(\gamma)}
=u_\gamma=\dot\gamma.
```

Thus $\Theta_f-\Theta_r-\gamma$ is constant. **If** articulation
zero denotes aligned chassis and the initial state satisfies
$\gamma=\text{wrap}(\Theta_f-\Theta_r)$, then that
identity holds throughout a valid trajectory. The **five values
contain only four independent configuration degrees of freedom**.

Eliminating $\Theta_r$ yields the previously documented four-state
equations **exactly** on the constraint manifold, but with
$(x,y,\Theta)=(x_f,y_f,\Theta_f)$, **not** a rear reference pose:

```math
\dot x_f=v\cos\Theta_f,\quad
\dot y_f=v\sin\Theta_f,\quad
\dot\Theta_f=\frac{v\sin\gamma+L_ru_\gamma}{D(\gamma)},\quad
\dot\gamma=u_\gamma,\quad
\Theta_r=\text{wrap}(\Theta_f-\gamma).
```

For model-facing documentation and outputs, retaining the explicit
$\Theta_r$ is useful. For an eventual efficient planner state
and state index, use four independent variables
$(x_f,y_f,\Theta_f,\gamma)$ **only when** the angle identity and
physical conventions have been confirmed. Reject inconsistent
input states rather than silently discarding the rear heading.
Angle wrap must not introduce discontinuous physical integration.

#### Meaning of the variables

| Symbol | Meaning established by the new equations | Still to confirm physically |
| --- | --- | --- |
| $x_f,y_f$ | World position of a point fixed in the **front** chassis; its velocity is $v(\cos\Theta_f,\sin\Theta_f)$ | Whether this is the front axle midpoint or another front reference |
| $\Theta_f$ | Front chassis heading | World-heading origin/positive rotation convention |
| $\Theta_r$ | Rear chassis heading | Physical rear axle/body reference origin |
| $\gamma$ | On the aligned-zero state manifold, $\Theta_f-\Theta_r$ | Confirmation of zero-angle alignment and angle wrapping |
| $v$ | Signed longitudinal velocity of the **front reference point** | Wheel, driveline and control-speed limits |
| $u_\gamma=\dot\gamma$ | Articulation angular-rate control/input | Positive actuator direction, joint-angle and rate limits |
| $L_f,L_r$ | Positive geometric lengths in the kinematic coupling | Physical endpoints and exact collision-body offsets |

A **conventional no-side-slip axle geometry that reproduces these
equations**, but is **not yet author-confirmed**, is:

- $F=(x_f,y_f)$, the midpoint of the front axle, heading $\Theta_f$;
- articulation pivot $H=F-L_fe(\Theta_f)$;
- rear axle midpoint $R=H-L_re(\Theta_r)$;
- $e(\theta)=(\cos\theta,\sin\theta)$, so $L_f$ measures
  front-axle-to-hinge and $L_r$ hinge-to-rear-axle distance
  along the respective chassis axes.

With front no-slip $\dot F=v e(\Theta_f)$, rear no-slip
$\dot R\cdot(-\sin\Theta_r,\cos\Theta_r)=0$, and
$\dot\gamma=\dot\Theta_f-\dot\Theta_r$, differentiation yields
**both supplied heading equations**. This is a consistent
derivation, not proof that those frame origins are the intended
real robot geometry. The **front/rear collision-body extents,
axle offsets, joint frames, possible wheel slip and actuator
limits still require the robot owner's input** (OQ-022).

#### Confirmed by author-supplied ROS 2 ODE simulator

The author's `OdeSimulation` source implements **exactly** the five-state
ODE above with `x_[0]=x_f`, `x_[1]=y_f`,
`theta1=x_[2]=Theta_f`, `theta2=x_[3]=Theta_r`,
`x_[4]=gamma`, `v_linear_=v`, and
`gamma_dot_=u_gamma`. The supplied ROS 2 source names these parameters `L1_` and `L2_`; in the planner specification these correspond to `L_f=L_r=0.5 m`.
The `/cmd_vel` fields `linear.x` and `angular.z`
supply the two controls.

The simulator explicitly calculates the **rear base pose** from the
front-integrated state:

```math
x_r=x_f-L_f\cos\Theta_f-L_r\cos\Theta_r,\qquad
y_r=y_f-L_f\sin\Theta_f-L_r\sin\Theta_r.
```

It broadcasts `odom -> base_link` with translation
$(x_r,y_r,z=0.25)$ and yaw $\Theta_r$, **not** the integrated
front position. Its distinct `/ode_pose` message publishes
$(x_f,y_f)$, with `position.z=\gamma` as an
application-specific encoding, not a physical height.
The initial state is all zero, consistent with
$\gamma=\Theta_f-\Theta_r$.

**Therefore the user's interpretation matches the source code**:
$(x_f,y_f)$ is the front reference point. $L_f$ is the unsigned
distance between that point and the hinge; $L_r$ is the unsigned
distance between the hinge and the rear reference. The coordinate
transform subtracts the corresponding heading-aligned displacement
vectors to compute the rear `base_link` pose. The source **encodes** those geometric offsets but
does not independently prove that either point is the actual
physical **axle midpoint**; confirm against mechanical design/URDF
before attaching collision polygons.

#### Equivalent rear-reference planning equations

Differentiating that exact transformation yields

```math
\dot x_r=v_r\cos\Theta_r,\quad
\dot y_r=v_r\sin\Theta_r,\quad
v_r=\frac{v(L_f+L_r\cos\gamma)+
          L_fL_r\sin\gamma\,u_\gamma}
         {L_f\cos\gamma+L_r},
```

```math
\dot\Theta_r=
\frac{v\sin\gamma-L_f\cos\gamma\,u_\gamma}
{L_f\cos\gamma+L_r},\qquad
\dot\gamma=u_\gamma .
```

This rear-position, rear-heading, articulation state is a
**four-independent-variable alternative** to the front-based
reduction. The commanded $v$ continues to mean the **front
reference longitudinal speed**; it must not be silently
reinterpreted as $v_r$. At $v=0$, $v_r$ can still be
nonzero for nonstraight articulation with $u_\gamma\ne0$.
The resulting rear-reference motion has no sideways velocity
under the confirmed heading relation.

**Simulator limitations for validation:** It integrates exactly
`0.01 s` per `/cmd_vel` callback, regardless of actual
message spacing; `joint_state.velocity[0]` is always sent as 0
instead of `gamma_dot_`; `/ode_pose` does not set a
meaningful orientation quaternion; and it does not check
$|L_f\cos\gamma+L_r|$ against zero or enforce
mechanical joint bounds. In particular with the supplied source values `L1_=L2_=0.5`,
the denominator vanishes at $\gamma=\pi$.
These are **integration/telemetry and numerical guards** to
address before using this simulator as an independent physical
reference or deploying a fourth planner model.

#### Consequences for planning and motion

- At $v=0$, the **front reference stays stationary**, but
  $\dot\Theta_f=L_ru_\gamma/D$ and
  $\dot\Theta_r=-L_f\cos\gamma\,u_\gamma/D$; the front/rear
  rigid bodies can rotate and sweep obstacles. **Do not** describe
  this as a stationary **rear** reference point.
- With $u_\gamma=0$ and $\gamma\ne0$, both headings change
  at rate $v\sin\gamma/D$; with $\gamma=0$ and
  $u_\gamma=0$, the chassis translate straight.
- $D(\gamma)$ must remain bounded away from zero throughout
  the admitted joint range. Check finite and positive geometry,
  joint-angle/rate/speed bounds, and the initial
  $\Theta_f-\Theta_r-\gamma$ invariant.
- At $v\ne0$, the spatial turning relationship is
  $d\Theta_f/ds=\sin\gamma/D+L_r(u_\gamma/v)/D$
  for **signed front-point travel**
  $ds/dt=v$. Never divide by $v$ when modeling
  articulation-only motion or passing through zero speed.
- An articulated primitive must retain or reconstruct **both body
  frames**, and collision checking must evaluate them at every
  required full-state sample (including $v=0$). A single
  rectangle at the front or rear pose is insufficient.
- Standard three-state Dubins/Reeds–Shepp connections generally
  do **not** satisfy the extra joint dynamics, so are not
  automatically valid articulated connectors.
- The model is a driftless, controlled system; derivatives can
  be integrated in time (e.g. RK4). The reported five-variable
  form has the invariant above, so preserve/check it in tests.

**Illustrative five-variable derivative (not production-integrated):**

```cpp
struct ArticulatedState {
  double xf, yf, thetaFront, thetaRear, gamma;
};
struct ArticulatedControl { double frontSpeed, gammaRate; };

ArticulatedState derivative(const ArticulatedState& q,
                            const ArticulatedControl& u,
                            double L_f, double L_r) {
  const double D = L_f*std::cos(q.gamma) + L_r;
  // Validate physical parameters, |D| >= safety threshold, finite
  // inputs/states and joint/control limits before evaluating.
  const double driveYaw = u.frontSpeed*std::sin(q.gamma) / D;
  return {
    u.frontSpeed*std::cos(q.thetaFront),
    u.frontSpeed*std::sin(q.thetaFront),
    driveYaw + L_r*u.gammaRate/D,
    driveYaw - L_f*std::cos(q.gamma)*u.gammaRate/D,
    u.gammaRate
  };
}
```

**Next confirmation:** The author's simulator corroborates the
front-to-rear offsets and `base_link` at the rear reference,
using `L_f=L_r=0.5` as example geometry. Still confirm
physical **axle midpoint locations**, the precise front/rear
collision-body dimensions and offsets, mechanical joint-angle
and speed/rate limits, and start/goal articulation semantics
before enabling the fourth planner model.

See [articulated-steering implementation gate](articulated_steering_integration.md)
and [OQ-022](open-questions.md) for remaining physical requirements.

### General articulated extension

The model supplies $\mathbf q=(x,y,\theta,\phi_1,\ldots,\phi_n,\ldots)$, its coupled ODE and all frame transforms $T_i(\mathbf q)$. Search, collision and indexing must not assume three-dimensional state or fixed body offsets. **ADR-0021 accepts geometric support for tractor and arbitrarily many trailers** by associating immutable independent circles/rectangles/convex polygons with model-defined transforms; it does *not* imply that multi-trailer reversing or search performance is solved.

## Illustrative C++20 contract (not yet final API)

```cpp
template<class M>
concept KinematicModel = requires(
    const M& model,
    const typename M::State& q,
    const typename M::Control& u,
    double t,
    FrameId frame)
{
    typename M::State;
    typename M::Control;
    typename M::Tangent;
    { model.derivative(q, u, t) }
        -> std::same_as<typename M::Tangent>;
    { model.validState(q) } -> std::convertible_to<bool>;
    { model.validControl(q, u) } -> std::convertible_to<bool>;
    { model.pose(q) } -> std::same_as<Pose2D>;
    { model.frameTransform(q, frame) }
        -> std::same_as<Transform2D>;
};
```

*Semantics*: the derivative is deterministic and map-independent; control limits may depend on state. Derivatives should avoid allocation or parsing. **Accepted ADR-0023:** a separate RK4-based primitive generator/integrator uses the vector field primarily when *constructing* reusable or state-dependent primitives, records complete-state samples, and stops translating primitives at a specified traveled length using integrated progress and terminal-step refinement. The search reuses generated motions when valid rather than needing to call RK4 per node. Exact closed-form solutions can serve as independent oracles and optional later fast paths. The integrator must respect the state manifold, including angular wrap.

**Accepted model/geometry separation (ADR-0021):** a model publishes transforms of known rigid-body frames while a separate immutable robot-geometry configuration holds circle, rectangle or convex-polygon collision bodies and their local offsets. A primitive provider chooses admissible controls and integration extents, integrates with RK4 during primitive construction (ADR-0023), and stores ordered complete-state trajectory points for visualization/debugging. It is separate from the ODE; the search needs only the resulting primitive evaluation/instantiation interface. The state indexer uses the complete state required for reachability/dominance. The search engine never branches on `isHolonomic()` to decide what motion is allowed; the model's valid control set is authoritative.

## Time-domain dynamics versus path progress

Time is the canonical derivative variable for these ODEs because it works for in-place rotation and simultaneous holonomic translation/rotation. A primitive of nominal duration $\Delta t$, with constant input, may be evaluated over a local parameter $\sigma\in[0,1]$:

```math
\frac{d\mathbf q}{d\sigma}=\Delta t\,f(\mathbf q,\mathbf u).
```

Here $\sigma$ is **primitive-local progress**, not a global execution schedule. A downstream MPC still chooses timing, but any rate-constrained property verified at a specific nominal duration must be rechecked if the motion is retimed.

**Accepted OQ-002 (ADR-0012): spatial extent for translation, model-specific configuration extent for stationary maneuvers.** Translating primitives can be generated using an explicit traveled length and a spatially parameterized control; a constant nominal speed is not required. For example, bicycle curvature is $d\theta/ds=\tan\delta/L$ for **signed** traveled distance $ds/dt=v$. For the supplied articulated model with $D(\gamma)=L_f\cos\gamma+L_r$, the **front reference** speed $v\ne0$ gives:

```math
\frac{d\gamma}{ds}=\frac{u_\gamma}{v},\qquad
\frac{d\Theta_f}{ds}=\frac{\sin\gamma}{D(\gamma)}
+\frac{L_r}{D(\gamma)}\frac{u_\gamma}{v}.
```

The spatial control $d\gamma/ds$ may be chosen directly for translating primitives; a realizable articulation rate then depends on the eventual longitudinal speed. This division is **singular when $v=0$** even though the original time-domain dynamics remain valid. Consequently, zero-translation articulation changes and pure rotations use their own angular/configuration extent (with time-domain integration available when appropriate). Geometric length alone cannot specify execution duration; absent a speed profile there is no unique $dt$. The accepted primitive design is recorded in [ADR-0012](../adr/0012-model-aware-geometric-primitive-extents.md).

**Accepted OQ-003:** the public path's *canonical address* is $(\text{segmentIndex},\lambda)$, $\lambda\in[0,1]$, not either time or arclength ([ADR-0011](../adr/0011-canonical-segment-path-position.md)). A translating segment may internally use length; a stationary rotation segment may use angle; both evaluate using the same normalized local $\lambda$.

The output path retains full model state, segmented progress, maneuver semantics, optional derivatives, and the map/model validity context; no universal forward/reverse-only label or curvature-by-arc-length requirement is imposed.

## Verification obligations

- [ ] Analytic derivative/propagation comparisons for all three reference models.
- [ ] Differential in-place rotation, no instantaneous lateral control.
- [ ] Mecanum lateral translation with constant yaw, and simultaneous translation and rotation.
- [ ] Mecanum coupled actuator bounds when modeled.
- [ ] Extended bicycle steering state and rate-limit checks.
- [ ] Pose/frame covariance and articulated body transforms.
- [ ] For the supplied four-state articulated model, verify derivatives at zero articulation, coupled heading/articulation changes at zero speed, denominator guard, and integration against independent numerical checks.
- [ ] Validate equivalent configurations with distinct articulation angles do not collapse to one search label.
- [ ] Validate articulation-only motions sweep full robot geometry despite zero reference-point translation.
- [ ] Angle wrap and state/tangent integration tests.
- [ ] Primitive trajectory vs path evaluation equivalence and swept-body validation.
- [ ] Benchmark exact vs numerical propagation and allocations.

## Unresolved details

OQ-002 (exact extent API) is still a design detail, but its **geometric parameterization principle is accepted**; OQ-003 (path address) is accepted. The basic kinematic-to-independent-multibody-geometry interface is accepted (ADR-0021), while OQ-021 retains manifold/integrator signatures and OQ-014 retains frame registry/self-collision details. Open issues include OQ-004 (cost units), OQ-019 (capability metadata), OQ-020 (mecanum milestone), OQ-021 (state/tangent integration customization), OQ-022 (articulated geometry/control conventions), and OQ-024 (speed-profile and retiming boundary). OQ-007's **three-state fixed-curvature Ackermann baseline is accepted** (ADR-0014); a four-state steering-rate extension remains optional.
