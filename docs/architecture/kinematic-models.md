# Kinematic model contract — proposed v0.1

The mathematical definition is a **controlled system of differential equations**, not a class hierarchy. See [interfaces](interfaces.md), [ADR-0002](../adr/0002-generic-kinematic-models.md), [ADR-0008](../adr/0008-motion-model-capabilities.md) and [open questions](open-questions.md).

## Formal model

\[
\dot{\mathbf q}(t)=f(\mathbf q(t),\mathbf u(t);\mathbf p),\qquad
\mathbf q\in\mathcal Q_{\mathrm{valid}},\qquad
\mathbf u\in\mathcal U(\mathbf q;\mathbf p).
\]

- **State** \(\mathbf q\): all variables needed to predict motion and locate collision bodies. Initially mostly configuration variables; extended models may include internal rates.
- **Control** \(\mathbf u\): independently commanded inputs under explicit idealizations.
- **Parameters** \(\mathbf p\): fixed wheelbase, geometry, limits and model assumptions during a request.
- **Vector field** \(f\): state derivative/tangent at a valid state/input.
- **Admissible state and control sets**: state/control constraints including coupled wheel-speed limits or steering bounds.
- **Geometric projections**: reference pose \(P(\mathbf q)\in SE(2)\) and each body transform \(T_i(\mathbf q)\in SE(2)\) for collision.

State is not necessarily a Euclidean array: yaw and articulation angles have circular topology. Specify tangent and local state-update operations for numerical integration rather than assuming ordinary vector addition always works. If a model has discrete modes such as gear, express mode changes explicitly instead of differentiating an enum.

## Three planar reference models

### Differential drive

\[
\mathbf q=(x,y,\theta),\quad \mathbf u=(v,\omega),\quad
\dot{\mathbf q}=(v\cos\theta,\ v\sin\theta,\ \omega).
\]

No-side-slip constraint:
\[
-\sin\theta\,\dot x+\cos\theta\,\dot y=0.
\]

It is nonholonomic despite in-place rotation (\(v=0,\omega\ne0\)). Optionally relate chassis commands to wheel angular rates by \(v=r(\omega_R+\omega_L)/2\) and \(\omega=r(\omega_R-\omega_L)/b\) for wheel radius \(r\) and track \(b\).

### Geometric bicycle / Ackermann

\[
\mathbf q=(x,y,\theta),\quad \mathbf u=(v,\delta),\quad
\dot{\mathbf q}=(v\cos\theta,\ v\sin\theta,\ v\tan\delta/L).
\]

For steering-rate constraints, extend state to \(\mathbf q=(x,y,\theta,\delta)\) and control to \(\mathbf u=(v,\dot\delta)\). This is a *different model* for indexing and analytic-connector assumptions; Dubins/Reeds–Shepp paths do not automatically respect finite steering rate.

### Ideal mecanum / omnidirectional chassis

\[
\mathbf q=(x,y,\theta),\quad \mathbf u=(v_x^b,v_y^b,\omega),
\]
\[
\dot{\mathbf q}=\begin{bmatrix}
\cos\theta\,v_x^b-\sin\theta\,v_y^b\\
\sin\theta\,v_x^b+\cos\theta\,v_y^b\\
\omega
\end{bmatrix}.
\]

The ideal chassis controls independent forward, lateral and yaw velocities, but wheel-speed saturation may couple input limits. This is a chassis-level holonomic abstraction, not an assertion about detailed roller-contact or slip behavior. Sideways travel at fixed yaw is allowed: **body heading is not travel tangent**.

### Supplied articulated-steering model (provisional interpretation)

This equation was supplied by the project author. The two input columns are **assumed** to correspond to longitudinal reference-point speed `v` and articulation angular rate `u_gamma = d(gamma)/dt`. Reference point, heading/joint angle sign convention, and geometry associated with `l1` and `l2` remain **unverified** (OQ-022). Do not construct collision frame transforms from guessed physical geometry.

Let `q=(x,y,Theta,gamma)`, `u=(v,u_gamma)`, and `D(gamma)=l1 cos(gamma)+l2`.

\[
\frac{d}{dt}\begin{bmatrix}x\\y\\\Theta\\\gamma\end{bmatrix}
=
\begin{bmatrix}
\cos\Theta & 0\\
\sin\Theta & 0\\
\frac{\sin\gamma}{l_1\cos\gamma+l_2} &
\frac{l_2}{l_1\cos\gamma+l_2}\\
0 & 1
\end{bmatrix}
\begin{bmatrix}v\\u_\gamma\end{bmatrix}.
\]

In scalar form:

\[
\dot x=v\cos\Theta,\quad
\dot y=v\sin\Theta,\quad
\dot\Theta=\frac{v\sin\gamma+l_2 u_\gamma}{D(\gamma)},\quad
\dot\gamma=u_\gamma.
\]

This is a **driftless control-affine** kinematic model with four continuous states and two inputs.

**Consequences**:

- `gamma` belongs in state and the discretization policy; equal `(x,y,Theta)` does not imply equivalent future reachability when articulation differs.
- At `v=0`, changing the articulation input changes `Theta` while `x,y` are stationary. This is *not* the same as independently commanded, unrestricted differential-drive rotation in place. With `v=0` and a nonsingular `D`, returning to the same `gamma` through articulation-only movements also restores the initial `Theta`.
- Body collision geometry can sweep at `v=0`; the checker must use all frame transforms at intermediate states, not only reference-point displacement.
- `D(gamma)` must remain nonzero (preferably bounded away from zero) over the configured joint range; also validate lengths, angle limits, speed and articulation-rate limits.
- For `v != 0`, `dTheta/ds = sin(gamma)/D(gamma) + l2*u_gamma/(v*D(gamma))` where signed `ds/dt=v`. Thus the fixed-joint curvature expression applies only when `u_gamma=0`. It is singular as `v -> 0` even though the time-domain ODE may be well defined.
- Standard Dubins and Reeds–Shepp connectors do **not** enforce changing articulation state or its coupling; only a specifically proved restricted connector may use those constructions.
- Use time-domain propagation for the complete state and preserve the full actuation trajectory for reconstruction; generic integrators such as RK4 can evaluate the vector field, but exact special cases and numerical convergence require tests.

**Illustrative derivative, not a finalized API**:

```cpp
struct ArticulatedState { double x, y, theta, gamma; };
struct ArticulatedControl { double speed, gammaRate; };

ArticulatedState derivative(
    const ArticulatedState& q,
    const ArticulatedControl& u,
    double l1,
    double l2)
{
    const double d = l1 * std::cos(q.gamma) + l2;
    // Validate |d| >= configured safety threshold before this call.
    return {
        u.speed * std::cos(q.theta),
        u.speed * std::sin(q.theta),
        (u.speed * std::sin(q.gamma) + l2 * u.gammaRate) / d,
        u.gammaRate
    };
}
```

A physical frame diagram and explicit kinematic sign conventions are required before accepting it as a production-ready robot model.

### General articulated extension

The model supplies \(\mathbf q=(x,y,\theta,\phi_1,\ldots,\phi_n,\ldots)\), its coupled ODE and all frame transforms \(T_i(\mathbf q)\). Search, collision and indexing must not assume three-dimensional state or fixed body offsets.

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

*Semantics*: the derivative is deterministic and map-independent; control limits may depend on state. Derivatives should avoid allocation or parsing. A **separate integrator** uses the vector field to propagate complete states and produce intermediate states/dense evaluation for swept collision checking. Exact closed-form propagation can be an optional model specialization. The integrator must respect the state manifold, including angular wrap.

A primitive provider chooses admissible controls and integration extents. It is separate from the ODE. The state indexer uses the complete state required for reachability/dominance. The search engine never branches on `isHolonomic()` to decide what motion is allowed; the model's valid control set is authoritative.

## Time-domain dynamics versus path progress

Time is the canonical derivative variable for these ODEs because it works for in-place rotation and simultaneous holonomic translation/rotation. A primitive of nominal duration \(\Delta t\), with constant input, may be evaluated over a local parameter \(\sigma\in[0,1]\):

\[
\frac{d\mathbf q}{d\sigma}=\Delta t\,f(\mathbf q,\mathbf u).
\]

Here \(\sigma\) is **primitive-local progress**, not a global execution schedule. A downstream MPC still chooses timing, but any rate-constrained property verified at a specific nominal duration must be rechecked if the motion is retimed. Arc-length parameterization alone is not universal: pure rotation has \(ds/dt=0\), and holonomic yaw is independent of the translational path tangent.

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

OQ-002 (primitive integration extent), OQ-003 (path progress), OQ-004 (cost units), OQ-007 (Ackermann steering state), OQ-019 (capability metadata), OQ-020 (mecanum milestone), OQ-021 (state/tangent integration customization) and OQ-022 (articulated model geometry/control conventions) remain open.
