# Articulated-steering integration gate and implementation plan

**Status (2026-11-04): the author has supplied an explicit
FIVE-variable front/rear chassis ODE.** The documented earlier four-state
ODE was mathematically equivalent but **misidentified the front reference
pose as a rear reference pose**. This correction is captured in the
[canonical articulated kinematic specification](kinematic-models.md).
Physical axle/joint locations, collision-body geometry and operating
limits still need author confirmation before fourth-model integration.

**Use the [dedicated kinematic model and frame-conventions review draft](articulated_steering_kinematic_model_review.md) for the owner's physical-robot sign-off.** It covers the exact ODE, front/rear frames, L_f/L_r, stationary steering and undecided conventions. Stages A–C and the experimental Planning Lab interface have been implemented using example-only geometry. Do not deploy them on a physical robot until its body frames, collision extents, joints and actuation limits have been confirmed.

**Implementation status (2026-11-04):** The five-variable C++ kinematic
model uses the project-owned RK4 integrator. A **separate experimental
articulated Hybrid A* entry point** has now been added in
[`articulated_steering/planner.hpp`](../../include/hybrid_astar/articulated_steering/planner.hpp)
and [`planner.cpp`](../../src/articulated_steering/planner.cpp).
It carries rear `(x,y,yaw,gamma)` search states and builds a
**request-local, reusable articulation-indexed primitive library**:
constant-`gamma` forward/reverse motion as analytic circular arcs,
and changes in `gamma` **only at zero commanded front speed** as
precomputed RK4 steering transitions. Every local template is
transformed to the expanded world rear pose; both collision bodies
are checked against the immutable occupancy map. Accepted paths
are independently re-integrated by RK4 and collision checked
at finer resolution before success.

**This is not production-ready robotic motion validation.**
The front/rear collision bodies, link geometry, joint limits and
actuator semantics are caller supplied and must match the actual robot.
Self-collision, continuous swept-volume certification, dynamic-obstacle
prediction, and timed actuator feasibility are not provided. The existing Ackermann/differential/mecanum planning behavior is
unchanged. Planning Lab now offers an **experimental articulated model**
that selects this separate 4D backend, with dedicated `gamma` controls,
`L_f/L_r`, two footprint definitions and full-state replay. The
complete C++/Docker build, live-browser visual review and end-to-end
real-robot testing remain **unverified in this environment**.

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

```math
\dot\Theta_f=\frac{L_r}{D(\gamma)}u_\gamma,\qquad
\dot\Theta_r=-\frac{L_f\cos\gamma}{D(\gamma)}u_\gamma,\qquad
D(\gamma)=L_f\cos\gamma+L_r.
```

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

```math
\dot\psi=\frac{L_r-L_f\cos\gamma}{2D(\gamma)}u_\gamma.
```

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

### Stage A — five-state kinematic model and own RK4 — implemented

- [x] Preserve the author-provided five-variable ODE
  `(xf,yf,Theta_f,Theta_r,gamma)` with invariant
  `Theta_f-Theta_r=gamma` modulo `2*pi`.
- [x] Provide front/hinge/rear frame conversions and the rear
  `base_link` reference configuration.
- [x] Use classical C++20 `motion::rk4Step`, without Boost.Odeint or ROS.
- [x] Validate supplied link distances, joint/control limits, denominator
  bounds, finite states and finite integration-step budgets.
- [x] Test analytic fixed-articulation arcs, reverse, joint-only turns,
  RK4 convergence and front/rear frame consistency.
- [ ] Confirm production frame locations, articulation encoder sign,
  permissible inputs and mechanical limits with robot owner.

### Stage B — four-dimensional bounded search — implemented experimentally

- [x] Add `articulated_steering::plan(Request)` with **rear x/y/yaw
  and articulation gamma** indexed independently; the incoming drive
  direction is also part of the label key.
- [x] Precompute **two fixed-`gamma` drive templates per
  articulation lattice angle** (forward and reverse) with the exact
  analytic rear-reference arc; no RK4 is used in search expansion
  for these edges.
- [x] Precompute **adjacent steering transitions** `gamma_i →
  gamma_{i±1}` by integrating the five-state ODE **once per
  articulation interval** with our own RK4 and `v=0`. Steering
  while driving is intentionally **not** an allowed primitive.
  Both body headings rotate; the front reference remains fixed
  and the rear `base_link` may move.
- [x] Include arbitrary exact start and goal articulation angles
  in the request-local gamma lattice. No state is teleported,
  rounded to a joint bin or allowed to jump instantly between
  fixed-articulation drive arcs.
- [x] Reuse all local templates in search by SE(2) transformation
  of their complete intermediate pose samples. Still check
  both collision bodies at **every transformed sample** and
  independently integrate the complete final path by RK4.
- [x] Retain complete five-variable states, rear reference poses,
  control, nominal duration and checked intermediate samples
  on reconstructed motion sections.
- [x] Add separate expansion, generated-node and integration-sample
  budgets, cancellation and typed failure outcomes.
- [x] Check four-state goal tolerances, **including gamma**, and
  revalidate a candidate independently before reporting success.
- [ ] Tune primitive times, key resolution, control families, label
  pruning and search quality on representative articulated maps.
- [ ] Integrate a compatible fourth model into the Planning Lab/browser
  UI only after validating physical robot geometry; the rigid
  `ackermann::PlanRequest` API deliberately cannot carry gamma.


### Primitive library invariants and cost

The grid's articulation spacing is configured by
`SearchOptions::articulationPrimitiveRadians` (default 0.2 rad).
Values $\gamma_{\min},\gamma_{\max}$, the **exact request start
angle** and the **exact request goal angle** are retained. Each
angle state has two analytic drive templates ($\dot\gamma=0$,
$v=\pm v_{\mathrm{configured}}$) and up to two neighboring
stationary steering templates ($v=0$,
$\dot\gamma=\pm u_{\mathrm{configured}}$). All templates are
constructed **once per plan**, not once per search expansion.

- **Driving:** $\dot\Theta_r=v\sin\gamma/D$;
  $v_r=v(L_f+L_r\cos\gamma)/D$. The local rear trajectory
  is a straight line for zero angular velocity and an exact
  circular arc otherwise. Both carriage headings rotate together,
  maintaining $\gamma$ exactly.
- **Stationary steering:** Generate a complete local trajectory
  using the **five-state RK4 ODE** at $v=0$ between neighboring
  angles. The front reference $F$ stays fixed, while both
  carriage headings and the rear reference $R$ evolve.
  Each transition has finite time $|\Delta\gamma|/|u_\gamma|$.
- **Reuse:** Precompute with the local rear reference at
  $(0,0,0)$; transport every sample to the node's actual rear
  pose using the standard SE(2) transform. $\gamma$ itself
  is unchanged by this frame transformation.
- **Collision:** Static maps do **not** permit precomputing
  universal collision outcomes; each transported sample
  must be checked against the pinned map and against **both**
  carriage rectangles. Intermediate samples are essential
  even for stationary steering.
- **Safety:** The final accepted candidate is independently
  re-integrated with finer RK4 steps and collision checked.
  Precomputation does **not** bypass this pass. Still no
  continuous swept-volume or physical actuator certificate.

The library is deliberately limited to **64 or fewer regular
articulation intervals**, plus explicit start/goal angle
insertion, and fails with a typed status if its sample
budget cannot cover the configured geometry. Instrumentation
exposes the number of analytic templates, RK4 steering
templates, stored checkpoints and template applications.

This is analogous to Ackermann's selection of a constant
steering command per arc, but **not** identical to
instantaneously changing Ackermann wheel angle: articulated
steering physically displaces carriage geometry, so its
joint-only transition must occupy its own collision-checked
edge in search.

### Stage C — front and rear sampled collision — implemented experimentally

- [x] Require explicit front and rear filled-rectangle geometry in
  their own carriage frames. No Ackermann geometry is silently reused.
- [x] Use `articulated_steering::BodyCollisionChecker` with two
  independent existing `ackermann::ExactCollisionChecker`
  instances against the **same immutable map**, occupancy/unknown
  policy, map-origin transform, boundary checks and metric inflation.
- [x] Evaluate **both** collision bodies at each full state, including
  joint-only steering where the front reference remains fixed.
- [x] Bound the sampled motion of body points using conservative
  reference-speed, yaw-rate and body-radius bounds; treat insufficient
  sample budgets as incomplete rather than safe.
- [x] For every candidate goal route, replay the complete control
  history from the initial state using **denser RK4 samples**, check
  both bodies at every sample, verify endpoint continuity and
  goal tolerances, and set success only after this pass.
- [ ] Add independently validated nonrectangular attachments, body-to-
  body self-collision where relevant and mechanically accurate offsets.
- [ ] Consider continuous swept checks or a certified motion envelope
  if hard safety claims are needed. Adaptive samples alone do **not**
  mathematically prove continuous collision freedom.

### Stage D — UI, performance and real-robot readiness — pending

- [ ] Confirm `L_f`, `L_r`, both chassis collision shapes/offsets,
  joint-angle/rate limits and whether steering at rest is physically
  achievable. These values remain **unapproved** for deployment.
- [ ] Run complete Catch2/CTest and regression tests in the real
  Docker checkout; measure reachable hard maps, cancellation,
  memory, sampling overhead and failed-final-validation outcomes.
- [x] Expose experimental articulated model selection, start/goal
  rear-reference pose and `gamma`, `L_f/L_r`, independent front/rear
  carriage rectangles and joint markers in Planning Lab.
- [x] Route the loopback-only `/api/plan` endpoint to
  `articulated_steering::plan` for this model, preserving all
  existing rigid-model routes.
- [x] Add headless JavaScript rendering/form regression, dedicated
  C++ demo-adapter tests and HTTP smoke coverage of the same path.
- [ ] Run full local Docker/CMake/Catch2/HTTP tests and inspect
  the articulated UI visually in a real browser before relying on it.
- [ ] Develop model-specific refinement, steering-rate continuity,
  velocity profiling, self-collision checks and controller tracking.
- [ ] Keep independent real-robot supervisory collision and motion
  safety checks; successful geometric planning does not imply
  actuator-feasible trajectories.

## Standalone C++ planning usage

```cpp
#include "hybrid_astar/articulated_steering/planner.hpp"
using namespace hybrid_astar::articulated_steering;

Request request;
request.map = pinnedMap;  // shared_ptr<const ackermann::MapSnapshot>
request.parameters = {
    .LfMeters = 0.5, .LrMeters = 0.5,
    .minGammaRadians = -1.2, .maxGammaRadians = 1.2,
    .maxAbsFrontSpeedMetersPerSecond = 1.0,
    .maxAbsGammaRateRadiansPerSecond = 0.8,
    .minDenominatorMeters = 0.1
};
// EXAMPLES ONLY: replace BOTH bodies and ALL parameters with real geometry.
request.bodies.front = {-0.3, 0.3, 0.15};
request.bodies.rear  = {-0.3, 0.3, 0.15};
request.start = {10.0, 10.0, 0.0, 0.0};
request.goal  = {12.0, 10.0, 0.0, 0.0};

const Result result = plan(request);
if (result.status == Status::Success &&
    result.independentlySampledBodyValidated) {
    // result.motions have full front/rear state checkpoints.
    // Still NOT a continuous swept/actuator feasibility certificate.
}
```

The `Request` does **not** default to usable geometry or
a calibrated robot: caller-supplied model parameters and both
physical footprints are mandatory. A successful response contains
independently re-integrated, denser-sampled collision checks, but
still requires actuator and continuous swept-volume safety validation
where applicable.

## Manual local verification

```bash
docker compose build test
docker compose --profile test run --rm test \
  ctest --test-dir build-docker --output-on-failure \
  -R 'articulated_steering_.*_test'
```

The executable tests are
[`articulated_steering_model_test.cpp`](../../tests/articulated_steering_model_test.cpp)
and [`articulated_steering_planner_test.cpp`](../../tests/articulated_steering_planner_test.cpp).
They cover kinematics, search-key gamma/gear separation,
straight/reverse/joint-only planning, front-only and rear-only
obstacle collisions, an obstacle struck *between* stationary-turn
endpoints, cancellation and finite sampling/validation budgets.

This branch's new search entry point still needs a full local
Catch2/Docker build; no GitHub Actions were dispatched or modified.

## Roadmap relationship

Stages A–C and experimental Planning Lab wiring have code and
regression tests. Stage D still requires the full local build,
browser checks and physical-robot geometry/actuator sign-off.
See [P0-B in the active implementation roadmap](../TODO.md) and
the [owner-review conventions](articulated_steering_kinematic_model_review.md).
