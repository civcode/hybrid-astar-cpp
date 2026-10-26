# W10 — local-frame primitives for Ackermann and differential drive

**Implemented 2026-10-26:** the W10 *model and primitive* milestone, not
the later W10 generic search, collision-policy generalization or Planning
Lab differential-drive mode. The existing Ackermann planner remains the
only executable Hybrid A* search in this increment.

## Architecture and exactly what is cached

Every fixed-control rigid planar primitive is stored in an **SE(2)-local
reference frame** with anchor `(0,0,0)`. The generic planar
`motion::LocalPrimitive` contains:

- the native extent: `TranslationMeters` or `RotationRadians`;
- the nonnegative extent value in its **specific** unit;
- signed reference-body traveled distance and heading change;
- ordered `{lambda, localPose}` samples, always including 0 and 1;
- `evaluate(lambda)`, continuously evaluated using the exact SE(2)
  exponential for constant body-frame twist.

At a continuous search state `worldAnchor`, instantiate **without RK4**:

```cpp
const auto endpoint = primitive.local.endpoint(worldAnchor);
for(const auto& sample:primitive.local.samples) {
    const auto worldPose = hybrid_astar::motion::compose(
        worldAnchor,sample.localPose); // pose in the map's frameId
}
```

A library user can call `evaluateWorld(worldAnchor, lambda)` for
arbitrary progress not coinciding with a stored sample. Samples are
debug/drawing/initial collision points, **not a polyline substitute for
the model's analytic curve**.

### Ackermann

`ackermann::buildPrimitiveCatalog` generates the six established controls
(Forward/Reverse × Left/Straight/Right) in the same deterministic order,
**once per plan request** using that request's primitive length, curvature,
footprint radius and search sampling policy.

During ordinary Hybrid A* successor expansion, the new implementation
applies a rigid transform to the precomputed local endpoint and each
stored collision-sampling pose. It does not numerically reintegrate the
motion or rebuild samples at each node. If the configured sample budget
is insufficient, the original exact checker reports the same
`ValidationIncomplete` instead of trusting undersampled trajectories.

The core still stores an exact `MotionSection` for path reconstruction,
evaluation at arbitrary progress and use by OMPL steering/refinement.
**Independent strict final validation deliberately re-evaluates that
analytic section at finer settings**, rather than trusting previously
checked search samples or a map-independent template.

`PlanDiagnostics` exposes `primitiveTemplatesBuilt` (six normally),
`primitiveTemplateInstantiations` and
`primitiveTemplatePoseChecks`. Note that template instantiation involves
trigonometry to rotate/translate a rigid pose; *integration* and
template-sample generation are the operations avoided.

### Differential drive

The new `differential_drive::Model` describes ideal planar no-slip
`q=(x,y,yaw)`, `u=(v,omega)` with
`xdot=v cos(yaw)`, `ydot=v sin(yaw)`, `yawdot=omega`.
The parameter contract validates wheel radius, track width, and each
wheel's maximum absolute angular rate using:

```text
left  = (v - omega * track / 2) / wheelRadius
right = (v + omega * track / 2) / wheelRadius
```

`differential_drive::buildCatalog` creates eight local reusable
primitives **once per model/configuration**:

- Forward and reverse straight;
- Forward/reverse × left/right constant-twist arcs;
- Clockwise and counterclockwise in-place rotation.

A pure spin has **zero** translated distance but positive angular
extent, nonzero heading change and multiple poses. Sample spacing
conservatively limits any body point's possible motion by the bound
`|reference translation| + maxBodyRadius * |heading change|`.
The caller must supply a radius covering its *actual* collision shapes.
This is adaptive **pose sampling**, not mathematically continuous
swept-volume certification.

**RK4 versus analytic:** constant `v,omega` (including reverse,
zero omega, and zero v) is solved analytically using a stable sinc-based
SE(2) exponential. The independently written RK4 oracle in
`tests/w10_motion_primitives_test.cpp` compares endpoints across all
control families. RK4 remains a planned generic integrator for
time/state-dependent input laws and articulated models; no RK4 call
belongs in the constant-control search hot path.

## Build and tests

```bash
# Dependency-free model and core libraries:
cmake -S . -B build-w10 -DCMAKE_BUILD_TYPE=Release \
  -DHYBRID_ASTAR_BUILD_TESTS=OFF
cmake --build build-w10 --parallel

# With Catch2 v3 installed:
cmake -S . -B build-w10-tests -DCMAKE_BUILD_TYPE=Release \
  -DHYBRID_ASTAR_BUILD_TESTS=ON
cmake --build build-w10-tests --parallel
ctest --test-dir build-w10-tests --output-on-failure
```

The registered `w10_motion_primitives_test` covers independent fine-step
RK4 agreement with straight/curved/reverse/spin motions, wheel-rate
bounds and bad controls, all eight differential templates, SE(2) sample
and endpoint consistency, the six Ackermann templates against exact
`makeSection`, and an obstacle that is hit **only partway through a
stationary half-turn** on a rotated occupancy map. It checks that an
Ackermann plan really uses cached sample poses while still obtaining
full-footprint adaptive-sampled final collision evidence.

**Verification limitation:** a standalone GCC 20-mode smoke compile
of the new differential-drive model/template logic passed in the
current environment. The exact remote checkout and the installed
Catch2/Docker matrix are not available here, so the new Catch2 test
registration and full planner integration need a dependency-provisioned
host run before being considered release evidence.

## Intentional next milestones

1. Extract model-parameterized search successors, complete state indexing
   and collision geometry from the Ackermann-only `plan()` implementation.
   Differential-drive rotation requires a **positive angular cost** even
   when translation is zero; the current Ackermann arc-length-only cost
   and Reeds–Shepp connector must not be reused unmodified.
2. Validate the entire sampled body sweep of in-place turns, including
   out-of-map/unknown cells, at **independent final-validation density**.
   Choose a model-specific rectangle/circle/compound footprint rather
   than assuming the rear-axle Ackermann rectangle.
3. Add model selection, renderer/body geometry and angular progress
   playback to Planning Lab. The current browser already uses map-frame
   world `[x,y,yaw]` poses for sampled Ackermann section drawings.
4. Consider configuration-keyed catalog caching across planning requests
   **only after** defining model parameter, footprint radius, control
   catalog, sample policy and version keys. A request-local catalog is
   currently both deterministic and sufficient.

No GitHub Actions workflows are added or run.
