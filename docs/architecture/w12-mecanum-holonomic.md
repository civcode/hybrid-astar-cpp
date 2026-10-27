# W12 — ideal mecanum/holonomic planner integration

**Branch:** `docs/architecture-baseline` · 2026-10-26.

## What is implemented

A third **ideal four-wheel mecanum** model reuses the W11 typed Hybrid A*
search kernel, its immutable map snapshot, deterministic key/queue logic and
independently checked final collision evidence. The model is not a claim
about tire slip, roller force, dynamic feasibility or timed actuation.

- `mecanum::Model` accepts independent body forward, lateral and yaw
  velocities. Four explicitly ordered wheel rates are bounded by wheel
  radius, wheelbase/track lever arm and a maximum angular rate.
- `motion::constantHolonomicTwist` is an **exact SE(2) exponential** for
  any constant planar body twist. It does not assume that travel tangent
  follows body yaw. Independent midpoint integration is part of W12 tests.
- Fourteen once-per-request local primitives include cardinal moves,
  diagonal moves, forward+turn, lateral+turn and pure spin. Templates carry
  complete pose samples with conservative
  `distance + bodyRadius * abs(yawChange)` sweep spacing bounds.
- `MotionKind::HolonomicTwist` carries signed body forward/lateral travel
  and yaw change. Its progress is actual reference-center translation in
  meters; pure spin still has angular progress in radians.
- `planning::MecanumPolicy` provides a no-gear state key, positive
  translation-plus-angle effort cost and checked rigid rectangular
  footprint. The same **shared** search, collision and final validation
  kernel is used. The certified Euclidean goal-region heuristic remains a
  lower bound because every translating edge costs at least its XY
  displacement.
- Planning Lab's selector and `--once --model mecanum` produce
  `kind: holonomic` path sections with independent forward/lateral/turn
  metadata. The browser renders the full footprint, colored holonomic
  sections and actual SE(2) heading samples. The HTTP API advertises a
  `mecanum` capability.

## Compatibility and safety

The default Ackermann and existing differential-drive models retain their
existing primitives and serialized segment types. Mecanum rejects
Reeds–Shepp, CSC arc shortcuts, distance-only analytic heuristic
connectors and clearance-weight integration. Its **separate, model-aware
holonomic refinement** is supported as described below. Wheel constraints
validate constant controls under finite time scaling, not acceleration,
jerk, traction or execution dynamics.

The collision checker still verifies a **single** planar rigid rectangle,
using discrete body-motion-adaptive sampling. This does not establish
continuous swept collision freedom, compound multibody collision or
actuator safety.

## Exact-goal and warehouse follow-up (2026-10-27)

The previous 14 fixed-length mecanum primitives could search many quantized
states without finding a strictly validated path to an arbitrary continuous
XY/yaw goal. A stricter final-collision sampler could also reject an edge
accepted at coarse search resolution, allowing the cheaper but invalid
single-label predecessor to suppress a viable alternative.

The planner now permits a **model-provided, bounded terminal connection**
within 1 metre of the goal: translation and stationary rotation in either
order. Wheel-rate feasibility is checked by slowing diagonal translations
if needed; both orders must still pass independent full-path endpoint,
continuity, footprint-collision and sample-budget validation. At most
32 nearby frontier states get a connection attempt. This is **not** a
continuous swept-safety certificate or time-parameterized controller plan.
The C++ runtime policy adapter forwards these optional capabilities so W11
native/adapter equivalence is retained; unrelated model policies are unchanged.

Mecanum's search-level footprint sampling now uses at least the final
checker resolution so near-contact edges fail before occupying their search
keys. Its heuristic adds a certified lower bound for the remaining heading
change, weighted by the positive geometric turn cost. The optional
noncertified obstacle-grid total settlement budget is now 100,000.

The library, built-in Planning Lab scenarios and browser control now
default to **1,000,000 expansions** (and 4,000,000 generated-node maximum).
This increases possible computation and memory expenditure on impossible
maps, so manual run budgets should be selected based on the investigation.

The W12 Catch2 suite now includes (a) an arbitrary off-lattice pose/yaw
with independent segment checks and runtime adapter equivalence, and
(b) the warehouse obstacle/pose reproduction from the reported failure:
start (4.77, 5.62, 0°), goal (31.01, 3.41, −90°).

**Verification pending:** These new revisions have been pushed on `dev`,
but the full exact-checkout build, warehouse regression, performance
benchmarks and sanitizer tests have not been run here. Do not treat
the new goal-connection behavior as proven until the manual Quick/CTest
suite passes. No GitHub Actions was triggered by this work.

## Performance profiling and 4-disc validation

A shared `bench/warehouse_scenarios.hpp` fixture now supplies the exact
warehouse map and off-lattice start/goal to both the W12 regression and the
manual hotpath benchmark. The planner reports terminal connection attempts,
candidate tails, strictly validated connections and rejected connections.
These values appear in the opt-in `--warehouse` profiling CSV alongside
collision calls, queue work and expansion counts.

The dedicated `hotpath_test` CTest suite exercises the 4-disc conservative
prefilter against the exact checker on rotated maps, boundary/contact poses,
near-obstacle configurations and too-narrow gaps. It also compares a compact
mecanum planning request with and without the prefilter. A direct prefilter
`definitelyClear` result must **never** hide an actual footprint collision,
and final validation still uses the independent exact checker.

The production default remains `discPrefilterCount=0`. Do **not** enable
four discs globally until the expanded invariance tests and profiled vs
unprofiled benchmark suite pass on representative maps. The benchmark
separately supports certified-Euclidean vs noncertified obstacle-grid A/B with
identical planning budgets and initial states.

See [hotpath benchmarking](w6-hotpath-profiling.md) for the commands.

## Tests and release gate

`w12_mecanum_test` has been registered under Catch2/CTest:

1. Four wheel-rate mapping and invalid controls/geometry.
2. Exact combined body twist versus independent numerical integration.
3. Fourteen bounded cached primitives and endpoints.
4. W11 runtime-policy adapter equivalence to the built-in mecanum entry point.
5. Deterministic end-to-end lateral-goal search, strict returned path checks
   against the pinned map and provenance.
6. Intermediate collision along a lateral-plus-turn segment.
7. Invalid wheel constraints and unsupported connector/clearance options.
8. The demo HTTP regression covers lateral section JSON and rejects
   incompatible OMPL options.

A local C++20 ASan/UBSan **standalone formula smoke** has passed for
holonomic exponential propagation, independent numerical integration,
lateral travel, spin and nominal wheel-rate limits. It is *not* a compile or
CTest pass of this exact repository.

**Still pending:** an exact GitHub checkout CMake/Catch2 run (including W11),
full installed OMPL package and Docker matrix, Clang/GCC sanitizers on the
actual library and demo, browser acceptance on target platforms, and
representative performance measurements. The current environment has no
network access for cloning the repository, no Docker engine and no HTTPS access for
the initial Catch2 FetchContent download or complete installed OMPL test
dependencies. Catch2 v3.16.1 is now automatically provisioned through CMake
on machines with Git and network access. A [manual-only Actions verification
workflow](../ci/manual-verification.md) has been configured on the feature
branch, but has not been dispatched. The default-branch activation requirement
and the exact-checkout verification gates are still pending.

### Focused local verification with Catch2 v3.16.1 (2026-10-26)

Using the user-supplied upstream `Catch2-3.16.1.zip` as
`FETCHCONTENT_SOURCE_DIR_CATCH2`, a **separate local CMake harness**
successfully compiled the six exact repository source/header blobs required
by the standalone mecanum kinematics and cached-primitive modules
(the local files' Git blob SHA-1 hashes matched the linked branch).
Three targeted Catch2 test cases passed: four wheel limits, combined
constant-body-twist integration, and the 14-template catalog.
**102 assertions / 3 cases passed**, and the same focused code compiled
with AddressSanitizer + UndefinedBehaviorSanitizer passed without findings.

**Scope limitation:** This was not an invocation of the repository's full
`tests/w12_mecanum_test.cpp` or root-project `ctest` matrix. The exact
repository checkout cannot be cloned directly in this sandbox, and
the focused local harness does not link the planner/search, validation,
demo or OMPL components. The W11/W12 exact-checkout gates remain unchecked.
The default CMake FetchContent pin is updated to `v3.16.1`.

### Manual verification on a provisioned host

```bash
cmake -S . -B build-w12 -DCMAKE_BUILD_TYPE=Debug \
  -DHYBRID_ASTAR_BUILD_TESTS=ON -DHYBRID_ASTAR_BUILD_APP=ON
cmake --build build-w12 --parallel
ctest --test-dir build-w12 --output-on-failure
./build-w12/hybrid_astar_demo --once --model mecanum \
  --scenario open --start 10 15 0 --goal 10 17 0
# Separately, with Docker and a network:
docker compose run --build --rm test
```

## Deferred

- Time-parameterization, acceleration, actuation, wheel-ground force and
  slip feasibility, and verified continuously swept volumes.
- Compound or articulated-body geometry and configuration state spaces
  beyond the planar rigid Pose2D + quantized XY/yaw key.
- Model-aware analytic shortcuts, obstacle-aware refinement and other
  performance optimizations; preserve the correct baseline first.
 
## Strict native holonomic path refinement (2026-10-27)

In addition to the search-time bounded stop removal, the C++ strict
`refinePath` API and Planning Lab `refine` checkbox now support mecanum
plans. The candidate is the exact inverse SE(2) constant body-twist
connecting two validated section-boundary poses; it combines longitudinal,
lateral and angular motion rather than introducing any new full stop.
Candidate wheel rates are checked under finite time scaling; acceleration,
jerk, and actuator tracking are not implied.

The refiner revalidates the original whole path against the pinned map and
configured metric footprint inflation, then tries bounded multi-scale
shortcuts. A candidate is accepted only after independent full-path
collision sampling and strictly lower weighted cost including the configured
stationary spin penalty. Any failed, exhausted or colliding shortcut retains
the validated original path. CSC/Reeds–Shepp candidates remain non-mecanum.

`differential_refinement_test` includes a small holonomic zigzag with
strict improvement and a blocked direct shortcut with 0.15 m inflation.
The default post-path refinement checkbox is still unchecked for all models;
it must be explicitly enabled by the user.
