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
existing primitives, costs and serialized segment types. Mecanum rejects
Reeds–Shepp, the Ackermann-only shortcut refiner, distance-only analytic
heuristic connectors and clearance-weight integration until a model-aware
variant is validated. The wheel constraints validate *per-primitive constant
controls*, not acceleration, jerk, traction or execution dynamics.

The collision checker still verifies a **single** planar rigid rectangle,
using discrete body-motion-adaptive sampling. This does not establish
continuous swept collision freedom, compound multibody collision or
actuator safety.

## Tests and release gate

`w12_mecanum_test` has been registered under Catch2/CTest:

1. Four wheel-rate mapping and invalid controls/geometry.
2. Exact combined body twist versus independent numerical integration.
3. Fourteen bounded cached primitives and endpoints.
4. Deterministic end-to-end lateral-goal search, strict returned path checks
   against the pinned map and provenance.
5. Intermediate collision along a lateral-plus-turn segment.
6. Invalid wheel constraints and unsupported connector/clearance options.
7. The demo HTTP regression covers lateral section JSON and rejects
   incompatible OMPL options.

A local C++20 ASan/UBSan **standalone formula smoke** has passed for
holonomic exponential propagation, independent numerical integration,
lateral travel, spin and nominal wheel-rate limits. It is *not* a compile or
CTest pass of this exact repository.

**Still pending:** an exact GitHub checkout CMake/Catch2 run (including W11),
full installed OMPL package and Docker matrix, Clang/GCC sanitizers on the
actual library and demo, browser acceptance on target platforms, and
representative performance measurements. The current environment has no
network access for cloning the repository, no Docker engine and no installed
Catch2/OMPL test dependencies. No GitHub Actions workflow is run or created.

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
