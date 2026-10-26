# W10 — differential-drive Hybrid A* integration

**Implemented:** 2026-10-26 on `docs/architecture-baseline`.

The existing C++ Hybrid A* now accepts `PlanOptions::Model::DifferentialDrive`
alongside `Ackermann`. The **same** open queue, deterministic label map,
quantized `(x,y,yaw,incoming gear)` key, occupancy snapshot, search budgets,
goal tolerance, exact raster footprint collision checker, and independent
strict final validation are reused. Incoming gear is deliberately preserved
through a pure rotation so subsequent direction-switch penalties remain
correct. The model-specific primitive catalog is built once per request
and its ordered local samples are rigidly transformed at each expansion.

## Kinematics and search controls

- **Ackermann:** existing six request-local cached fixed-curvature
  forward/reverse primitives, default model and unchanged analytic RS/refiner.
- **Differential drive:** eight cached constant-`(v,omega)` trajectories:
  forward/reverse straight; forward/reverse left/right arcs; clockwise and
  anticlockwise **stationary rotations**.
- **Stationary rotation:** `MotionSection::kind=RotateInPlace`, signed
  `turnRadians`, positive **angular** `progressExtent()`, and
  `lengthMeters=0`. Never confuse angular progress with traveled metres.
- **Cost:** translating sections reuse distance-based forward/reverse and
  switch costs. Rotation adds strictly positive
  `rotationCostMetersPerRadian * abs(turnRadians)`, default `0.40`.
  A spin does not erase the previous incoming gear.
- **Indexing:** same three-pose-coordinate search bins; incoming gear
  persists on spin transitions. This remains **single-label approximate
  search**, not globally complete or optimal.
- **Heuristic:** certified Euclidean-to-XY-goal-region remains available
  (a nonnegative lower bound with zero cost for on-site rotation).
  The optional relaxed occupancy-grid heuristic remains noncertified.
  Ackermann-specific Reeds–Shepp analytic connector and clearance scorer
  are **rejected** for differential mode until independently generalized.

## Collision checks and path output

Exact raster full-body pose checking uses the requested pinned map
revision, occupancy policy and rectangular footprint for both models.
The differential demo footprint is a compact center-referenced rectangle
`rearX=-0.35, frontX=0.35, halfWidth=0.25` metres. A consumer of
the native C++ API must provide its own actual footprint.

A spinning reference point travels zero metres, but the body does not:
the collision sample count is bounded from below by
`maxBodyRadius * abs(turnRadians) / maxBodyPointMotionMeters`.
Both local precomputed search samples and **independently generated**
strict-final section samples evaluate intermediate headings. If a work
limit or a collision prevents validation, the planner must not claim
`Success` or collision evidence. This is still **pose-sampled** collision
assurance, *not* a continuous swept-volume proof.

The existing JSON `original.sections` now includes
`kind: "translation" | "rotation"`, `turnRadians`, and ordered
`points: [[x,y,yaw],...]` for every section. A rotation has
`length: 0` and positive nonzero angular change; its points have
identical reference positions and changing heading. Each point is in
the occupancy map's **world frame** (`frameId`), as before, and the
browser transforms it with the map origin including any nonzero yaw.

## Planning Lab and CLI

```bash
# Pure rotation on the built-in open map; output is a validated path JSON.
./build/hybrid_astar_demo --once --model differential \
  --scenario open --start 10 15 0 --goal 10 15 90

# Rotated ROS occupancy-grid origin.
./build/hybrid_astar_demo --once --model differential \
  --map-yaml apps/demo/maps/sample.yaml
```

HTTP accepts `model=differential` with existing GET or bounded POST
`/api/plan`; the browser supplies it from the new model selector.
Switching models immediately invalidates stale paths, replans, draws the
appropriate footprint, and visualizes in-place turns with a violet turn
marker and animated robot heading. Ackermann-only OMPL and shortcut
refinement controls are disabled in differential mode. Requests that try
to combine these unsupported options return an explicit error.

## Regression tests

- `tests/w10_differential_search_test.cpp`:
  positive-cost zero-translation 90° turn; reproducibility; strict
  full-footprint collision evidence; rotated map origin; body swept during
  intermediate headings even if the endpoints are clear; unsupported
  analytic/zero-spin-cost cases; preservation of the Ackermann default.
- `tests/demo_http_test.py`: model switch, bounded POST, successful pure
  turn, JSON angular samples and unsupported option rejection.
- `demo_cli_differential_spin` CTest invokes the same operation via CLI.
- `tests/w10_motion_primitives_test.cpp`: analytic-vs-RK4 propagation
  and catalog endpoint/sampling contract from the previous W10 increment.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DHYBRID_ASTAR_BUILD_TESTS=ON -DHYBRID_ASTAR_BUILD_APP=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

**Verification limitation:** exact-checkout Catch2/OMPL/Docker execution
is not available in the current environment. The code and tests are
committed, but these assertions have not been run end-to-end here.
Run the above CTest suite before release or declaring performance and
behavioral compatibility across all maps.

## Further engineering

The shared search implementation now branches on the model when building
the catalog and calculating turn costs. The **generic model concept /
state-index / collision-shape abstraction** remains future refactoring;
this increment does not claim arbitrary articulated or mecanum support.
Likewise, actuator-rate/acceleration feasibility and continuous
swept-volume collision certification remain outside this geometric
planner. No GitHub Actions workflows are required.
