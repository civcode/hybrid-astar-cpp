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

## Finite yaw-lattice robustness (2026-10-27)

Differential-drive primitive spins use fixed 22.5-degree increments. The
default target tolerance is 10 degrees; the exact numerical difference
between a requested `-80 deg` start and `-90 deg` goal can evaluate
*slightly greater* than 10 degrees even though the angles are exactly
10 degrees apart mathematically. The general goal test now uses the same
`1e-9`-radian/meter boundary slack as independent final goal validation.

A more general issue occurs if the target yaw lies between the discrete
lattice headings. The differential-drive policy now offers **one optional
final stationary turn** once the current XY position is within the specified
goal tolerance, making precise heading correction possible without changing
the global 22.5-degree primitive catalog. The policy charges its configured
nonzero rotation cost. The existing common search kernel checks the terminal
rotation *and the entire preceding path* against the exact full-footprint
checker and strict final sampling; an occupied swept footprint is rejected.

Added regression coverage for the exact tolerance boundary, an off-lattice
goal heading reachable by a strictly checked terminal rotation, and a
collision during that terminal rotation. The motivating edited warehouse
map contained 782 client-added occupancy edits; the exact edit payload is
not part of the test fixture. This fix addresses two confirmed numerical/
reachability hazards but does **not** by itself prove that the reported
million-expansion case is resolved. The demo JSON now emits generated
nodes, key pruning, grid settlements/fallbacks, and terminal attempt/
validation/rejection counters to help reproduce failures.

 
## Strict differential-drive path refinement (2026-10-27)

The core `ackermann::refinePath(request, validatedPlan, connector, options)`
now accepts `PlanOptions::Model::DifferentialDrive`, in addition to
Ackermann. The model's search and collision checking are not changed. The
post-search refiner independently revalidates the **entire original** against
the pinned map/revision and full rectangular footprint before considering any
replacement; malformed, stale, budget-starved, or geometrically colliding
inputs cannot turn into an advertised validated success.

For differential drive, native candidates consist of:

- Straight-line chords between compatible pose/heading boundaries, with
  no newly inserted in-place rotations. Existing spins are not removed if
  no strictly validated cost-reducing replacement exists.
- Optional fixed-gear CSC curves (LSL/LSR/RSL/RSR at a configurable 0.75 m
  candidate radius). Each arc is an ideal differential-drive constant
  curvature motion. The checker confirms that a positive finite control speed
  exists under the differential wheel-rate limits; it may be lower than the
  configured primitive translation speed.

Differential rolling shortcut joins now avoid retained in-place rotations
immediately outside the candidate window. New splices also cannot add a
previously absent gear reversal or a sharper curvature discontinuity than
the original seam. These are kinematic/visual smoothness checks in addition
to collision safety, not acceleration-continuity guarantees.

The strict verifier evaluates every exact motion section, including body-swept
in-place rotations, at the configured final sampling density, with finite
candidate attempts, accepted replacements and total pose-check limits.
It recomputes all path costs (including angular rotation cost and gear-switch
penalties), terminal state, gear/cusp metadata and validation evidence.
Replacing a window requires a strict **whole-path cost reduction**;
otherwise the verified original is retained. Motion sections are geometrically
continuous, but curvature transitions, wheel accelerations, jerk and physical
tracking dynamics are **not** certified.

The differential search sometimes emits hundreds of small sections, so the
candidate window schedule spreads trials across the route at several scales
rather than burning every attempt on the first path prefix. The Planning Lab
sets differential refinement budgets to 160 candidate windows, 8 accepted
replacements and 200,000 strict pose checks; the C++ library API remains
separately tunable. Enabling `refine` alone tries straight chords;
also enabling `curved` adds rolling CSC arcs. Neither mode generates
stationary spin shortcuts. Both remain opt-in.

Unit tests in `differential_refinement_test` cover a reducible zigzag,
a blocked shortcut, a colliding swept spin, strict work-budget behavior and
unsupported mecanum. `demo_test` and `demo_http_test` cover the Planning
Lab integration. Run these plus `refinement_test`, `curved_shortcuts_test`
and `w10_differential_search_test` before treating the feature as verified.

## Reported differential warehouse strict-final failure (2026-10-27)

The default differential policy already requests final-density collision
sampling during search. A `StrictValidationFailed` search result means a
goal-region state was encountered and its reconstructed route did not pass
the independent final path checks. It is **not** a proof of infeasibility,
and is not necessarily a swept-collision failure. The planner now records
`strictGoalCandidates`, `strictRejectedCollision`,
`strictRejectedContinuity`, and `strictRejectedGoalMismatch`.
These distinguish the principal rejection classes in the demo JSON/UI;
terminal connector-specific rejects are counted separately.

Reproduce the reported user case on the unedited warehouse layout manually
(not as a default, potentially expensive CTest):

```bash
./build-docker/hybrid_astar_demo --once --model differential \
  --scenario warehouse --start 5.23 21.21 0 \
  --goal 42.87 35.28 0 --inflate 0.5 --budget 1000000
```

A 0.5 m hard margin can change the reachable free configuration space
substantially. This diagnostic change does not by itself guarantee a valid
route or resolve single-key search-label pruning. The final validator must
never be bypassed to make the test succeed.
