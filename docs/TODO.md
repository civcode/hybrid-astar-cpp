> **2026-10-26 implementation update:** The external-review [consolidated
> specification](architecture/Hybrid_AStar_Consolidated_Specification.md)
> has been imported. The normative current behavior and remaining W0-W9 gates
> are tracked in [current-implementation.md](architecture/current-implementation.md).
> The current Ackermann primitives use exact constant-curvature integration, not RK4. This older checklist and ADR references may predate the verified vertical
> slice and must not override the current contract. **Still pending:** full OMPL package/export build and exact-checkout
> verification (the real OMPL 2.0.2 source-runtime tests now pass, W5),
> representative multi-seed performance thresholds and any reachability proof (W6),
> optional EDT/disc proof review and broader benchmarks (W7),
> clearance-weight tuning and broader curvature-aware post-path smoothing (W8; a bounded strict shortcut refiner is now implemented),
> future chassis/actuation and ROS adapters (W9). No swept/actuator safety
> certification or validated global optimality is claimed.

**Latest executable gates (2026-10-26):** The native signed adapter,
true OMPL 2.0.2 Reeds–Shepp source-runtime smoke and 12,000-case randomized
test, and independent C++20 search/refiner suites now run through a
[manual offline source bridge](../tests/ompl_source_bridge/CMakeLists.txt).
See [W5 runtime verification](architecture/w5-ompl-source-verification.md).
The separate W4 cumulative strict-final-check budget is implemented
(default 250,000 checks), including explicit incomplete status or preservation
of an earlier strictly validated incumbent. Full installed OMPL package
verification, complete GitHub checkout build, geometric swept certification
and steering actuator modeling remain future work.

## W12 mecanum / holonomic search — 2026-10-26

- [x] Add ideal four-wheel mecanum forward/lateral/angular control mapping,
  validation, four wheel-rate bounds and exact SE(2) constant-body-twist
  motion with signed body lateral travel.
- [x] Introduce 14 cached cardinal, diagonal, lateral+turn, forward+turn
  and spin primitives; use model-aware body-motion sampling bounds.
- [x] Integrate a `MecanumPolicy` with the existing W11 shared Hybrid A*
  kernel and strict collision checker, no Ackermann steering branches.
- [x] Expose mecanum selection and holonomic path JSON in Planning Lab,
  browser and CLI, with Catch2 and HTTP regression tests registered.
- [x] Raise the library, demo scenarios and browser default expansion ceiling
  to 1,000,000, with up to 4,000,000 generated nodes; keep explicit
  per-request budget overrides.
- [x] Introduce bounded, independently validated mecanum exact terminal
  connectors, strict-resolution search samples and positive yaw-cost lower bound.
- [x] Register off-lattice exact-terminal and reported warehouse obstacle
  regressions in W12 Catch2.
- [ ] Execute the **exact-checkout** Catch2/CTest, Docker-OMPL, sanitizer and
  browser verification matrix; the new W12 regressions are **not yet verified**
  as passed. Profile high-expansion/no-path search memory and wall-clock costs.
- [ ] Add model-aware optimization, actuator/slip verification and eventually
  generalized multibody state and geometry, only with separate acceptance
  evidence.

See [W12 mecanum implementation](architecture/w12-mecanum-holonomic.md).

## W11 model-policy Hybrid A* extraction — 2026-10-26

- [x] Introduce C++20 `PlanarSearchModel` policy for state/key/hash,
  immutable primitive catalog, successor section, incoming mode,
  geometric footprint, cost and model option validation.
- [x] Compile one typed search loop for Ackermann/differential drive,
  removing chassis branches from the per-node successor hot path.
- [x] Add public `planWithPolicy` adapter; test identical Ackermann and
  differential-drive outcomes and a third standalone pivot-only policy.
- [x] Reject zero/nonfinite third-party edge costs and unverified legacy
  analytic/clearance hooks; require exact section endpoint/continuity
  checks before a model-provided path can be accepted.
- [x] Register W11 Catch2 equivalence, custom footprint, invalid policy,
  and dishonest-endpoint regression tests.
- [ ] Run exact GitHub checkout with Catch2/Docker/OMPL and sanitizers;
  source-level verification is not a release build.
- [ ] Expand model state and key dimensionality beyond SE(2), with
  model-supplied multibody frame transforms and compound collision geometry.
- [ ] Add lateral holonomic/mecanum geometric sections, not only rigid
  Ackermann arcs and zero-translation turns; verify cost positivity
  and independent full-body motion feasibility.

See [W11 model-policy implementation](architecture/w11-model-policy-kernel.md).

## W10 differential-drive and precomputed primitives — 2026-10-26

- [x] Introduce shared local SE2 constant-twist primitives with model-aware
  translating/metres and stationary-rotation/radians extents, normalized
  progress, complete Pose2D samples and continuous analytic evaluation.
- [x] Implement a differential-drive `(x,y,yaw)` model with independent
  commanded `v,omega`, wheel-rate feasibility and eight precomputed
  straight/arc/spin templates.
- [x] Build six Ackermann relative primitives once per plan and transform
  their **cached local endpoints and collision samples** at each search
  expansion, retaining independent analytic strict final validation.
- [x] Add Catch2 analytic-vs-independent-RK4, SE2 transform, wheel-limit,
  sampled rotating-body collision and Ackermann search regression tests.
- [ ] Complete official dependency-provisioned Catch2/Docker regression
  matrix on an exact checkout (not available in this environment).
- [x] Integrate differential-drive primitive catalog into the existing
  Hybrid A* successor loop, carry the incoming gear across turns, and add
  positive angular effort cost with stricter independent rotation checks.
- [x] Expose the model in Planning Lab and CLI, with a distinct rectangular
  footprint, spin-marker drawing, heading animation and HTTP regressions.
- [ ] Generalize the remaining Ackermann-shaped API/types into a
  model-parametric state/collision/motion policy, then extend beyond
  planar rigid chassis and circle/rectangle geometry as needed.

See [W10 primitives contract](architecture/w10-differential-primitives.md)
 and [W10 search and demo integration](architecture/w10-differential-search.md).

## W6 hash/collision/queue hotpath instrumentation — 2026-10-26

- [x] Instrument label map finds/hits/inserts/updates/rehashes, hash calls,
  exact full-footprint collision evaluations, search/final arc work and
  both search queues with compile-time opt-in nanosecond timers.
- [x] Avoid redundant hashing on improved-key replacement by writing
  through the existing label iterator; cache immutable map-origin trigonometry.
- [x] Add opt-in label-table reserve and controlled comparison with the
  conservative 4-disc collision prefilter.
- [x] Register Catch2 reserve/collision invariants and a manual hard-scenario
  p50/p95 benchmark with profiled versus nonprofiled runs.
- [ ] Execute exact checkout Catch2/Docker build plus profiled and
  profiling-OFF benchmark on one representative host; establish actual CPU
  hotspots and RSS before promoting any reserve/prefilter configuration.

See [W6 hotpath profiling guide](architecture/w6-hotpath-profiling.md).

## W6 priority-queue profiling — implemented 2026-10-26

- [x] Instrument both Hybrid A* open and lazy reverse-grid frontiers for
  pushes, pops, stale pops, peak live entries and approximate buffer storage.
- [x] Implement independent opt-in binary lazy, four-ary lazy, indexed
  binary and indexed four-ary variants with identical deterministic order.
- [x] Avoid full `Node`/`MotionSection` copies when expanding children;
  retain a safe small snapshot instead of holding invalidatable references.
- [x] Benchmark all queue combinations on seven deterministic hard maps,
  checking expansion/path/cost equivalence and reporting p50/p95 and memory.
- [x] Add Catch2 priority/decrease-key and all-scenario equivalence tests;
  local standalone and Clang sanitizer harnesses pass.
- [x] Retain the binary-lazy default: indexed alternatives remove stale
  entries but can cost more memory and latency, particularly for grid cells.
- [ ] Run exact-checkout Catch2/Docker suite and repeat performance trials
  on product hardware before changing a queue default.

See [priority-queue verification](architecture/w6-priority-queue-verification.md).

## W3/W6 planner milestones — implemented 2026-10-26

- [x] Share seven deterministic reverse/parking/three-point/narrow-corridor/
  dead-end/detour/impassable-gap fixtures across Catch2 and manually run
  benchmark; classify successes, finite work-limit outcomes, reproducibility,
  map provenance and independent strict path revalidation.
- [x] Compare certified primitive-only Euclidean and noncertified
  obstacle-grid search with identical map/pose/footprint/budgets.
  Conditional OMPL/Reeds–Shepp modes and bounded CSC-refinement rows are
  included in both regression tests and CSV benchmark.
- [x] Seed grid heuristic from tolerance-disk cells in its clipped bounding
  box rather than scanning all map cells; preserve free tolerance fringe seeds.
- [x] Apply request-global reverse Dijkstra settlement budget, finite fallback
  for uncomputed/blocked cells, and default diagonal corner prevention.
  Expose diagnostic counts and keep certified Euclidean mode unchanged.
- [ ] Run exact-checkout Catch2 and full installed OMPL/Docker matrix, then
  gather representative multi-run p50/p95/p99 performance data before
  enabling the noncertified heuristic by default.
- [x] Investigate both failures: dead-end's original goal collided with
  its back wall; corrected reverse-escape fixture succeeds. The long detour's
  7k grid-work cap exhausted before producing useful search guidance; 20k
  grid settlements plus opt-in heuristic weight 1.2 recover a strictly
  validated route within the original 8k expansion limit.
- [x] Check 18 XY-bin/primitive-length/gear-switch variants; none solved
  detour under the original 7k grid cap. Do not infer one-label dominance
  is globally harmless from this narrow diagnostic.
- [ ] Design independent feasible narrow-passage fixtures and diagnose
  single-label dominance with a multi-arrival reference under bounded
  memory/time constraints before considering a search-state change.

See [W3/W6 failure root-cause investigation](architecture/w3-w6-failure-investigation.md) and [W3/W6 hard-scenario evidence](architecture/w3-w6-hard-scenarios.md).

## Planning Lab obstacle editor — implemented 2026-10-26

- [x] Add blocked/free/unknown/restore brushes, configurable radius and
  drag painting on built-in/PGM/YAML maps, with rotated local-cell indexing.
- [x] Invalidate old path evidence on map or pose changes and trigger fresh
  planning on pointer release or edit reset.
- [x] Apply bounded sorted edit records to a **new** request-local
  `MapSnapshot` at a new revision; retain original map and previous
  validated results without mutation.
- [x] Add a bounded `POST /api/plan` form endpoint (64 KiB body, maximum
  4096 changes) while keeping GET planning backward compatible.
- [x] Register Catch2 edit/revision/collision tests, Node brush math tests,
  and HTTP regression coverage for collision, restoration and malformed input.
- [ ] Execute the newly registered Catch2 + HTTP CTest suites with a
  Docker-provisioned build; the local source/parser and Node probes alone
  are not full integration certification.

## Next Planning Lab increment — rotated ROS YAML (2026-10-26)

- [x] Load flat ROS map-server YAML with a sibling/relative PGM image.
- [x] Respect resolution, translated/rotated origin, negate and trinary
  occupancy thresholds; preserve unknown cells conservatively.
- [x] Transform C++ default start/goal and browser map/world XY
  consistently. Add a checked-in 90-degree sample map descriptor.
- [x] Register Catch2 YAML/PGM threshold tests and Node geometry
  round-trip tests (Node also provisioned by Docker).
- [ ] Execute new Catch2/Docker-OMPL matrix on a Docker host; direct local
  Clang sanitizer, GCC planner and Node geometry probes already passed.

## Subsequent Planning Lab application increment — 2026-10-26

- [x] Add P2/P5 parser bounds, orientation/unknown-occupancy Catch2 tests,
  and live imported-map server, CLI and malformed-input regressions.
- [x] Add automatic custom-map option in the browser and JSON export control.
- [ ] Run new Catch2 and full Docker-OMPL matrices on a host with provisioned
  dependencies; local standalone GCC/Clang sanitizer probes and HTTP checks
  are not substitutes for a real Catch2/Docker run.

## Docker and Catch2 test framework — implemented 2026-10-26

- [x] Reorganize developer applications under `apps/demo/`; remove the legacy
  root `app/` directory and update all CMake/includes/docs references.
- [x] Add multi-stage Ubuntu 24.04 Dockerfile and Compose configuration,
  with pinned **OMPL 2.0.2**, Eigen3, Boost, FLANN, ODE, Python, Ninja and
  **Catch2 v3** dependencies.
- [x] Expose separate manual Docker `demo` and `test` targets with host
  loopback-only port publication; retain native loopback binding as default.
- [x] Wire Catch2 v3 as the actual C++ test runner; default CMake builds
  now use pinned v3.16.1 via `FetchContent`, with explicit opt-in
  `HYBRID_ASTAR_USE_SYSTEM_CATCH2=ON` for an installed CMake package.
  Native tests use Catch2 cases and `INFO`/`REQUIRE` assertions.
- [x] Add native Catch2 `SECTION` fixtures for kinematic propagation and
  strict validated path evidence.
- [x] Configure an opt-in `workflow_dispatch`-only GitHub Actions workflow,
  with one selected suite (`quick`, `sanitizers` or `ompl`) per run, bounded
  parallelism, short job timeouts and no automatic triggers.
- [ ] Activate the manual workflow via a standalone workflow-file change on
  `main` (GitHub requires workflow-dispatch files on the default branch).
- [ ] Run the W11/W12 full Catch2/CTest matrix and the OMPL/Docker matrix
  using manual GitHub Actions selections; neither has been dispatched yet.
- [ ] Continue splitting larger legacy Catch2-wrapped test bodies into
  independently named `TEST_CASE`/`SECTION` fixtures for finer diagnostics.

See [Docker/testing guide](docker.md) and [demo usage](../apps/demo/README.md).

## Demo application milestone — implemented 2026-10-26

- [x] Create separate `apps/demo/` directory and opt-in `HYBRID_ASTAR_BUILD_APP`
  CMake target without changing core-only builds.
- [x] Add local loopback C++ HTTP/CLI demo, five occupancy scenarios,
  start/goal yaw editing and interactive click-to-place world coordinates.
- [x] Visualize the real exact-arc Hybrid A* path, forward/reverse sections,
  sampled footprint animation, weighted cost, search effort, planning time,
  strict success/failure, and optional original/refined comparison.
- [x] Support optional real OMPL RS and noncertified performance heuristic,
  bounded validated-incumbent search and post-search refinement.
- [x] Add native demo regression and no-dependency optional Python localhost
  API smoke test for scenarios, HTTP assets/errors and invalid input.
- [x] Add bounded P2/P5 **static occupancy PGM import** with ROS-compatible
  top-row-to-positive-Y conversion and planner-pinned map provenance; support
  a built-in sample map and custom server startup map selection.
- [x] Export planning output as CLI JSON (`--once --output`) or browser
  download, including motion sections, provenance, sampled collision evidence,
  termination reason and explicit lack of actuator feasibility.
- [x] Implement restricted ROS map-server YAML (PGM, trinary mode) with
  `image` resolution, world-frame origin/yaw, negate and thresholds.
  Custom-map default poses and browser XY conversions now honor rotation.
- [ ] Add comprehensive YAML features, PNG raster, frame metadata and
  ROS OccupancyGrid integration only with a separately validated schema.
- [ ] Add downstream timed trajectory and steering-actuator visualization
  only after those models are actually implemented and validated.
- [ ] Verify the browser UI on target operating systems and build against a
  complete installed OMPL package and exact GitHub checkout.

The standalone app is a **developer tool**, not a ROS/Nav2 integration or a
certified safe vehicle execution controller. See [apps/demo/README.md](../apps/demo/README.md).

# Implementation and architecture TODOs

A prioritized, incremental checklist. **All tasks unchecked as of initial documentation baseline**; this is a proposed plan, not a report of completed code. See [architecture index](architecture/README.md) and [open questions](architecture/open-questions.md).

## M0 — Review and accept architecture baseline

- [ ] Review [requirements](architecture/requirements.md): correct scope, vehicle models, non-goals.
- [ ] Review [components](architecture/components.md): dependency direction, ownership and map snapshots.
- [ ] Review [interfaces](architecture/interfaces.md): state/control types, primitive/domain, indexed labels, path evaluation.
- [ ] Review [controlled-ODE model specification](architecture/kinematic-models.md) for derivative, tangent, constraints, body transforms and integration semantics.
- [ ] Review [ROS-compatible map coordinate specification](architecture/map-coordinates.md) and [ADR-0009](adr/0009-ros-map-coordinate-conventions.md); settle the native occupancy/frame representation (OQ-023).
- [ ] Confirm outdoor world frame uses local ENU (+X east, +Y north, +Z up) with explicit geodetic reference when GNSS is used; indoor frame may be building-aligned.
- [ ] Review [ADR-0008](adr/0008-motion-model-capabilities.md): model control constraints, ideal mecanum as third reference and holonomic travel semantics.
- [ ] **Later model milestone:** confirm supplied four-state articulated-steering ODE input interpretation, joint/body frames and link lengths (OQ-022); this is **not** a first Ackermann MVP prerequisite.
- [x] Accept OQ-001 **single retained node per model-specific discretized key** (ADR-0013, superseding ADR-0010); [x] accept OQ-003 canonical `(segmentIndex, lambda)` path position (ADR-0011).
- [ ] Benchmark single-key pruning against feasible-path recovery and post-refinement quality in narrow passages; revisit extra arrival labels only if evidence warrants a separate future decision.
- [x] Accept OQ-002 geometric length for translation and angular/configuration extent for stationary motion (ADR-0012).
- [ ] Define only the initial Ackermann fixed-control primitive extent and exact closed-form arc execution contract needed for the first vertical slice (OQ-002). Defer generalized typed extent APIs and OQ-024 downstream speed-profile feasibility/retiming unless a concrete test reveals a blocker.
- [x] Retain the historical optional **combined RS+obstacle-field noncertified performance mode** alongside the current **certified Euclidean goal-region default**, with truthful reporting (ADRs 0015,0027; current contract supersedes their older default choice).
- [x] Accept OQ-007 three-state fixed-curvature Ackermann primitives (ADR-0014).
- [x] Accept OQ-008 pose-tolerance default of 0.25 m / 10 degrees with optional extra-state goal constraints (ADR-0016).
- [x] Accept OQ-006 unknown-cell semantics/policy and known-free frontier default (ADR-0017); selectable per request, map data unchanged.
- [x] Accept OQ-006 configurable known-cell max occupancy, default zero (ADR-0018).
- [x] Accept OQ-006 immutable full-grid shared snapshots with atomic publication, revision-keyed on-demand bounded caching and strict full-snapshot `Reject` validation (ADR-0019).
- [x] Accept **lean MVP and evidence-driven complexity**: rigorous contracts/tests, early working vertical slice, defer speculative modules (ADR-0020).
- [x] Clarify intended **search → downstream path optimization** boundary: search is a collision-checked, model-feasible path initializer; evaluate quality after refinement rather than overengineering raw search, with independently validated optimized output and raw-path fallback (ADR-0020).
- [x] Accept **obstacle-aware optimization whenever refinement is enabled**: use map obstacles/clearance during optimization, pinned snapshot and occupancy policy, full transformed collision bodies and independent final validation (ADR-0024).
- [x] Accept initial Ackermann search cost weights, resolution/gear key, **exact closed-form primitive** and collision sample defaults (ADR-0025; historical RK4 wording superseded); further model-generalized effort costs remain OQ-004/OQ-024.
- [ ] Finish native occupancy/frame/handle ABI (OQ-023) and mutable cache concurrency (OQ-012) only to the extent necessary for first runnable MVP.
- [ ] Agree [quality metrics](architecture/quality.md), reference hardware/scenarios and initial budgets where feasible.
- [ ] Review [verification matrix](architecture/verification.md), Catch2 property and oracle tests.
- [ ] Accept/revise ADRs and mark architecture baseline v0.1 reviewed.

**Exit:** contracts and initial assumptions agreed; no unresolved blocker for M1.

## MVP integration priority (ADR-0020)

- [ ] Define the smallest useful end-to-end supported robot/map/scenario slice with explicit validation and diagnostic contract; **do not block it on optional advanced designs**.
- [ ] Run the selected model, map snapshot, primitive propagation, collision, search, and exact geometric-path reconstruction together against deterministic fixtures as early as the foundational modules permit.
- [ ] Expand to other reference models and optional refinement only with targeted model/geometry verification; mark unsupported or partially supported features honestly.
- [ ] Defer multi-label search, speed-state search, speculative effort optimization, incremental heuristic repair, map tiling and sophisticated concurrent-cache machinery until real scenario evidence motivates them.
- [ ] Once optional refinement is available, compare search design choices on **final validated output** and total search+refinement latency; preserve feasibility, route/maneuver choice and original-path fallback, not expensive raw-path smoothness.
- [ ] Log integration failures with reproducing fixtures and compare planning latency, memory, validity and final path quality before adopting new mechanisms.

## M1 — Standalone foundation (no search yet)

- [ ] **Minimum compiling C++20 CMake library** with focused Ackermann-native pose/config/core types, CTest smoke tests and **isolated OMPL Reeds–Shepp steering adapter**. Do not design all future generic C++ model ABIs before first Ackermann slice; omit ROS dependencies.
- [ ] Format/lint/test/benchmark commands documented for local use.
- [ ] Common types: poses, transforms, frame ids, angles, units, result/status.
- [ ] Immutable validated native configuration; YAML (yaml-cpp) and JSON (nlohmann/json) adapters.
- [ ] Immutable owning occupancy grid snapshot and atomic publication, REP-103/105 frame metadata, SE(2) origin/world-grid conversions, row-major X-fastest layout, per-request unknown/out-of-bounds policy and source identity/revision (ADRs 0017–0019).
- [ ] Convert ROS OccupancyGrid signed occupancy cells explicitly and disallow raw Nav2 uint8 costmap reinterpretation.
- [x] Accept independent frame-attached multibody collision geometry and model-provided frame transforms, supporting geometric trailer chains (ADR-0021).
- [ ] Implement circle/rectangle/convex-polygon collision shape primitives and immutable multibody sets; validate model frame references and compound shape offsets. Exact registry and self-collision pair rules remain OQ-014.
- [ ] Catch2 tests for foundational types, serialization and ROS-compatible grid semantics: zero/rotated origins, half-cell offsets, negative/border coordinates, unknown occupancy, frame mismatches and Y-axis direction.
- [ ] Validate frontier known-free full-footprint traversal, three request-level unknown policies, default max-traversable-known-occupancy `0`, threshold equality, optimistic-path assurance and cache-policy separation.
- [ ] Implement accepted atomic publication of privately built immutable shared map snapshots, revision-keyed on-demand cache rebuild with bounded eviction, and up-front full-snapshot strict `Reject` validation (ADR-0019); define exact API types and synchronization at implementation review.
- [ ] Catch2 outdoor ENU axis/yaw fixtures, with independent rotated-grid origin; optional adapter tests for geodetic datum/reference and GNSS-to-local conversion.
- [ ] cxxopts-based CLI fixture runner (optional initial example).

**Exit:** standalone core configure/build/test succeeds locally.

## M2 — Kinematics, primitives and collision

- [ ] Differential-drive model including in-place rotation; compile-time concept checks.
- [ ] Three-state geometric bicycle/Ackermann model with exact fixed-curvature left/straight/right (optional intermediate) forward/reverse arcs; preserve joins and annotate finite steering-rate assumptions (ADR-0014).
- [ ] **Later generic-model milestone:** ideal mecanum reference model: independent chassis body lateral/forward/yaw velocities, admissible bounds, concept tests and analytic propagation fixtures; **not a gate** for first Ackermann search.
- [x] Accept RK4-based primitive generation to prescribed actual traveled length (terminal iterative refinement when required), retained ordered full-state visualization/debug points, and search-facing instantiation of cached evaluable motions (ADR-0023).
- [x] Accept **0.50 m** primitive length, **0.05 s RK4** at reference **1 m/s**, **0.0001 m** arc-length tolerance, **0.10 m** stored-sample spacing, and six forward/reverse left/straight/right controls for Ackermann (ADR-0025).
- [ ] Define the controlled ODE/tangent generator interface and build the native RK4 integration plus accumulated distance, failure/tolerance handling; use analytic constant-curvature formulas as tests and optional later optimized evaluators.
- [ ] Model-specific control enumeration, primitive templates and cache invalidation; distinguish SE(2)-equivariant reusable catalogs from initial articulation-state-dependent generation.
- [ ] Implement OQ-002 RK4-generated traveled-length translating primitives plus model-aware angular/configuration extents for stationary maneuvers; keep full control-law numerical evaluation across generated checkpoints, including reverse/stall guards, and ensure no division by longitudinal speed at zero.
- [ ] Implement **single retained continuous node per model-defined discrete key** with stable parents, incoming primitive metadata, reopen/stale-queue handling and no multi-label storage (ADR-0013).
- [ ] Deterministic key replacement/pruning tests, including loss of alternative continuous arrivals and history-dependent transition-cost approximation.
- [ ] Geometry frame transforms from hypothetical state; add a tractor/two-trailer synthetic state-transform fixture without assuming a production ODE; retain the supplied four-state articulated-steering model fixture only after its physical frame conventions are confirmed.
- [ ] Implement articulated steering derivative with safe denominator checks, articulation state indexing, rate constraints and zero-translation articulation sweep tests (extension milestone, not blocking differential/Ackermann).
- [x] Keep all six stored 0.10 m-spaced collision poses per initial 0.50 m Ackermann primitive and adaptive subdivision disabled by default; narrow-phase geometric overlap implementation **updated to default raster boundary, **optional** interior guard and conservative hull mask** (ADRs 0025,0029).
- [ ] Implement **conservative supercover rasterized footprint edges by default** and **optional** scanline-filled interior blocked-cell checks behind `CollisionCheckOptions.checkFootprintInterior=false` default; support circle/compound geometry, unknown and map-boundary policies. Distinguish weaker boundary-only from complete-footprint sampled coverage (ADR-0029).
- [ ] Precompute orientation-hull **collision-possibility grid** from pinned occupancy map, robot geometry and continuous heading/position bin margins; lookup at **each stored primitive or RS segment pose**. False means provably clear; true triggers default boundary raster checking and optional interior checking; the flag itself **does not** turn boundary-only results into complete-footprint validation. Use the nearest-obstacle EDT **only for the MVP collision-candidate mask**, enforcing map/policy/geometry cache invalidation; defer 0.40 m clearance-influence flags and EDT path-cost lookup to low priority (ADR-0029).
- [ ] Implement candidate rejection at every **stored full-state primitive sample** using conservative hull lookup and rasterized actual footprint (with interior only when requested), not a separate collision schedule; optional adaptive additional states only if enabled. Report independent **boundary-only/full-footprint coverage** and sampled-trajectory assurance.
- [ ] Build a nearest-obstacle 2D EDT per pinned map/policy **for the conservative collision mask only** (not the reverse-Dijkstra-to-goal field). Verify subcell and boundary margins, EDT cache validity and mask performance (ADR-0029).
- [ ] Numerical, collision and maneuver property tests including mecanum lateral/combined motion; baseline microbenchmarks.

**First Ackermann vertical-slice exit:** the 3-state Ackermann model generates six tested primitives and poses; sampled collision works on fixture grids. Differential-drive, mecanum and articulated model reference tests remain generalization work **after** this first end-to-end planner (ADR-0020).

## M3 — Steering and heuristics

- [ ] **OMPL-backed Dubins adapter:** provide distance/interpolation/segments behind native project API; defer full native solver (ADR-0028).
- [ ] **Required for first Ackermann end-to-end MVP:** OMPL-backed Reeds–Shepp adapter for signed straight/arcs, path length, interpolation and cusps; self-authored solver low priority (ADR-0028).
- [ ] Segment/direction representation, robust normalization and degenerate handling.
- [ ] Validate OMPL steering adapter against analytic endpoints, invariances, signed segments and cusp/degenerate fixtures; optional independent native solver comparison only if later implemented (ADR-0028).
- [ ] **Required in first Ackermann search:** implement **lazy, cached, multi-source reverse Dijkstra**, seeded from all traversable XY cells intersecting the goal tolerance region. Keep contiguous settled/tentative distance and status arrays plus reusable frontier/heap, expand only to settle requested cells, with explicit bounded-work fallback and cache compatibility (ADR-0027).
- [ ] Combine OMPL RS distance with holonomic obstacle-aware goal-rooted reverse Dijkstra by `max` (ADR-0027). Seed **all traversable grid cells intersecting the XY goal tolerance disk**, even if the nominal center cell is blocked; preserve independent heading/actual footprint checks at terminal states. Store settled vs unavailable/disconnected field states (not a fabricated infinite score), and retain the unfinished frontier for compatible replan/cache hits and fall back to **finite RS/Euclidean** without prematurely pruning Hybrid A*. Expose uncertified performance mode and strict certified fallback; test nonzero goal tolerance/grid overestimation.
- [ ] Test lazy-field cache hit/reuse and frontier persistence across repeat plans; enforce **one active plan() per planner instance**, fail fast with Busy on overlap, and retain the instance-owned mutable cached frontier. Independent instances may share immutable map snapshots. Decide/test 8-neighbor diagonal corner policy, exact map-meter weights and finite heuristic settlement budget. Profile OMPL distance-only calls and combined-heuristic preparation/search time.

**Exit:** OMPL steering adapter passes independent analytic/invariance tests, combined heuristic is implemented with explicit guarantee mode, and obstacle distance-field checks pass grid fixtures.

## M4 — Hybrid A* baseline

- [ ] Node pool/open heap, stable indices, stale entry strategy, parent/action records.
- [ ] Generic search loop with primitive expansion and **required OMPL Reeds–Shepp analytic expansion with first-node and adaptive retry scheduling** for geometric Ackermann (ADRs 0026–0028).
- [x] Accept Ackermann **0.25 m XY / 5° yaw (72 bins) + incoming gear** search key, reverse multiplier **1.10**, and gear-switch **0.50 m** for MVP edge costs. Defer the previously proposed **0.15/0.40 m EDT-based clearance penalty** to a low-priority opt-in extension (ADRs 0025,0029). Combined RS/2D obstacle heuristic remains required (ADR-0027).
- [ ] Implement **MVP edge costs only:** equivalent-meter forward length (1.00), reverse length (1.10) and 0.50 m per direction switch (none for first choice). Apply the same active weights to OMPL Reeds–Shepp straight/arc sections and internal/incoming gear switches. Do **not** compute any clearance integral or per-pose nearest-obstacle EDT score. Tie breaks, reopen/dominance, and validated feasible-first analytic termination remain required.
- [ ] Cancellation and time/expansion/memory budgets, structured diagnostics.
- [ ] Exact path reconstruction and validated piecewise path output using canonical `(segmentIndex, lambda)` evaluation; complete state and one-sided cusp semantics.
- [ ] Validate continuous goal tolerance and complete sampled motions. **No near-goal XY gate:** implement a per-request Nav2-style Reeds–Shepp scheduler that tries the first expanded node, then retries periodically with `analyticExpansionRatio` (provisional 3.5) based on closest observed finite heuristic in **map-cell-equivalent units**. Each scheduled attempt is one complete OMPL Reeds–Shepp path (signed sections/cusps). Validate at ≤0.10 m per segment, finite model/goal/collision/cost, optional disabled-by-default weighted-cost cap, and full prefix reconstruction; never cap connector length. Failed attempts continue ordinary Hybrid A*. Include attempt counts, retry-cadence diagnostics and ratio tuning benchmarks (ADR-0026).
- [ ] **First Ackermann scenario tests:** open field, wall/doorway, narrow turns, reverse parking, blocked exact goal with a usable tolerance-region goal and unreachable goal. Add mecanum sideways travel and other model-specific fixtures in later generic-model milestones.
- [ ] Compare geometric raw versus **finally validated refined** path planning outcomes and total pipeline latency, including paths lost to single-label key pruning, without promising refinability of missing homotopy classes.
- [ ] Local latency/memory/quality baseline with and without obstacle-aware heuristic.

**Exit:** model-independent, validated baseline paths with measured performance.

## Low-priority backlog — optional search clearance scoring

- [ ] **Only if justified after benchmarking:** add an explicit opt-in EDT-based footprint-boundary clearance edge cost with coefficient **0.15** and influence radius **0.40 m**. Reuse collision-preprocessing EDT, derive an additional influence mask, read actual oriented footprint raster cells, evaluate five trapezoidal intervals for a 0.50 m six-sample primitive, and score RS connectors consistently. This occurs after collision testing and before `g` relaxation, **not during primitive precomputation** (ADRs 0025,0029).
- [ ] Benchmark resulting total planner/search+refinement latency and final validated path quality against the zero-clearance-cost MVP; retain only if it improves the product.

## M5 — Path representation and optional refinement

**Current W8 increment:** `ackermann::refinePath` implements opt-in bounded
section-boundary shortcuts, opt-in native fixed-gear CSC curves, signed-provider injection, recomputed weighted
cost and independent strict full-path validation. Invalid proposals keep the
verified original. See [W8 refinement verification](architecture/w8-refinement-verification.md).
A curvature-continuous (steer-rate-aware) smoother, complete RS/CCC alternatives, real-map quality benchmarks,
and actuator feasibility checks remain separate follow-up tasks.


- [ ] Segment-level continuous evaluation using canonical `(segmentIndex, lambda)`, derivative availability, progress/cusp/rotation semantics, and lateral holonomic motion without heading-as-tangent assumptions.
- [ ] Define external speed-profile input requirements: segment derivatives, rate/acceleration/jerk limits, start/end speeds and mandatory stop conditions (OQ-024).
- [ ] Validate final timed trajectory against model actuation limits and moving-obstacle constraints; reject/adjust/replan if a geometric path cannot be executed safely.
- [ ] Exact start/end full configurations and map/model provenance.
- [ ] Native curve math (B-spline / Hermite etc.) without mandating one family in API.
- [ ] Implement **Stanford Junior-style obstacle-aware optimization** (ADR-0028) with independently validated full-body output, preserving raw path fallback. Benchmark against Nav2-inspired simple smoothing in total pipeline latency and final quality; optional Ceres remains private (ADR-0024).
- [ ] Curvature, steering/articulation, endpoint and **pose-sampled collision validator** with explicit non-continuous assurance.
- [ ] Independently validate model, endpoints and full-body collision of optimizer output under the selected sampled-assurance level; return untouched original path upon failed optimization, exhausted budget or collision (ADR-0024).
- [ ] Controller-facing evaluation/projection benchmarks.

**Exit:** refined path never silently displaces valid raw path.

## M6 — External integration (later)

- [ ] YAML/JSON example configs for differential, Ackermann, ideal mecanum and articulated geometry.
- [ ] Standalone examples and CLI traces using cxxopts.
- [ ] Optional ROS 2 adapter and Nav2 plugin without ROS headers in core.
- [ ] Rich path output preserving segment metadata beyond `nav_msgs/Path`.
- [ ] Integration test with separate MPC/trajectory planner contract (no MPC inside core).
- [ ] Release/package documentation, license and dependency notices.

## Working agreements

- Implement on feature branches with **small commits pushed after meaningful progress**; do not merge without review.
- Run local tests/benchmarks first; manually dispatch GitHub Actions only on
  explicit request and select the smallest applicable suite.
- No automatic workflow triggers (`push`, `pull_request`, `schedule` etc.) unless explicitly requested.
- Keep requirement IDs, interface docs, ADRs, verification and code changes synchronized.
