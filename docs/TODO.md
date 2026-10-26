> **2026-10-26 implementation update:** The external-review [consolidated
> specification](architecture/Hybrid_AStar_Consolidated_Specification.md)
> has been imported. The normative current behavior and remaining W0-W9 gates
> are tracked in [current-implementation.md](architecture/current-implementation.md).
> The current Ackermann primitives use exact constant-curvature integration, not RK4. This older checklist and ADR references may predate the verified vertical
> slice and must not override the current contract. **Still pending:** full OMPL package/export build and exact-checkout
> verification (the real OMPL 2.0.2 source-runtime tests now pass, W5),
> performance-grid benchmarks and any reachability proof (W6),
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

## Demo application milestone — implemented 2026-10-26

- [x] Create separate `app/` directory and opt-in `HYBRID_ASTAR_BUILD_APP`
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
- [ ] Add file-based map import and exporting structured path results when a
  real application map schema is chosen.
- [ ] Add downstream timed trajectory and steering-actuator visualization
  only after those models are actually implemented and validated.
- [ ] Verify the browser UI on target operating systems and build against a
  complete installed OMPL package and exact GitHub checkout.

The standalone app is a **developer tool**, not a ROS/Nav2 integration or a
certified safe vehicle execution controller. See [app/README.md](../app/README.md).

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
- Run local tests/benchmarks using available tooling; document limitations rather than dispatch GitHub Actions.
- No automatic workflow triggers (`push`, `pull_request`, `schedule` etc.) unless explicitly requested.
- Keep requirement IDs, interface docs, ADRs, verification and code changes synchronized.
