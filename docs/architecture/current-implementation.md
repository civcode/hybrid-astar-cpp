# Current Ackermann implementation contract

**Status:** initial executable, sampled-validation vertical slice, 2026-10-26. This
file is the **current implementation contract**, not a declaration that all
external-review recommendations are complete. The source synthesis is
[Hybrid_AStar_Consolidated_Specification.md](Hybrid_AStar_Consolidated_Specification.md).
The consolidated review labels Consensus, Recommended synthesis, Alternative,
and Decision required must remain distinct. ADRs 0013–0029 describe historical
choices and may no longer express the shipped defaults.

## What is implemented

The standalone C++20 `hybrid_astar::ackermann::plan()` accepts one
`shared_ptr<const MapSnapshot>` plus a fixed policy, footprint, goal and
request-local options. It performs:

1. Whole-map shape/value validation, including full-snapshot
   `UnknownPolicy::Reject`. Known cells are free through the inclusive
   threshold (default 0); unknown defaults to blocking; map exterior always
   blocks. A caller must **publish genuinely immutable** map storage, with a
   new object/revision for updates.
2. Full-filled-rectangle versus occupied **closed cell squares** using SAT,
   in map-local coordinates obtained by the inverse origin SE(2). Interior
   cells, contacts, out-of-bounds body corners and unknown cells cannot be
   skipped. A configurable **request-local conservative 3–5-disc (or 1–64-disc)**
   EDT pose prefilter is now available **OFF by default**, with uncertain
   classifications falling back to the exact checker; strict final validation
   never uses the prefilter. Boundary-only checking is not used by the planner.
3. Six exact closed-form rear-axle Ackermann controls: forward/reverse times
   left/straight/right at `1/Rmin`. Nonnegative progress is evaluable at
   arbitrary positions, including exact endpoints; the evaluator does not
   replace arcs with chords.
4. Continuous arrivals indexed by quantized XY/yaw/**incoming gear**.
   A readable search key, append-only ancestry, active-label replacement,
   stale-entry skipping and reopening preserve reconstructible parent chains.
   Lower g wins within one approximate key; this can suppress a feasible
   sub-bin arrival, so **no resolution-completeness or global-optimality
   claim** is justified.
5. Search queue ordered by finite `g+h`. The queue's default remains `std::priority_queue` with
   deterministic tie-breaking by `(f,h,node ID)`; optional binary/4-ary
   indexed variants are available for opt-in A/B testing. Both the open
   and the reverse-grid heap expose push/pop/stale/peak/storage diagnostics.
   Expansion copies only necessary scalar/pose/key fields rather than the
   stored motion section. **No queue alternative is globally enabled** on
   the exploratory hardware timing data alone. Default `h` is the certified
   goal-region Euclidean lower bound; an optional **noncertified** performance
   mode combines finite fallback, lazy goal-region-seeded reverse Dijkstra
   (settled distances only) and an optional injected RS exact-goal distance.
   No 2D graph can prune search as a `NoPath` proof. Edges use 1.0x
   forward, 1.10x reverse and +0.50m actual gear changes.
6. Required full-footprint checks at every search segment start and sample,
   including each endpoint. Spacing obeys BOTH reference spacing and a
   translation-plus-rotation body-point movement bound
   `(1 + Rbody*abs(curvature))*deltaS`. Required work exceeding the
   finite per-section sample limit returns `ValidationIncomplete`.
7. An **independent, stricter final resampling pass** on reconstructed actual
   motion sections: default 0.05 m reference and 0.05 m body-point movement
   ceiling, with its own `maxFinalSamplesPerSection` budget (default 10000)
   and cumulative `maxTotalFinalPoseChecks` budget (default 250000), both
   independent of the search sampler's limit. It calls exact collision directly and never trusts cached
   search classifications. Only a completed strict pass permits `Success`.
8. Optional scheduled signed analytic connector supplied as a native callback.
   It is attempted at the first eligible node and at search-bin-relative
   intervals; independently validates every section's direction, curvature,
   endpoint continuity, incoming/internal cusp costs, search collision and
   final whole-path collision. Invalid analytic candidates fall back to the
   primitive search; an optional incremental weighted-cost gate is available.
   `ReedsSheppEvaluator` in the **OMPL-enabled target** materializes signed
   OMPL sections using reusable per-instance state-space/scratch resources.
   The OMPL adapter now delegates signed section normalization to a core-native,
   independently regression-tested `decodeSignedSections()`, and provides
   `steering::planWithReedsShepp()` as an explicit opt-in complete-planning entry
   point with safely scoped provider callbacks. A focused true-OMPL-2.0.2
   source build now executes the actual RS implementation, passing 12000
   randomized pairs. A full installed OMPL CMake package build remains
   **unverified**. The additional `planAndRefineWithReedsShepp()` entry point
   reuses one evaluator for planning and independently checked refinement.
9. Optional `ImproveIncumbent` quality mode retains only strictly validated
   lower-weighted-cost paths for a bounded extra-expansion budget (illustrative
   default 2500; not benchmark-ratified). First-validated remains the default.
   The result reports solution quality, candidate/improvement counts, and the
   reason for terminating continued search. A previously verified incumbent
   can be returned on count-budget exhaustion, validation-budget exhaustion,
   or post-incumbent cancellation with explicit termination metadata.
10. Optional **exact-oriented-rectangle** nearest blocked-cell-square/map-edge
    clearance score is integrated using nonnegative trapezoidal arc-progress
    penalties. It applies equally to ordinary primitives and signed analytic
    sections, is computed before active-label relaxation, and defaults to zero.
11. Opt-in post-search `refinePath(request, validatedPlan)` in
    `ackermann/refinement.{hpp,cpp}` tries deterministic exact straight
    section-boundary shortcuts, opt-in fixed-gear curvature-bounded
    left/straight/right circle-tangent CSC candidates, or an injected signed
    analytic provider.
    It independently revalidates the original against the pinned request,
    reconstructs/rechecks every candidate arc and the full candidate path
    using exact filled-footprint collision and the independent strict sampling
    limits, and accepts only reduced configured weighted cost. Finite attempt,
    accepted-replacement and total pose-check budgets bound work. On bad
    candidate, cancellation or budget limit AFTER original validation, the
    last strictly verified path remains available. Failed original validation
    cannot yield a purported success. New quality metadata distinguishes
    `FeasibleAfterRefinement`. This is not a time-continuous swept proof.
12. Explicit statuses for input/map/unknown errors, cooperative cancellation, start collision,
   work budgets, incomplete/failed final checking and approximate
   search exhaustion; true terminal pose/error/gear, weighted path cost,
   effective map provenance, counts and achieved *sampled* collision
   evidence are exposed. `trajectoryActuatorFeasible` remains false.

The request owns a const map pointer; the algorithm has no process-global
mutable cache and creates request-local nodes/queue/labels. Concurrency
requires callers to keep map contents immutable throughout requests.
The scheduler API now takes **search XY-bin meters**, not map resolution.
Legacy collision options default to full-footprint, while explicitly
boundary-only diagnostic mode cannot be used by `plan()`.

## Optional developer demonstration application

The independent `apps/demo/` **Planning Lab** runs the shipped Ackermann
`plan()` directly from a localhost-only C++20 HTTP/CLI executable.
Build with `HYBRID_ASTAR_BUILD_APP=ON` (default **OFF**); browse at
`http://127.0.0.1:8765`. Five preconfigured 50 × 40 m occupancy maps,
editable/clickable XY/yaw start and goal, online replanning, reverse/forward
section displays, full vehicle footprint playback, strict status and
cost/expansion/time readouts are supplied without any ROS/browser framework.
The mode can invoke strict `refinePath()`, native CSC shortcuts, optional
performance-grid heuristic and bounded improvement. If the separate optional
OMPL target is built, its signed Reeds–Shepp planning/refinement is available.
The native UI uses the library's actual returned exact arcs, not a distinct
browser pathfinding implementation. Local C++ demo and loopback HTTP smoke
tests are separately registered when the demo is built; Python 3 enables
the network smoke test where available. See [app usage](../../apps/demo/README.md).

A subsequent app-only extension supports local P2/P5 PGM import and
restricted ROS `mode: trinary` map-server YAML, including `image`,
`resolution`, `origin=[x,y,yaw]`, `negate` and occupancy thresholds.
Custom pose defaults, path visualization and canvas click world coordinates
now use the same nonzero map translation/yaw as the core planner.
The `apps/demo/maps/sample.yaml` fixture uses a 90-degree origin rotation;
the C++ loader and independent JavaScript transform have local tests.
Unsupported arbitrary YAML schemas/PNG/ROS message integration remain deferred.
The browser/CLI can export JSON but this is still not a timed ROS Path.

The browser now supports interactive **obstacle/free/unknown/restore**
brush editing in map-local coordinates (including nonzero YAML map yaw).
Each planning request uses a bounded, canonical cell-edit list, applied
to a new immutable `MapSnapshot` with a fresh browser edit generation.
Old path evidence is hidden immediately when map cells change; search,
strict final validation and optional refinement all use the copied revision.
A capped `POST /api/plan` URL-encoded form handles brush edits, while
existing unedited GET requests remain available. This is request-local
editing, not a transactional shared mutable map server. See
[edit protocol](../../apps/demo/README.md#interactive-obstacle-editing-immutable-map-revisions).

This is a **single-user developer demo**, not a dynamic-map/ROS/Nav2
integration or remote/public HTTP service. It advertises only the core's
static-map **adaptive-sampled geometric evidence**, not certified swept
clearance, timed steering/actuator feasibility or global optimum.

## Test framework and Docker development environment

C++ test executables now use **Catch2 v3** as their actual CTest-backed
runner and assertion framework; they are no longer individual handwritten
`main()` tests. The earlier raw-assertion vertical slice was dependency-minimal
but did not conform to the documented Catch2 selection. New core contract
tests use `TEST_CASE`/`SECTION`/`REQUIRE`; legacy suites retain their
existing checks through registered Catch2 cases and assertion helpers.

The root [Dockerfile](../../Dockerfile) and [Compose services](../../compose.yaml)
provision the compiler, **OMPL 2.0.2**, Catch2 3, Boost and Eigen for a
developer image. The default demo is now in `apps/demo/`; the `test`
service runs only when requested. See [Docker guide](../docker.md).
**Docker has not yet been executed here** because the engine is unavailable.
The relocated core-only demo has been compiled and smoke-tested locally.

## Important limitations (never advertise as implemented)

- **W5:** The signed connector and reusable OMPL evaluator are implemented.
  An application can explicitly supply its own `analyticConnector`, or use
  `steering::planWithReedsShepp()` from the optional OMPL target. Its second
  parameter optionally selects the noncertified RS-plus-grid heuristic.
  The native signed decoder and real OMPL 2.0.2 state-space code have now
  passed 12000 randomized configurations and integration tests through an
  offline source-subset harness. `steering::planAndRefineWithReedsShepp()`
  also passes. The full official **OMPL-ON package build remains unverified**.
- **W6:** Request-local raw-occupancy lazy reverse Dijkstra is an optional
  **noncertified** performance provider. It now seeds only cells in the
  goal tolerance disk's grid bounding box (not the whole map), blocks
  diagonal corner cutting by default, limits total settlements per request
  (`gridMaxTotalSettlements=20000`), and directly skips blocked target cells.
  Unavailable, disconnected and over-budget grid values use a finite fallback;
  there is **no** graph-based `NoPath` or footprint-aware reachability proof.
  New diagnostics count seed candidates, actual seeds and budget fallbacks.
  `performanceHeuristicWeight` defaults to 1.0 (valid [1,3]); setting it
  above 1.0 explicitly inflates **only** the noncertified heuristic to seek
  a quicker first path, potentially increasing cost. No weighted-A* solution
  bound is asserted due to grid relaxation and single-label pruning.
  One-label dominance and gear/reverse cost semantics are unchanged.
- **W7:** Conservative request-local blocked-center Euclidean distance transform
  and constructive whole-body disc covers are implemented as an optional
  search-only safe bypass with exact fallback, map-border checks and counts.
  Default is **off** pending independent proof review, randomized regression
  expansion and whole-planner benefit on representative maps. Never infer
  collision from disc overlap; **no continuous swept certification**.
  Adaptive samples still control *density*, not the continuous swept set.
- **W8:** Optional nonnegative oriented-footprint clearance integration,
  cost-gated analytic candidates and bounded validated incumbent improvement
  are implemented, defaulting to disabled/first-feasible. A separately callable
  **bounded post-path shortcut refiner** is now available, also opt-in, with
  exact obstacle-aware strict revalidation and raw-path fallback. It currently
  proposes aligned straight shortcuts by default. With explicit
  `RefinementOptions::useCurvatureBoundedShortcuts=true`, it also generates
  forward-only and reverse-only LSL/LSR/RSL/RSR candidates. Multi-cusp
  alternatives still require an injected provider. A certified nonnegative
  base-cost lower bound prunes uncompetitive candidates before strict checks.
  A bounded strict-validation retry and curvature-continuous smoothing
  remain deferred. The clearance
  experiment `lambda=0.15` caused a distant-doorway regression under the
  tested work budget: do not adopt it as a product default.
- **W9:** No timed steering/actuator checks, dynamic-obstacle predictions,
  ROS/Nav2 adapter, articulated/self-collision or alternative chassis model.
- Cooperative request-local atomic cancellation is implemented and polled
  during search, sample checks and strict validation. Strict final validation
  now has both a **separately metered per-section sample budget** and a
  cumulative global strict pose-check budget; any incomplete required
  checking without an incumbent prevents success. There is no wall-clock
  or allocator-level memory budget in this initial interface.
- Named tests cover open field, reverse costs, tolerance-region goals,
  unknown/boundary policy, adaptive-budget/cancellation and a deliberately
  missed between-sample obstacle rejected by strict validation. An opt-in
  **manual** scenario benchmark now compares exact-only, 4/5-disc covers and
  experimental clearance settings across forward/reverse and distant-doorway
  (region-Euclidean versus obstacle-grid ordering); neither is a complete
  parking/regression matrix, randomized geometric proof, validated latency
  threshold, full toolchain matrix or swept/actuator certification.

## Work-package traceability

| Package | State | Code and next exit gate |
| --- | --- | --- |
| W0 Baseline/tests | In progress | Contract, imported synthesis, named tests, manual `bench/planner_bench.cpp`; add reference hardware/Release and additional geometry fixtures |
| W1 Map/exact geometry | Implemented for filled Ackermann rectangle | `MapSnapshot`, `ExactCollisionChecker`, tests for rotated/boundary/unknown/interior; expand contact/property cases |
| W2 Ackermann motion | Exact arcs and reusable local templates implemented | `MotionSection` analytic evaluator, six request-local sampled templates transformed per search pose; independent final evaluation preserved; stress test numerical contacts/equivariance |
| W3 Minimal search | Deterministic difficult-map suite implemented | Seven reverse/parking/corridor/dead-end/detour/blocked-gap fixtures; certified and performance modes run repeatable finite tests, successful paths independently rechecked, budget outcomes explicit; exhaustive feasibility and global optimum still unproved |
| W4 Final validation | Independent per-section and cumulative strict budgets | Stricter exact resampling, weakened search fixture, final per-section and total pose-check budgets; validated incumbent retained on validation-budget exhaustion; failure location and swept proof pending |
| W5 Analytic RS | Genuine 2.0.2 source-runtime integration passes, package install unverified | Offline CMake state-space subset, 12000 randomized OMPL pose pairs, signed cusps, wall/heuristic integration, plan/refine workflow and sanitizer runs pass locally; complete OMPL CMake package/export and exact repository checkout remain pending |
| W6 Obstacle heuristic | Detour failure diagnosed and opt-in weighting added | Original long-wall cap of 7k settlements offered no useful relaxed-grid values; 20k plus weight 1.2 recovers validated detour in 3530 expansions at higher cost; zero-weight-change certified mode, bounded seeding/corner handling preserved; optimality unproved |
| W7 Disc/EDT | Opt-in prototype, disabled by default | Blocked-center exact EDT, conservative rectangle disc cover, strict exact fallback; 3/4/5-disc randomized differential tests and staged end-to-end benchmarks; proof review and further map/geometry testing pending |
| W8 Quality/refiner | Optional CSC shortcut extension implemented | Validated incumbent continuation, exact-oriented clearance weighting, strict section-boundary refinement, and opt-in forward/reverse CSC candidates with cost lower-bound pruning; 2000 seeded pose tests and local benchmark pass. Steering-rate smoothing, broader benchmarks and final retry remain pending |
| W9 Other models | Deferred | Add only after a second real model exists; actuator/ROS contracts independent |

## Provisional configuration / decision log

| Area | Current behavior | Decision pending |
| --- | --- | --- |
| Footprint | 4.6×1.8 m rear-axle fixture (-1.0..3.6, ±0.9), configurable | Application vehicle geometry |
| Search | 0.25 m XY, 72 yaw bins, 0.50 m primitive | Fixed vs auto-derived length; packed keys benchmark |
| Model | Rmin=4.0 m, six controls, exact arcs | Different model/progress policy |
| Validation | Exact filled pose; optional OFF-by-default 1–64-disc prefilter for search only; adaptive sampled at <=0.10m/0.05m search/final | Continuous swept certification, body-point thresholds |
| Occupancy | threshold 0, Unknown=Occupied, optional Free/Reject | Exploration's risk policy |
| Cost | F=1.0, R=1.10, switch=0.50m; optional clearance lambda=0, influence=0.40m | Clearance/routing objective and safe settings for constrained maps |
| Heuristic | Certified region Euclidean default; optional noncertified lazy grid plus injected RS exact-goal distance | Benchmark performance and separate reachability proof |
| Termination | First strictly validated default, opt-in bounded `ImproveIncumbent` with explicit reasons | Improve-mode expansion budget and strict retry policy |
| RS | First-node/search-bin scheduler; signed native hook, reused real OMPL evaluator, native combined plan/refine wrapper | Full OMPL-ON package build, exact checkout and scheduling/quality benchmarks |
| Refinement | Explicit `refinePath()`; straight-only default; opt-in CSC or signed provider; optional real-OMPL combined plan/refine; strict cheaper full-path revalidation and original fallback | CCC/RS completeness, curvature-continuous steering smoothing, multi-seed cost/latency benchmarks |
| Result | Geometric sampled evidence, no actuator guarantee | Consumer execution assurance policy |

Search hotpath instrumentation counts label finds/hits/inserts/rehashes,
exact pose checks and queue work. An opt-in CMake timing build measures
hashing, label writes/lookups, exact SAT collisions, full motion checks and
both queue push/pops; nested timings must not be summed. The map checker now
caches immutable map-origin trigonometry; successful key updates reuse
the found map iterator instead of performing an extra lookup. Label-table
pre-reservation is opt-in (`searchLabelReserve=0` baseline), not
a new benchmark-ratified default. See [hotpath profiling
contract](w6-hotpath-profiling.md). A complete profiled toolchain run is
still pending.

**W10 model layer update:** Ackermann ordinary search now builds its six
constant-curvature local motion templates **once per request**, then rigidly
transforms cached endpoints and search samples at each continuous node.
The final full-footprint validator still independently re-evaluates the
exact analytic section with its own stricter sampling configuration.
A new isolated differential-drive `Model` and eight-local-primitive catalog
supports exact constant body twist and stationary in-place turns, including
sample points and bounded wheel-rate controls. **The same native Hybrid A* loop now runs differential drive** with
eight cached SE(2) local primitives, explicit positive-cost angular
sections, wheel-rate configuration checking, rotated map-aware exact
footprint collision sampling and independent strict validation.
The localhost Planning Lab supports model selection and animates
stationary turns. This reuses planar pose/gear indexing; a fully generic
model-parametric core and broad shape registry are still deferred.
See [W10 differential-drive primitive status](w10-differential-primitives.md)
and [working search/demo contract](w10-differential-search.md).

See [W6 priority queue verification](w6-priority-queue-verification.md) for both-heap instrumentation, indexed and four-ary A/B timings, memory costs and unchanged defaults.
See [W3/W6 failure investigation](w3-w6-failure-investigation.md) for the corrected dead-end fixture, detour heuristic-budget root cause and optional weight tradeoffs.
See [hard-scenario regression and W6 A/B evidence](w3-w6-hard-scenarios.md) for current test fixtures, counters and remaining search-efficiency limitations.
See [real OMPL 2.0.2 source integration evidence](w5-ompl-source-verification.md) for the offline 10-suite Release/sanitizer matrix and remaining official-package gate.
See [W8 curvature-bounded CSC verification](w8-csc-verification.md) for geometric construction, safe cost pruning and local performance limits.
See [W3–W5 verification notes](w3-w5-verification-notes.md) for new acceptance fixtures and explicit OMPL verification limits.

See synthesis §1.3, §17 and §18 for alternative approaches and mathematical
proof obligations. **All settings are provisional; initial measurements are exploratory and not release benchmarks.** A
successful sampled plan is a static-map geometric initializer, not an
execution-safe timed trajectory, proof of swept collision freedom, globally
optimal path or a motion command.
