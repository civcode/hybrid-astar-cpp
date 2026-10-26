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
5. Search queue ordered by finite `g+h`. Default `h` is the certified
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
   point with safely scoped provider callbacks. Actual OMPL-ON compilation
   and execution are **still unverified**.
9. Optional `ImproveIncumbent` quality mode retains only strictly validated
   lower-weighted-cost paths for a bounded extra-expansion budget (illustrative
   default 2500; not benchmark-ratified). First-validated remains the default.
   The result reports solution quality, candidate/improvement counts, and the
   reason for terminating continued search. A previously verified incumbent
   can be returned on count-budget exhaustion or post-incumbent cancellation
   with explicit termination metadata.
10. Optional **exact-oriented-rectangle** nearest blocked-cell-square/map-edge
    clearance score is integrated using nonnegative trapezoidal arc-progress
    penalties. It applies equally to ordinary primitives and signed analytic
    sections, is computed before active-label relaxation, and defaults to zero.
11. Explicit statuses for input/map/unknown errors, cooperative cancellation, start collision,
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

## Important limitations (never advertise as implemented)

- **W5:** The signed connector and reusable OMPL evaluator are implemented.
  An application can explicitly supply its own `analyticConnector`, or use
  `steering::planWithReedsShepp()` from the optional OMPL target. Its second
  parameter optionally selects the noncertified RS-plus-grid heuristic.
  The provider-neutral signed decoder has passed local tests, but **OMPL-ON
  build and real OMPL tests remain unverified** in this environment.
- **W6:** Request-local raw-occupancy lazy reverse Dijkstra is an optional
  noncertified performance provider with deterministic count-based settlements,
  goal-region seeds and finite fallback; it is **not** a separately proved
  footprint-aware reachability oracle. No early graph-based `NoPath`.
- **W7:** Conservative request-local blocked-center Euclidean distance transform
  and constructive whole-body disc covers are implemented as an optional
  search-only safe bypass with exact fallback, map-border checks and counts.
  Default is **off** pending independent proof review, randomized regression
  expansion and whole-planner benefit on representative maps. Never infer
  collision from disc overlap; **no continuous swept certification**.
  Adaptive samples still control *density*, not the continuous swept set.
- **W8:** Optional nonnegative oriented-footprint clearance integration,
  cost-gated analytic candidates and bounded validated incumbent improvement
  are implemented, defaulting to disabled/first-feasible. A post-path refiner
  and a bounded strict-validation retry remain deferred. The clearance
  experiment `lambda=0.15` caused a distant-doorway regression under the
  tested work budget: do not adopt it as a product default.
- **W9:** No timed steering/actuator checks, dynamic-obstacle predictions,
  ROS/Nav2 adapter, articulated/self-collision or alternative chassis model.
- Cooperative request-local atomic cancellation is implemented and polled
  during search, sample checks and strict validation. Strict final validation
  now has a **separately metered per-section sample budget**, but there is no
  wall-clock budget, global final-work budget, or allocator-level memory
  budget in this initial interface.
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
| W2 Ackermann motion | Implemented for fixed curvature | `MotionSection` analytic evaluator; test more numerical extreme values/equivariance |
| W3 Minimal search | Initial vertical slice, additional acceptance fixtures | Immutable nodes/reopen/base costs/region Euclidean and cancellation; primitive-only reverse and two-cusp parking, narrow-gap and revision tests now pass locally; further negative/large-map scenarios pending |
| W4 Final validation | Strict sampled gate with separate section budget | Stricter exact resampling, weakened search fixture and independent final sample cap with incomplete-on-budget test pass; failure location, global work cap and swept guarantee pending |
| W5 Analytic RS | Native decoder and wrapper added, OMPL-ON unverified | Signed native decoder and two-cusp cost/fallback tests pass locally; adapter and `planWithReedsShepp` wired, real OMPL build/integration matrix remains an explicit required gate |
| W6 Obstacle heuristic | Initial performance mode | Goal-region-seeded request-local lazy reverse Dijkstra, optional RS metric, finite fallback; compare maps/settlement budgets and no unproven `NoPath` |
| W7 Disc/EDT | Opt-in prototype, disabled by default | Blocked-center exact EDT, conservative rectangle disc cover, strict exact fallback; 3/4/5-disc randomized differential tests and staged end-to-end benchmarks; proof review and further map/geometry testing pending |
| W8 Quality/refiner | Initial opt-in quality increments | Strictly validated incumbent continuation and oriented exact-square clearance weight implemented; quality regression suite passes; objective tuning and full refiner/retry pending |
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
| RS | First-node/search-bin scheduler; signed native hook; optional reusable OMPL evaluator | OMPL-ON build, integration and scheduling/quality benchmarks |
| Result | Geometric sampled evidence, no actuator guarantee | Consumer execution assurance policy |

See [W3–W5 verification notes](w3-w5-verification-notes.md) for new acceptance fixtures and explicit OMPL verification limits.\n\nSee synthesis §1.3, §17 and §18 for alternative approaches and mathematical
proof obligations. **All settings are provisional; initial measurements are exploratory and not release benchmarks.** A
successful sampled plan is a static-map geometric initializer, not an
execution-safe timed trajectory, proof of swept collision freedom, globally
optimal path or a motion command.
