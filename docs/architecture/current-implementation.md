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
   skipped. There is **no EDT or boundary-only fast path** in the new planner.
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
5. Search queue ordered by finite `g + max(0,distXY-to-goal - XY tolerance)`
   (equivalent-meter, goal-region Euclidean lower-bound ordering).
   Edges use 1.0x forward, 1.10x reverse and +0.50m actual gear changes.
6. Required full-footprint checks at every search segment start and sample,
   including each endpoint. Spacing obeys BOTH reference spacing and a
   translation-plus-rotation body-point movement bound
   `(1 + Rbody*abs(curvature))*deltaS`. Required work exceeding the
   finite per-section sample limit returns `ValidationIncomplete`.
7. An **independent, stricter final resampling pass** on reconstructed actual
   motion sections: default 0.05 m reference and 0.05 m body-point movement
   ceiling. It calls exact collision directly and never trusts cached
   search classifications. Only a completed strict pass permits `Success`.
8. Explicit statuses for input/map/unknown errors, start collision,
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

- **W5:** The separate OMPL target still only provides distance
  `reedsSheppDistance`; this vertical slice does not yet accept analytic
  connector segments or use the attempt scheduler during search. Optional
  OMPL-ON build remains unverified in this environment.
- **W6:** No reverse-Dijkstra/lazy obstacle-performance heuristic yet.
  The planner uses the goal-region Euclidean fallback only; no graph
  unreachable pruning. There is no `max(RS,grid)` priority in this slice.
- **W7:** No EDT, multi-disc cover or swept certification. Search/final
  adaptive sampling controls *density*, **not the continuous swept set**.
- **W8:** No clearance integral, incumbent-improvement mode, cost-gated RS,
  post-path refiner or bounded final-validation retry.
- **W9:** No timed steering/actuator checks, dynamic-obstacle predictions,
  ROS/Nav2 adapter, articulated/self-collision or alternative chassis model.
- There is no cancellation token, wall-clock budget, separate final-check
  budget, or allocator-level memory budget in this initial interface.
- The initial test set exercises open-field feasibility and correctness
  corner cases, **not** full wall/parking/benchmark, randomized property,
  sanitizer-matrix, or proven SAT/EDT/sweep assurance release gates.

## Work-package traceability

| Package | State | Code and next exit gate |
| --- | --- | --- |
| W0 Baseline/tests | In progress | This contract + imported synthesis, named `planner_test`; reconcile further ADR prose and add benchmark harness |
| W1 Map/exact geometry | Implemented for filled Ackermann rectangle | `MapSnapshot`, `ExactCollisionChecker`, tests for rotated/boundary/unknown/interior; expand contact/property cases |
| W2 Ackermann motion | Implemented for fixed curvature | `MotionSection` analytic evaluator; test more numerical extreme values/equivariance |
| W3 Minimal search | Initial vertical slice | Immutable nodes/reopen/base costs/region Euclidean; add wall detour, parking and negative tests |
| W4 Final validation | Initial adaptive-sampled gate | Stricter exact resampling; independently inject weaker search path and verify failure, verify separate resource budget |
| W5 Analytic RS | Pending | OMPL reusable evaluator, signed sections, cusps and cost, scheduler wired into search |
| W6 Obstacle heuristic | Pending | Goal-region-seeded request-local reverse Dijkstra with finite fallback; no unproven `NoPath` |
| W7 Disc/EDT | Deferred | Conservatism proof + exact fallback + whole-planner benchmark |
| W8 Quality/refiner | Deferred | Benchmarkable optional clearance, validated incumbent improvement, full-path revalidation |
| W9 Other models | Deferred | Add only after a second real model exists; actuator/ROS contracts independent |

## Provisional configuration / decision log

| Area | Current behavior | Decision pending |
| --- | --- | --- |
| Footprint | 4.6×1.8 m rear-axle fixture (-1.0..3.6, ±0.9), configurable | Application vehicle geometry |
| Search | 0.25 m XY, 72 yaw bins, 0.50 m primitive | Fixed vs auto-derived length; packed keys benchmark |
| Model | Rmin=4.0 m, six controls, exact arcs | Different model/progress policy |
| Validation | Exact filled pose; adaptive sampled at <=0.10m/0.05m search/final reference | Continuous swept certification, body-point thresholds |
| Occupancy | threshold 0, Unknown=Occupied, optional Free/Reject | Exploration's risk policy |
| Cost | F=1.0, R=1.10, switch=0.50m, clearance=0 | Clearance/routing objective |
| Heuristic | Certified region Euclidean numerical lower bound | Performance RS+grid, graph reachability proof |
| Termination | First strictly validated path | Bounded incumbent improvement, retry policy |
| RS | Scheduler uses search-bin units; no planner connector | Full path adapter, attempt cadence benchmark |
| Result | Geometric sampled evidence, no actuator guarantee | Consumer execution assurance policy |

See synthesis §1.3, §17 and §18 for alternative approaches and mathematical
proof obligations. **All settings are provisional and unbenchmarked.** A
successful sampled plan is a static-map geometric initializer, not an
execution-safe timed trajectory, proof of swept collision freedom, globally
optimal path or a motion command.
