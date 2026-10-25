# Requirements and constraints — draft v0.1

IDs are stable references for design discussions and tests. **Status: accepted direction, detailed contract proposed unless otherwise stated.** A requirement is fulfilled only when implemented and verified, not when documented.

## Functional requirements

| ID | Requirement | Initial scope / acceptance evidence |
| --- | --- | --- |
| FR-001 | Generic kinematic models | Search compiles against differential-drive and Ackermann model policies, plus an ideal mecanum reference, without drive-type branches |
| FR-002 | Configurable motion primitives | Admissible controls generate forward/reverse, in-place rotation, lateral and combined motions where supported; full trajectory preserved |
| FR-003 | Model-aware state indexing | One retained continuous node per complete model-specific discretized key; initial Ackermann uses **0.25 m XY, 5° yaw (72 bins), and incoming forward/reverse mode** to account for switch penalties (ADR-0025). Index articulation/other model-specific dimensions when relevant; preserve lossy merge/reopen semantics and test pruning effects. |
| FR-004 | Collision geometry | Multiple frame-attached rigid 2D bodies; articulated poses derived from hypothetical state |
| FR-005 | Primitive-coupled sampled motion validation | During each expansion, apply a **conservative all-orientation footprint hull collision-possibility lookup**, then **rasterized polygon/rectangle boundary-cell occupancy checks** for flagged poses by default, with an optional `checkFootprintInterior=true` API flag enabling filled-interior occupancy checks at **each stored RK4 primitive sample** (including start/end). Initial Ackermann: **0.10 m stored sample spacing**, no independent default collision-sampling schedule; additional adaptive subdivision optional and **disabled by default**. No intermediate samples become search nodes. Report the **boundary-only sampled** (default) versus **full-footprint sampled** (opt-in) coverage honestly; the default can miss fully enclosed obstacles, and neither mode certifies continuous swept collision freedom (ADRs 0022,0025,0029). |
| FR-006 | Static occupancy environment | Immutable whole-grid, shared owning snapshot with source identity, revision, frame, origin and bounds; atomic publication of privately built new revisions; per-request snapshot pinning (ADR-0019) |
| FR-007 | Combined nonholonomic + obstacle-aware heuristic | **Required for first Ackermann MVP:** OMPL-backed obstacle-free Reeds–Shepp kinematic distance and map-aware **lazy, cached, multi-source 2D reverse-Dijkstra** obstacle distance (reusing settled distances and pending heap across compatible requests), combined with `max`; evaluate only settled grid costs and fall back to a finite RS/Euclidean estimate for unavailable/disconnected cells, with safe bounded/synchronized mutable-cache access (ADR-0027). Raw combined heuristic is not automatically a certified goal-region lower bound; retain separately certified Euclidean/zero fallback (ADR-0015), report guarantee status and profile both modes. |
| FR-008 | OMPL-backed steering with adaptive analytic connections | The first Ackermann MVP uses OMPL Reeds–Shepp from the popped node's **continuous** state to the exact requested goal as a **single complete piecewise path** (including signed sections/cusps). **Attempt on the first non-stale expanded node regardless of XY distance**, then retry from later nodes with a **Nav2-style adaptive heuristic-distance-based expansion interval** (`analyticExpansionRatio` initially 3.5, configurable). **No fixed XY trigger or path-length cap.** Sample all analytic sections at ≤0.10 m including cusp/endpoints; enforce finite model/goal validity and collision checks, full reconstructed-path validation, and consistent forward/reverse/gear-switch costs (no clearance integral in MVP). Optional weighted-cost cap is **off by default**. Failed analytic attempts always continue ordinary search (ADRs 0026–0028). |
| FR-009 | Configurable search and clearance cost | Open/closed handling; accepted reverse/switch costs and **no EDT clearance cost in the first implementation**. The nearest-obstacle EDT is used for the collision candidate mask only. **Low-priority future option:** reuse it for actual-footprint boundary clearance scoring (0.15 weight, 0.40 m influence), with matched primitive/analytic cost semantics (ADRs 0025,0029). EDT is **not** the goal-rooted 2D obstacle-aware reverse-Dijkstra heuristic. Respect pinned map/occupancy policy, cancellation and bounded resources. |
| FR-010 | Exact path reconstruction | Ordered, evaluable motion segments including gear changes and in-place rotations |
| FR-011 | Optional but obstacle-aware refinement | When implemented/enabled, optimization uses static map obstacles/clearance **during** candidate improvement, not merely final collision rejection. Use the same pinned map snapshot and request occupancy policy, and full frame-attached body geometry including articulation; independently validate final endpoints, model feasibility and complete-body collision. Keep original valid raw path on failure (ADR-0024). |
| FR-012 | Controller-facing output | Piecewise geometric path; endpoint conditions, progress, directional and configuration information |
| FR-013 | External configuration | Typed native config; optional YAML and JSON adapters with validation; SI units, radians |
| FR-014 | Optional ROS 2 integration | Isolated adapter(s), including Nav2 option; core does not depend on ROS |
| FR-015 | Deterministic diagnostics | Failure reason, map/config versions, counters, timings, and reproducible test inputs |
| FR-016 | Controlled-ODE model contract | Model defines state, control, state derivative, validity constraints and frame transforms; integrator evaluates model and exposes intermediate states |
| FR-017 | Holonomic-capable motion semantics | Holonomic sideways/combined motion can have heading independent of path tangent; no binary holonomicity dispatch |
| FR-018 | ROS-compatible map coordinates | Adopt REP-103/105 right-handed frames, map-origin SE(2), lower-left grid origin, x-fastest row-major storage, explicit occupancy values; outdoor maps use local ENU (+X east, +Y north, +Z up) with explicit geodetic reference metadata when GNSS is involved |
| FR-019 | Pose-tolerant goal acceptance | Default Euclidean XY <= 0.25 m and wrapped yaw <= 10 degrees on continuous endpoint; independent configurable tolerances, optional extra model-state constraints, full motion validity |
| FR-020 | Fixed-curvature Ackermann baseline | Three-state geometric bicycle with bounded-curvature arcs (straight/left/right and optional intermediate); no implied finite steering-rate feasibility |
| FR-021 | Explicit heuristic guarantee modes | Ackermann performance default combines OMPL RS obstacle-free and obstacle-aware grid estimates without an admissibility guarantee; provide a separately selectable certified Euclidean-to-goal-region/zero mode (ADRs 0015,0027). Neither mode implies continuous-space optimality under quantized-key single-label pruning. |
| FR-022 | Request-level occupancy and unknown policy | Expose `Occupied` (default), `Free` (optimistic), `Reject` for `-1` cells; known cells traverse iff `c <= maxTraversableOccupancy` (configurable `0..100`, default `0`); `Reject` fails upfront on `-1` anywhere in the supplied map; external frontier goals target known-free area (ADRs 0017–0019) |

## Architectural constraints

| ID | Rule |
| --- | --- |
| AC-001 | Core and tests build outside ROS 2; ROS types cannot occur in core public API |
| AC-002 | Generic search hot path avoids mandatory virtual dispatch, avoidable heap allocations and format parsing |
| AC-003 | Kinematic state and search key are separate; no universal `(x,y,heading)` key assumption |
| AC-004 | Collision depends on entire model state and all collision bodies. Conservative precomputed footprint-envelope lookup may skip collision only when safe is proven for the pinned map/policy and continuous pose; otherwise rasterize footprint edges and **optionally** test enclosed blocked cells when `checkFootprintInterior=true`. The default boundary-only result is **not complete-footprint collision assurance**, because enclosed blocked cells can be missed (ADR-0029). |
| AC-005 | If refinement is enabled it must consider obstacle information **during** optimization, evaluate full robot geometry on the pinned map/policy, and independently validate the proposed result; on failure keep the untouched raw feasible path (ADR-0024) |
| AC-006 | OMPL is allowed **only as an isolated runtime Dubins/Reeds–Shepp steering backend** (ADR-0028); do not expose its types in generic search/model/path APIs, link it into unrelated modules or incorporate civcode demo-repository code. |
| AC-007 | Separate dynamic trajectory planning/MPC handles predicted moving obstacles and timing |
| AC-008 | Any future GitHub Actions workflows are manual-only unless explicitly authorized |
| AC-009 | Model's admissible controls are authoritative; optional capability metadata cannot substitute for validity checks |
| AC-010 | Search and path representations cannot universally equate body heading with velocity direction or require forward/reverse-only motion |
| AC-011 | All native map/pose coordinates follow ROS-compatible conventions; only adapters access ROS APIs or resolve frame transforms |
| AC-012 | Initial search uses one retained continuous node per model-defined discretized key; no required multi-label policy/storage; record approximate dominance and history-dependent cost limitations |
| AC-013 | An admissible heuristic does not by itself establish continuous-space optimality when distinct continuous arrivals share a discretized key |
| AC-014 | Unknown occupancy and known-cell maximum traversable occupancy are independently configurable per planning request and consistent across collision and heuristics; derived caches must distinguish effective policy/map identity/revision and goal/algorithm inputs |
| AC-015 | A map revision is immutable after atomic publication; requests pin shared owning snapshot handles without map mutation or per-query map lock; stale paths are not automatically valid for newer revisions |
| AC-016 | Strict `UnknownCellPolicy::Reject` validates the entire supplied snapshot before search and returns dedicated unknown-occupancy request failure independent of search traversal order |

## Non-goals (initial milestones)

- No time-expanded Hybrid A*, multi-agent reservation planning, or dynamic-obstacle prediction in the library.
- No MPC, velocity profile optimizer, ROS 2 node, or Nav2 plugin in the initial core milestone.
- No claim of global optimality, completeness, continuous collision certification, or real-time deadlines until assumptions and tests justify it.
- No requirement to use B-splines (or any particular curve family) at the controller boundary.
- No support for arbitrary 3D mesh collision in v0.1; plan a future extension point.

## Traceability

Acceptance tests and thresholds are defined in [verification.md](verification.md) and [quality.md](quality.md). Use stable requirement IDs in tests / benchmark manifests and PR descriptions. Open questions are tracked in [open-questions.md](open-questions.md).
