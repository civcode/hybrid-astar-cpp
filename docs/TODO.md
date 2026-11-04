# Implementation roadmap — current priorities

## Current priorities — 2026-11-04

**This section is the active priority order.** The W/M checklists below are
historical implementation and architecture work items, **not** a competing
priority queue. Some older unchecked proposals have already been implemented,
superseded or measured; use the
[current implementation contract](architecture/current-implementation.md),
[measured warehouse findings](warehouse_performance_findings_2026-11-04.md)
and the code for current behavior. Do not mistake a historic checkbox for
a report that a working feature is absent.

**Current baseline:** Hybrid A* has independently sampled strict full-footprint
validation, opt-in cost-reducing straight/CSC/RS or model-specific refinement,
a measured map-specific label-reserve dial-in (tier series v2), and profiler
support. It produces a **geometric path initializer**; it does not establish
continuous swept-body safety, time-parameterized actuator feasibility, safe
real-robot tracking or dynamic-obstacle avoidance.

### Immediate prerequisite gate — complete BEFORE smoothing-quality benchmarks

**The two explicit user-requested prerequisites (in order) supersede
the earlier generic, unspecified prerequisite gate:**

**P0-A — Planning Lab footprint at editable start and goal** (UI change
implemented; regression review pending):

- [x] Draw the full **model-specific rotated rectangular footprint at
  BOTH start and goal** before planning, while planning is running,
  after failure, and while a route is stale. Retain each pose's marker
  and yaw ray. Keep the replay body's independent rendering.
- [x] Show footprint inflation when enabled, using the displayed
  metric round-joined margin as a **visual aid**, not a substitute
  for the exact C++ collision checker. Distinguish start (cyan), goal
  (gold) and replay (mint).
- [x] Exercise the actual updated app drawing logic against a headless
  canvas/DOM: initial start/goal footprints, editable 90-degree yaw,
  Ackermann/differential sizes and the inflation band all passed.
  A manual Node regression is provided at
  `tests/demo_pose_footprint_ui_test.js`.
- [ ] Visually verify the UI in a real browser: scenario/model changes,
  pose editing, goal shift-click, rotated map origin, inflation,
  pre-plan loading, stale routes and failed searches. Do not require
  planning success for either footprint to appear.

**P0-B — Articulated-steering kinematic model** (the author has
now supplied the full **five-variable front/rear-heading ODE**;
**physical conventions and planner integration pending**).
See [articulated-steering integration stages and remaining physical questions](architecture/articulated_steering_integration.md):

- [x] Confirm the supplied five-variable state
  `(xf,yf,Theta_f,Theta_r,gamma)` in
  [kinematic-models](architecture/kinematic-models.md) and its
  `(v,u_gamma)` input matrix with `D=L1*cos(gamma)+L2`.
  **Correct the earlier "rear-reference" label:** the four-state
  ODE is exactly the **front-reference** reduction when
  `gamma=wrap(Theta_f-Theta_r)`, initialized consistently.
  Do not misrepresent five reported states as five independent
  configuration degrees of freedom.
- [x] Derive a physically consistent candidate frame interpretation:
  `F=(xf,yf)`, `H=F-L1*heading(Theta_f)`,
  `R=H-L2*heading(Theta_r)`, with front/rear axle no-slip.
  This algebra is established; its **identification with the real
  robot geometry is still provisional**.
- [ ] **Confirm OQ-022 with the robot owner:** whether the front
  reference really is the front axle midpoint, the physical
  endpoints for `L1/L2`, zero/positive articulation convention,
  front/rear collision-body dimensions and offsets, input meaning,
  articulation/speed bounds and joint-angle goal semantics.
- [ ] Introduce a standalone validated model retaining **both
  reported headings** for diagnostics and checking the invariant;
  use a four-independent-state reduction for search if the robot
  geometry confirms it. Require finite-parameter/denominator
  safeguards, angle-aware integration, admissible controls and
  deterministic time-domain tests, including `v=0`,
  finite articulation rate and reverse.
- [ ] Integrate the **full physical configuration** with model-specific
  primitive generation, `gamma`-aware search indexing/goal acceptance, exact
  interpolation and full-state path reconstruction. Never shoehorn
  articulation into the existing three-state Ackermann/differential
  `Pose2D` keys or use standard RS connectors without proof.
- [ ] Attach independently defined front/rear collision bodies
  to the model's confirmed `SE(2)` transforms; validate both at
  **every sampled configuration**, including articulation motion
  when the **front reference point** does not move. Verify swept-motion
  assumptions, map boundaries, unknown occupancy, inflation,
  start/goal and strict final checks before accepting route success.
- [ ] Expose only genuinely supported articulated UI/CLI controls
  (pose, `gamma`, speed/joint limits and body overlays) after
  state/collision integration, then test deterministic maps,
  narrow corridors, forward/reverse turns, joint-only sweeps and
  cancellation/budgets. Provide explicit unsupported-model status
  until the full path pipeline is validated.

**Acceptance gate to start smoothing-quality benchmarking:**
P0-A is visually and functionally verified, and P0-B has a
physics-consistent, independently checked planning path (not merely
a derivative function). Define smoothing metrics that apply to
each supported robot: Ackermann curvature/steering rate, differential
wheel-rate continuity, mecanum body twist, and articulated `gamma`,
`u_gamma`, front/rear body clearance and joint-limit compliance.
All benchmarks must preserve the validated raw-path fallback, pinned
map/occupancy/geometry, independently strict sampled collision checks
and finite work budgets.

### Ordered engineering priorities

| Rank | Priority / scope | Target outcome | Current next action |
| ---: | --- | --- | --- |
| **1** | **Curvature-continuous, model-feasible smoothing** | Fewer sudden steering/wheel-command changes without breaking collision, kinematics, or route feasibility | **After P0-A and P0-B are verified:** create a matched quality baseline for all supported robot models, then test bounded optional curvature/joint-rate-continuous smoothing with strict validated fallback |
| **2** | **Faster, smarter post-search refinement** | More quality per millisecond | Profile existing refiner; prioritize high-value sharp turns/cusps/stops, avoid redundant whole-route checks for candidates while retaining independent strict validation of published results |
| **3** | **Time parameterization / acceleration profiling** | Turn geometric motion sections into controller-ready timed candidates | Define independent speed, acceleration, steering-rate/wheel-rate, jerk, braking and mandatory-stop/cusp contracts; validate physically meaningful limits, not just wheel-rate feasibility |
| **4** | **End-to-end profiling and quality acceptance** | Choose improvements by total validated pipeline result | Track search, heuristic/map build, collision, refinement, strict validation, smoothing, and future timing stages separately; report p50/p95/p99, failure rate and final-path quality |
| **5** | **Map-dependent data reuse across requests** | Faster repeated planning on immutable maps | A/B cold versus warm reuse of occupancy prefix, bounded grid/heuristic data and collision acceleration; key by map snapshot/fingerprint + occupancy/geometry/options and invalidate on edits |
| **6** | **Further hash/EDT/clearance optimization** | Incremental speed, memory or optional wide-clearance quality | First verify measured bottlenecks; keep clearance scoring and EDT opt-in, and preserve the measured versioned label-reservation policy |

The ranks describe **product payoff**, not an instruction to implement each
item without measurements. Priority 4's *instrumentation and acceptance
criteria* must underpin all other experiments from their first benchmark.

### Priority 1 — smoothing quality: evidence before new curves

**After P0-A and P0-B pass**, use matched immutable-map routes and
identical robot, occupancy policy, hard margin and strict validation budgets
to compare: (A) raw Hybrid A* output, (B) current straight/CSC or model
appropriate refinement, (C) optional steering-variation objective where
supported, and (D) a *future* curvature-continuous alternative.
Current W8 supports (A)–(C); **(D) is not implemented**.

- [ ] Establish reproducible route fixtures covering open space, bends,
  zigzags, tight corridors, parking/reverse cusps, and blocked alternatives.
  Retain original seed/map fingerprint and budget-limited failures.
- [ ] Record **final validated**, not merely raw, length/weighted cost,
  stop/reverse/cusp counts, section count, turning-radius/wheel-rate feasibility,
  steering/curvature discontinuities, curvature variation, sampled
  minimum/P10 obstacle clearance and search+refinement latency.
- [ ] Evaluate curvature-continuous candidate representations (e.g.
  clothoid transitions), separately for Ackermann, differential and
  mecanum motion constraints; for articulated steering, constrain
  **joint angle/rate, both body transforms and steering-induced rear-yaw
  dynamics** from the confirmed physical model. Do **not** assume a
  generic XY spline preserves orientation, no-slip constraints,
  articulation limits, wheel limits or obstacles.
- [ ] Bound solver work/time and ensure immutable map/occupancy,
  inflation and model provenance. Preserve exact start/goal semantics,
  intended stops/cusps and original validated route if any candidate
  fails model/collision/strict check or a work budget.
- [ ] Demonstrate meaningful reduction in curvature/steering discontinuity
  (and ideally controller-tracking effort), without unacceptable
  degradation of path cost, corridor clearance, success rate or total
  p95 latency. Quantitative limits need explicit robot/hardware agreement.

### Priority 2 — refinement throughput / candidate selection

- [ ] Profile `refinePath()` by candidate generation, cheap objective
  lower-bound pruning, collision pose checks, exact clearance calculations,
  whole-path revalidation, accepted windows and elapsed time.
- [ ] Compare current multiscale window scheduling with budgeted,
  quality-targeted proposals around avoidable stops, reversals,
  high-curvature jumps and costly detours.
- [ ] Investigate incremental scoring and **local** verification of changed
  candidate sections, with pinned map/options and safe splice-boundary
  accounting. Preserve a **fresh independent whole-path strict validation**
  before publishing any accepted final route; retain current verified-path
  fallback. Never substitute a stale cached certificate for a map change.
- [ ] Accept only measured reductions in total validation/refinement time
  or improved final validated quality at an equal budget; no new invalid
  paths or changed search semantics.

### Priority 3 — downstream timed trajectories

- [ ] Specify `path sections -> timed trajectory` contract independently
  of the Hybrid A* search core, including direction/cusp, in-place spin,
  lateral mecanum motion and start/goal speed.
- [ ] Implement bounded speed/acceleration/braking/jerk profiling with
  Ackermann steering position/rate, differential wheel-rate/acceleration
  and mecanum wheel-rate/acceleration constraints as appropriate.
- [ ] Recheck tracking feasibility and full robot motion along the
  time-parameterized trajectory; retain explicit unsupported-result
  statuses. Moving obstacles, uncertainties and closed-loop controls
  require separate validated components and are **not delivered** by
  geometric search or smoothing alone.

### Priority 4 — representative pipeline benchmarks

- [ ] Establish reference hardware, compiler/build flags and
  p50/p95/p99 latency, memory/RSS and timeout targets for easy, hard,
  unsuccessful, inflated-footprint and repeated-plan workloads.
- [ ] Profile complete work breakdown: map/prefix/heuristic initialization,
  search labels/queues/collision, reconstruction, post-refinement,
  independent final checks and (when implemented) smoothing and retiming.
  Record failure reasons and compare quality of the **final validated path**.
- [ ] Use uninstrumented builds for real speed results; opt-in
  `PROFILE_HOTPATHS` for hotspot attribution only. Do not add overlapping
  nested hot-path timers, treat bucket-array estimates as total RSS,
  dispatch GitHub Actions or add event-triggered workflows.

### Priority 5 — map reuse; priority 6 — lower-return tuning

- [ ] Benchmark unchanged-map repeated requests against cold searches.
  Consider versioned, bounded reusable occupancy prefix and heuristic
  assets only when measured savings exceed construction, invalidation
  and memory overhead. Avoid sharing mutable model/occupancy-dependent
  caches across incompatible requests.
- [ ] Preserve opt-in `searchLabelReserve` and the versioned map-specific
  calibration. The 32-route large-footprint warehouse run verified zero
  rehashes for **v2 tier 13 (524,288)** and **v2 tier 14 (655,360)**,
  with essentially tied paired search latency; the 20%-headroom
  recommendation is tier 14 for **that specific clearance-OFF workload**.
  Calibrate again after changing objective, footprint, map or budgets.
- [ ] Keep the expensive obstacle-clearance objective and its EDT
  accelerator explicitly opt-in. The corrected 2-route smoke showed
  EDT about 30.2% faster than exact clearance scoring but still
  3.57–4.84× slower than clearance-OFF planning. Complete the larger
  clearance benchmark and measure bottlenecks before pursuing
  caching/approximation.
- [ ] Defer alternative hash tables, further queue changes and speculative
  full-map preprocessing unless a reproducible workload shows their
  impact on **end-to-end**, strictly validated output.

### Decision and safety guardrails

- [ ] Use measured *final-output* quality versus p95 latency and RSS,
  with strict goal/model/footprint validation, to accept each change.
  Keep failures observable; do not silently relax sampling or work caps.
- [ ] Before any physical-robot claim, explicitly verify actuator/time
  limits, localization/tracking tolerances, complete swept-volume
  assumptions and relevant dynamic-obstacle/control integrations.
  The existing sampled geometric validator is **not** a physical
  deployment certification.
- [ ] Work locally (Docker/CTest and explicit manual benchmark commands).
  No GitHub Actions run or automatic triggers unless explicitly requested.

---

## Historical implementation checklists (preserved for traceability)

These dated W/M work items record implementation history and previously
proposed designs. The current priority order is above; follow the
[current implementation contract](architecture/current-implementation.md)
when dated entries conflict with shipping behavior.

## Ackermann default collision fast paths — 2026-10-27

- [x] Make four covering discs the Ackermann default; keep explicit
  `discPrefilterCount=0` off and default other drive models unchanged.
- [x] Retain obstacle-grid heuristic as the planning default.
- [x] Implement immutable-policy-aware blocked-cell prefix sums for exact
  zero-occupied-cell inflated-AABB bypass after map boundary checks.
- [x] Preserve full rectangle SAT and Euclidean geometric inflation
  whenever the occupied cell AABB is nonempty.
- [x] Extend strict collision work counters with empty-region query/bypass
  measurements, and expose effective defaults in demo JSON.
- [x] Add equivalence regressions for occupancy policies, rotations,
  inflated geometry and cross-model default resolution.
- [x] Add easy-map, warehouse and fixed-pose prefix enabled/disabled
  A/B benchmarks without adding costly workloads to CTest.
- [ ] Run full Docker/CTest suite and gather the three new CSVs to
  validate net speedup versus memory/setup overhead on easy and hard maps.

## Ackermann collision-work measurement matrix — 2026-10-27

- [x] Add optional exact collision work counters for AABB grid cells
  visited, occupied cells reaching SAT, Euclidean distance checks,
  and finite-boundary rejects; keep ordinary checks unchanged by default.
- [x] Add opt-in Ackermann factorial matrix: Euclidean/obstacle-grid
  × 0/0.50 m inflation × 0/4 covering discs, preserving old CSV columns.
- [x] Separate uninstrumented wall-clock results from work counting
  results; enforce deterministic prefilter outcome equivalence.
- [x] Extend narrow/rotated conservative-cover tests with the inflated
  Ackermann rectangle and counter invariants.
- [ ] Run CTest and manual Docker CSV benchmarks locally, quantify
  exact pose calls, cells scanned per call, grid heuristics and time.
- [ ] Evaluate enabling discs by default only after cross-model safety
  and reproducible uninstrumented A/B performance results.

## Diagonal differential refinement / Ackermann warehouse profiling — 2026-10-27

- [x] Add multi-radius nonstationary CSC candidates, ordered by length,
  without changing exact validation or continuity/splice constraints.
- [x] Add fixed-vs-multi-radius A/B control and diagonal frame-rotation
  regression to distinguish geometry from map orientation.
- [x] Add manual diagonal warehouse refinement CSV benchmark.
- [x] Add manual Ackermann warehouse hot-path CSV cases based on user poses,
  with 0/0.10/0.50 m inflation cases where the screenshot margin was hidden.
- [ ] Run exact Docker/CTest suite; capture disabled/enabled profiler CSV,
  fixed/multi-radius refinement benchmarks and comparison screenshots.
- [ ] If Ackermann expansions dominate, compare to OMPL Reeds–Shepp
  analytic terminal connector without changing the strict validator.

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

## Differential strict search/final density parity — 2026-10-27

- [x] Diagnose screenshot with differential start (5.68, 28.97),
  goal (30.25, 38.17) and 0.10 m inflation: seven independent
  collision rejections before search termination.
- [x] Correct DifferentialDrivePolicy strictSearchSampling() to ensure
  search uses final-density sampling and regenerates matching motion
  templates. Add a focused inflated-footprint regression.
- [ ] Rerun exact warehouse screenshot manually to confirm path recovery
  and measure expansion/checking changes.
- [ ] Reproduce Ackermann 1M-expansion case with screenshot's exact
  (currently unknown) inflation value and compare OMPL analytic connector
  enabled/disabled before retuning heuristics.

## Planning Lab smoothing / strict-failure diagnostics — 2026-10-27

- [x] Set metric inflation number spinner to 0.05 m increments.
- [x] Reject differential rolling-refinement splices adjacent to retained
  stops, new gear reversals and worsened curvature discontinuities.
- [x] Enable opt-in strict mecanum post-refinement with independently
  validated wheel-rate-feasible SE(2) holonomic twist candidates.
- [x] Report strict final-revalidation failure categories in API/UI instead
  of assuming every rejection is an obstacle/swept collision.
- [ ] Run user-reported 0.5 m margin warehouse coordinates locally,
  review 22%-path visual junction, and rerun CTest; no claim yet that
  the previously failed differential goal is reachable.

## Motion continuity: avoid unnecessary stopping rotations — 2026-10-27

- [x] Stop generating rotate–straight–rotate post-refinement candidates for
  differential drive; retain collision-checked straight and rolling CSC.
- [x] Prefer moving turns to stationary spins in mecanum search with a
  nonnegative 0.35 m stop penalty, configurable via PlanOptions.
- [x] Run bounded local mecanum stop cleanup at plan return using exact SE(2)
  moving-twist connectors, four-wheel rate feasibility and strict whole-path
  revalidation. Keep the original if no cost-reducing safe merge is found.
- [x] Add regressions for no new differential spins and the specific 0.5 m
  inflated mecanum warehouse route shown in Planning Lab.
- [ ] Verify the full Docker/CTest suite and actual path screenshots; a spin
  can remain when stopping is geometrically required or cleanup fails.

## Continuous footprint inflation — 2026-10-27

- [x] Introduce `PlanOptions::footprintInflationMeters` default 0 with
  exact metric rectangle-to-cell distance and zero-margin SAT fast path.
- [x] Apply the geometric margin to all model search/final checks, rotated
  map boundaries, conservative disc prefilter and strict refinement.
- [x] Add Planning Lab checkbox and meter input, HTTP/CLI options, JSON
  metadata and visual halo for the configured margin.
- [x] Add threshold/corner/rotated-map/boundary/prefilter/refinement tests.
- [ ] Verify Docker/CTest and performance on representative 0/.05/.1m maps;
  evaluate actual robot localization and tracking error before relying on
  this margin for deployed clearance guarantees.

## Differential-drive post-path smoothing — 2026-10-27

- [x] Extend independently collision-checked, bounded post-path refinement
  to differential drive, including spin/translation costs and wheel-rate
  feasibility checks for constant-curvature candidates.
- [x] Add strict straight-chord plus optional 0.75 m radius CSC connectors,
  and spread bounded attempts across long paths at multiple scales.
  Refinement does not introduce stop-and-spin candidates.
- [x] Enable both Planning Lab refinement toggles for differential drive
  while retaining Ackermann-only OMPL; strict mecanum refinement was added
  later with holonomic rather than CSC shortcuts.
- [x] Add strict geometric regression cases, demo contract and HTTP checks.
- [ ] Run the C++/Docker test suite and inspect Planning Lab smoothing on a
  real zigzag warehouse map; measure cost, sections and elapsed time.

## Differential-drive near-goal robustness — 2026-10-27

- [x] Use a consistent 1e-9 numerical slack for ordinary goal acceptance,
  matching the existing independent final validator.
- [x] Permit a separately costed, strictly validated differential-drive
  terminal rotation when within goal XY tolerance and outside yaw tolerance;
  reject any swept-footprint collision.
- [x] Add finite heading-lattice and blocked-rotation regressions and expose
  grid/terminal diagnostics in Planning Lab exported path JSON.
- [ ] Verify the user-reported two nearby warehouse starts on the exact
  edited map and ensure both either reach strictly validated paths or
  report useful bounded-failure diagnostics. Run CTest/Docker locally.

## Performance follow-up — 2026-10-27

- [x] Add manual mecanum warehouse profiling with shared exact regression
  fixture, per-stage counters and terminal candidate/strict rejection counts.
- [x] Add rotated-map/contact/narrow-gap/mecanum 4-disc prefilter equivalence
  tests; retain default OFF pending execution and verification.
- [x] Add exact-input certified Euclidean vs obstacle-grid heuristic A/B
  benchmarks across hard scenarios and optional warehouse target.
- [x] Make `PROFILE_HOTPATHS` an opt-in Docker Compose build argument.
- [ ] Run CTest and representative ON/OFF profile comparisons; document
  expansion, memory and latency outcomes before changing default behavior.

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
  Expose diagnostic counts and retain certified Euclidean as an explicit mode.
- [x] Select bounded lazy obstacle-grid as the default (weight 1.0),
  using the supplied cross-scenario and warehouse performance evidence.
  Certified Euclidean remains explicitly selectable for testing and
  applications requiring an admissible lower-bound heuristic.
- [ ] Verify the changed default with exact-checkout Catch2 and full
  OMPL/Docker regression tests; rerun representative multi-repeat profiling-OFF
  performance/quality trials. No claim of passing tests until executed.
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
- [x] Make the bounded lazy **obstacle-grid heuristic the default** at weight 1.0, with optional OMPL RS distance guidance and explicit certified Euclidean fallback. Historical ADRs 0015/0027 used certified Euclidean as the default; this newer evidence-driven decision supersedes that choice.
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

**Updated November 4, 2026:** The current priorities are the
[roadmap at the top of this file](#current-priorities--2026-11-04).
The previously proposed generic smoother is now split into:
(1) prerequisite-gated matched smoothing-quality benchmark,
(2) optional model-feasible curvature-continuous smoothing,
(3) separately benchmarked refiner throughput, and
(4) downstream time parameterization. These are **not** shipped
features and must retain a strictly validated original-path fallback.
Legacy checkboxes below remain historical proposals and may overlap
with capabilities already implemented in W8.

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
