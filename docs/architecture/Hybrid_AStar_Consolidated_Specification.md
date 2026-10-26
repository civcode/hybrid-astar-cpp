# Hybrid A* C++ — Consolidated Architecture, Data and Implementation Specification

**Document type:** Cross-review synthesis and detailed proposed engineering specification  
**Date:** 2026-10-26  
**Project:** `civcode/hybrid-astar-cpp`  
**Baseline reviewed:** branch `docs/architecture-baseline`, commit `dff94bcbcd772aaf4758067794d497bc28c3475b`  
**Status:** Consolidation of three review documents. **Not an approved ADR, a repository change, or evidence that the planner has been implemented.**  
**Initial scope:** Standalone C++20, rear-axle Ackermann Hybrid A* vertical slice; explicitly distinguish deferred models and integrations.

> **Interpretation rule.** The three inputs are proposals/reviews, not three simultaneously binding standards. This specification labels **Consensus** (the three substantially agree), **Baseline** (documented settings from the reviewed branch as reported by the reviews), **Recommended synthesis** (a conservative implementation choice made for this consolidated document), **Alternative** (a source-proposed option), and **Decision required** (genuine disagreement or proof/benchmark still outstanding). Do not interpret an illustrative C++ snippet, acceptance metric, or prospective ADR number as a frozen public API.

## Contents

1. [Introductory comparison: agreement, differences and disposition](#1-introductory-comparison-agreement-differences-and-disposition)
2. [Evidence, scope and claims](#2-evidence-scope-and-claims)
3. [System context and architectural invariants](#3-system-context-and-architectural-invariants)
4. [Collected baseline and configurable parameters](#4-collected-baseline-and-configurable-parameters)
5. [Map, coordinate frames, occupancy policy and snapshots](#5-map-coordinate-frames-occupancy-policy-and-snapshots)
6. [Model state, geometry and motion representation](#6-model-state-geometry-and-motion-representation)
7. [Collision geometry, exact checker and optimization](#7-collision-geometry-exact-checker-and-optimization)
8. [Motion validation, swept-volume options and final checking](#8-motion-validation-swept-volume-options-and-final-checking)
9. [Search graph, keys, costs and priority queue](#9-search-graph-keys-costs-and-priority-queue)
10. [Heuristic providers and mathematical obligations](#10-heuristic-providers-and-mathematical-obligations)
11. [Reeds–Shepp connector and solution-selection policies](#11-reedsshepp-connector-and-solution-selection-policies)
12. [Clearance scoring and path refinement](#12-clearance-scoring-and-path-refinement)
13. [Path, result, validity evidence and consumer contract](#13-path-result-validity-evidence-and-consumer-contract)
14. [Ownership, concurrency, caching and portability](#14-ownership-concurrency-caching-and-portability)
15. [Proposed internal interfaces and execution flow](#15-proposed-internal-interfaces-and-execution-flow)
16. [Verification, tests and benchmarks](#16-verification-tests-and-benchmarks)
17. [Work packages, acceptance gates and dependencies](#17-work-packages-acceptance-gates-and-dependencies)
18. [Decision register and remaining proof obligations](#18-decision-register-and-remaining-proof-obligations)
19. [Documentation reconciliation and traceability](#19-documentation-reconciliation-and-traceability)
20. [References and source boundaries](#20-references-and-source-boundaries)

---

## 1. Introductory comparison: agreement, differences and disposition

### 1.1 The three source documents

For compact traceability, this document uses these labels:

- **A — Review:** `hybrid-astar-architecture-review.md`, sections 1–6, especially §2–5. A deliberately simplifying review with a detailed alternative collision pipeline, footprint-aware grid heuristic and milestone plan.
- **B — Improvement specification:** `hybrid_astar_architecture_review_and_implementation_spec(1).md`, sections 1–21. An implementation-oriented assessment with eleven numbered improvements (IMP-01…IMP-11), validity/evidence contracts, quality-aware analytic termination and phased delivery.
- **C — Verdict/specification:** `hybrid_astar_review_verdict_and_implementation_spec(1).md`, sections 1–13. A more proof-conservative decision-by-decision review, exact collision algorithms, rigorous heuristic/reachability qualifications and work-package gates.

All review the **same repository baseline** as of 2026-10-26. A reports one successful local OMPL-disabled CMake/CTest smoke build with GCC 13.3; B explicitly says that its assessment did not locally build or benchmark; C reports design/scaffold evidence and no repository implementation changes. These accounts are **not contradictory**: they describe different review activities. None demonstrates the completed planner, OMPL-enabled connector or execution safety. [A §1.1; B §1.2; C §1]

### 1.2 Areas of substantive agreement

| Topic | Shared conclusion | Design implication | Source locations |
|---|---|---|---|
| Overall architecture | Keep a standalone ROS-independent C++20 core and optional adapters; the underlying Hybrid A* search decomposition is sound. | Do not replace the architecture wholesale. | A §2, §3.8; B §2–4; C §1–2, §8 |
| First deliverable | Build a complete Ackermann vertical slice before mecanum/articulated or speculative generic registries. | Generalize from a working second model, not hypothetical APIs. | A §2.2, §3.8, §5; B IMP-07, §18; C §8, §10 |
| Hybrid state/index | Continuous arrivals and true motion endpoints; discretized `(x,y,yaw,incoming gear)` keys and initially one active arrival per key. | Acknowledge loss of completeness/optimality; preserve immutable ancestry. | A §2 A1–A2, §4.4; B §4, §11; C §1, §6 |
| Collision default | Perimeter-only checks against an un-inflated/raw grid can miss fully enclosed blocked cells; not acceptable as ordinary full-body validation. | Use complete footprint vs blocked cell squares; keep diagnostic boundary-only mode separate, if at all. | A B1, §3.1, §4.1; B IMP-01; C §3 |
| Broad-phase correctness | EDT/disc masks may safely bypass only when clearance is **proved**; uncertain => exact footprint check. | No broad-phase false-safe, no disc-overlap-as-collision shortcut. | A §4.1; B IMP-08; C §3.3 |
| Motion checking | Fixed 0.10 m rear-axle samples do not establish continuous swept safety for a long body. | Account for whole-body motion, preserve cusps/endpoints and distinguish sampled vs certified assurance. | A §3.3, §4.1.4; B IMP-02; C §3.4–3.5 |
| Final validation | Do not trust the same weak/optimized collision path as the last gate; use strict complete-body checking. | A candidate failing final validation cannot become `Success`. | A §4.1.6; B IMP-01/11; C §3.5 |
| Ackermann integration | Six fixed-curvature actions can be propagated analytically; RK4 is needless in the Ackermann hot path. | Use closed-form SE(2) arcs; retain generic numerical integration for later models. | A C1, §4.3; B §11.3; C §5 |
| Search costs | Forward 1.00× length, reverse 1.10×, gear change +0.50 equivalent meters are provisional baseline terms. | Finite nonnegative edge costs, consistent for primitives and signed RS sections. | A A6, §4.6; B §13, §16; C §2, §7 |
| Heuristic structure | The RS-plus-obstacle-aware-grid **idea** is useful; guarantee claims need careful semantics. | Implement goal-region-correct fallback and benchmark the combined performance heuristic. | A A3/C2; B IMP-06; C §4 |
| Reeds–Shepp | Attempt scheduler should use search-relative units, not map resolution; OMPL evaluator should reuse resources. | Keep first-node attempt and adaptive cadence; measure RS attempt cost. | A A5, §3.10, §4.5; B IMP-03/05; C §6.4, §8.3 |
| Map integrity | Immutable, versioned snapshots and one effective occupancy policy per request. | Cache compatibility is strict; mapping changes never mutate an active plan. | A D1, §4.7; B §14; C §1, §8.2 |
| Results | Distinguish geometric feasibility, collision evidence and physically executable timed motion. | Return metadata, actual endpoint/gear/cusps; external trajectory/actuator validation remains necessary. | A §4.4–4.7; B IMP-04/11; C §1, §3, §6 |
| Evidence and prioritization | Parameter defaults are starting points, not verified performance or safety guarantees. | Tests first, benchmarks before accelerator/caching claims; reconcile contradictory ADRs. | A §3.9, §4.8–5; B §17–20; C §2, §10–12 |

### 1.3 Where the sources propose meaningfully different concepts or implementations

This is the primary **difference register**. Rows represent alternatives, not minor differences in wording. The last column is a **recommended synthesis**, not an assertion of three-way agreement.

| ID | Issue | A — Review | B — Improvement specification | C — Verdict/spec | Consolidated disposition |
|---|---|---|---|---|---|
| D01 | Swept-collision assurance | Derives resolution-based sample spacing and **inflated sampled footprints** using a half-step body-motion margin; argues this can certify the sweep for fixed-curvature rigid motions. | Body-point-motion adaptive subdivision as execution-facing default; sampling alone **is not** certification; certification needs proven swept envelope/clearance. | Fixed ≤0.10 m search and ≤0.05 m strict final samples are **sampled** assurance only; conservative swept proof is future. | **Decision required.** Ship explicit full-footprint sampled/adaptive evidence. Treat A's inflated-envelope method as a separate certifying implementation candidate that requires proof and independent tests. |
| D02 | Collision prefilter timing | 4-disc cover and EDT are integral to proposed initial collision pipeline. | First ship exact checker; make old mask or alternative envelope a measured optional accelerator. | Exact checker first; optional 4/5-disc EDT cover only after soundness and total-planning benchmark. | **Recommended:** exact checker first; later optional verified multi-disc prefilter. |
| D03 | Disc count | Default 4 longitudinal covering discs. | Does not mandate multi-disc count; discusses existing all-yaw disk/per-heading alternative. | Benchmark 4 vs 5; neither universally optimal. | Benchmark per geometry; 4-disc trial is not a release invariant. |
| D04 | Footprint-aware 2D field | Treat inscribed-radius inflation as core improvement; prune disconnected graph (`pruneUnreachable=true`) and early `NoPath`. | Keep ordinary grid heuristic as **guidance**; no `NoPath` from approximate graph. | Separate permissive, fully exhausted reachability graph may prove disconnection; ordinary performance field cannot. | **Recommended:** guidance never causes early `NoPath`. Optional separate permissive reachability graph only after its complete superset proof. |
| D05 | Grid cell infeasibility test | `D[c] + sqrt(2)*r < rho_in` (blocked-center EDT) and restrictive diagonal corner rule. | Conservative full-body caution; no certified reachability formula. | `Dcenter[c] + r/sqrt(2) < rho_in` and **permissive** vertex/diagonal adjacency for reachability. | **Decision required for proof.** Do not treat A's field/adjacency as proven `NoPath` oracle. Preserve C's proposed permissive graph independently of performance field. |
| D06 | Octile scaling/admissibility | One heuristic; contemplates footprint-aware graph distances with caveat that numeric admissibility is unproven. | Two explicit certified/performance modes; default combined guidance. | Explicitly rejects universal certification by dividing octile by ≈1.082392; separates distance bound and connectivity proof. | Keep region-aware Euclidean certified baseline and combined noncertified performance mode; no universal octile proof claim. |
| D07 | API heuristic modes | Drop production-facing certified/performance distinction; keep zero/Euclidean for tests. | Expose typed `CertifiedEuclideanToGoalRegion` and `PerformanceReedsSheppAndObstacleGrid`. | Retain certified vs performance meanings, possibly simpler API. | **Recommended:** retain typed guarantee metadata and two selectable modes, with minimal code. |
| D08 | Search reopening | Start with closed-no-reopen; benchmark before adding reopen. | Keep stale heap plus reopening. | Keep versioned heap and reopen by default; benchmark disablement. | **Recommended:** reopen; optional no-reopen benchmark only. |
| D09 | Key representation | Pack map-relative `(x,y,yaw,gear)` into fixed 64 bits immediately. | Readable key initially; memory layout profiling first. | Packing deferred until bounded injective indexing is proved and profiling warrants it. | **Recommended:** readable bounded key; pack only with exact range proof/benchmark. |
| D10 | Primitive length | Auto-compute from bins/radius, e.g. 1.25× maximum scale; round to samples. | Keep 0.50 m fixed baseline initially; measure. | 0.50 m provisional; offer auto-from-discretization candidate but no uniqueness guarantee. | **Recommended:** fixed 0.50 m default, optional auto policy benchmark. |
| D11 | Clearance-cost phase | Implement early; disc EDT lookups allegedly nearly free, `w=0.15`, `d=0.40`. | Disabled for first correct release, optional measured P2; compute from actual oriented body/boundary. | Add after functioning end-to-end feasibility; disc-cover proxy can over-penalize tight passages. | **Recommended:** default 0 at MVP, separately benchmark exact/boundary-based clearance term and disc proxy. |
| D12 | Analytic termination | First collision-free RS attempt may terminate; no extensive incumbent policy initially. | `FirstFeasible` or **default-proposed bounded `ImproveIncumbent`** (`2500` extra expansions illustrated). | `FirstStrictlyValidated` baseline; optional cost-gated or continue-improvement. | Make both explicit; MVP `FirstStrictlyValidated`, optional benchmarked improvement; default for product remains decision. |
| D13 | Strict final failure handling | Immediately return `ValidationFailed` when strict independent check rejects path. | Return structured failure; preserve only already validated incumbent; raw fallback subject to consumer constraints. | Optional one-time bounded exact-mode retry, then structured failure. | **Recommended:** never success for failed candidate; one bounded strict retry is opt-in, not unbounded recursion. |
| D14 | Final-validation density | `min(ds,r/2)` with extra 0.5r inflation (A's separate validation policy). | Adaptive based on body point movement; no universal fixed bound. | Initially ≤0.05 m with body-motion subdivision as needed; no unearned swept assurance. | Strict full-body final checker, independent of EDT bypass; adaptive and threshold profile remains configurable. |
| D15 | `Unknown::Reject` | Drop/defer entire-map Reject. | Retain an explicit full-snapshot strict option. | Retain `Reject` as optional strict map-validation mode, default `Occupied`. | **Recommended:** preserve optional strict behavior but not normal SLAM default; never silently change it to ROI-only. |
| D16 | Snapshot cache | Simpler request-local fields; cross-request frontier deferred. | Document coherent map+policy+goal keyed caches; allow later reuse if beneficial. | Request-local first; cross-request retention benchmark gated. | Request-local mutable frontier first; optional immutable caches only after profiling. |
| D17 | ADR strategy | Collapse/supersede many ADRs; keep one concise normative spec and tuning log. | Add new numbered superseding ADRs, keep provenance. | Annotate historical ADRs clearly; one current normative architecture contract. | Maintain historical ADR lineage; publish one current normative spec and a parameter/tuning log; exact ADR numbering optional. |
| D18 | Validation/execution scope | Full collision envelope is major focus. | Explicit four-stage geometric/sampled/swept/actuator evidence separation. | Strongly distinguishes sampled-vs-swept, no physical actuator guarantee. | Use B/C evidence taxonomy and A's possible sweep algorithm only if independently established. |
| D19 | Build evidence | OMPL-OFF local smoke passed on one GCC version. | No build performed in B; OMPL ON unverified. | No claim of repository changes; toolchain support open. | No claim of completed implementation or OMPL-enabled test coverage. |

### 1.4 A conservative integration profile (proposed, not ratified)

The remainder of this document specifies a buildable **common-core path** while retaining explicit alternatives: a full-footprint exact checker; immutable request snapshot; six exact Ackermann arcs; `(x,y,yaw,gear)` one-label search with versioned heap/reopening; finite goal-region Euclidean fallback; strict independent post-reconstruction validation; reusable native-wrapped OMPL RS; stable path/event metadata. Sophisticated collision covers, lazy obstacle heuristic, clearance cost, incumbent improvement, reachability proof, smoother and additional models are independently selectable increments. Where A makes stronger claims than B/C, the stronger claim is **not inherited by default**.

**Blocking questions before advertising the output as execution-ready:** What sampled or swept evidence does the consuming robot require? Which steering/gear timing constraints are imposed downstream? Is a validator failure retried or only reported? These are application decisions, not facts established by the three reviews.

---

## 2. Evidence, scope and claims

### 2.1 Observed baseline versus future system

The reviewed branch is mostly architecture/ADR documentation plus limited Ackermann configuration, Reeds–Shepp distance adapter, analytic-attempt scheduler and smoke tests. A reports approximately 3,000 documentation lines and approximately 300 code lines at review time; figures are its estimates, not newly recounted here. Absent or incomplete in the reviewed branch are the full Hybrid A* search, primitive-generator integration, complete map/collision subsystem, 2D Dijkstra heuristic, full signed analytic connector and refinement. The three documents specify proposed components rather than verified production behavior. [A §1; B §1; C §1]

### 2.2 Boundary of this specification

- **In:** C++20 library component boundaries, map and geometry definitions, exact Ackermann motion, search, heuristics, collision and final path validation, RS adapter, diagnostics, acceptance and release criteria.
- **Interface only for now:** optional Nav2/ROS 2 adapter, model-general concepts, trajectory generator/MPC, path refiner, cross-request caches.
- **Out of the first implementation:** timed vehicle dynamics, steering-rate guarantees, dynamic obstacle prediction, multi-trailer self-collision, proven globally optimal search, resolution completeness, automatic safety certification for real vehicles.
- **External precedents:** Stanford Junior's 4D Hybrid A* state / two heuristic concepts / smoothing; Nav2 Smac's analytic-expansion and engineering choices. Neither historical design authorizes a proof or tuning value for this project. [A §1.2; B §2.4; C §1]

### 2.3 Requirement levels

- **P0 — correctness/output gate:** no erroneous full-body success, proper map/unknown handling, correct provenance, final validator, explicit incomplete status.
- **P1 — connected vertical slice:** continuous Ackermann state, search key/parent chain, finite costs, reconstruction, RS if enabled, deterministic controls and basic tests.
- **P2 — informed quality/acceleration:** combined heuristic, optional EDT/disc, clearance preferences, improvement mode, measurable caching.
- **Future:** models beyond Ackermann, robust timed execution, incremental maps, a formally verified swept checker where required.

These priorities synthesize B's IMP and C's WP ordering; A advocates bringing some P2 elements (EDT, clearance, grid inflation) forward. That disagreement is intentionally visible in §1.3.

---

## 3. System context and architectural invariants

### 3.1 Component relationship

```text
Native application/CLI                       Optional ROS 2 / Nav2 adapter
         \                                               /
          +---------------- Planner Facade --------------+
                            |  PlanRequest
                     Immutable PlanContext
           pinned MapSnapshot + model/geometry + policies
                            |
                     HybridAStarSearch
         +------------------+------------------+
         |                  |                  |
  Motion/Primitive     HeuristicProvider   CollisionValidator
       Model          Euclidean/RS/Grid    exact / optional EDT
         |                  |                  |
         +----------- AnalyticConnector ------+
                            |
                  Accepted segment records
                            |
                   Exact path reconstruction
                            |
                  Strict FinalValidator
                            |
                  Validated GeometricPath
                            |
                   Optional PathRefiner
                            |
                   Independent revalidation
                            |
            External trajectory / actuator layer
```

### 3.2 Invariants to test and expose

1. Search nodes retain **continuous full states**; indexing is approximate but must not overwrite actual endpoints.
2. The complete key includes reachability-relevant discretized coordinates and historical mode (incoming gear for the first Ackermann model).
3. Every accepted transition is a model-evaluated segment; every recorded parent/action reference survives later label improvement.
4. All costs are finite, nonnegative and in **equivalent meters** for the baseline; nominal motion length cannot be negative.
5. Full-body collision at each *claimed checked pose* includes interior occupied cells and map boundary contact; broad phase can only safely bypass exact validation when proven conservative.
6. Search, Dijkstra, EDT, RS connector, final validator and refiner use one pinned map/occupancy interpretation for a request.
7. A goal is a tolerance **region**; actual reached state is returned. Never replace it with an untraversed nominal target.
8. Sampled-pose checking, adaptive sampled checking, continuous swept certification, kinematic feasibility and timed actuator feasibility have distinct evidence.
9. Failing, cancelled or budget-exhausted **required validation** cannot be upgraded to success.
10. Finite count-based work budgets and stable tie breaking are deterministic for identical input and cache lifecycle; wall-clock aborts can change termination.
11. Default is a feasible-path geometric initializer, not classical globally optimal A*, resolution-complete planning or automatic executable commands.
12. No ROS, OMPL, TF, YAML or library-specific polygon types leak into a model-independent core API unless an explicit adapter boundary is chosen.

---

## 4. Collected baseline and configurable parameters

These values describe the **reviewed configuration baseline/fixture**, not universally suitable defaults. The new behavioral options in later sections are proposals.

| Parameter | Reported baseline | Scope / guarantee |
|---|---|---|
| Search XY bin `b_xy` | 0.25 m | Independent of occupancy-map resolution |
| Heading bins | 72 = 5° each | Wrapped yaw indexing |
| Key mode | incoming forward/reverse gear; special no-gear start | Not just `(x,y,yaw)` |
| Active labels | one continuous arrival / complete key | Approximate dominance by lower `g` |
| Minimum turning radius `R_min` | 4.0 m | Ackermann test fixture |
| Footprint | x ∈ [−1.0, 3.6] m, y ∈ [−0.9, 0.9] m | 4.6×1.8 m rectangle about rear axle |
| Body maximum radius `rho_max` | `hypot(3.6,0.9)` ≈ 3.711 m | Farthest footprint point |
| Inscribed reference disc `rho_in` | 0.9 m | For this fixture only |
| Primitive controls | F/R × left / straight / right | Six constant-curvature motions |
| Primitive length | 0.50 m | Provisional tuning setting |
| Stored primitive/check samples | ≤0.10 m arc-length | Not continuous swept guarantee |
| Strict re-sampling candidate | ≤0.05 m | C's recommendation, not universal proof |
| Goal XY acceptance | ≤0.25 m | Independent configurable tolerance |
| Goal yaw acceptance | ≤10° wrapped difference | Independent configurable tolerance |
| Cost forward | `1.00 × L` | Equivalent meters |
| Cost reverse | `1.10 × L` | Equivalent meters |
| Cost gear switch | `+0.50 m` | Includes incoming→first analytic gear when changed |
| Analytic scheduler ratio | 3.5 | Search-relative units recommended |
| Combined performance heuristic | `max(Reeds–Shepp, obstacle grid)` | **Not certified** for tolerant goal |
| Unknown policy | `Occupied` | `Free` is provisional; `Reject` strict optional |
| Known-cell traversability | `cell <= threshold`, default threshold 0 | `-1` handled separately |
| Illustrative map resolution | 0.20 m | Fixture; also test 0.10, 0.05, 0.025 m |
| Optional clearance `lambda` | 0.15 | Experimental only; not first MVP default here |
| Optional clearance influence radius | 0.40 m | Experimental only |

### 4.1 Parameterization decisions to avoid premature standardization

`primitiveLengthMeters=0.50` is retained for the first measured series. A alternative proposes `L_auto = 1.25 max(sqrt(2) b_xy, R_min DeltaTheta)` (for the fixture approximately 0.44 m before rounding); its appendix rounds this to ~0.5 m. C proposes the same scales as an **initial tuning mode** rather than a guarantee that actions map to distinct keys. Record `FixedMeters` and optionally `AutoFromSearchDiscretization` as separate configuration modes; do not make map-cell resolution determine primitive length. [A §4.3, Appendix B; B §16; C §5.2]

### 4.2 Configuration validation

Validate fixed map/model/configuration once when constructing a request context or validated configuration object. Check positive finite map resolution, turning radius, primitive extent, bins, spacings, occupancy threshold range, nonnegative finite cost weights, valid goal tolerances, finite start and goal states, finite model transforms, time/cancellation budgets and compatible enabled providers. Only **dynamic state** and derived numeric results require per-query validation. Do not unconditionally run entire static `validate()` from `quantize()` and `edgeCost()` on the hot path. [A §3.11; B §9; C §8.3]

---

## 5. Map, coordinate frames, occupancy policy and snapshots

### 5.1 MapSnapshot contract

The incoming occupancy grid is a width×height X-fastest row-major array of `-1` for unknown or `0…100` for known occupancy (not Nav2 8-bit graded costs). Capture resolution, full SE(2) world-to-grid origin (translation + yaw), frame/source identity and revision. Pin `shared_ptr<const MapData>` with every request. World position transforms as `p_mapLocal = T_worldMap^{-1} p_world`; use **floor**, not truncation, for negative coordinate indices. A cell is a **closed physical square** for contact testing; numerical edge/tangency tolerance must be explicitly defined. Outside the finite map is blocking even when unknown map cells are provisionally free. [A §4.7; B §5, §14; C §3]

### 5.2 Occupancy classification

- `UnknownCellPolicy::Occupied`: `-1` blocks movement; conservative default.
- `UnknownCellPolicy::Free`: `-1` is provisionally traversable **within supplied map bounds**; do not claim real-world occupancy certainty.
- `UnknownCellPolicy::Reject`: reject request/map if *any* cell in the full snapshot is unknown; B/C retain it as a strict optional mode while A proposes dropping/deferment. If retained, do not silently replace it with a region-of-interest scan.
- Known value `v` is traversable iff `0 <= v <= maxTraversableOccupancy`; threshold equality is free. Malformed values trigger map validation errors.

Pin the effective known-cell threshold, unknown policy and map revision for the entire request. Explicitly distinguish `InvalidMap`, `RejectedUnknownMap`, `NoPath` and `StartInCollision`/`Goal` conditions.

### 5.3 Mutable source, immutable request

Build a new map privately, then publish a shared immutable snapshot using a toolchain-compatible publication method. `std::atomic<std::shared_ptr<...>>` availability must be assessed against supported compiler/libstdc++ (GCC 11/ROS Humble is flagged); a mutex-protected publication handle is a portable option. The planner must not observe mid-request SLAM updates; an external execution monitor decides whether a planned path is still fresh. [A §3.11; B §14; C §8.2]

---

## 6. Model state, geometry and motion representation

### 6.1 Minimal Ackermann model

Initial state: `(x, y, yaw)` at rear-axle center; incoming gear is search history, not a physical fourth coordinate. Six controls: direction `sigma ∈ {-1,+1}` and signed steering curvature `kappa ∈ {-1/R_min,0,+1/R_min}`. Distances `s` are nonnegative traveled reference arc length; for a segment:

```text
dx/ds   = sigma * cos(yaw)
dy/ds   = sigma * sin(yaw)
dyaw/ds = sigma * kappa
```

For `kappa != 0`, from `(x0,y0,theta0)` to progress `s`:

```text
theta(s) = theta0 + sigma*kappa*s
x(s)     = x0 + (sin(theta(s)) - sin(theta0))/kappa
y(s)     = y0 + (cos(theta0) - cos(theta(s)))/kappa
```

For `kappa=0`:

```text
x(s) = x0 + sigma*s*cos(theta0)
y(s) = y0 + sigma*s*sin(theta0)
theta(s) = theta0
```

Use stable near-zero-curvature evaluation (sinc/half-angle or verified straight limit); do not wrap the internal arc calculation across discontinuities, only normalize output/indexing as required. Equivalent signed-distance notation is `d = sigma s`, so other documentation may write yaw change `kappa*d`. The primitive's **stored geometric extent remains positive**, gear explicit. [A §4.3; B §11.3; C §5]

### 6.2 Primitive catalogue and samples

Precompute model-local sections for six actions, or use one exact analytic evaluator on demand; apply current node's full SE(2) pose to instantiate a section. Provide `evaluate(s)` for every `0 <= s <= L`, deterministic ordered nominal knots including `s=0`, `s=L` and intermediate `s_i=i L/N` where `N=ceil(L/step)`; avoid floating incremental drift. Keep the exact evaluator for later strict subdivision rather than interpolating endpoints with straight chords. Test forward/reverse symmetry, heading wrapping, SE(2) equivariance, near-zero curvature, finite outputs, exact endpoints and duplicate-key statistics across sub-bin starts.

### 6.3 Attached collision geometry and future model seams

The Ackermann fixture is a filled rigid rectangle with body-frame corners `(-1,-0.9), (3.6,-0.9), (3.6,0.9), (-1,0.9)`. Preserve abstraction for circles, rectangles, convex polygons and compound collision bodies, each attached to a named model body frame with a local transform. An articulated model must supply complete joint state and correct state→body transforms. The initial search kernel should not require every model to have gear, wheelbase, radius or translational `s`; differential-drive in-place yaw and mecanum lateral motion motivate a model-specific progress contract in later milestones. [B §8, §11; C §8]

### 6.4 Segmented geometric motion semantics

A path is an ordered sequence of **evaluable full-state sections**, not a list of straight chords between sampled poses. Each section records start/end full state, control/curvature, direction/gear, geometric extent or model progress, cusp/gear-change events, numeric integration/evaluation identity and join continuity. Model progress can increase with **zero translational distance** for a future differential-drive rotation. Preserve actual final reached state and independent goal XY/yaw errors.

---

## 7. Collision geometry, exact checker and optimization

### 7.1 Normative initial full-footprint pose checker — Consensus

For each checked pose, evaluate the **complete transformed filled** collision body against blocked occupancy **cell squares**. Contact (edge, corner, tangency) counts as collision subject only to a documented numeric epsilon. A perimeter supercover alone is insufficient: an occupied square entirely enclosed by a 4.6×1.8 m body can be missed, particularly on fine grids. A's standalone straight-line experiment reported ~49.5% missed enclosed single-cell obstacles at 0.05 m and ~74.9% at 0.025 m with 0.10 m stored samples; these are scenario-specific simulated numbers, not universal miss rates. [A §3.1, Appendix A; B §5; C §3]

Algorithm for each body and pose:

1. Validate numeric state/body transforms, snapshot geometry and policy.
2. Compose model body transform and shape offset; transform shape into **map-local metric** coordinates using inverse grid-origin SE(2).
3. Compute conservative map-local axis-aligned bounding range, including edge-touching cells.
4. Check entire shape is inside the finite map without boundary contact under the defined policy; out of bounds is blocking.
5. For each candidate **blocked** cell square, use rectangle/convex-polygon vs square separating-axis theorem (SAT), circle vs square nearest-point distance, or another proven no-false-negative geometry test.
6. Return `Colliding` on first intersection; otherwise `ClearAtThisPose`; propagate `InvalidState`, `InvalidMap` and numerical errors distinctly.
7. Count visited candidate cells, blocked cells, exact tests and time. Keep checker usable with **no EDT**.

It is permissible to accelerate **blocked-cell enumeration** (spatial index/row runs), but not to replace interior testing with outline-only testing. Compound bodies are a union: any blocked constituent intersects => collision. For later trailer systems, body–body self-collision is separate and cannot be assumed solved by grid checking.

### 7.2 Conservative covering-disc EDT optimization — Optional

A's preferred 4-disc optimization and C's 4-vs-5-disc experiment use a constructive rectangle cover. For x-range `[x_min,x_max]`, total length `W`, half-height `H`, divide into `k` longitudinal strips:

```text
center_i = (x_min + (i+0.5)*W/k, 0), i=0..k-1
radius_k = sqrt(H^2 + (W/(2k))^2)
```

Each rectangular strip is contained in its disc, so the union covers the **entire rectangle for every yaw** when rigidly transformed. Fixture `W=4.6`, `H=0.9`: `k=3 -> ~1.18 m`, `k=4 -> ~1.068 m`, `k=5 -> ~1.011 m` per disc; single rear-axle-centered all-heading cover has radius `~3.711 m`. The number of discs trades over-cover and query count. Only verified body-specific covers may be used for polygons/articulation; otherwise exact fallback. The cover's circles may extend outside the real vehicle, so overlap is **uncertain, not definitely colliding**. [A §4.1.2; B §12; C §3.3]

Define `D[c]` in meters as the nearest blocked **cell-center** Euclidean distance from grid-cell center `c` for the pinned map/policy. For transformed disc center `p` lying in cell `c`, a sufficient obstacle-free condition is:

```text
D[c] > discRadius + sqrt(2)*mapResolution + numericalMargin
```

The `sqrt(2)*r` allowance comprises up to half a cell diagonal from query point to query-cell center and half a diagonal from blocked-cell center to its physical square. **Also** require the entire covering disc strictly within all four map-local boundaries (or choose exact fallback when it extends outside). If any of these checks fails, an EDT is missing/stale, or geometry/policy metadata differs, return `NeedsExactCheck`. Only all-disc `DefinitelyClear` may skip narrow phase; never infer `DefinitelyColliding` from cover intersection. For a future proved inflated swept cover, the necessary margin must be added consistently to disc radii and boundary checks; do not silently use a pose-only clear proof as an interval proof.

Suggested internal result:

```cpp
enum class BroadPhaseResult { DefinitelyClear, NeedsExactCheck, InvalidInput };
enum class PoseCheckStatus  { Clear, Colliding, InvalidState, InvalidMap };
```

A residual all-heading hull mask or per-heading-bin cover may be benchmarked, but its yaw-bin subangle and subcell margin must cover **every continuous configuration** it claims to classify. Test `DefinitelyClear => exact oracle Clear` across randomized continuous yaw, subcell offsets, obstacles, borders, changed revision and unknown policy. Zero observed false-safe classifications plus a supporting geometric argument is mandatory; random tests alone do not prove soundness. EDT build cost, memory, exact fallback share, total end-to-end planner time and path success determine whether this optimization remains enabled.

### 7.3 Disc roles must never be conflated

- **Outer covering discs:** supersets of actual footprint; if all outer discs are obstacle-free, real footprint is free. They are **safe-bypass evidence**.
- **Inscribed reference disc:** subset of footprint; a vehicle-feasible reference point must allow the disc. It can be used to reject certainly impossible reference **positions** for a separate permissive reachability relaxation. It does **not** establish that full footprint is clear.
- **Obstacle-center EDT:** a discrete distance data structure; cell-center distance is not automatically full-body clearance or continuous-space motion safety.

---

## 8. Motion validation, swept-volume options and final checking

### 8.1 Levels of assurance

| Evidence level | Meaning | What it cannot automatically claim |
|---|---|---|
| `FullFootprintPoseSampled` | Every required declared pose was checked against entire footprint (or safely bypassed). | Continuous swept clearance between poses |
| `FullFootprintBodyMotionAdaptiveSampled` | Extra poses checked until a configured conservative body-point movement threshold/budget was met. | Swept clearance **without** a verified envelope/clearance bound |
| `ContinuousSweptCertified` | A proved and implemented envelope or clearance-bound algorithm covers **all progress between samples**. | Physical steering-rate/acceleration/trajectory feasibility |
| `TrajectoryActuatorFeasible` | Separately verified timed actuation and vehicle-specific constraints. | Real-world obstacle-free future motion on an aging SLAM map |

Labels in B and C vary slightly; this is a normalized taxonomy. The **achieved** level is assigned only after checks have actually completed, not from options flags. [B §8, §15; C §3]

### 8.2 Body-point motion bound and sampling

For rigid planar motion over an interval, if reference translation path length is `DeltaS`, total **absolute** body yaw variation is `DeltaThetaAbs` and the farthest body point is `R` from the reference, a conservative maximum point-travel bound is:

```text
B <= DeltaS + R*DeltaThetaAbs
```

For constant curvature `kappa`, `DeltaThetaAbs = abs(kappa)*DeltaS` and hence `B <= (1 + R*abs(kappa))*DeltaS`. For the fixture at max curvature 1/4 m and `R≈3.711 m`, factor `1+R/R_min≈1.928`; a 0.10 m rear-axle step permits nearly 0.193 m of point travel. Endpoint **wrapped yaw difference is insufficient** if motion turns and turns back; use accumulated variation. Articulation and stationary rotation require per-body frame/joint terms. [A §3.3; B §6.3; C §3.4]

**Sampling profile (recommended MVP):** retain nominal knots ≤0.10 m, including both ends of primitives and every analytic signed-section endpoint/cusp. Subdivide where a verified bound exceeds configured `maxBodyPointMotionMeters`; reevaluate **actual motion at midpoint progress**, not linear interpolation of endpoint poses. At work/recursion/cancellation exhaustion return `ValidationIncomplete`. The threshold is a density control, not a mathematical safety margin by itself.

### 8.3 A's proposed rigid swept-envelope variant — unadopted until verified

A proposes for fixed-curvature rigid sections:

```text
k_disp  = 1 + abs(kappa)*rho_max
DeltaS  <= r/k_disp
m_sweep = 0.5*DeltaS*k_disp
```

and checks **inflated footprints** at each sample by `m_sweep`. Its argument: every intermediate body point lies within a conservative travel-distance bound of its corresponding point on the nearest sample, hence the swept region is contained in the union of inflated sampled footprints. This is **meaningfully stronger than checking uninflated poses more densely**. However, B/C do not accept a particular implementation as already certified. Before enabling `ContinuousSweptCertified`, independently establish and test: (1) exact conservative inflation/dilation for shape–cell geometry; (2) the nearest-sample half-interval bound for *every* point and every section including cusps; (3) reference/body rotational movement under actual evaluator; (4) border checks on inflated bodies; (5) joint/multi-body handling or explicit model restriction; (6) numeric error margins and adaptive termination. An inability to prove or complete a difficult interval must produce incomplete/failed certification, not a weaker successful label. [A §4.1.4–4.1.6; B §6.2–6.4; C §3.4–3.5]

### 8.4 Parent validation reuse

Expansion may omit primitive `s=0` only when parent state was already validated under the **same pinned snapshot, occupancy policy, geometry, numeric validity assumptions and required level**. Validate the global start. Standalone primitive evaluation and strict final path checking cannot inherit an implicit parent shortcut. For analytic sections include the exact section endpoints, every cusp and start/end; avoid missing short degenerate sections. [C §3.4; B §6]

### 8.5 Independent final validator and optional recovery

Reconstruct exact accepted segments first. Execute a separate strict validation policy on the **same pinned map**. At minimum: full-body exact shape–blocked-square checks without trusting search's EDT/cover safe-bypass result, more detailed/rigidly bounded motion resampling and separate budget/evidence. A suggests `min(ds,r/2)` and inflated footprint; C proposes ≤0.05 m initial final sample spacing and optional adaptive body-point movement. B requires body-aware adaptive validation and no unsupported certification. The exact numeric profile and any genuine sweep-certifying algorithm remain application choices. [A §4.1.6; B §5–6, §15; C §3.5]

Failure behavior:

1. On first strict collision, numeric error or incomplete required validation, **discard candidate as success** and record failing state, segment, map revision and validation level.
2. **Minimum acceptable behavior:** return a structured `StrictValidationFailed` / `ValidationIncomplete` without the invalid path as successful output (A/B).
3. **Optional C extension:** retry search **at most once** within an explicit separate budget, with exact checks replacing the disc bypass and stronger configured sampling. Revalidate new candidate strictly. Never retry recursively or fall back to the invalid original candidate.
4. A previously **independently validated incumbent** may remain available after unrelated later candidate failure, but whether cancellation/timeouts return it and with which status must be in a documented consumer contract.
5. Optimization/refinement can replace a path only after equal-or-stronger final validation; if refinement fails, keep the original **validated** path and original evidence, not a fabricated upgraded result.


---

## 9. Search graph, keys, costs and priority queue

### 9.1 Continuous state and one active search label

**Baseline:** the Ackermann search key is `(xBin,yBin,headingBin,incomingGear)` with separate initial no-gear semantics. Bin XY coordinates independently of the occupancy-grid resolution (0.25 m baseline); normalize yaw into 72 bins (5°), preserving the baseline convention of yaw bins centered on multiples of 5° while XY uses floor-aligned bins. Quantize using validated finite coordinates and overflow/bounds checks. The stored node pose remains its **actual continuous arrival**, not a cell center. [A §4.4; B §4.2, §11.5; C §6]

Only one active lower-`g` arrival per complete key is retained initially, acknowledging that a higher-`g` arrival at a different continuous subbin position may have better future reachability. **Do not claim completeness, global optimality or standard weighted-A* cost bounds.** A state that reaches the XY tolerance but violates yaw tolerance is not a goal. Goal acceptance is computed on actual continuous state using `hypot(dx,dy)<=goalXYTol` and `abs(wrap(yaw-goalYaw))<=goalYawTol` and independently validated model/footprint constraints.

### 9.2 Stable ancestry and generational queue

Recommended implementation (B/C): append-only stable `NodeRecord`s that contain pose/full state, stable ID, `g`, `hAtInsertion`, key, incoming gear, immutable parent record ID, accepted primitive/connector descriptor, gear/cusp data and map/context provenance. A separate active-label table maps keys to latest accepted `(nodeID, generation, bestG)`. The open heap has `(f, tieBreak, nodeID, generation)`. On pop, discard a stale generation; when a better arrival replaces an already expanded label, enqueue it again (**reopen**). Never mutate a prior ancestor's pose/parent, because existing child chains may still refer to it. [B §4.2, §11.5; C §6.3]

A differs: immediate packed 64-bit key and **closed-no-reopen** default. Both are **benchmark alternatives**, not consensus requirements. Packed key `[2 gear | 7 yaw | 27 y | 28 x]` is its illustrative map-relative layout, which is valid only over a known checked finite range. The current exposed key includes signed 64-bit XY bins, so lossless packing cannot be assumed. Preserve descriptive struct key until injectivity/range proof and profiling justify packing. [A §4.4.1–4.4.3; C §8.4]

### 9.3 Edge costs and dominance order

Initial cost, in equivalent meters:

```text
C_base(edge) = 1.00*L_forward + 1.10*L_reverse
             + 0.50m*(number of actual incoming/internal gear switches)
C_total(edge) = C_base(edge) + C_clear(edge)   // optional, nonnegative
```

For a normal fixed-gear 0.50 m primitive, one outgoing gear is compared to incoming gear (the initial no-gear state has a separately documented rule). For analytic connectors, break into signed sections and count switches at the boundary from parent gear to first connector section **and** each internal cusp. `L_forward`, `L_reverse` are nonnegative length magnitudes. All nonfinite/negative costs are errors. B/C require the same configured objective for primitive and connector; never compare RS geometric distance directly to a reverse/switch-weighted incumbent.

C's safe successor validation order is:

```text
endpoint = primitive.evaluate(parent.state, L)        // true closed-form state
validate finite/model-state/mode
key      = quantize(endpoint, outgoingGear)
lowerG   = parent.g + C_base(primitive)                // lower bound if C_clear >= 0
if retainedBestG(key) <= lowerG: skip this dominated arrival
else:
    validate actual motion samples / required intervals
    if not valid: reject (or record ValidationIncomplete)
    compute optional nonnegative clearance integral
    tentativeG = parent.g + C_base + C_clear
    if retainedBestG(key) <= tentativeG: skip
    else append immutable node record, replace active label, push heap entry
```

**Important distinction:** pre-collision pruning on nominal cost is only valid within the chosen **approximate one-label key policy**, not a proof of continuous-state dominance. A's suggested early use of `closed[key]` to prune permanently reflects its no-reopen alternative and must not be combined silently with the recommended reopening implementation. When a rejected early-dominated edge is never accepted into the retained search graph, it need not incur expensive collision checks. A clearance cost that may be negative or unknown at pruning time would invalidate the nominal lower bound. [A §4.1.5; B §11.5; C §6.2]

### 9.4 Expansion, budgets and deterministic behavior

Pop best non-stale label; check work budget/cancel/timeout; evaluate continuous goal and analytic-attempt policy; expand six model-defined actions; update incumbent according to selected solution policy. Stable tie order should include `(f, h or other explicit tie, insertion node ID/key)` and fixed primitive/neighbor enumeration order; never depend on pointer addresses or nondeterministic hash iteration. Preserve counts of generated, pruned by key, reopened, expanded, analytic attempts, exact checks, heuristic settlements and final retry work.

Limit types include maximum expansions, maximum generated nodes, queue/memory cap, maximum heuristic settlements/heap operations, maximum adaptive interval subdivisions and an overall optional wall-clock timeout. Count-based limits preserve deterministic work sequence; wall-clock limits are operational aborts and can change output across machines. Differentiate `NoPath` (exhausted search under its stated finite graph semantics) from `BudgetExceeded`, `Cancelled`, `ValidationIncomplete`, `InvalidRequest` and `InternalError`. If one-label quantization prevents a route, an algorithmic `NoPath` is relative to the chosen approximate search configuration, not a claim that the continuous robot cannot reach the goal.

---

## 10. Heuristic providers and mathematical obligations

### 10.1 Why two meanings are necessary

**Guaranteed numeric fallback (recommended):** where each translating edge's cost is at least traveled reference length and nonnegative penalties are added, distance to the **goal position tolerance region** is a lower bound:

```text
h_region(q) = max(0, norm(xy(q)-xy(goal)) - xyGoalTolerance)
```

If a later model's objective is not length-dominated in these units, use `h=0` or establish a new bound. Even a proved lower bound cannot restore global optimality/completeness after merging different continuous states into one discretized key.

**Fast ordering estimate:** the baseline idea is `h_perf(q)=max(h_RS_exactGoal(q),h_grid(q))`; optionally include `h_region` in this max for a robust finite fallback. Because an already accepted tolerance-region state can have **positive exact-goal RS distance**, `h_RS_exactGoal` is not generally an admissible distance to the accepted region. Similarly a cell-center octile grid distance can overestimate continuous travel around obstacles/corners. Report `heuristicCertifiedAdmissible=false` for this performance mode. `h_perf` can order expansion but cannot support admissible `f`-based optimality pruning without separate proofs. [A §3.5; B §10; C §4.1–4.2]

**Mode policy difference:** A prefers a single production-facing combined heuristic and relegates zero/Euclidean to testing. B advocates explicitly selectable `CertifiedEuclideanToGoalRegion` and `PerformanceReedsSheppAndObstacleGrid`; C agrees that certified/performance **semantics** must not be collapsed, though API can be minimal. Recommended small interface retains the two meanings plus `DisabledZero` test mode.

### 10.2 Goal-rooted reverse Dijkstra specification

Construct a deterministic graph over map-local XY cells with 4- or 8-neighbor edges (8 is the reported baseline family); edge length `r` for axial moves and `sqrt(2)*r` for diagonal, unless a documented **separate** cost metric is chosen. Seed every permitted cell whose physical square intersects the accepted **XY tolerance disc**, not just the nominal goal-center cell; the goal-center cell may be blocked while some region endpoint remains feasible. A 2D seed does not itself establish full-body final goal validity or yaw feasibility.

For a lazy field:

1. Build or bind request-local frontier, distance array and states (`Unvisited`, `Queued/Tentative`, `SettledFinite`, `ExhaustedUnreachable`, `WorkLimitReached`). A field may maintain fewer state labels if it represents these distinctions without ambiguity.
2. Use deterministic seed order, `(distance,cellIndex)` heap tie and fixed neighbor order.
3. Settle on pop with a complete monotonic Dijkstra relaxation; only **settled** distances can be reported as completed numeric grid distances.
4. On a query, advance the frontier up to a deterministic settled-node/heap-operation budget. If queried cell remains unsettled or the budget is reached, use finite goal-region Euclidean/RS fallback; **do not return +infinity or fail the plan** merely because the graph entry is missing.
5. At complete frontier exhaustion, mark remaining cells graph-unreachable; such a result is not automatically a physical `NoPath` certificate (§10.4).
6. Freeze a node's `hAtInsertion` in its queued priority for MVP. If later settlement changes the available field value, do not silently rewrite existing priorities; re-evaluation-on-pop would require a separate heap update/evidence contract.
7. Request-local mutable frontier is the default; any reuse requires exactly compatible map revision, policy, threshold, goal seed set, graph and weights.

A proposes node-count/unlimited budgets and restricted diagonals for the inflated 2D field. B/C agree on count-based work limits and settled-only results, but warn about premature unreachable pruning. [A §4.2; B §10.3; C §4.4]

### 10.3 Footprint-aware **performance** field

A wants to improve obstacle guidance using an **inscribed disc** around the rear-axle reference. Its proposed forbidden cell test is:

```text
forbidden_A(c) = blocked(c) OR (Dcenter(c) + sqrt(2)*r < rho_in)
```

and it proposes 8-neighbor Dijkstra with diagonal corners allowed **only if both adjacent orthogonal cells are nonforbidden**, with `pruneUnreachable=true`. That graph may produce more useful guidance in narrow gaps than a raw occupancy grid. B/C support testing footprint-aware relaxations, but not automatically granting `NoPath` from a graph that might exclude a possible continuous cell trace. **Do not unify** performance-graph corner restrictions and certified reachability connectivity. [A §4.2.2; B §10; C §4.3]

The numerical Dijkstra cost on even a permissive XY grid is **not thereby a certified continuous-space distance lower bound**. In particular the unobstructed octile overestimate factor:

```text
alpha = sqrt(1+(sqrt(2)-1)^2) ≈ 1.082392
```

addresses certain orientation errors between cell-center endpoints; dividing all obstacle-graph costs by `alpha` does **not** prove a lower bound under cell rasterization, diagonal rules, subcell queries and tolerant goal regions. C explicitly rejects that universal proof shortcut. [C §4.2]

### 10.4 Separate permissive graph for optional sound `NoPath`

C specifies a stronger **logical connectivity** notion separate from numeric heuristic cost. For the fixture, the vehicle contains a reference-point disc of radius `rho_in=0.9` at every yaw. Any collision-free vehicle reference trajectory therefore requires the corresponding disc to avoid every blocked square. A candidate cell may be declared **certainly impossible** using blocked-cell **center** distance:

```text
Dcenter(c) + r/sqrt(2) < rho_in    // C's conservative proposed cell rule
```

Explanation: any point inside the query cell is at most `r/sqrt(2)` from its center. If nearest blocked **center** remains strictly inside the reference disc for every point, that cell cannot contain a feasible reference point. This is a conservative necessary-condition test; using blocked *centers* is weaker but simpler than exact dilation of occupied squares. The term differs from A's `sqrt(2)*r`; **do not substitute one for the other** without proving the desired containment relation. Treat threshold equality in the permissive direction unless an exact proof handles tangency.

Build a graph whose admissible vertices are all **potentially feasible** map cells, with an **over-permissive adjacency** that includes vertex-touching/diagonal transitions, including cases that strict corner-cut rules would forbid. Include every potentially feasible start cell and every potentially feasible cell whose square intersects XY goal tolerance. Under a verified property that **every continuous valid reference-point trajectory traces connected permitted cells**, a **completely exhausted** reverse connected component that excludes the start is evidence of infeasibility in the *specified static pinned map/policy*, not a statement about unknown real-world occupancy. Heading constraints can only make reachability harder but may not be used to revoke the XY graph's required superset assumption.

**Early `NoPath` may be enabled only if all of these hold:** verified map and inradius, rigorously conservative cell rejection, permissive adjacency and goal seeds, exact map/boundary semantics, complete frontier exhaustion (no settlement/time budget exit), and tested trace inclusion. If any is absent: return `Unsettled/Unknown` and continue ordinary search. B's default prohibition of early `NoPath` and C's conditional proof-based allowance are compatible as a phased design. A's default `pruneUnreachable=true` is a competing **unproven** option for this stricter purpose, not incorporated here. [B §10.3; C §4.3.1–4.3.2]

### 10.5 Heuristic evidence and experiments

Required tests: exact-goal RS positive inside accepted tolerance; goal-center blocked/free subregion; empty-grid octile counterexample and path-around-corner example; raw versus inscribed-disc obstacle field around sub-vehicle doorway; deterministic partial lazy settlement; budget-exhausted graph **cannot** claim unreachable; permissive vertex-adjacent trace; cache invalidation after unknown-policy/goal-map changes; correct finite fallback. Record expansions for zero, region Euclidean, RS-only, grid-only and combined on open field, wall/doorway and U-shaped dead-end. A cites Junior Figure 16 as qualitative motivation (e.g. 21,515 Euclidean expansions versus 10,588 combined in one published example), **not** a quantitative acceptance target for this different project/map. [A §1.2, §4.2.4; B §17; C §4.5]

---

## 11. Reeds–Shepp connector and solution-selection policies

### 11.1 Two separate OMPL operations

Implement a native `ReedsSheppEvaluator` owning an OMPL `ReedsSheppStateSpace` and reusable scratch/state resources per request/planner instance. `distance(from,to)` is a hot-path geometric-length **heuristic**, which must avoid new state-space/state allocation per call; `shortestGeometricPath(from,to)` materializes **signed** RS sections for a scheduled analytic candidate. OMPL types remain inside a private implementation target; thread ownership or locking must be explicit. When OMPL is disabled/unavailable, do not silently advertise RS heuristic/path functionality that is not present. [A §3.10; B §9; C §8.3]

Each connector result must reconstruct section kind (straight/left/right), signed length or gear, exact start/end states, arc-length evaluator, zero/degenerate sections and cusp events. Validate that sum of absolute signed section lengths agrees with OMPL reported geometric length within a defined tolerance and that evaluating all sections reaches the actual nominal target within numeric tolerance. Sample every signed section separately, including cusps and endpoints, and apply the same required collision/strict validation assurance as primitive edges. Failed analytic expansion must **not abort** ordinary grid search.

### 11.2 Scheduler normalized to search, not occupancy resolution

Preserve an RS attempt on the **first eligible popped non-stale search node**, then a per-request adaptive countdown. Replace the existing scheduler argument of closest heuristic distance divided by `map.resolution` with e.g. `closestHeuristicMeters / xySearchBinMeters` (or measured primitive-length units). Initial `attemptRatio=3.5` is provisional. A illustrates:

```text
interval = max(floor(closestSearchBins/ratio), ceil(ratio))
```

Validate the exact existing cadence semantics against scheduler unit tests rather than assuming the expression is the only allowed implementation. Vary occupancy resolution while keeping search bins and scene geometry unchanged: attempt timing should not change solely due to cell-size differences when the input heuristic meters are equal. No hard XY-proximity trigger is required in the baseline. B cautions against silently reintroducing Nav2's optional analytic maximum-length guard; a weighted-cost gate is an alternative quality control. [A §4.5; B §7; C §6.4]

### 11.3 Costing connector sections

Connector geometric length is **not** the configured objective. Sum `1.0*L_forward + 1.10*L_reverse + 0.50m*switches`, counting the first-section gear against the search parent's incoming gear and all internal cusps; apply identical enabled clearance integration to each section. An RS shortest-length family can be more expensive than an alternative in reverse-heavy weighted cost even when shorter geometrically. Validate incoming gear, section signs, endpoint/cusp consistency and no unexpected zero/nonfinite lengths.

### 11.4 Distinct termination/quality concepts — Decision required

**Policy F — `FirstStrictlyValidated` (A/C baseline, B selectable):** return the first path that has passed required complete-path strict validation; fast feasible-path initializer without quality bound. A far-field but feasible long RS connector may be returned; do not assert it is cost-optimal.

**Policy I — `ImproveIncumbent` (B's suggested quality-oriented default; C optional future):** when first strictly valid ordinary or analytic path is found, store actual weighted cost, evidence and immutable complete path. Continue for a **separate bounded number** of expansions (B illustrates 2,500, but this is unmeasured) and optionally a separate time budget. Replace incumbent only by strictly lower **fully validated** configured cost. Return best verified incumbent with `ImprovementBudgetExhausted` termination reason if allowed by public contract. Do not prune based on `h_perf` as though it were admissible, and do not claim an A* suboptimality factor.

**Policy G — optional `CostGated` connector:** reject analytic candidates exceeding a finite configured weighted-cost guard or failing to improve an already verified incumbent; a length cap is optional and can rule out otherwise feasible paths. Trade latency versus reverse-heavy shortcuts empirically.

A consumer must know which policy ran and whether search stopped because of first candidate, improvement count limit, timeout/cancellation or fully exhausted open set. A validated incumbent survives later invalid analytic attempts. Cancellation behavior with incumbent remains an API choice (`CancelledWithIncumbent` versus success-with-termination-reason). [A §4.5; B §7; C §6.4]

---

## 12. Clearance scoring and path refinement

### 12.1 Why clearance cost is controversial

All three accept that a geometric shortest raw path can graze obstacles and that a downstream optimizer may fail to change a route's **homotopy class**. They differ on timing and proxy accuracy. A advocates clearance in an early milestone as nearly free using existing covering-disc EDT lookups. B/C regard it as an **optional post-correctness quality increment**: disc covers protrude beyond the actual footprint, so their nearest-disc distance can severely undervalue clearance in tight but safe corridors. **Recommended MVP value is zero**, preserving exact baseline cost until whole-planner benchmarks establish benefit. [A §3.7, §4.6; B §13; C §7]

### 12.2 Cost equation and measurement choices

For accepted segment progress `s` of geometric extent `L` and a chosen oriented-full-body clearance estimate `d_F(s)`:

```text
phi(d) = max(0, 1-d/d_influence)^2
C_clear = lambda * integral_0^L phi(d_F(s)) ds
```

`lambda >= 0` and `d_influence > 0`; experimental fixture values `lambda=0.15`, `d_influence=0.40m` are unverified. Use deterministic trapezoidal integration over segment samples including endpoints/cusps. Add cost **before** active-label dominance relaxation; because it is nonnegative, base cost is still a valid cheap predecessor-to-key lower bound. Apply to normal primitives and complete analytic signed paths alike.

**A's inexpensive proxy:**

```text
d_cover ≈ max(0, min_i[D(cell(discCenter_i)) - sqrt(2)*r - discRadius_i])
```

This is an approximate conservative-in-one-direction obstacle-distance proxy for over-covering discs, **not the actual exact oriented body clearance**, and may force inappropriate long detours. **B/C alternative:** sample transformed physical boundary/full-body geometry against EDT with documented cell-square correction, or compute exact footprint–square nearest distance if needed. Neither approximate clearance score proves collision-free movement. Separate map-boundary clearance from obstacle EDT if required. Experimentally compare `lambda=0` with explicit boundary-based clearance, disc proxy and (where feasible) exact body clearance, measuring final validated minimum clearance, route selection and latency.

### 12.3 Optional post-search refinement

The raw geometric path is a model-feasible sequence of explicit primitives/RS sections; it is **not** automatically a speed-planned command sequence. An optional refiner must respect locked start/goal tolerance, full body geometry, static obstacles, curvature, cusp/gear topology and model controls. Consider local obstacle-aware shortcutting/low-cost smoothing first; a heavier Junior-style conjugate-gradient or control-parameter optimizer needs application-specific benefit. Post-optimization controls/sections must be re-evaluated and independently validated on the same pinned snapshot. If refinement fails or produces a colliding/curvature-invalid path, retain the previously validated raw path **only for consumers allowed to use its stated geometric/evidence level**. It must not be advertised as actuator-rate-feasible merely because it is collision-checked. [B §8, §15; C §7]

---

## 13. Path, result, validity evidence and consumer contract

### 13.1 GeometricPath contract

Each accepted segment stores at least: model and validated config identity, complete start/end states, unique segment ID/kind, reproducible evaluator `state(progress)`, numeric extent and progress meaning, gear and signed direction, control/curvature, stable parent/action origin and explicit cusps/gear-switch events. Joins record position/yaw continuity, curvature/steering discontinuity, motion-direction changes and stationary events as relevant. Preserve actual reached terminal pose, XY/yaw tolerances and map revision. Path reconstruction must follow immutable ancestry in chronological order; do not replace arcs with straight interpolation for validity checks. [B §8, §15; C §6]

### 13.2 Achieved validity attributes are independent

- `modelGeometryValid`: state/control and geometric constraints for evaluated sections.
- `collisionEvidence`: `None`, `BoundaryOnlyDiagnostic` (never ordinary success), `FullFootprintPoseSampled`, `FullFootprintBodyMotionAdaptiveSampled`, optional `ContinuousSweptCertified`.
- `solutionQuality`: `FeasibleFirst`, `FeasibleAfterImprovement`, optionally `CostGatedFeasible`; no optimality guarantee implied.
- `heuristicGuarantee`: `CertifiedRegionEuclidean` or `PerformanceNoncertified` (for specific priority calculation).
- `trajectoryActuatorFeasible`: normally **false/unverified** until an external timing/actuation validator checks steering-rate, speed, acceleration, jerk, stop/gear-change constraints.
- `validationRequested` and `validationAchieved`: separate; no flag creates evidence before checks finish.
- `mapProvenance`: source/revision/frame, resolution, occupancy/unknown policy and relevant geometry/config hashes where supported.

### 13.3 Error/status semantics

Illustrative internal status set (final ABI decision pending):

```cpp
enum class PlanStatus {
  Success, NoPath, InvalidRequest, InvalidMap, RejectedUnknownMap,
  InvalidStart, Busy, Cancelled, BudgetExceeded,
  ValidationIncomplete, StrictValidationFailed,
  MissingSteeringProvider, InternalError
};
```

`Success` means a path meeting the explicitly advertised **required validation level** was completed. `NoPath` must be distinguished from approximate search exhaustion and any separately proved static-grid disconnection reason. A search limit is not a no-path proof. A strict candidate failure cannot be success. A validated incumbent returned on a later budget/cancellation event must have a documented status/termination pair that makes its **quality incomplete** visible; no invalid raw path may be returned as executable fallback.

### 13.4 Diagnostic record

Recommended counters/metadata: generated, expanded, reopened, pruned-by-cost, pruned-by-heuristic (with proof source if permitted), heap stale-pops and peak, exact shape–cell tests, EDT fast-path acceptances/fallbacks, collision extra samples and validation failures, heuristic settlements/budget fallbacks, RS attempts/valid connections/candidate cost/improvements, strict retries, solution weighted cost, geometric length, reverse length, gear-switch count, map identity/revision and occupancy policy, check levels, total/per-stage duration, cancellation/timeout reason. Benchmark logs should additionally record commit, hardware, compiler, OMPL build availability and deterministic seed. [A §4.4; B §15, §17; C §11]

---

## 14. Ownership, concurrency, caching and portability

### 14.1 Request and evaluator lifecycle

`PlanContext` holds const map, validated config, model, attached geometry, effective occupancy/unknown policy, goals, request work limits and cancellation token. `RequestScratch` owns node pool, active label map, priority queue, statistics, lazy Dijkstra frontier and, if used, request-local RS evaluator/EDT. One active request per planner instance and fail-fast `Busy` is an acceptable baseline; independent instances may share immutable maps but must not share mutable OMPL scratch/heap/heuristic frontier without concurrency support. [B §4, §9, §14; C §4.4]

### 14.2 Cache compatibility matrix

| Cache/data | Invalidation dependencies | Initial ownership |
|---|---|---|
| Six Ackermann local primitives | `R_min`, model/control sign conventions, length, sample policy | Planner/model immutable cache |
| EDT blocked-cell-center field | map source/revision/content, dimensions/resolution/origin, known threshold, unknown policy | Request-local or immutable exact-key cache |
| Cover/mask | footprint/body offsets, model geometry, cover count/margins, map/EDT identity | Planner immutable cover; request-bound EDT |
| Reverse 2D Dijkstra | map/occupancy identity, goal tolerance seeds, connectivity/corner rules, inflation/cell rejection and edge weights, budgets/settlement semantics | Request-local mutable frontier |
| Optional permissive reachability | all Dijkstra identity plus inradius and proof-backed cell/adjacency rules | Separate request-local field |
| Optional RS lookup | radius, relative pose coordinate frame, gear/sign treatment, binning/error bound | Future, benchmark gated |
| Final path evidence | entire accepted path, pinned map/policy/geometry and exact validation version/config | Result immutable provenance |

The active mutable lazy-field heap/distances/statuses are **one coherent object**; never reuse an incompatible partially settled frontier. A changing SLAM map can make exact revision caches ineffective; quantify publication frequency, warm hit rate, retained old snapshot memory, EDT/Dijkstra build time and total planning latency before incremental repair, region-of-interest grids, tiled maps or cross-request resumption. [A §3.8; B §14; C §4.4]

### 14.3 Portability and build

- C++20 native public API; OMPL private to adapter target; ROS 2 integration optional and kept outside core.
- Validate supported Linux/compiler/libstdc++/OMPL matrices, especially atomic shared-pointer publication on older GCC 11/Ubuntu 22.04 environments.
- `HYBRID_ASTAR_ENABLE_OMPL=OFF` builds can verify non-RS core but **cannot** establish the complete RS-equipped planner.
- Use diagnostic assertions with meaningful failure context (Catch2 or equivalent); avoid bare integer smoke exit codes as test suite grows.
- Execute validation and benchmarks locally by default; this specification adds or runs no GitHub Actions workflows.

---

## 15. Proposed internal interfaces and execution flow

### 15.1 Native C++20 design sketch (illustrative, not public ABI)

```cpp
struct Pose2D { double x, y, yaw; };
enum class Gear : std::uint8_t { Start, Forward, Reverse };

struct MapSnapshot { /* immutable cells, size, resolution, SE(2) origin,
                         source+revision/frame provenance */ };
struct OccupancyPolicy { /* threshold, UnknownCellPolicy */ };
struct ValidatedConfig { /* bins, controls, costs, budgets, assurance */ };

struct MotionSegment {
    /* model-specific full start/end state and progress extent */
    /* gear/signed sections, curvature/control, evaluator, cusps */
};

struct SearchKey { /* checked x,y bins, wrapped yaw bin, incoming gear */ };
struct NodeRecord {
    /* stable ID, complete continuous state, g, hAtInsertion, key */
    /* immutable parent ID and accepted MotionSegment reference */
};

struct PlanContext {
    std::shared_ptr<const MapSnapshot> map;
    /* immutable model and collision geometry */
    OccupancyPolicy occupancy;
    ValidatedConfig config;
    /* exact goal target and tolerance region, cancellation */
};

struct RequestScratch {
    /* stable node records, generational active labels and heap */
    /* per-request heuristic frontier and optional OMPL evaluator */
    /* counters, incumbent and retry budget */
};

class ExactGridCollisionChecker {
public:
    PoseCheckStatus checkFullFootprint(const Pose2D&) const;
};
class OptionalEdtDiscPrefilter {
public:
    BroadPhaseResult classify(const Pose2D&) const;
};
class MotionValidator {
public:
    /* checks declared samples/adaptive intervals; returns achieved evidence */
};
class FinalValidator {
public:
    /* strict independent full-path model/collision pass */
};
class ReedsSheppEvaluator {
public:
    double distance(const Pose2D&, const Pose2D&) const;
    std::optional</*SignedGeometricPath*/ MotionSegment>
        shortestGeometricPath(const Pose2D&, const Pose2D&) const;
};
```

This API deliberately hides unimplemented generic model machinery; replace placeholder `MotionSegment` with model-correct `GeometricPath`/section types before stabilizing binary compatibility.

### 15.2 Integrated planning algorithm

```text
plan(request):
  validate static config, map, effective policy, model and input states
  pin immutable snapshot / geometry; reject whole-map unknown if strict Reject
  validate start full body and relevant goal constraints
  build request-local search / exact checker; optional precomputed caches
  initialize no-gear start node with finite region-aware heuristic
  incumbent = none

  while a non-stale queue item exists and outer work budgets permit:
      pop active latest-generation continuous arrival
      if continuous goal region reached:
          reconstruct accepted prefix; strict-validate complete path
          if valid: consider costed incumbent; maybe terminate (policy F)
          else: record failure (strict retry only if configured)
      if analytic scheduler fires and nominal target is suitable:
          request complete signed RS connector
          validate each model segment/cusp and full-body motion
          evaluate full connector base+optional clearance cost
          reconstruct prefix+tail and strictly validate complete candidate
          if valid: consider incumbent; maybe terminate (policy F)
          if invalid: continue ordinary search
      for primitive in stable six-action order:
          derive exact continuous successor and checked key
          apply nominal-cost lower-bound one-label pruning
          perform required full-body motion check (EDT bypass only if proved)
          evaluate true finite nonnegative edge cost
          recheck retained label and push immutable generation if improved
      if incumbent and bounded improvement consumed: return verified incumbent

  if independently validated incumbent exists:
      return incumbent with explicit stop/quality reason
  else if search exhausted: return appropriate finite-graph NoPath status
  else: return explicit timeout/cancel/resource/validation failure
```

**Separation of concerns:** the fast search validator and strict final validator may share thoroughly tested coordinate/geometry primitives, but strict final success must not depend on search's unsafe fast-path decision. The true evidence is assigned after the final validator completes. [A §4.1, §4.4; B §4, §7, §15; C §6, §3]

---

## 16. Verification, tests and benchmarks

### 16.1 Correctness acceptance matrix

| ID | Fixture / test | Required assertion | Provenance |
|---|---|---|---|
| MAP-01 | Rotated map yaw `pi/2` or 0.3, translated/negative world points | Inverse SE(2), floor indices, identical rotated-back occupancy answer | A COL-5; B §17; C §3.6 |
| MAP-02 | Map boundary, grazing edge/corner, occupancy equality, unknown -1 | Contact/out-of-bounds blocks; Occupied/Free/Reject consistent | A COL-4; B §5; C §3.6 |
| MAP-03 | Published next map revision during planning | Old request uses old pinned cells; next request uses new revision | B §14; C §8 |
| COL-01 | Blocked **single cell wholly enclosed** in body at `r=0.20,0.10,0.05,0.025` | Exact checker and strict final validator detect every overlapping checked pose | A COL-1; B §5; C §3.6 |
| COL-02 | Rectangle on blocked corner / map edge / contact tangent | Full-body exact result follows one declared numeric contact convention | B §5; C §3.2 |
| COL-03 | Random continuous subcell + yaw; EDT cover enabled | `DefinitelyClear` never contradicts independent exact oracle, under every policy/border case | A COL-2; B §12; C §3.6 |
| COL-04 | EDT stale/missing, footprint changed, map revision changed | Exact fallback or explicit setup error, **never safe bypass** | B §12; C §3.3–3.6 |
| COL-05 | Tight valid corridor smaller than outer cover | Disc overlap triggers exact fallback, **not false collision** | A §4.1; C §3.3 |
| MOT-01 | Thin 1-cell obstacle between nominal poses on maximum-curvature arc | Adaptive checking or proved swept-envelope catches it; if not certified evidence remains sampled | A COL-3; B §6; C §3.4 |
| MOT-02 | Forward/reverse arcs, cusps, short/zero signed sections | Segment endpoints/cusps always covered; no chord interpolation | B §6, §9; C §3.4 |
| MOT-03 | Later stationary differential yaw / trailer joint motion | Body bound detects motion with nearly zero reference translation | B §6; C §8 |
| MOT-04 | Subdivision/strict budget exhausted | `ValidationIncomplete`, never silent validation downgrade | B §6; C §3.5 |
| VAL-01 | Inject deliberate search prefilter false-safe behavior | Independent strict oracle rejects candidate; planner never returns invalid Success | A COL-7; B §5; C §3.5 |
| VAL-02 | Strict failure after complete path reconstruction | Failure location/segment/revision recorded, optional single retry bounded | C §3.5 |
| ACK-01 | Six exact arcs, near-zero `kappa`, yaw wrap, reverse signs | Correct analytic endpoints, SE(2) equivariance and symmetric identities | A §4.3; B §11; C §5 |
| KEY-01 | Two states quantize to same key; better arrival after expansion | Immutability of old ancestry, generation staleness, correct reopen if enabled | B §11.5; C §6 |
| COST-01 | RS with incoming switch, reverse sections and 2+ cusps | Hand-computed equivalent-meter cost matches implementation | B §7, §17; C §6.4 |
| GOAL-01 | Nominal goal center blocked; tolerance zone has free pose | Ordinary search may accept valid tolerance endpoint; never fake exact nominal pose | A §4.2; B §7; C §4 |
| HEU-01 | A state already inside goal tolerance | Certified region Euclidean is zero; exact-nominal-goal RS may be positive/noncertified | B §10; C §4 |
| HEU-02 | Relaxed graph disconnected vs settlement budget-limited | Only independently proved superset graph after **full exhaustion** may early `NoPath` | C §4.3–4.5 |
| HEU-03 | Same map/search geometry, changed occupancy resolution | Analytic scheduler uses search units; equal metric inputs give same attempts | A §4.5; B §7; C §6 |
| PATH-01 | Multi-gear maneuver reconstruction | Exact full states, signed sections, cusp boundaries, reached endpoint retained | B §15; C §6 |
| PATH-02 | Failed optional refiner | Verified raw geometric path preserved without raising its assurance | B §15; C §7 |
| API-01 | Actuator-invalid fixed-curvature joins | No `TrajectoryActuatorFeasible` claim from geometric planner alone | B §8 |
| API-02 | Cancel / work limit / missing OMPL / invalid map | Discrete non-success statuses; no misleading `NoPath`/guarantee | B §9, §15; C §12 |

### 16.2 Performance and quality scenarios

Run empty field, straight free RS case, wall with distant doorway, U-shaped dead end, reference point through sub-vehicle-width gap, tight corridor, map boundary overhang, impossible pocket, parking/multiple reverse cusps, long reverse-heavy early RS candidate, thin between-sample obstacle, two homotopy alternatives, changed map revisions. In a subsequent model milestone add differential in-place turn, mecanum lateral motion and articulated compound geometry.

Compare controlled variants:

- Exact checker versus old all-heading mask versus verified 4/5-disc prefilter, reporting build cost and fraction of exact fallbacks.
- Nominal 0.10 m vs adaptive body-point criterion vs optional *proved* inflated sweep method; track validation completeness/cost separately from speed.
- `h=0`, region Euclidean, RS, raw Dijkstra, footprint-aware Dijkstra, combined; eager vs lazy and cold vs warm.
- Reopening vs no reopening; packed vs struct key only if injected bit range has been verified.
- Fixed `L=0.50` vs auto-from-search-bins length; subbin duplicate successor statistics and route success.
- `FirstStrictlyValidated` vs `ImproveIncumbent` vs weighted-cost-gated analytic acceptance.
- Clearance weight disabled vs sample-boundary vs cover-proxy vs exact-oriented clearance, all evaluated on **strictly validated final paths**.
- Optional refinement disabled vs simple obstacle-aware shortcut/smoother vs heavier optimization when available.

### 16.3 Performance measurement protocol

Controlled Release builds (and sanitizer builds separately), pinned scenario/seed/map/geometry, identical relevant configuration, specified CPU/compiler/OMPL versions. Run sufficient repetitions for p50/p95/p99 latency and peak memory; cold/warm distinct. Split total time into snapshot handling, EDT construction, Dijkstra settlement, search/queue, collision exact/broad-phase, OMPL, reconstruction, strict validation and refinement. Collect success/failure rate, node counts, retry counts, verified minimum clearance, raw/final length and weighted cost, reverse length, gear-switch count and chosen homotopy.

No single latency or improvement factor is asserted in any input as verified for this project's completed planner. Release thresholds for p95/p99, valid-path rate and memory must be chosen on reference hardware with real application scenarios; do not infer them from Junior's published counts or Nav2's different costmap assumptions.

### 16.4 Suggested local verification commands

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DHYBRID_ASTAR_ENABLE_OMPL=OFF -DHYBRID_ASTAR_BUILD_TESTS=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure

# Separately, with supported OMPL installed:
cmake -S . -B build-ompl -DCMAKE_BUILD_TYPE=Release \
  -DHYBRID_ASTAR_ENABLE_OMPL=ON -DHYBRID_ASTAR_BUILD_TESTS=ON
cmake --build build-ompl --parallel
ctest --test-dir build-ompl --output-on-failure
```

These are **future implementation commands** taken from B's guidance, not commands run to create this synthesis. Use named tests, property fixtures and sanitizers; no automatic CI trigger or GitHub Actions dispatch is implied. [B §17.4]

---

## 17. Work packages, acceptance gates and dependencies

The ordering below merges A's M1–M7, B's Phases A–E and C's WP-0…WP-9. It follows the **C/B correctness-first** path where it conflicts with A's preference to introduce EDT/clearance early. Each gate describes implemented behavior to verify in a future repository task; none has been executed as part of this document.

| WP | Dependencies | Deliverable | Exit gate | Position / conflicts |
|---|---|---|---|---|
| **W0 Baseline and tests** | Reviewed config/docs | One normative current design, provisional config log, named assertion harness, map/geometry fixtures | No contradictory default collision/heuristic/RS scheduling contract | All agree; A suggests more aggressive ADR consolidation |
| **W1 Map and exact geometry (P0)** | W0 | Immutable snapshot, occupancy policy, inverse SE(2), exact whole-footprint square checker | COL-01, boundary, unknown, rotated-map pass; no boundary-only success | Consensus |
| **W2 Exact Ackermann motions (P1)** | W0–W1 | Six closed-form arcs, exact evaluator, segment/cusp representation | Analytic, reverse, yaw and SE(2) tests pass | Consensus; fixed vs auto length still benchmarkable |
| **W3 Minimal search (P1)** | W1–W2 | Gear-aware one-label graph, immutable parent pool, versioned heap/reopen, base costs, finite region Euclidean | Open field, wall detour, parking, unreachable/limit tests; exact reconstruction | A alternative: no reopening / packed key |
| **W4 Strict final gate (P0)** | W1–W3 | Independent no-EDT full-body path validator, body-aware sampling, evidence, optional single retry | Deliberately weakened fast checker never yields invalid successful output | Consensus on gate; sweep-certification implementation undecided |
| **W5 RS connector/termination (P1)** | W2–W4 | Reusable OMPL evaluator, complete signed sections and cost, search-relative scheduler | Correct cusps/costs, first attempt, valid fallback and blocked-goal tolerance tests | Quality policy default disputed |
| **W6 Obstacle grid heuristics (P2)** | W3 and RS for combined | Deterministic request-local lazy reverse field, goal-region seed, finite fallback, performance metadata | No false admissibility; no unproven early `NoPath` | A wants aggressive inflation/pruning early |
| **W7 Optional safe-bypass (P2)** | W1, benchmarks | EDT plus verified covers, exact fallback, instrumentation | No observed false-safe, conservative proof, useful total planner benefit | A would build with W1; B/C prefer gated |
| **W8 Quality and refinement (P2/P3)** | W4–W7 as needed | Optional clearance, incumbent improvement, obstacle-aware refiner | Measurable improvement in validated final path without success/latency regression | A wants earlier clearance; B wants proposed incumbent-default |
| **W9 Later systems** | Stable W1–W8 interfaces | Differential drive, mecanum, articulation, advanced swept certification, cache/ROS adapters | Full-state/body/evidence contracts hold for each actual model | Defer speculative generality |

### 17.1 Definition of done — first validated Ackermann geometric path

- [ ] Full-footprint exact collision checking at all evaluated poses and no perimeter-only `Success`.
- [ ] Pinned immutable map, correct origin transform, boundaries, thresholds and unknown policy.
- [ ] Validated start state and model; six deterministic analytic Ackermann motions with complete evaluable segments.
- [ ] Continuous-state gear-aware one-label search with stable ancestry, explicit queuing/reopening policy and deterministic limits.
- [ ] Continuous XY/yaw tolerance goal predicate, true reached endpoint and baseline equivalent-meter cost.
- [ ] Independent stricter final full-body path validation; incomplete/failing required checks cannot succeed.
- [ ] Search vs final evidence, finite fallback heuristic and no unearned optimality/swept/actuator promises.
- [ ] Local end-to-end map/parking/reverse/collision regression tests pass, with a recorded compiler/configuration matrix.

### 17.2 Definition of done — first RS-equipped complete planner

- [ ] OMPL-enabled build and tests pass locally on supported toolchain, with evaluator resource reuse.
- [ ] Exact signed RS section reconstruction and endpoint/cusp/gear cost integrity.
- [ ] First eligible search node attempts analytic connector; cadence independent of occupancy resolution under equal metric inputs.
- [ ] Failed connector attempts do not abort ordinary search; valid tolerant goal endpoints remain reachable.
- [ ] Strict full-path validation governs all successful analytic candidates; quality policy and termination reason are visible.
- [ ] Combined performance heuristic explicitly noncertified; any graph-based early `NoPath` disabled until separate proof.

---

## 18. Decision register and remaining proof obligations

### 18.1 Decisions still requiring an owner or benchmark

| ID | Required decision or experiment | Why it remains open | Safe default for initial implementation |
|---|---|---|---|
| O01 | Required output assurance: sampled/adaptive or certified sweep? | Application speed, geometry/map uncertainty and external monitoring differ | Full footprint + strict adaptive sampled; report level |
| O02 | Is A's half-step inflated footprint envelope proven and useful? | Need actual shape inflation, numeric/border and section proofs; B/C avoid premature certification | Do not claim continuous swept guarantee |
| O03 | How much body-point motion triggers subdivision? | No universal safe threshold from map resolution alone | Configurable, bounded, fail incomplete |
| O04 | Can 4/5-disc EDT beat exact checker end-to-end? | Build/fallback/cost depends on resolution and clutter | Exact fallback always active; EDT disabled until measured |
| O05 | Should the 2D performance grid be footprint-aware? | Restrictive graph improves guidance but can distort connectivity | Experiment separately; no graph-pruning guarantee |
| O06 | Can a permissive inscribed-disc graph certify `NoPath`? | Cell rejection, cell trace adjacency, goal seeding and full exhaustion require proof | No early `NoPath` from 2D field |
| O07 | Can obstacle-grid numeric distances ever be admissible? | Octile 1.082 factor alone is insufficient for blocked/corner/subcell/tolerance semantics | Region Euclidean certified only |
| O08 | Keep two heuristic modes in public API? | A simplifies; B/C preserve guarantees | At least explicit guarantee metadata and selectable baseline |
| O09 | Reopen or no-reopen? | Inconsistent/noncertified priorities and one-label policy affect behavior | Versioned heap with reopening; benchmark alternative |
| O10 | Fixed 0.50 m or auto-derived primitive length? | Subbin aliasing and route feasibility are workload-dependent | 0.50 m fixture baseline; tune |
| O11 | Pack key into 64 bits? | Map-relative bit widths/injectivity/performance not established | Readable key, checked indexing |
| O12 | First valid or bounded incumbent improvement default? | Latency vs weighted route quality varies | First strictly valid for minimal MVP; both tested |
| O13 | Hard RS connector length/cost guard? | Long feasible connectors may be inefficient but caps exclude feasible paths | No mandatory length cap; optional cost gate |
| O14 | Clearance proxy/weight and deployment phase? | Covering discs overhang actual vehicle and may bias route | Disabled, benchmark actual-oriented clearance |
| O15 | Strict validator retry policy? | Recovery effort/latency budget product-specific | Fail explicitly; optional one bounded exact retry |
| O16 | Keep `Unknown::Reject`? | Useful strict mode, impractical for incomplete SLAM | Optional explicit full-grid strict validation |
| O17 | Cross-request partial-frontier/cache reuse? | Frequent revisions may wipe out reuse, affect determinism | Request-local mutable field |
| O18 | Publish snapshots via atomic shared ptr or mutex? | C++20/library/compiler support varies | Toolchain-compatible publication policy |
| O19 | Minimum actuator requirements of downstream consumer? | Geometric curvature jumps/cusps not timed executable commands | Expose continuity/events; external trajectory layer |
| O20 | Supported ROS/OMPL/compiler platform matrix? | OMPL ON and older libstdc++ were not validated in the reviews | Explicitly qualify supported builds |
| O21 | Governance of superseding ADRs/tuning defaults? | A favors collapsing; B/C favor provenance and current normative doc | Single current contract + supersession lineage |

### 18.2 Mathematical and engineering proof obligations

1. **Exact full-shape checker:** robust SAT/contact and map-boundary rules, including transformed rotated grids, no false-free interior misses.
2. **Disc cover:** each transformed cover geometrically contains every covered body point, at every supported continuous yaw/state.
3. **EDT margin:** both query-subcell and blocked-square half-diagonal errors, plus numeric margin and boundary containment, correctly handled.
4. **Continuous motion certification (only if attempted):** entire swept collision geometry inside tested conservative envelopes, including all sections/cusps, joint motion, and numeric/interval termination.
5. **Heuristic lower bound:** all compared costs in same units and goal acceptance **region**; exact-goal RS and grid cost cannot be called admissible without separate proofs.
6. **Reachability:** no genuinely feasible reference trajectory is removed by forbidden-cell rule or adjacency, and `NoPath` only after fully exhausted permitted goal-connected component.
7. **Key packing:** injective over a checked, supported finite map-relative range; no silent signed-coordinate collisions.
8. **Dominance:** `g + nominalBase` is a lower bound on final nonnegative cost; do not claim full continuous-state dominance from one-label keys.
9. **Final-result evidence:** claimed validation and path cost derive from performed checks, complete accepted sections and pinned policy; no flag-only success.

---

## 19. Documentation reconciliation and traceability

### 19.1 Known baseline documentation inconsistencies noted by the reviews

- **ADR-0025 vs ADR-0027:** older table calls Euclidean the default while newer combined `max(RS,grid)` is the performance setting. Distinguish certified Euclidean fallback from noncertified combined ordering.
- **ADR-0015 / OQ-004:** admissibility wording must be goal-region-specific; retaining one-label search does not imply optimality.
- **ADR-0022 / ADR-0029:** historical SAT/adaptive wording and boundary-only default must not both read as current normative requirements. Explicitly change the default collision assurance and document supersession.
- **ADR-0023:** for the initial constant-curvature Ackermann, the closed-form evaluator supersedes runtime RK4/root-finding baseline; generic numerical techniques remain available for later models.
- **ADR-0026:** scheduler units must reference search bin/primitive length rather than occupancy resolution; record first-node/long-connector and quality policy.
- **ADR-0027:** distinguish useful non-certified obstacle heuristic, candidate footprint-aware relaxation, and any independently proved connectivity `NoPath` oracle.
- **ADR-0013:** preserve/measure reopening rather than silently removing it when the performance priority may be inconsistent.
- **`quality.md`:** repair the incomplete NFR-005 Markdown table row noted by C.
- **Smoke/tests and README:** express scaffold limitations, not implemented planner/collision/OMPL-enabled success; replace bare integer assertion returns as tests grow.

### 19.2 Source-of-truth policy

Keep a single concise **current architecture algorithm/contract** with owned invariants and assurance semantics, plus a configuration-baseline/tuning log and a verification matrix. Treat ADRs as historical decisions with explicit superseded-by links, not parallel conflicting specifications. B proposes new ADRs for full-body default, motion validation, incumbent termination, execution validity, OMPL lifetime and achieved evidence; C proposes narrower targeted supersession; A proposes a more aggressive collapse of accepted structural ADRs. The **governance method is open**, but the end state must prevent old docs from silently overriding current requirements.

### 19.3 Trace map from consolidated sections to the inputs

| Consolidated section | A source | B source | C source |
|---|---|---|---|
| §1–2 Comparison/evidence | §§1–3, 6 | §§1–3, 20 | §§1–2, 12 |
| §3 Architecture | §§2, 3.8 | §4, IMP-07 | §§1, 8–9 |
| §4–5 Parameters/maps | §§3.8–3.11, 4.7 | §§14, 16 | §§2, 8 |
| §6 Ackermann/geometry | §§4.3, 4.4 | §§8, 11, 15 | §§5–6 |
| §7 Collision | §§3.1–3.2, 4.1 | §§5, 12 | §3 |
| §8 Motion/final | §§3.3–3.4, 4.1 | §§6, 15 | §3.4–3.5 |
| §9 Search | §§4.1.5, 4.4 | §§4, 11 | §6 |
| §10 Heuristics | §§3.5, 4.2 | §10 | §4 |
| §11 RS/quality | §§3.10, 4.5 | §§7, 9 | §§6.4, 8.3 |
| §12 Clearance/refiner | §§3.7, 4.6, 5 | §§13, 15 | §7 |
| §13 Results | §§4.4–4.7 | §§8, 15 | §§1, 3, 6, 12 |
| §14 Ownership | §§3.8, 4.7 | §§9, 14 | §§4, 8 |
| §15 Code design | §§4–5 | §§4–16 | §§3–10 |
| §16 Tests | §§4.1.8, 4.2.4, 4.9 | §17 | §§3.6, 4.5, 11 |
| §17 Work packages | §5 | §18 | §10 |
| §18 Open decisions | §6 | §20 | §12 |
| §19 Docs | §§3.9, 3.12, 4.8 | §19 | §2.2 |

---

## 20. References and source boundaries

### 20.1 Primary inputs (all three reviewed in full for this synthesis)

- **A:** `hybrid-astar-architecture-review.md` (2026-10-26), architecture review with source-specific collision/heuristic redesign, code notes, reproduction-script appendix and milestones. Relevant topics cited using its section numbers above.
- **B:** `hybrid_astar_architecture_review_and_implementation_spec(1).md` (2026-10-26), eleven IMP items, detailed API/verification/phase proposals, architectural approval gates and external references.
- **C:** `hybrid_astar_review_verdict_and_implementation_spec(1).md` (checked 2026-10-26), technical verdict, proof-conservative collision/heuristic analysis, illustrative file layout, work packages and unresolved decision list.

### 20.2 Reviewed project baseline and external precedents

- [Commit-pinned `civcode/hybrid-astar-cpp`](https://github.com/civcode/hybrid-astar-cpp/tree/dff94bcbcd772aaf4758067794d497bc28c3475b) — **reported** baseline, not freshly verified by this synthesis.
- [Architecture algorithms](https://github.com/civcode/hybrid-astar-cpp/blob/dff94bcbcd772aaf4758067794d497bc28c3475b/docs/architecture/algorithms.md) and the ADRs 0013–0029 cited by the inputs.
- [Stanford Junior paper](https://robots.stanford.edu/papers/junior08.pdf), particularly Hybrid A* §6.3: continuous arrivals in discretized search, nonholonomic+holonomic heuristics, reverse/switch and obstacle costs and smoothing; does **not** directly prescribe all later RS analytic-expansion details.
- [Nav2 Smac Hybrid planner configuration](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/smac_hybrid/configuring_smac_hybrid/) and [Smac source](https://github.com/ros-navigation/navigation2/tree/main/nav2_smac_planner) — comparative precedents, not a proof of correctness or a transferable costmap configuration for this raw occupancy grid.
- [OMPL ReedsSheppStateSpace](https://ompl.kavrakilab.org/classompl_1_1base_1_1ReedsSheppStateSpace.html) — geometric RS distance and path machinery, not reverse/gear-switch-weighted optimality.

**Provenance note:** The quantitative baseline values, reported local smoke test, proposed algorithms and engineering alternatives above are taken from the **three provided reviews** and explicitly marked when they disagree. This consolidation does **not** assert that additional public sources or live repository state were newly inspected, that proposed C++ classes exist, that source A's simulations were rerun, or that any proof/benchmark gate has already been passed.