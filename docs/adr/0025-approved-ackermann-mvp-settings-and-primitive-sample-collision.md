# ADR-0025: Approved Ackermann MVP settings and collision checks at primitive samples

**Status:** Accepted (first implementation baseline; tune after integrated tests)  
**Date:** 2026-10-25  
**Related:** [ADR-0013](0013-single-label-search-baseline.md), [ADR-0015](0015-admissible-default-heuristics.md), [ADR-0020](0020-evidence-driven-mvp-simplicity.md), [ADR-0022](0022-opt-in-adaptive-collision-subdivision.md), [ADR-0023](0023-rk4-arc-length-primitives-and-samples.md)

**Updates:** [ADR-0027](0027-combined-hybrid-a-star-heuristic-and-guarded-analytic-expansion.md) replaces the initial Euclidean-only performance heuristic with combined Reeds–Shepp / obstacle-field estimates (strict certified fallback remains); [ADR-0029](0029-conservative-orientation-hull-mask-and-raster-footprint-collision.md) replaces the per-pose polygon–cell intersection algorithm with conservative hull-mask + raster boundary **and interior guard**. The approved numeric primitive, discretization and search cost settings and primitive sampling interval remain unchanged.

## Context

The user reviewed and accepted twelve concrete primitive/search/cost settings and subsequently approved a simpler collision schedule: **the initial Hybrid A* collision validator checks the complete robot footprint at the already stored RK4 primitive points**. Do not introduce a separate default fixed-sampling scheduler or its own translation/rotation thresholds. These are starting defaults, not claims of universal optimality, safety or final tuning.

This ADR specifies the first **three-state Ackermann** end-to-end slice. It does not freeze a general kinematic-model API or impose Ackermann-specific concepts on other models.

## Accepted primitive generation defaults

| Setting | Baseline |
| --- | --- |
| Translational extent per primitive | **0.50 m** actual traveled arc length |
| Reference generation speed for RK4 | **1.0 m/s** (nominal generation parameter, not commanded execution speed) |
| Nominal RK4 step at reference speed | **0.05 s** (corresponds to ~0.05 m traveled per step at constant 1 m/s) |
| Maximum accepted arc-length endpoint error | **0.0001 m** |
| Stored full-state sample spacing | **0.10 m** traveled arc length, including both endpoints |
| Actions | **Forward/reverse × left/straight/right** (six candidates; `kappa = -1/R_min, 0, +1/R_min` with a consistent sign convention) |

Use RK4 to generate the complete trajectory according to ADR-0023. The generator must include exact *designated sample locations* and the numerically computed endpoint, refining the terminal integration interval if necessary. For a 0.50 m primitive with 0.10 m sample spacing this gives **six stored states**, at `s={0,0.1,0.2,0.3,0.4,0.5}` m. Internally there may be **more RK4 integration steps than stored debug/collision sample states**; they need not coincide. Reverse is an explicitly identified direction, with nonnegative traveled length. A constant-speed action can determine its nominal integration duration directly without an unnecessary iterative length root search; keep the traveled-length tolerance.

The reference fixture discussed before review—wheelbase 2.7 m, minimum turning radius 4.5 m, 4.5 × 1.9 m rectangular body attached relative to the rear axle and 0.20 m map resolution—is a **proposed test-fixture profile**, not part of the twelve individually approved parameters. Keep vehicle geometry, wheelbase and map resolution configurable and verify the chosen fixture when writing tests. The maximum length-solver iterations, stopping failure status and analytic-check tolerances are also implementation details, not silently adopted as approved numeric defaults.

## Accepted Hybrid A* search discretization

| Setting | Baseline |
| --- | --- |
| XY discretization | **0.25 m** in each axis |
| Yaw discretization | **10 degrees** (36 wrapped bins) |
| Driving mode in search key | **Include incoming forward/reverse mode**, alongside quantized `(x,y,yaw)` |
| Retained arrivals | **One continuous state per complete key** (ADR-0013) |
| Default heuristic | **Euclidean distance to the accepted goal XY region**, i.e. `max(0, hypot(dx,dy)-xyTolerance)`, measured in meters |

The physical Ackermann model stays `State=(x,y,yaw)`; travel direction is a **search-label mode**, not an added continuous ODE coordinate. A start node has no prior gear; the first direction chosen does not pay a switch penalty. Retain stable parent/action records for reconstruction. The heuristic only claims an admissible translational-distance lower bound when all enabled edge penalties are nonnegative and the translation cost is at least traveled length. Extra penalties are *not* included in this heuristic; our single-label quantization still precludes global optimality claims.

## Accepted initial edge-cost settings

Use **equivalent meters** (not measured execution time) and evaluate along the actual primitive samples:

```text
cost = L * (1 + 0.10 * isReverse)
     + 0.50 m * isDirectionSwitch
     + 0.15 * integral_primitive phi(clearance(q(s))) ds

phi(d) = max(0, 1 - d / 0.40 m)^2
```

| Setting | Baseline |
| --- | --- |
| Forward distance multiplier | **1.00** |
| Reverse distance multiplier | **1.10** |
| Forward/reverse transition cost | **0.50 m**, only between actual successive directions |
| Whole-footprint clearance weight | **0.15** (dimensionless) |
| Clearance influence radius | **0.40 m** |
| Steering-change and curvature/smoothness penalties in search | **0**; addressed by downstream obstacle-aware refinement (ADR-0024) |

The entire transformed robot footprint, not only the rear-axle point or shape vertices, determines geometric obstacle clearance. The occupancy threshold and unknown/out-of-bounds policies used for clearance must agree with hard collision testing. `phi` is a nonnegative preference for extra clearance **within** the 0.40 m influence band; it cannot override collision rejection. Integrate/evaluate this soft cost using the same stored primitive samples in the first implementation (e.g. deterministic trapezoidal quadrature over arc length); this is an approximation, and higher-fidelity clearance queries are deferred unless tests justify them. Reuse map data if practical but **do not require a sophisticated cached distance field solely to implement the initial preference**.

These cost weights are approved starting defaults, subject to measurement and tuning, not values claimed to originate from Stanford Junior's paper. **ADR-0026 requires a native near-goal Reeds–Shepp connector in the first Ackermann implementation.** Apply the *same* costs to each straight/arc connector section, including incoming-gear and internal cusp switches, and full-body clearance sampled along the connector.

## Revised collision checking — coupled to primitive expansion

1. **Default:** at each Hybrid A* expansion, instantiate the generated RK4 primitive from the current *continuous* search state. Transform its **stored complete-state samples** into the map frame. At each stored state—including start, intermediates and end—test the **entire model-transformed collision shape(s)** for intersection with map cells treated as blocked. Reject the candidate if any sample collides. **Only the endpoint becomes a successor search node**; samples are not additional search nodes.
2. **No independent default collision sampling schedule.** Do not introduce a separate translation step, angular step, resolution parameter or required interpolation between planner search nodes. The **0.10 m primitive sample spacing is the first Ackermann collision-check spacing**. At 0.50 m primitive length it gives six checks, prior to harmless duplicate-state elimination. Do **not** replace curved RK4 trajectories with straight-line interpolation between the two search nodes.
3. **Optional adaptive subdivision remains request-level, default `false` (ADR-0022).** If requested and implemented, the checker may evaluate additional complete intermediate states *between stored primitive samples* using the retained numerical trajectory/control-law representation, applying model-aware body motion. It must not insert extra Hybrid A* nodes or silently claim continuous safety; exact adaptive threshold/depth/budget defaults are **not adopted** from the earlier unapproved proposal.
4. **Future stationary maneuvers** (in-place yaw/articulation) have zero translational arc length, so their generator must store intermediate full states at an appropriate **angular/configuration extent resolution** before using these samples for collision checking (ADR-0012). The initial translating Ackermann slice does not force a universal 2-degree rotation interval.
5. **Final path validation** may reuse the same model/map collision service and accepted sample semantics; obstacle-aware path refinement (when implemented) must validate its altered trajectory using its own generated/evaluable states. Preserve and report the actual `sampled` assurance, sample spacing, occupancy policy and exact pinned map revision. Checking finite samples is **not** continuous swept-collision certification.
6. Geometry overlap at any state is **shape-versus-cell**, with circles, rectangles, convex polygons or compound multi-frame collections (ADR-0021). It is not a vertex-only, perimeter-only or interior-point-sampling test. Out-of-bounds, unknown and occupancy thresholds retain the existing map contracts.

**Interaction with ADR-0022:** this ADR **refines/supersedes only ADR-0022's separate configurable deterministic fixed-sample schedule for the first Ackermann implementation**. ADR-0022's complete-shape intersection, adaptive option default `false`, sampled assurance and budget failure semantics remain in force. **Interaction with ADR-0023:** stored RK4 primitive points become the *default* collision schedule (not merely visualization suggestions); ADR-0023's ability to evaluate between points is still needed for optional adaptive checking, path evaluation or future refiners.

## Validation before further tuning

- Verify a 0.50 m forward/reverse straight or curved primitive returns six ordered samples at 0.10 m increments, RK4 convergence and traveled-length error no more than 0.0001 m.
- Verify six controls at every valid ordinary Ackermann state, their transformed ends and stored samples; test 0.25 m XY and 10-degree wrapped yaw bins, distinct forward/reverse labels with one retained arrival *per complete key*, and no charge for selecting the first gear.
- On a simple map, check candidate rejection when any stored shape overlaps an occupied cell and when a cell lies **inside** a large footprint without crossing its edges; test rotated grid frame, unknown/free/reject/out-of-bounds policy.
- Demonstrate that no separate collision-only fixed sampling is generated by default; additional states are queried only when `adaptiveSubdivision=true`. Build at least one between-samples collision example showing the default's explicitly approximate nature.
- Check all initial cost terms against a hand-computed toy primitive, including clearance integral units, forward vs reverse and gear switches; verify Euclidean goal-region lower bound is no larger than such nonnegative edge-cost paths.
- Test start-validity, goal tolerances, accepted parent-edge reconstruction, input validation and infeasible/unreachable maps; log sampled-only assurance rather than certified collision freedom.

## Outside approved numeric baseline

No optimizer method, timed execution profile, articulated-mode sample resolution, RK4 root-solver iteration limit, numeric distance-query tolerance, adaptive-refinement depth/threshold, concurrent cache policy or performance guarantee is fixed here. **Native near-goal Reeds–Shepp is now required (ADR-0026)**; its proposed numeric trigger radius and performance settings remain for implementation review.
