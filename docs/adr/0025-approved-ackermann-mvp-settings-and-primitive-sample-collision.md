# ADR-0025: Approved Ackermann MVP settings and collision checks at primitive samples

**Status:** Accepted (first implementation baseline; tune after integrated tests)  
**Date:** 2026-10-25  
**Related:** [ADR-0013](0013-single-label-search-baseline.md), [ADR-0015](0015-admissible-default-heuristics.md), [ADR-0020](0020-evidence-driven-mvp-simplicity.md), [ADR-0022](0022-opt-in-adaptive-collision-subdivision.md), [ADR-0023](0023-rk4-arc-length-primitives-and-samples.md)

**Updates:** [ADR-0027](0027-combined-hybrid-a-star-heuristic-and-guarded-analytic-expansion.md) replaces the initial Euclidean-only performance heuristic with combined Reeds–Shepp / obstacle-field estimates (strict certified fallback remains); [ADR-0029](0029-conservative-orientation-hull-mask-and-raster-footprint-collision.md) replaces the per-pose polygon–cell intersection algorithm with conservative hull-mask + **boundary-only raster by default with optional interior coverage**; default assurance is intentionally weaker for enclosed obstacles. The approved numeric primitive, discretization and **distance/reverse/switch** search cost settings and primitive sampling interval remain unchanged. **MVP change:** the previously accepted 0.15/0.40 m clearance term is **deferred to a low-priority optional extension** (see below); it is **not** evaluated in the first implementation.

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

**Approved compact passenger-car test fixture** ([details and manufacturer-source references](../architecture/passenger-car-test-fixture.md)): 4.0 m minimum **rear-axle** turning radius, 2.7 m wheelbase, a 4.6 × 1.8 m rectangular body spanning local x ∈ [-1.0,+3.6] m and y ∈ [-0.9,+0.9] m, and sample map resolution 0.20 m. This is an illustrative fixture, **not** a library default. A manufacturer turning-circle measurement is not the same as a rear-axle centerline turning radius. At 0.50 m primitive length, the fixture's max-turn Δyaw is 0.125 rad ≈ 7.16°. The maximum length-solver iterations, stopping failure status and analytic-check tolerances are also implementation details, not silently adopted as approved numeric defaults.

## Accepted Hybrid A* search discretization

**2026-10-25 revision:** the first Ackermann search uses **5° heading bins (72 bins)** to reduce aliasing between 0.50 m straight/turn primitives. This changes **only the search-key quantization**, not the independently approved **10° heading tolerance at the goal** (ADR-0016). Test turning/straight successor uniqueness across several subcell initial poses; 5° does not mathematically guarantee every maneuver creates a distinct key.

| Setting | Baseline |
| --- | --- |
| XY discretization | **0.25 m** in each axis |
| Yaw discretization | **5 degrees** (**72** wrapped bins); revised from the prior 10-degree/36-bin search-key default |
| Driving mode in search key | **Include incoming forward/reverse mode**, alongside quantized `(x,y,yaw)` |
| Retained arrivals | **One continuous state per complete key** (ADR-0013) |
| Default heuristic | **Euclidean distance to the accepted goal XY region**, i.e. `max(0, hypot(dx,dy)-xyTolerance)`, measured in meters |

The physical Ackermann model stays `State=(x,y,yaw)`; travel direction is a **search-label mode**, not an added continuous ODE coordinate. A start node has no prior gear; the first direction chosen does not pay a switch penalty. Retain stable parent/action records for reconstruction. The heuristic only claims an admissible translational-distance lower bound when all enabled edge penalties are nonnegative and the translation cost is at least traveled length. Extra penalties are *not* included in this heuristic; our single-label quantization still precludes global optimality claims.

## Accepted initial edge-cost settings

Use **equivalent meters** (not measured execution time) and evaluate along the actual primitive samples:

```text
MVP cost = L * (1 + 0.10 * isReverse)
         + 0.50 m * isDirectionSwitch

# LOW-PRIORITY OPTIONAL EXTENSION; disabled/not implemented in MVP:
# cost += 0.15 * integral_primitive phi(clearance(q(s))) ds
# phi(d) = max(0, 1 - d / 0.40 m)^2
```

| Setting | Baseline |
| --- | --- |
| Forward distance multiplier | **1.00** |
| Reverse distance multiplier | **1.10** |
| Forward/reverse transition cost | **0.50 m**, only between actual successive directions |
| Whole-footprint clearance weight | **0 in MVP (disabled)**; optional future weight **0.15** (dimensionless) |
| Clearance influence radius | **Not used in MVP edge costs**; optional future **0.40 m** |
| Steering-change and curvature/smoothness penalties in search | **0**; addressed by downstream obstacle-aware refinement (ADR-0024) |

**MVP scope:** compute **no clearance integral**, no per-pose clearance distances and no clearance-influence mask. Edge-cost evaluation needs only already-known primitive length, travel direction and incoming gear. This does **not** remove the nearest-obstacle EDT used to build the collision-candidate mask (ADR-0029); it removes only **clearance scoring** from the first implementation.

**Low-priority future option:** the preserved coefficient **0.15** and influence radius **0.40 m** can be used for a footprint-based soft penalty. If enabled later, evaluate clearance at the **stored complete-state samples of each already collision-accepted primitive**, then integrate over its actual traveled arc length (e.g., trapezoidal quadrature) **before relaxing the successor with `tentativeG = current.g + edgeCost`**. Read the nearest-obstacle EDT at cells along the *actual transformed robot footprint boundary*, not just at the axle cell; use a conservative grid-cell correction and compatible map policy. Its value is a soft routing preference, never proof of collision freedom. See ADR-0029 for the deferred algorithm.

These cost weights are approved starting defaults, subject to measurement and tuning, not values claimed to originate from Stanford Junior's paper. **ADRs 0026–0028 require an OMPL-backed near-goal Reeds–Shepp connector in the first Ackermann implementation.** Apply the *same active MVP costs* to each straight/arc connector section, including incoming-gear and internal cusp switches. The optional clearance integral is **not** evaluated for analytic connectors until the low-priority clearance-cost feature is enabled.

## Revised collision checking — coupled to primitive expansion

1. **Default:** at each Hybrid A* expansion, instantiate the generated RK4 primitive from the current *continuous* search state. Transform its **stored complete-state samples** into the map frame. At each stored state—including start, intermediates and end—test the **entire model-transformed collision shape(s)** for intersection with map cells treated as blocked. Reject the candidate if any sample collides. **Only the endpoint becomes a successor search node**; samples are not additional search nodes.
2. **No independent default collision sampling schedule.** Do not introduce a separate translation step, angular step, resolution parameter or required interpolation between planner search nodes. The **0.10 m primitive sample spacing is the first Ackermann collision-check spacing**. At 0.50 m primitive length it gives six checks, prior to harmless duplicate-state elimination. Do **not** replace curved RK4 trajectories with straight-line interpolation between the two search nodes.
3. **Optional adaptive subdivision remains request-level, default `false` (ADR-0022).** If requested and implemented, the checker may evaluate additional complete intermediate states *between stored primitive samples* using the retained numerical trajectory/control-law representation, applying model-aware body motion. It must not insert extra Hybrid A* nodes or silently claim continuous safety; exact adaptive threshold/depth/budget defaults are **not adopted** from the earlier unapproved proposal.
4. **Future stationary maneuvers** (in-place yaw/articulation) have zero translational arc length, so their generator must store intermediate full states at an appropriate **angular/configuration extent resolution** before using these samples for collision checking (ADR-0012). The initial translating Ackermann slice does not force a universal 2-degree rotation interval.
5. **Final path validation** may reuse the same model/map collision service and accepted sample semantics; obstacle-aware path refinement (when implemented) must validate its altered trajectory using its own generated/evaluable states. Preserve and report the actual `sampled` assurance, sample spacing, occupancy policy and exact pinned map revision. Checking finite samples is **not** continuous swept-collision certification.
6. The initial collision narrow phase is now **rasterized polygon/rectangle boundary by default** with an **optional** scanline-filled interior cell check `checkFootprintInterior=true` (ADR-0029). A boundary-only check may miss an enclosed blocked cell, even if the mask flags the reference-point cell; report the reduced assurance. Other circle/compound body coverage must be explicitly defined (ADR-0021). Out-of-bounds, unknown and occupancy thresholds retain the existing map contracts.

**Interaction with ADR-0022:** this ADR **refines/supersedes only ADR-0022's separate configurable deterministic fixed-sample schedule for the first Ackermann implementation**. ADR-0022's complete-shape intersection, adaptive option default `false`, sampled assurance and budget failure semantics remain in force. **Interaction with ADR-0023:** stored RK4 primitive points become the *default* collision schedule (not merely visualization suggestions); ADR-0023's ability to evaluate between points is still needed for optional adaptive checking, path evaluation or future refiners.

## Validation before further tuning

- Verify a 0.50 m forward/reverse straight or curved primitive returns six ordered samples at 0.10 m increments, RK4 convergence and traveled-length error no more than 0.0001 m.
- Verify six controls at every valid ordinary Ackermann state, their transformed ends and stored samples; test 0.25 m XY and **5-degree/72-bin** wrapped yaw keys, distinct forward/reverse labels with one retained arrival *per complete key*, and no charge for selecting the first gear.
- On a simple map, check candidate rejection when any stored shape overlaps an occupied cell and when a cell lies **inside** a large footprint without crossing its edges; test rotated grid frame, unknown/free/reject/out-of-bounds policy.
- Demonstrate that no separate collision-only fixed sampling is generated by default; additional states are queried only when `adaptiveSubdivision=true`. Build at least one between-samples collision example showing the default's explicitly approximate nature.
- Check first-implementation edge costs against hand-computed forward/reverse and gear-switch primitives; assert **no nearest-obstacle EDT lookup or clearance integration is performed for the MVP edge cost**. Verify goal-region Euclidean lower-bound behavior where certified. Keep clearance-integral/quadrature tests on the low-priority extension checklist.
- Test start-validity, goal tolerances, accepted parent-edge reconstruction, input validation and infeasible/unreachable maps; log sampled-only assurance rather than certified collision freedom.

## Outside approved numeric baseline

No optimizer method, timed execution profile, articulated-mode sample resolution, RK4 root-solver iteration limit, numeric distance-query tolerance, adaptive-refinement depth/threshold, concurrent cache policy or performance guarantee is fixed here. **OMPL-backed near-goal Reeds–Shepp is now required (ADR-0026)**; its proposed numeric trigger radius and performance settings remain for implementation review.
