# ADR-0029: Conservative collision-candidate mask and configurable rasterized footprint checks

**Status:** Accepted (orientation-envelope broad phase; boundary-only narrow phase is the user-selected MVP default, with optional interior checking and explicitly weaker assurance)  
**Date:** 2026-10-25  
**Updates:** [ADR-0021](0021-independent-multibody-collision-geometry.md), [ADR-0022](0022-opt-in-adaptive-collision-subdivision.md), [ADR-0025](0025-approved-ackermann-mvp-settings-and-primitive-sample-collision.md), [ADR-0026](0026-reeds-shepp-near-goal-analytic-expansion.md)

## Context

The user favors Nav2-style rasterized polygon edges over repeated polygon–occupied-cell SAT checks, and wants a map-preprocessed lookup telling Hybrid A* whether a detailed footprint check can be skipped at any sampled pose. The **default** for rectangle/polygon footprints is **boundary-only rasterization**; scanning footprint **interiors is an optional per-request API setting**, off by default. Our occupancy format remains signed `-1/0..100`, *not* Nav2's inflated costmap. These differences limit what the default result can claim.

## Building the collision-possibility mask

For a static **rigid-body** footprint `F` in its local reference frame, define an all-orientation conservative envelope

```text
E ⊇ union_{θ∈[0,2π)} rotate(F, θ)
```

including local offsets and all compound rigid bodies. A simple correct MVP choice is a **disk of radius R** about the reference point, where `R = max_{p∈F} ||p||`: this is conservative for every *continuous* yaw, including yaw values between search bins. Conceptually this disk is the convex envelope of the footprint through all possible rotations. It can be quite loose for long vehicles; optional later refinements are conservative envelopes **per heading bin**, with an explicit bound covering **every** heading in that bin. Never use the hull of a *finite set* of rotated footprints without accounting for angles between them.

Let `B_k` denote the *entire square area* of a blocked occupancy-grid cell under the pinned request occupancy/unknown policy. A reference position `q_xy` can collide at some orientation only if

```text
q_xy ∈ union_k (B_k ⊕ (-E)) .
```

This is the Minkowski dilation of blocked cell squares by the **reflected** footprint envelope. **Lookup cells** are the map-grid's own squares `C_ij`, not the separate Hybrid A* 0.25 m search bins. Set:

```text
flagged[i,j] = true  if C_ij intersects union_k (B_k ⊕ (-E))
             = false otherwise.
```

The **whole reference-point lookup cell** must be considered, because the retained Hybrid A* pose is continuous and is not necessarily located at the cell center. The map origin's rotation is handled by working in map-grid-local coordinates. Outside-map areas are blocking for footprint checking, even when unknown `-1` is allowed as free.

**Practical first build:**

1. Classify cells as blocking under `maxTraversableOccupancy` and `UnknownCellPolicy`, with the request's pinned map snapshot. Unknown-`Reject` prevalidates the **entire** snapshot first (ADR-0019).
2. Create a 2D bitset/byte `flagged` table at **map resolution**. For each blocked cell square, conservatively dilate by the reflected all-orientation disk `R`, marking **every reference-point cell it intersects**. A deliberately looser implementation may use an axis-aligned square of radius `R` and mark all intersecting reference cells; this produces more flagged cells but no false-safe cells. Include map-border proximity/out-of-bounds as another blocking region. For the accepted combined collision-and-cost preprocessing, compute a **single 2D Euclidean distance transform** from **blocking-cell centers** to every lookup-cell center. On a square occupancy grid of resolution `r`, conservatively **flag** a lookup cell whose nearest blocking-cell-center distance is `<= R + sqrt(2)*r` (the `sqrt(2)*r` term covers the half-diagonals of **both** cell squares, including arbitrary subcell reference positions); mark cells within `R` plus a lookup half-diagonal of the **map boundary** as flagged too. This center-distance threshold is deliberately looser than exact Minkowski cell-square intersection and may produce false positives, but should not produce a false-safe cell if the footprint disk and boundary handling are valid. Compare the computed mask against the exact region definition above in tests.
3. For each sampled continuous world pose, transform its `x,y` into the **map-local** frame and select the containing **map cell** by floor. Out-of-map lookup is flagged and must be rejected/validated, never treated as safe. If `flagged=false`, the envelope guarantees no blocked cell can overlap the complete rigid footprint **at that sampled pose**, irrespective of heading. If `flagged=true`, collision is merely possible; check the actual oriented footprint.
4. Rebuild/cache by map source identity+revision, occupancy threshold, unknown policy, geometry/offsets/envelope, map origin/resolution/dimensions and any conservative margin. A missing or stale mask cannot return `SAFE`; fall back to the actual footprint checker.
5. A static all-heading rigid envelope is **not automatically valid for arbitrary articulated internal states**. Later articulated bodies need an envelope proven over all allowed joint states or a model/state-specific conservative lookup; otherwise bypass this optimization.

This is a **broad phase**, not a precomputed collision oracle. A large all-orientation disk may flag much of a cluttered map, reducing its performance benefit; benchmark and consider per-yaw masks only if justified.

## Narrow-phase rectangle/polygon rasterization

When a pose is flagged, transform the *actual* footprint using its full continuous state and independent frame transforms. For each rectangle or polygon, rasterize its boundary edges using a **conservative supercover grid-line traversal**; query the occupancy of every visited map cell, including touched grid-cell corners/boundaries and out-of-map space.

**User-approved per-request default:**

```cpp
struct CollisionCheckOptions {
    bool checkFootprintInterior = false;  // default: polygon/rectangle boundary only
    bool adaptiveSubdivision = false;     // sample interpolation off by default
    // Future adaptive body-movement settings.
};
```

- **`checkFootprintInterior = false` (default):** check boundary-rasterized grid cells only where the broad phase flags a pose. This is faster, but a blocked map cell **completely enclosed by the robot** without intersecting a boundary can be missed. The result **must be identified as `BoundaryOnlyPoseSampled`** (or equivalent), not as complete-footprint collision validity or certified collision freedom.
- **`checkFootprintInterior = true` (optional):** also enumerate all grid cells overlapped by the filled polygon (for example, conservative scanline fill) and reject if any are blocking. Combined with sound supercover/boundary/filled-cell rasterization, this provides **whole-footprint collision testing at the checked poses**, but still *not* continuous swept-motion certification. Mark this as `FullFootprintPoseSampled` or equivalent.
- Circles must use an explicitly documented raster/coverage procedure consistent with the selected assurance; approximating by a few vertices is not an acceptable whole-body guarantee. Compound/multi-body geometry checks its constituent transformed shapes. Self-collision remains a separate policy (OQ-014).
- The broad-phase safe result is stronger than boundary-only detailed checking **at that pose** when the mask is sound. But once the mask says *possible*, a negative **boundary-only** test cannot rule out enclosed obstacles. In particular, **mask flagging does not repair boundary-only false negatives**.

The default is knowingly a performance-oriented **weaker collision check** on our non-inflated occupancy grid. Document this clearly in configuration, result diagnostics, logs and API contracts. A consumer requiring complete-footprint sampled validation must explicitly set `checkFootprintInterior=true`; a path produced in boundary-only mode must not silently be advertised as fully collision-free or unconditionally safe to execute.

## Motion samples, clearance and result semantics

1. **Keep previously approved trajectory sample poses.** During normal Ackermann expansion check its **six stored RK4 poses** (0.10 m separation) and during Reeds–Shepp analytic expansion check each segment at ≤0.10 m including cusps. A safe broad-phase answer may skip rasterization **for that pose**; inspecting only the successor cell without considering intermediate poses is not sufficient.
2. **No continuous swept-volume testing or guarantee.** Both default and optional interior mode are discrete **pose-sampled** checks. `adaptiveSubdivision` remains independent, default false, and does not make the result mathematically continuous-collision certified. Always distinguish (a) boundary-only vs full-interior coverage and (b) primitive-sampled vs adaptively refined density.
3. **Use that same precomputed distance transform for the 0.40 m soft clearance cost**, while keeping it distinct from the binary collision decision. A collision-safe mask does not imply zero soft clearance penalty. Threshold the **same field** again, with the footprint envelope + 0.40 m influence radius and the necessary cell-square correction, for a clearance-influence lookup; no second distance-transform build is needed. At poses within clearance influence, sample the field **along the actual transformed rasterized footprint boundary**, not only at the robot reference-point cell. Cache the field by map/policy; cache geometry-specific collision and clearance masks by compatible footprint/envelope and threshold settings. Details below.
4. **An optimizer cannot silently upgrade a boundary-only path into a validated full-footprint solution.** Validation of a refined path must report its actual configured coverage and sampling assurance; a consumer requiring full-footprint validity must use the interior-inclusive mode and its final validation. The resulting geometric path may still need steering/time constraints and later execution-stage validation.

## Shared distance transform for obstacle-clearance costs

For the **first Ackermann implementation**, compute one 2D Euclidean distance transform `D[c]` in meters from each map-cell center to its nearest **blocking-cell center**, consistent with the pinned `MapSnapshot`, unknown policy and occupancy threshold. Handle out-of-map boundaries as blocking separately or include a conservative boundary-distance field. An all-free interior map has `D=+infinity` away from boundaries; cap only at the finite influence thresholds used for decisions. The transform is **distance to the nearest obstacle**, **not** the separate goal-rooted reverse-Dijkstra field used by the Hybrid A* obstacle-aware heuristic (ADR-0027).

Derive both the collision-candidate flag and clearance-influence flag from this **one** field for the static rigid Ackermann envelope radius `R`. For square grid resolution `r`, a deliberately conservative cell-center test is:

```text
collisionCandidate[c] = (D[c] <= R + sqrt(2)*r) or nearMapBoundary(c, R)
clearanceCandidate[c] = (D[c] <= R + 0.40m + sqrt(2)*r)
                        or nearMapBoundary(c, R + 0.40m)
```

`collisionCandidate=false` can eliminate detailed geometry checking **at that pose** if the conservative geometry, subcell and boundary guarantees have been tested. `clearanceCandidate=false` implies the 0.40 m soft clearance penalty is zero for the actual rigid footprint, assuming those same guarantees. **They are different thresholds over the same distance field**, not two separate distance transforms. With a long vehicle, the conservative all-heading disk can make both flags common; tighter per-heading envelopes remain an optimization only if benchmarks justify them.

When clearance could matter, rasterize the **actual oriented footprint boundary** with the existing conservative supercover iterator and read the already-precomputed `D` for its visited grid cells. Define the first soft-cost estimate for a pose as:

```text
d_clear_hat(q) = max(0, min_{c in rasterBoundary(q)} D[c] - sqrt(2)*r)
phi(q) = max(0, 1 - d_clear_hat(q) / 0.40m)^2
J_clearance(primitive) ≈ 0.15 * trapezoidal_integral_s phi(q(s)) ds
```

The `sqrt(2)*r` subtraction covers the half-diagonals of **the visited boundary cell and the obstacle cell** in the worst case, so this is a deliberately conservative (possibly low) **clearance estimate**, not the exact continuous shape-to-obstacle distance. Account for map boundaries in the minimum. The `0.15` coefficient, `0.40 m` radius and existing 0.10 m primitive-state samples are unchanged (ADR-0025). Apply the same approximation to every signed segment of an analytic Reeds–Shepp connector and the correct gear-switch costs; no nearest-obstacle search during individual expansions.

**Do not** use only `D[rearAxleCell]` as full-footprint clearance: a long body may come near an obstacle while its rear axle remains far away. **Do not** call this estimate a collision validator: with default `checkFootprintInterior=false`, a blocked cell enclosed by the footprint may still be missed by both the edge collision check and boundary-based soft clearance. Interior-inclusive checking remains the opt-in mode required to detect such cases at sampled poses. Neither cost mode changes the pose-sampled (not continuous-swept) assurance.

Cache the shared nearest-obstacle EDT using the *effective* map source/revision, occupancy/unknown policy, map geometry and blocking interpretation. The field itself is independent of the robot footprint and goal. Geometry-specific flags additionally depend on the envelope/body transforms and influence radius. Reuse only with matching effective inputs; no Nav2 costmap is introduced.

## Tests and benchmark requirements

- A small occupied grid cell fully enclosed by a large robot footprint, with every footprint edge cell free: **default boundary-only mode may miss it and MUST report weaker coverage**; `checkFootprintInterior=true` MUST detect it.
- Check supercover edge traversal under corner grazing, rotated map origin, negative map coordinates, occupancy threshold equality, unknown Occupied/Free/Reject and all outside-map contact cases.
- Confirm the conservative hull mask marks **every** reference-point map cell whose area could intersect a dilated occupied square, including arbitrary subcell positions, yaw values between sampling bins and full map borders. Compare randomly sampled poses against a separate whole-shape intersection oracle; **mask safe → oracle safe**, with no false negatives permitted.
- Verify unknown-`Free` is optimistic, not true obstacle-freedom; a newly published map revision invalidates old mask reuse for new requests without invalidating already pinned requests.
- Ensure collision checks apply to every primitive sample and Reeds–Shepp cusp/segment sample. Confirm adaptive density and interior coverage are separate knobs and their descriptors are independent.
- Test the shared nearest-obstacle EDT on tiny maps against brute-force blocked-cell-center distances, including unknown-policy variants, rotated map frames, out-of-map boundaries and empty/full blocked maps. Verify both masks use the **same** field and that clearance flags cannot skip cost for an obstacle within 0.40 m of the *actual footprint*.
- Compare boundary-cell EDT clearance against brute-force continuous rectangle/polygon-to-blocked-square distances in fixtures; test `sqrt(2)*r` conservativeness, straight and turning paths, rear-axle-far/body-near configurations, footprint interior false negatives and the accepted trapezoidal cost coefficient/units. Distinguish non-exact soft-clearance estimates from full-footprint collision validity.
- Measure EDT and mask build/reuse time and memory, flagged fraction, edge rasterization, per-primitive EDT lookups, clearance-scoring runtime, optional interior-fill cost, false-safe counts (must be zero) and collision outcomes. Benchmark all-heading disk against future tighter per-bin envelopes only when justified.

## Deferred

Native Nav2 costmap inflation, a mathematically certified **continuous** swept-collision checker, high-fidelity geometric polygon-vs-square SAT in the hot path, yaw-bin mask optimization and arbitrary articulated all-state envelope preprocessing are not required for the first Ackermann slice. The model-independent geometry contract is unchanged.
