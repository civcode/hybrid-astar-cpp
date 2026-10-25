# ADR-0029: Conservative footprint collision mask and rasterized-geometry narrow phase

**Status:** Accepted direction; **boundary-only sufficiency is explicitly unproven and unsafe without an interior guard**  
**Date:** 2026-10-25  
**Updates:** [ADR-0021](0021-independent-multibody-collision-geometry.md), [ADR-0022](0022-opt-in-adaptive-collision-subdivision.md), [ADR-0025](0025-approved-ackermann-mvp-settings-and-primitive-sample-collision.md), [ADR-0026](0026-reeds-shepp-near-goal-analytic-expansion.md)

## Context

The user favors Nav2-style rasterized footprint-boundary checks over performing per-blocking-cell polygon/square SAT intersection at every pose and proposes preprocessing conservative collision-candidate cells using a convex hull/envelope of robot footprints at different orientations. The occupancy map retains our own signed 0..100/-1 encoding, without adopting a Nav2 layered/inflated costmap.

## Decision

1. **Replace mandatory per-pose polygon-vs-occupied-square SAT with a rasterized footprint narrow phase for the first Ackermann MVP.** For each evaluated pose, transform the polygon boundary, map its edges into integer grid coordinates and rasterize with a **conservative/supercover grid-line iterator** that visits cells touched by each edge; reject if encountered cells are blocking under the request-level occupancy/unknown/out-of-bounds policy. Circle footprints require an explicit conservative rasterization/coverage rule; do not silently replace them with only a few sampled points. Compound models check all transformed bodies. Keep the geometry independent of kinematics.
2. **Safety caveat: boundary-only rasterization is not a sufficient general collision test on a raw occupancy map.** An occupied cell may lie **entirely inside** a large footprint without touching the polygon's boundary; blocking-cell corners grazing a polygon can also be missed by a non-supercover grid line iterator. Nav2's edge-first check is used with **inflated costmaps and center-cell logic** that our raw occupancy grid does not have. A conservative broad-phase mask indicating *possible* collision does not solve this; its `false` answer can safely certify free (given a sound mask), but its `true` answer is not a collision decision. **Before calling a pose collision-free, provide an interior-occupancy guard**, e.g. a raster scanline-fill of the footprint cells or checking blocked candidate cells for containment. This is not a return to per-cell polygon-vs-cell SAT. Until the guard is implemented and tested, do not claim the raster-edge-only fallback validates whole-footprint collision freedom.
3. **Precompute an orientation-aware conservative broad-phase lookup** for immutable grid snapshots. For the first rigid Ackermann footprint, construct the **convex hull of all robot-body footprint vertices across heading orientations** as an envelope relative to the reference point, accounting for local offsets; optionally use separate conservative envelopes **per orientation bin** for tighter masks. An envelope across sampled headings needs an explicit bound for orientations *between* samples (e.g. padding from `R_max * deltaYaw`, or a conservative circumscribed-radius envelope). Expand the projected footprint/reference-location reach to cover **quantization error**, entire grid-cell squares, transformed grid origin and out-of-map bounds. Build a map-dependent table marking **reference-point grid cells where intersection *may* occur** with any blocking occupied/unknown/out-of-bounds cell. A `false` lookup may bypass narrow phase only with verified conservativeness; a `true` lookup invokes rasterized footprint checking **plus** the interior guard. False positives cost CPU but are safe; false negatives are not acceptable.
4. **Optimize the common case without assuming a Nav2 costmap.** The broad-phase table is a bitset/byte grid or optional orientation-indexed table. Build/reuse it under map identity/revision, occupancy threshold, unknown policy, geometry, local footprint/hull, yaw-bin assumptions and margin; invalidate/rebuild when any effective input changes (ADR-0019). For an untrusted/stale/missing table or atypical state/shape, **fall back to geometric checking**, never optimistic free. Under strict unknown-`Reject`, prevalidate the entire provided map as already accepted. If map boundary is crossed by any body, do not use a `safe` mask entry produced without that out-of-bounds condition.
5. **Keep motion sampling but drop continuous swept-volume claims.** At each ordinary Ackermann primitive, check the **six stored full-state poses** (0.10 m spacing) and each analytic Reeds–Shepp segment at ≤0.10 m including cusps; intermediate poses do not become search nodes. This is **pose-sampled collision checking**, not a continuous sweep test and not endpoint-only validation. A broad-phase lookup is performed **for every evaluated pose or through a proven conservative bound for the whole interval**—checking the successor reference cell alone is not a valid replacement for path sampling. The optional adaptive subdivision (off by default) remains possible but is not a continuous swept-volume guarantee.
6. **Separate hard collision and 0.40 m soft clearance.** A collision broad-phase bit marked `false` means the footprint is collision-free at that pose, but it does **not** imply the nearest obstacle is at least 0.40 m away. For shortcutting soft clearance, build a separately expanded conservative **clearance-influence mask** (footprint envelope + 0.40 m), or run the bounded clearance query. Do not silently treat unflagged collision cells as zero-clearance-cost cells.
7. **Keep exact assurance language.** Even with the interior guard, pose-sampled collision validation cannot certify the continuous motion between poses. Do not report mathematical continuous-sweep validity or exact shape-vs-cell intersection unless the implemented rasterization method actually guarantees the relevant occupied-cell contact semantics.

## Suggested first implementation mechanism (not fixed C++ API)

```text
preprocess(snapshot, occupancyPolicy, rigidFootprint):
    E = conservative hull/envelope across allowed orientations + offsets
    mask = dilation(blockingCells, reflected(E) + positionBinMargin + gridMargin)
    clearanceMask = dilation(blockingCells, reflected(E) + positionBinMargin + gridMargin + 0.40m)
    # use correct world/grid transforms; out-of-bounds is blocking

for each stored full-state pose in a candidate motion:
    if mask[referencePoseGridCell] == SAFE:
        poseCollisionFree = true               # only when mask construction is sound
    else:
        polygon = transformFootprint(pose)
        if rasterizedEdgeTouchesBlockedCell(polygon, policy): reject candidate
        if interiorScanlineOrContainmentTouchesBlockedCell(polygon, policy): reject candidate
    if clearanceMask[referencePoseGridCell] != SAFE:
        evaluateFullBodyClearanceWithin0_40m(pose)
```

A tight per-heading bitset may be constructed later. The simplest conservative all-heading hull avoids making the fast-path result depend on yaw quantization and helps with arbitrary continuous approach poses at the cost of more false positives.

## Tests and benchmarks

- Tiny blocked cell completely enclosed by a large robot footprint, with every footprint boundary cell free: boundary-only **must not** pass this collision after the interior guard is added.
- Grazing at cell corners, rotated map origins, negative world/map coordinates, occupancy threshold equality, unknown Occupied/Free/Reject, near map edge/out-of-bounds and nonconvex compound bodies.
- Validate the conservative mask independently by comparing **mask-safe implies geometry-free** over randomized continuous `x,y,theta`, including poses inside the same search bin at different sub-cell offsets and headings between hull samples. If it fails, disable the mask optimization.
- Bench raw raster scanning, scanline interior guard and precomputed broad phase vs a full polygon/square intersection reference **in test code only**. Measure mask builds per map revision, bitset size, lookup counts, cache invalidations and total search time.
- Verify the collision mask alone is not used to suppress the clearance preference inside 0.40 m, and that the same mask logic applies to all Reeds–Shepp cusp/segment poses.

## Deferred

Native Nav2 costmap inflation, continuous swept collision, per-heading mask packing, exact shape-vs-cell SAT runtime implementation and broader articulated envelope precomputation are not prerequisites for the first Ackermann slice. The multi-body model contract is unchanged.
