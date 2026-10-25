# ADR-0026: Reeds–Shepp near-goal analytic expansion in the Ackermann MVP

**Status:** Accepted; OMPL runtime steering, 20 m XY trigger and collision-first feasibility; no mandatory connector length limit  
**Date:** 2026-10-25  
**Related:** [ADR-0004](0004-native-steering-functions.md), [ADR-0014](0014-fixed-curvature-ackermann-baseline.md), [ADR-0016](0016-default-pose-goal-tolerances.md), [ADR-0022](0022-opt-in-adaptive-collision-subdivision.md), [ADR-0025](0025-approved-ackermann-mvp-settings-and-primitive-sample-collision.md)

**Updates:** [ADR-0028](0028-ompl-backed-steering-and-junior-style-refinement.md) changes the required connector implementation from native to an OMPL-backed adapter. [ADR-0027](0027-combined-hybrid-a-star-heuristic-and-guarded-analytic-expansion.md) retains actual weighted-cost evaluation with an **optional** cost cap; a hard path-length cap is **not used** in the MVP. The XY trigger is now **20.0 m**. [ADR-0029](0029-conservative-orientation-hull-mask-and-raster-footprint-collision.md) changes the geometric collision narrow phase, but not 0.10 m connector pose sampling.

## Context

Short fixed-length RK4-generated motion primitives can approach a goal while missing a precise feasible finishing connection. The user requires a **Reeds–Shepp path-checking attempt when Hybrid A* comes close to the goal pose** in the very first implementation, rather than deferring analytic goal connections. Reeds–Shepp is valid for the baseline three-state geometric Ackermann model under its fixed minimum-radius forward/reverse assumptions, not a general articulated model or a proof of steering-rate continuity.

## Decision

1. **An OMPL-backed Reeds–Shepp near-goal analytic connection is required in the first Ackermann MVP** behind the project-native steering interface (ADR-0028). The first model remains `(x,y,yaw)` with fixed minimum turning radius `R_min` and reversible constant-curvature sections (ADR-0014).
2. **Attempt an analytic connection from the actual continuous state of a selected/expanded Hybrid A* node to the requested goal pose** when the node is sufficiently near the goal. Use a cheap **configurable Euclidean XY radius** to gate attempts; do not require the yaw already to match, because the connector is intended to correct it. **Accepted MVP near-goal trigger: 20.0 m XY distance**, configurable per planning request. The larger window can generate many expensive connector attempts; honor budgets, try at most once per normally expanded node, and retain counter/timing diagnostics. Attempt at most once per normally expanded (non-stale) node; keep attempts bounded by existing planning budgets and report attempts and successes.
3. **Represent the entire analytic candidate as an evaluable piecewise path**, with straight/left/right sections and explicit forward/reverse directions, internal gear-change cusps and endpoint. A geometric shortest Reeds–Shepp candidate is a *connection proposal*, not necessarily least-cost for our weighted reverse/switch/clearance objective. Exact Dubins/Reeds–Shepp geometry is evaluated analytically; these connectors are **not** generated using the baseline primitive RK4 process.
4. **Sample and collision-check before accepting the candidate**. Generate complete-state samples on **each** Reeds–Shepp segment at **no more than 0.10 m of traveled arc length** for the initial Ackermann baseline, including the current node, every segment boundary/gear-change cusp, and the goal endpoint. At every sample, apply the **same complete transformed robot-body shape-versus-blocking-occupancy-cell checker** and pinned immutable map revision/occupancy policy as ordinary primitives. Validate curvature, finite states, model limitations, and start/end continuity. Use the same request-level `adaptiveSubdivision=false` default; when true and implemented, additional between-sample evaluations may be requested. Never substitute straight-line interpolation between endpoints or assert continuous swept-collision certification.
5. **Cost the entire candidate consistently with normal search primitives** ([ADR-0025](0025-approved-ackermann-mvp-settings-and-primitive-sample-collision.md)): forward length weight 1.00, reverse length weight 1.10, **0.50 m for each actual direction switch** including between the incoming search-node direction and the first Reeds–Shepp segment, using the same equivalent-distance units and occupancy interpretation. **No clearance integral is part of the first implementation**; the previously proposed 0.15/0.40 m whole-footprint preference is reserved for a low-priority opt-in cost extension, applied consistently to primitive and analytic edges if ever enabled. Do not silently rank a Reeds–Shepp candidate by geometric length alone as if that were the configured full path cost.
6. **Accept only a valid connection and apply goal semantics.** The initial analytic connector targets the **exact requested goal pose**, which necessarily passes the configured `0.25 m / 10°` default pose tolerances if successfully reached. Preserve the exact reached endpoint rather than snapping a different endpoint; validate against the actual goal predicates and any extra terminal restrictions. If the nominal goal center is obstructed or the connector fails, **continue ordinary Hybrid A***: an alternative endpoint *within* the allowed goal region may still be valid (ADR-0016). The analytic connector does not replace basic goal-region checking on ordinary nodes.
7. **Deterministic feasible-first acceptance in initial implementation:** first test whether the expanded node already satisfies the continuous goal region. Otherwise, if its actual continuous XY position is within **20.0 m** of the target, try the OMPL connector **once** for that expanded node. Accept only if the full connector has finite/valid length, endpoint and controls, its weighted forward/reverse/gear-switch cost is finite and satisfies an **optional explicitly enabled** `maxConnectorCost`, every sampled footprint passes the configured collision coverage, all request budgets hold, and the reconstructed full path passes final validation. **No hard path-length cutoff** is used by default. The single connector may consist of up to five signed curved/straight sections; it is one Reeds–Shepp path, not multiple independently solved connections chained together. Optional cost-cap exceptions, if later supported, must be explicit and reported; **no hidden exceptions**. A rejected candidate never aborts ordinary primitive expansion. The first accepted fully validated connector may terminate feasible-first, **without a global optimality guarantee**. Preserve stable ancestry, the actual connector segments and direction metadata, then independently validate the reconstructed complete path before reporting success. Failing candidates are simply discarded and ordinary search continues. More elaborate best-candidate stopping policies (OQ-011) are not a first-MVP prerequisite.
8. **Keep generic solver boundaries generic:** the search calls a model-compatible optional `tryConnect(currentFullState, targetPose)`-style connector. Reeds–Shepp is enabled for the compatible three-state Ackermann model only; it is not forced onto differential drive, mecanum or articulated models merely because they share `x,y,yaw`.

## Illustrative algorithm (not frozen API)

```cpp
if (connector && xyDistance(node.state, goal.pose) <= goalConnectionRadius) {
    auto candidate = connector->connect(node.state, goal.pose); // Reeds–Shepp
    if (candidate &&
        validatesFullModelAndGoal(*candidate) &&
        collisionChecker.validAtAllSegmentSamples(*candidate, map, options)) {
        candidate->cost = costWholeConnector(*candidate, incomingGear,
                                             searchCostOptions);
        // For lean feasible-first MVP: reconstruct and independently validate
        // the full prefix + connector before returning success.
        return validatedPathFrom(node, *candidate);
    }
}
// If no accepted analytic connection, expand ordinary RK4 primitive successors.
```

## Implementation parameters and verification

```cpp
struct AnalyticExpansionOptions {
    double triggerXYMeters = 20.0;              // accepted default
    std::optional<double> maxConnectorCostMeters; // independently configurable weighted-cost guard
    // explicit exceptional close-quarters policy may be added later
};
```

The 20.0 m trigger is an XY **attempt gate**, not a cap on the distance traveled by the returned analytic path; it is also unrelated to Nav2's 20.0 m kinematic-distance lookup window. Cost guards use the active MVP cost only (forward length ×1.0, reverse ×1.1, 0.50 m per gear transition); the soft clearance integral is deferred. Path length must be **finite and reported**, but does **not** disqualify an otherwise valid collision-checked Reeds–Shepp connection; the OMPL candidate is already shortest by geometric length for the configured turning radius. Optional weighted-cost screening remains user-configurable and disabled by default. Use the incoming search mode for the first connector switch; score internal cusps too. Log trigger checks, attempts, cost/length rejections, collision rejections and accepted connectors.

## Verification

- Native Reeds–Shepp path-family, length, endpoint, direction, cusp and numerical degenerate-case tests; independent analytic or optional test-only OMPL oracle checks (ADR-0004).
- Near goal with mismatched yaw: trigger by XY radius, build a connector to exact goal; confirm continuous endpoint and every segment's start/end match.
- Connector generated sampling: no greater than 0.10 m arc-length gaps **per segment**, include every cusp/boundary and endpoint, check the full rotated footprint against blocked cells and unknown/out-of-map semantics. Construct cases where endpoints are free but a Reeds–Shepp arc or reverse cusp collides; reject and keep searching.
- Correctly cost mixed forward/reverse connector, initial gear transition and internal cusp switches **without a clearance integral in the MVP**; ensure nonnegative equivalent-distance edge costs and stable path reconstruction. Future optional footprint-clearance costs must apply consistently to connectors and primitives.
- With an occupied exact target pose but a valid endpoint elsewhere inside goal tolerances, never reject the whole request just because the exact Reeds–Shepp connection fails.
- Attempt only within configured radius at expanded nodes, record diagnostics and honor existing cancellation/expansion/time limits.
- Report the accepted raw path's sampled-only collision assurance and lack of certified optimality; do not imply finite steering-rate feasibility.

## Deferred details

The near-goal XY trigger is **approved at 20.0 m** (configurable). **No path-length cap is used**; optional weighted connector cost cap is independently configurable and disabled by default. An attempted connection is one full OMPL Reeds–Shepp path containing up to five elementary signed sections. As in Nav2 and MATLAB Hybrid A*, unsuccessful analytic connections may be retried from later popped search nodes, not stitched into a chain of separately solved RS connections. Nav2 adaptively throttles attempts with `analytic_expansion_ratio` and MATLAB uses a configurable `AnalyticExpansionInterval` (5 by default). The specific retry cadence for our 20 m gate is **not yet approved**; instrument attempts, collision failures and wall-clock cost, then select a deterministic throttle if needed. End-to-end budget and cancellation still apply. Exact solver API, attempt throttling beyond once per expanded node, and better candidate choice remain OQ-011 implementation details.
