# ADR-0026: Reeds–Shepp near-goal analytic expansion in the Ackermann MVP

**Status:** Accepted (include connector in first Ackermann end-to-end implementation; initial trigger radius is provisional/tunable)  
**Date:** 2026-10-25  
**Related:** [ADR-0004](0004-native-steering-functions.md), [ADR-0014](0014-fixed-curvature-ackermann-baseline.md), [ADR-0016](0016-default-pose-goal-tolerances.md), [ADR-0022](0022-opt-in-adaptive-collision-subdivision.md), [ADR-0025](0025-approved-ackermann-mvp-settings-and-primitive-sample-collision.md)

## Context

Short fixed-length RK4-generated motion primitives can approach a goal while missing a precise feasible finishing connection. The user requires a **Reeds–Shepp path-checking attempt when Hybrid A* comes close to the goal pose** in the very first implementation, rather than deferring analytic goal connections. Reeds–Shepp is valid for the baseline three-state geometric Ackermann model under its fixed minimum-radius forward/reverse assumptions, not a general articulated model or a proof of steering-rate continuity.

## Decision

1. **A native Reeds–Shepp near-goal analytic connection is required in the first Ackermann MVP.** Use the native Reeds–Shepp implementation of [ADR-0004](0004-native-steering-functions.md), not an OMPL runtime dependency. The first model remains `(x,y,yaw)` with fixed minimum turning radius `R_min` and reversible constant-curvature sections (ADR-0014).
2. **Attempt an analytic connection from the actual continuous state of a selected/expanded Hybrid A* node to the requested goal pose** when the node is sufficiently near the goal. Use a cheap **configurable Euclidean XY radius** to gate attempts; do not require the yaw already to match, because the connector is intended to correct it. **Provisional first test value: 5.0 m**; this value is a suggested tuning default, not yet individually approved by the user. Attempt at most once per normally expanded (non-stale) node; keep attempts bounded by existing planning budgets and report attempts and successes.
3. **Represent the entire analytic candidate as an evaluable piecewise path**, with straight/left/right sections and explicit forward/reverse directions, internal gear-change cusps and endpoint. A geometric shortest Reeds–Shepp candidate is a *connection proposal*, not necessarily least-cost for our weighted reverse/switch/clearance objective. Exact Dubins/Reeds–Shepp geometry is evaluated analytically; these connectors are **not** generated using the baseline primitive RK4 process.
4. **Sample and collision-check before accepting the candidate**. Generate complete-state samples on **each** Reeds–Shepp segment at **no more than 0.10 m of traveled arc length** for the initial Ackermann baseline, including the current node, every segment boundary/gear-change cusp, and the goal endpoint. At every sample, apply the **same complete transformed robot-body shape-versus-blocking-occupancy-cell checker** and pinned immutable map revision/occupancy policy as ordinary primitives. Validate curvature, finite states, model limitations, and start/end continuity. Use the same request-level `adaptiveSubdivision=false` default; when true and implemented, additional between-sample evaluations may be requested. Never substitute straight-line interpolation between endpoints or assert continuous swept-collision certification.
5. **Cost the entire candidate consistently with normal search primitives** ([ADR-0025](0025-approved-ackermann-mvp-settings-and-primitive-sample-collision.md)): forward length weight 1.00, reverse length weight 1.10, **0.50 m for each actual direction switch** including between the incoming search-node direction and the first Reeds–Shepp segment, plus whole-footprint sampled-clearance preference 0.15 with 0.40 m influence radius, using the same equivalent-distance units and occupancy interpretation. Do not silently rank a Reeds–Shepp candidate by geometric length alone as if that were the configured full path cost.
6. **Accept only a valid connection and apply goal semantics.** The initial analytic connector targets the **exact requested goal pose**, which necessarily passes the configured `0.25 m / 10°` default pose tolerances if successfully reached. Preserve the exact reached endpoint rather than snapping a different endpoint; validate against the actual goal predicates and any extra terminal restrictions. If the nominal goal center is obstructed or the connector fails, **continue ordinary Hybrid A***: an alternative endpoint *within* the allowed goal region may still be valid (ADR-0016). The analytic connector does not replace basic goal-region checking on ordinary nodes.
7. **Lean termination for initial working implementation:** when the search selects a node within the trigger radius and finds a **fully validated** analytic connection, it may accept that complete candidate and finish with a successful raw path, **without a global optimality guarantee**. Preserve stable ancestry, the actual connector segments and direction metadata, then independently validate the reconstructed complete path before reporting success. Failing candidates are simply discarded and ordinary search continues. More elaborate best-candidate stopping policies (OQ-011) are not a first-MVP prerequisite.
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

## Verification

- Native Reeds–Shepp path-family, length, endpoint, direction, cusp and numerical degenerate-case tests; independent analytic or optional test-only OMPL oracle checks (ADR-0004).
- Near goal with mismatched yaw: trigger by XY radius, build a connector to exact goal; confirm continuous endpoint and every segment's start/end match.
- Connector generated sampling: no greater than 0.10 m arc-length gaps **per segment**, include every cusp/boundary and endpoint, check the full rotated footprint against blocked cells and unknown/out-of-map semantics. Construct cases where endpoints are free but a Reeds–Shepp arc or reverse cusp collides; reject and keep searching.
- Correctly cost mixed forward/reverse connector, initial gear transition, internal cusp switches and full-footprint clearance; ensure nonnegative equivalent-distance edge costs and stable path reconstruction.
- With an occupied exact target pose but a valid endpoint elsewhere inside goal tolerances, never reject the whole request just because the exact Reeds–Shepp connection fails.
- Attempt only within configured radius at expanded nodes, record diagnostics and honor existing cancellation/expansion/time limits.
- Report the accepted raw path's sampled-only collision assurance and lack of certified optimality; do not imply finite steering-rate feasibility.

## Deferred details

The default near-goal radius is **suggested at 5.0 m for initial test review**, not a previously approved numeric setting. Exact connector interface signatures, optional attempt throttling, performance tuning, multi-family weighted-cost candidate selection, complete goal-region steering search, and a provably cost-optimal termination rule remain implementation-facing OQ-011 details.
