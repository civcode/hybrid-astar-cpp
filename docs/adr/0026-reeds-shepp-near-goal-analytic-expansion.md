# ADR-0026: Adaptive Reeds–Shepp analytic expansion in the Ackermann MVP

**Status:** Accepted (Nav2-style adaptive attempt scheduling; no proximity gate)  
**Date:** 2026-10-25  
**Related:** [ADR-0025](0025-approved-ackermann-mvp-settings-and-primitive-sample-collision.md), [ADR-0027](0027-combined-hybrid-a-star-heuristic-and-guarded-analytic-expansion.md), [ADR-0028](0028-ompl-backed-steering-and-junior-style-refinement.md), [ADR-0029](0029-conservative-orientation-hull-mask-and-raster-footprint-collision.md)

## Context

A single analytic Reeds–Shepp solution can connect a continuous search state to the exact requested goal pose. That **one solution** may contain multiple forward/reverse straight or constant-curvature sections and direction-change cusps; it is **not** a chain of independently searched Reeds–Shepp connectors. An obstacle may block the first candidate while a later Hybrid A* node provides a valid connection. Thus, a single attempt **per planning request** is insufficient.

Nav2 Smac Hybrid attempts analytic expansion at the beginning and adaptively retries as the best observed heuristic distance to the goal decreases (see `analytic_expansion_ratio` in its `analytic_expansion_impl.hpp`). This replaces our earlier **fixed XY radius trigger**: there is **no spatial proximity threshold** or `triggerXYMeters` in the initial planner.

## Accepted decision

1. **Use OMPL** behind a project-native steering adapter for the 3-state reversible fixed-minimum-turn-radius Ackermann model. The adapter provides a single complete Reeds–Shepp solution from the **actual continuous popped-node pose** to the **exact goal pose**, with signed sections, endpoints and cusps (ADR-0028). Other motion models are not obliged to support it.
2. **Schedule by search progress, not XY proximity.** For every popped **non-stale** search node that has not already satisfied the goal predicate, evaluate an adaptive scheduling decision. **Attempt at the first expanded node regardless of distance**. If unsuccessful, continue ordinary primitive expansion and schedule a later independent attempt from a different popped node. At most one complete Reeds–Shepp candidate is generated per scheduled node. Heuristic scheduling does **not** reject candidate poses for being far from the goal.
3. **MVP adaptive cadence**, closely following Nav2:
   - Maintain the **minimum observed finite nonnegative heuristic distance** from popped nodes so far, converted from our equivalent-meter heuristic to **map-grid-cell equivalents** by dividing by map resolution. This quantity is **only a scheduling signal**; it need not be an admissible estimate.
   - With configurable `analyticExpansionRatio` (initial **3.5**, chosen as a Nav2-like tuning starting point rather than a proven optimal value), set `desiredInterval = max(floor(closestHeuristicCells / analyticExpansionRatio), ceil(analyticExpansionRatio))`. It is **at least one**. Keep a countdown, shorten its remaining delay if `desiredInterval` decreases, and retry when it reaches zero. Reset the countdown after every attempt. A smaller heuristic distance generally yields more frequent attempts but not an attempt at every node.
   - Scheduler state is **per planning request**, reset on each new `plan()` even if the lazy goal-rooted Dijkstra field is cached for compatible replans. Count attempts/failures/successes, and enforce cancellation, time, search expansion and resource budgets.
4. **Collision and model validity are mandatory.** Retain exact Reeds–Shepp straight/arc sections. Sample each section at ≤**0.10 m** of traveled arc length including section boundaries, cusps, actual start and exact goal. Apply the pinned occupancy policy and the selected collision checker (default boundary-only supercover footprint rasterization; optional interior checking, ADR-0029). A valid sampled path is **not** certified continuously swept collision-free.
5. **Preserve full maneuver cost**, computed from actual forward length ×1.00, reverse length ×1.10 and **0.50 m per actual forward/reverse switch**, including the incoming search gear and internal cusps. The soft clearance integral is **not** an MVP cost. Verify connector cost is finite; optionally enforce `maxWeightedCostMeters` if explicitly set (**disabled by default**). **No hard path-length cap**: record finite actual path length but do not reject a feasible connection solely because it is long.
6. **Deterministic feasible-first acceptance:** if a scheduled connector passes full model/goal/geometry, cost gate (if enabled), sampled collision tests and **independent validation of the complete prefix plus connector**, it may finish with success; no global weighted-cost optimality claim. If the analytic target exact goal pose is blocked or the candidate fails, continue ordinary Hybrid A* and allow a different terminal state within the accepted **0.25 m XY / 10° heading goal tolerance**. Do not turn analytic failures into search failure.
7. **Simplicity:** no per-heading cache, extra model-generalized API, attempt-length gate or complex adaptive policy beyond this bounded scheduler is necessary before benchmarking. Attempt interval tuning is separate from collision accuracy and from the goal-rooted lazy heuristic's cache state.

## Illustrative control flow (conceptual)

```cpp
AnalyticExpansionScheduler analyticScheduler(analyticOptions); // per plan() call
while (auto node = popNonStaleNode()) {
    if (passesActualContinuousGoalPredicate(*node)) {
        return independentlyValidatedReconstruction(*node);
    }

    // Heuristic in meter-equivalent search-cost units; map resolution in m/cell.
    if (analyticScheduler.shouldAttempt(node->heuristicMeters, map.resolutionMeters)) {
        auto rs = omplAdapter.shortestReedsShepp(node->continuousPose, goal);
        if (rs && validFiniteModelPath(*rs) &&
            validSignedManeuverCost(*rs, node->incomingGear) &&
            optionalWeightedCostGuardPasses(*rs) &&
            collisionChecker.validEveryStoredSectionPose(*rs) &&
            independentlyValidatePrefixPlusConnector(*node, *rs)) {
            return successWithCompleteConnection(*node, *rs);
        }
    }

    expandAndValidateOrdinaryRk4Primitives(*node);
}
```

## API sketch

```cpp
struct AnalyticExpansionOptions {
    double attemptRatio = 3.5; // provisional Nav2-like tuning seed
    std::optional<double> maxWeightedCostMeters; // disabled by default
};
class AnalyticExpansionScheduler {
public:
    bool shouldAttempt(double heuristicCostMeters, double mapResolutionMeters);
    void reset();
};
```

The adapter must not conflate the returned complete RS path's **multiple geometric sections** with repeated attempts; **only** the outer scheduling policy determines when to request a new independent path.

## Verification

- First expanded node triggers an attempt even when far from the goal; **no threshold on Euclidean XY distance** exists. Each subsequent popped node makes at most one scheduling decision.
- With 3.5 ratio, large heuristic-cell distances produce relatively long intervals; approaching the goal reduces the interval to at least `ceil(3.5)=4` popped nodes. Test monotone closest-observed value, mid-interval acceleration, reset between planning requests, extreme finite heuristic/map-resolution inputs and invalid configurations.
- A failed analytic collision/cost/model check **continues ordinary expansion**; later scheduled attempts originate at later popped nodes. Count all attempted complete RS candidates and accept at most one candidate as the feasible-first final connection.
- Validate signed straight/arc lengths, cusp gear switches, exact continuous endpoints, ≤0.10 m analytic segment samples and complete-ancestry reconstruction. Boundary-only and optional full-footprint sampled collision-assurance descriptors remain distinct.
- Verify optional weighted-cost cap (off by default) and **absence** of any hard length or proximity gate; allow long, otherwise valid multi-point maneuvers.
- Benchmark attempts, OMPL work, intermediate collision checks, total search duration and path quality on open-space, doorway and reverse parking scenarios.

## Nav2 source

- Adaptive attempt interval: https://github.com/ros-navigation/navigation2/blob/main/nav2_smac_planner/include/nav2_smac_planner/analytic_expansion_impl.hpp
- Nav2 Smac planner options (includes `analytic_expansion_ratio`): https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/smac_hybrid/configuring_smac_hybrid.html
