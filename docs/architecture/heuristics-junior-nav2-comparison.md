# Two-component Hybrid A* heuristic: Junior, Nav2, and our MVP

**Status:** Architecture comparison and first-implementation guidance (2026-10-25)  
**Decision:** [ADR-0027](../adr/0027-combined-hybrid-a-star-heuristic-and-guarded-analytic-expansion.md)

## Shared core concept

All three combine (by `max`) a **nonholonomic obstacle-free** distance estimate with a **holonomic obstacle-aware** 2D estimate:

```text
h = max(h_vehicle_geometry, h_goal_rooted_obstacle_graph)
```

- **Stanford Junior (2008), §6.3**, uses a precomputed 4D kinematic heuristic (location, yaw and forward/reverse direction) and an online 2D dynamic-programming obstacle heuristic. It calls its two components admissible under its assumptions and takes their maximum. The publication does **not** prescribe how to seed a nonzero goal-tolerance region, handle an obstructed exact goal, or treat a disconnected/unsettled finite-cost grid cell for our input contract. Figure 16 compares their qualitative expansion behavior.
- **Nav2 Smac Hybrid** uses OMPL-backed Dubins/Reeds–Shepp distance (with a local lookup table), and a **lazy** 8-connected goal-rooted obstacle heuristic expanded as required. Its obstacle estimate may incorporate **graded costmap penalties and downsampling**, neither of which our signed occupancy-grid MVP adopts. Its node heuristic takes the maximum. Nav2's `GoalManager::removeInvalidGoals()` accepts goals that are valid at the exact configuration or have a valid configuration in the allowed tolerance region; it reports `GoalOccupied` if no goal candidate is usable. In its current `ObstacleHeuristic::getObstacleHeuristic()`, a requested cell without a finite settled field value is not assigned `+infinity` as a definitive no-path proof; the combined heuristic can fall back to its vehicle-motion estimate.
- **Our initial Ackermann MVP** computes nonholonomic obstacle-free distance from an OMPL Reeds–Shepp adapter and a goal-rooted 2D reverse-Dijkstra grid field over **our own** occupancy policy; then `h=max(h_RS,h_grid)` in an explicitly performance-oriented **not-proven-admissible** mode. Certified mode retains a separately proven Euclidean-to-goal-region or zero fallback. We do **not** claim Stanford's admissibility proof transfers to our cell graph, cost units, finite goal tolerances or quantized one-arrival-per-key search.

## First-implementation rules

1. **Multi-source XY goal region:** seed the 2D reverse-Dijkstra from **all traversable cells whose square intersects the configured XY tolerance disk** around the requested goal. Do not require the nominal goal-center cell itself to be free. Check actual robot heading tolerance, full physical state, configured collision coverage and the final continuous XY error **only when accepting a real terminal Hybrid A* state**. A seed cell is not itself a full-pose goal.
2. **No invented certainty from grid graph exhaustion:** store whether a 2D value is `Unvisited`, `Unsettled`, `SettledFinite`, or `UnreachableByGrid` (equivalent naming is fine). Use `h_grid` only when finite and settled. Otherwise use finite `h_RS` or Euclidean/zero fallback; **never set the Hybrid A* search key's f-score to infinity or return NoPath solely from the 2D grid field**. A coarse 2D graph may be disconnected under its discretization even when a valid continuous motion exists. If no seed is available, document the missing seed and rely on normal final-state validation and request budgets.
3. **Goal-dependent cache:** obstacle-field cache keys include pinned map source/revision/geometry, occupancy/unknown policy, effective goal region and grid settings. A new goal region or map version must not reuse incompatible field values.
4. **Local conservative 2D heuristic limits:** goal-region seeding and finite fallback avoid gross correctness failures; they **do not automatically make the raw 8-neighbor graph value an admissible lower bound** on arbitrary continuous trajectories. Keep the performance-vs-certified mode distinction.
5. **Separate distance fields:** reverse-Dijkstra distance **to the goal** guides node order. Nearest-obstacle EDT distance **to blocked cells** is used only to build the conservative collision-candidate lookup in the first implementation (ADR-0029). Per-edge clearance integration is deferred.
6. **Near-goal OMPL analytic connector:** check at most once per normally expanded node within the configured **20 m XY trigger**, but only accept an entire valid connector after actual length, weighted forward/reverse/switch cost, endpoint, sample-based collision and parent-chain validation. No path-length cap is used; an **optional** weighted-cost cap is disabled by default. A single Reeds–Shepp candidate may have several signed curved/straight segments, and failed attempts may be retried from later search nodes (Nav2 adaptively, MATLAB at a configurable node interval). Continue normal expansion if rejected. The 20 m is **not** a path-length cap and not Nav2's separately named 20 m kinematic lookup-window size.
7. **Planner-instance ownership:** only **one active plan request per instance**; overlapping calls fail fast with Busy, while independent instances may share immutable maps without sharing mutable lazy frontiers.
8. **Search heading key vs goal tolerance:** use **5°/72** wrapped Ackermann search heading bins, with a separate **10°** continuous goal-heading tolerance. Keep collision assurance explicit: boundary-only sampled is default; interior coverage is opt-in.

## Sources

- Stanford Junior paper, §6.3, Figure 16: https://robots.stanford.edu/papers/junior08.pdf
- Nav2 Hybrid node heuristic and max: https://github.com/ros-navigation/navigation2/blob/main/nav2_smac_planner/src/node_hybrid.cpp
- Nav2 lazy 2D obstacle heuristic and zero/unsettled fallback: https://github.com/ros-navigation/navigation2/blob/main/nav2_smac_planner/src/obstacle_heuristic.cpp
- Nav2 OMPL/lookup kinematic distance: https://github.com/ros-navigation/navigation2/blob/main/nav2_smac_planner/src/distance_heuristic.cpp
- Nav2 goal validation / tolerance-zone logic: https://github.com/ros-navigation/navigation2/blob/main/nav2_smac_planner/include/nav2_smac_planner/goal_manager.hpp
- Nav2 Smac Hybrid configuration: https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/smac_hybrid/configuring_smac_hybrid/

## Tests

Construct blocked-exact-goal/free-tolerance-region, entire-goal-region-blocked, disconnected grid graph, intentionally unsettled lazy field, yaw-mismatch within XY region, rotated map and changed occupancy-policy fixtures. Show finite search priorities and no invalid early rejection for the first three, consistent acceptance for the others, and compare combined-vs-single-component expansions and runtime. The full Hybrid A* and heuristic implementation are **not yet written**; this document guides those tests.
