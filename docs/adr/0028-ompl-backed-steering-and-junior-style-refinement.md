# ADR-0028: OMPL-backed steering now; Junior-style optimizer and Nav2 smoothing lesson

**Status:** Accepted  
**Date:** 2026-10-25  
**Supersedes:** [ADR-0004](0004-native-steering-functions.md) requirement for native-first Dubins/Reeds–Shepp and test-only OMPL  
**Updates:** [ADR-0007](0007-dependencies-verification.md), [ADR-0024](0024-obstacle-aware-path-optimization.md), [ADR-0026](0026-reeds-shepp-near-goal-analytic-expansion.md)

## Context

A complete native Dubins/Reeds–Shepp solver is considerable engineering effort not essential for the first working planner. Nav2 Smac Hybrid uses OMPL geometric state spaces for its Dubins/Reeds–Shepp steering. We want to prioritize the combined heuristic, a guarded near-goal connector and the full end-to-end search pipeline. Stanford Junior's post-search optimization remains the intended algorithmic direction.

## Decision

1. **Use OMPL in the initial runtime steering module** for Dubins and Reeds–Shepp **distance, shortest-geometric-path extraction and interpolation**, especially the required Ackermann near-goal connector and kinematic heuristic. OMPL is an intentional **scoped runtime dependency**, not merely a test oracle. Keep it behind a small C++20 adapter exposing project-native poses, signed segments, cusp and sampling semantics; **do not leak OMPL types into the generic planner public API** or require OMPL for geometry, maps, collision checking or unrelated models. Match radius/frame/forward-reverse conventions.
2. **Native in-house Dubins/Reeds–Shepp becomes low priority**. Keep it on the backlog as an optional replacement only if runtime dependency, measured performance, packaging or needed control of behavior warrants it; do not block the first Ackermann MVP on a parallel solver. Test the OMPL-backed adapter with analytic endpoint, path-family, signed-segment, cusp, numerical-degeneracy and normalization fixtures.
3. **Retain the Stanford Junior-style downstream optimizer**: obstacle-aware path optimization/refinement after feasible Hybrid A*, with length, smoothness, curvature/steering, reversals and full-body obstacle constraints or costs; separately validate every produced path and keep the original feasible raw path upon failure (ADR-0024). This is the selected target strategy, not a new requirement to implement sophisticated optimization before the raw planner runs.
4. **Record Nav2's contrary practical experience as an explicit benchmark warning.** Nav2's Smac README explains that an earlier **conjugate-gradient-style smoother inspired by the original Hybrid A*** was retired from the default planner path because it was expensive and inconsistent in useful outcomes; a simpler/faster smoother became the default, and a cost-aware smoother is available separately through the smoothing server. This **does not disprove the Junior strategy** for our vehicles, but it means we must measure the optimization success fraction, final validated feasibility/clearance and total search+optimization latency, with a simple obstacle-aware smoother as a baseline. Do not assume the optimizer always improves the result.
5. Preserve our signed occupancy-grid / request-level unknown/threshold semantics for now. **Do not adopt Nav2's graded costmap encoding** in the current MVP. A future separate design review may add costmap concepts; interoperability would require explicit conversion, never reinterpretation.

## Sources

- Stanford Junior (2008), §6.3: https://robots.stanford.edu/papers/junior08.pdf
- Nav2 Smac Planner README and its discussion of CG smoother retirement: https://github.com/ros-navigation/navigation2/blob/main/nav2_smac_planner/README.md
- OMPL Reeds–Shepp space API: https://ompl.kavrakilab.org/classompl_1_1base_1_1ReedsSheppStateSpace.html

## Verification

- Build the standalone planner with OMPL isolated behind a steering adapter, without exposing OMPL in the generic public model/search/path types.
- Test RS shortest-length, interpolation and gear-cusp metadata, including exact / near-exact start and goal, rotations, reverse segments and zero-length cases.
- Benchmark kinematic-heuristic distance-only lookup against full RS path construction.
- When the Junior-style optimizer is added, compare its complete validated output against simpler obstacle-aware smoothing, using identical maps/scenarios and total pipeline latency, not raw smoothness alone.

## Deferred

Exact OMPL version/discovery and linkage defaults, native implementation, optimization solver/Ceres choices and later Nav2-style costmap integration.
