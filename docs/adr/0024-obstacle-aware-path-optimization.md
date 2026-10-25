# ADR-0024: The path optimizer must be obstacle-aware

**Status:** Accepted (architectural requirement; numerical optimization method remains OQ-013)  
**Date:** 2026-10-25  
**Related:** [ADR-0020](0020-evidence-driven-mvp-simplicity.md), [ADR-0021](0021-independent-multibody-collision-geometry.md), [ADR-0022](0022-opt-in-adaptive-collision-subdivision.md), [ADR-0019](0019-live-map-snapshot-cache-and-reject.md)

## Context

The intended pipeline uses Hybrid A* to produce a collision-checked, model-feasible initial path, then a separate optimizer to improve geometric smoothness, length, clearance, and maneuver quality. An optimizer that ignores the environment can repeatedly produce smooth but colliding candidates which are then rejected by a final collision checker. That makes poor use of the search result and computation, especially near obstacles or for multi-body articulated vehicles.

## Decision

1. **Whenever path optimization/refinement is enabled, it must account for static obstacles *during* optimization**, not only as a final binary accept/reject step. The optimizer consumes the validated initial path, the same immutable map snapshot (identity and revision) and request-specific occupancy interpretation as the search, and the complete model-dependent collision-body geometry. For an application that deliberately optimizes against a newer map, require a new explicitly supplied snapshot and corresponding validation context; never silently switch revisions.
2. **Combine path-quality objectives with obstacle information**: e.g., clearance constraints, a collision/clearance penalty, or a local feasible-update/line-search rule. The initial implementation may use simple discrete shape-vs-cell collision queries or a map-derived clearance/distance field if measured useful. Avoid requiring a specific optimizer, solver, distance transform, gradient method, smooth penalty, continuous-collision engine or large optimization framework now (OQ-013 remains open).
3. **Use actual complete robot geometry, not reference-point-only obstacle distances.** For every evaluated configuration and intermediate motion sample, apply each model-provided frame transform to all independent circles/rectangles/convex polygons, including articulated bodies and trailers. A point-path clearance proxy is not a substitute for full-body collision validity.
4. **Preserve search's essential decisions and model validity**. Optimize within model/control, start/goal, joint limits, gear/maneuver and configuration constraints. Local smoothing cannot be assumed to repair missing homotopy classes, disconnected corridors, or missing reverse maneuvers. New candidates may be temporarily infeasible inside an iterative solver if the algorithm handles them, but no infeasible final candidate may be accepted.
5. **Independently validate every proposed optimized replacement** against model validity, endpoint/boundary conditions, and the configured collision checker, **including whether interior cells were checked**, with fixed/adaptive pose sampling reported independently (ADR-0029). The default boundary-only raster mode cannot itself establish full-footprint collision validity. An obstacle-aware penalty/clearance metric is not collision certification. A failed or unverified candidate must not replace the original validated raw path; report failure and preserve the path and map/assurance provenance.
6. **Keep scope lean and staged**. A standalone first working search pipeline may ship without refinement (ADR-0020); *when* an optimizer is added, obstacle awareness is mandatory. Optimization against moving objects and final timing/rate validation remain responsibilities of the later trajectory planner/controller, not the static-map geometric optimizer.

## Candidate lightweight implementation (not a frozen cost or API)

An obstacle clearance term can be represented schematically by

`J = J_smooth + w_length J_length + w_clear J_clear`,

where `J_clear` penalizes insufficient clearance of the **entire transformed footprint** along the full geometric path under the fixed map policy. Appropriate weight scaling, non-smooth obstacle fields, sampling density, feasible-set treatment and solver choice remain OQ-013. Penalizing collisions does not replace a final constraint/collision validation gate.

Map-dependent distance fields used for obstacle clearance are **distinct** from reverse-Dijkstra cost-to-*goal* heuristic fields: neither their keys nor their admissibility claims can be assumed equivalent. Reuse compatible immutable map-derived data only with matching map identity/revision, occupancy policy, grid geometry, robot geometry/inflation and relevant options (ADR-0019).

## Verification

- A naïve smoothing or shortcut would cut through an occupied cell or unknown-as-occupied region; the obstacle-aware optimizer must account for this during candidate selection. **Default boundary-only collision coverage may miss enclosed obstacles; a complete-footprint final-validation requirement needs `checkFootprintInterior=true`** (ADR-0029).
- Check full-body configurations and intermediate states for rotated rectangles, nonconvex compound unions, articulated chassis and trailer chains; reference-point-only clearance must not pass a colliding candidate.
- Test immutable map snapshot and exact per-request threshold/unknown semantics, including a new map revision published while refinement uses its pinned revision.
- Deliberately fail optimization (unsatisfiable clearance or iteration budget) and verify original validated path remains untouched with diagnostics.
- Compare final validated path quality, optimization success rate and total search+refinement latency, not only raw smoothing costs.

## Open details

The numerical objective, obstacle-distance/clearance representation, solver/parameterization, differentiability, self-collision pair policy, smoothing feasibility guarantees and collision sampling resolution are still OQ-013/OQ-014/OQ-009. This ADR only resolves *when* obstacle information must be considered and the mandatory final validation/fallback behavior.
