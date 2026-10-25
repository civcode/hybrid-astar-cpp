# ADR-0022: Complete-shape grid intersection with opt-in adaptive collision subdivision

**Status:** Accepted (MVP validation method and default; exact sampling resolution and continuous assurance remain implementation details)  
**Date:** 2026-10-25  
**Related:** [ADR-0021](0021-independent-multibody-collision-geometry.md), [ADR-0017](0017-explicit-unknown-occupancy-policy.md), [ADR-0018](0018-known-free-only-occupancy-threshold.md), [ADR-0020](0020-evidence-driven-mvp-simplicity.md)

## Context

The planner checks circles, rectangles and convex polygons associated with independent model-defined robot frames against occupancy-grid cells along actual kinematic trajectories. Sampling vertices or edges alone can miss a cell completely inside a large robot footprint. Sampling every point of a polygon interior is unnecessary, however: polygon/cell overlap can be tested geometrically. The first viable implementation should favor a simple fixed-sampling baseline while allowing finer trajectory checks when explicitly enabled.

## Decision

1. **At each evaluated configuration, test the complete transformed shape against traversability-blocking map cells.** Do not sample polygon interiors, vertices or edges as proxies for shape occupancy. For convex polygons/rectangles versus occupancy-cell squares, use a sound convex intersection test (e.g., separating-axis theorem); circles may use a circle/rectangle intersection test. Treat occupied/unknown/out-of-bounds according to the validated map and request policies. Query candidate cells through the shape's map-grid-local bounding box, with narrow-phase tests only for potentially colliding cells. Conservative contact/boundary semantics and numerical robustness require tests.
2. **Use non-adaptive (fixed) sampling by default.** A motion validator always checks the start and end and a **configurable, deterministically spaced sequence of intermediate complete states** along the *actual propagated motion*, including in-place and articulated motion. The exact fixed step/resolution units and initial default value remain an implementation-facing OQ-009 detail; `adaptiveSubdivision=false` does **not** mean endpoint-only validation. Avoid asserting a particular numeric step without scenario benchmarks and shape-scale evidence.
3. **Expose adaptive subdivision as an opt-in request-level collision-validation option, defaulting to `false`.** When enabled, begin from the same fixed samples, then subdivide an interval using a **per-body bound/estimate of point displacement** from translation and rotation of each body frame, including articulation; compare the worst affected body with an explicitly configured refinement threshold. This must not be based solely on reference-point motion or endpoint angular difference, which can miss intermediate excursions. Vertices can be used to bound convex polygon radius relative to the attached frame; circles use center offset plus radius.
4. **Maintain sampling budgets and unambiguous failures.** Limit adaptive depth/number of evaluations and report inability to meet the configured refinement criterion (e.g., exhausted budget or invalid propagation), rather than silently declaring an unchecked interval valid. The exact option names, thresholds and limit defaults are not frozen ABI.
5. **Both fixed and adaptive *sampling alone* are approximate swept-motion validation**: exact shape-vs-cell intersection at checked states does not prove no collision between states. Return a validation-assurance descriptor identifying fixed/adaptive sampled status and effective sampling parameters; neither may claim conservative continuous collision certification. A certified swept-volume/distance-bound mode may be introduced later as a separately proven and explicitly named guarantee, not implicitly by enabling adaptive sampling.
6. Geometry and kinematics remain separate. The checker queries every collision body at each evaluated state through `frameTransform(q, frame)`; a long trailer or a stationary-reference articulation can require subdivision even when the tractor's reference point barely moves. The exact model-independent pairwise self-collision policy remains OQ-014.

## Illustrative request options (not final ABI)

```cpp
struct CollisionCheckOptions {
    bool adaptiveSubdivision = false; // accepted default
    // Independently configured fixed-sample spacing / extent.
    // If adaptive: max body-point displacement, budget/depth.
};
```

The baseline fixed sampling parameters **must be defined and reported** before relying on the implementation, and an appropriate step size is not implied by the boolean alone.

## Tests

- Complete-shape collision with an occupied cell fully *inside* a rectangle or convex polygon, with no vertex or edge intersection; polygon-cell edge crossing, containment in both directions and boundary tangencies.
- Rotated occupancy-grid origin, circles, rectangles, convex polygons and compound multi-frame bodies, including unknown policy, out-of-map treatment and threshold behavior.
- Fixed default `adaptiveSubdivision=false`: deterministic scheduled start/end/intermediate checks; no additional recursive subdivision.
- Enabled adaptive mode: more evaluations for high angular sweep of distant trailer corners, zero-reference-translation rotation/articulation, and intervals with large body movement; avoid inferring zero movement from coinciding endpoint transforms.
- Sampling-budget exhaustion produces a distinct incomplete-validation outcome, not a successful collision-free assurance.
- Fixed and adaptive modes both report sampled-only assurance; construct a trajectory whose mid-interval collision is missed by a coarse fixed schedule and detected after refinement, and document that refinement still does not guarantee detection in all cases.

## Outside this decision

Selection of concrete fixed-sampling resolution/default, geometric numeric tolerances, admissible conservative bounding method and any future certified continuous checker remain OQ-009 implementation matters. Self-collision pair inclusion/exclusion remains OQ-014. This ADR does not introduce meshes, interior point sampling or new vehicle-model methods.
