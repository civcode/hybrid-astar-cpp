# ADR-0011: Canonical path position is (segment index, local parameter)

**Status:** Accepted  
**Date:** 2026-10-24

## Context

The planner must represent translating, reversing, rotating-in-place, steering/articulation-only and holonomic combined motions. Translational arc length is zero for valid stationary configuration changes; elapsed time would commit the geometric planner to an arbitrary speed policy. A single scalar world-progress coordinate is therefore not universally meaningful.

## Decision

1. The **canonical address** of a configuration on a path is **`PathPosition{segmentIndex, lambda}`**, with `lambda in [0,1]`. It refers to one identified motion segment and a normalized segment-local evaluation parameter, not elapsed time or necessarily translational distance.
2. Each segment exposes a continuously evaluable **full model state**, with explicit endpoints, maneuver/control provenance, and relevant derivative availability/validity. Body heading and travel tangent need not align for holonomic motion.
3. Adjacent segments must meet in continuous configuration when describing a valid path. Control, tangent, curvature and their derivatives may be discontinuous or undefined across reversing cusps and changes of maneuver; no fictitious global smoothness claim is made.
4. The canonical address preserves segment identity at boundaries. API must document boundary canonicalization and one-sided derivative evaluation; `(i,1)` and `(i+1,0)` may be equal in configuration but have different maneuver semantics. A single right-/left-side rule should be selected explicitly before coding.
5. **Optional derived coordinates** may include traveled arc length for translating intervals, cumulative maneuver costs, normalized total progress, or model-dependent generalized progress. They do not replace the canonical address.
6. Zero-change/degenerated segments must not create artificial progress or break evaluation; genuine zero-translation rotation/joint motions remain nondegenerate segments with a valid local parameter.
7. Geometric segment evaluation must not require ROS headers, MPC time profiles or a particular spline family.

Illustrative API sketch (not final):

```cpp
struct PathPosition {
    std::size_t segmentIndex;
    double lambda;                 // [0,1]
};

// path.evaluate(position) -> complete model-specific State
// path.segment(position.segmentIndex).evaluate(position.lambda)
// optional: path.translationalArcLength(...)
```

## Alternatives considered

- Global translational arc length alone: cannot distinguish locations along stationary rotation/joint movement.
- Global elapsed time: forces an arbitrary nominal execution schedule onto geometry.
- Weighted joint-space generalized length as mandatory public address: requires model- and configuration-specific weights, obscuring exact primitive boundaries.

## Consequences

Path consumers must evaluate segment-aware positions, including boundary/cusp semantics. Segment derivation can preserve exact arcs, numerical integration traces and future curve-family refinements. A separate MPC handles time parameterization with dynamically feasible controls.

## Follow-up / unresolved details

Decide exact endpoint canonicalization, optional segment prefix indexing, scalar length query semantics for discontinuous derivative points and nonunique geometrical positions; update interface and property tests. The status of OQ-003 is accepted at the architectural level.
