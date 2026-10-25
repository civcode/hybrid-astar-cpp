# ADR-0023: RK4 primitive generation, geometric length termination, and stored trajectory samples

**Status:** Accepted (baseline approach; solver tolerances and concrete signatures remain implementation details)  
**Date:** 2026-10-25  
**Related:** [ADR-0011](0011-canonical-segment-path-position.md), [ADR-0012](0012-model-aware-geometric-primitive-extents.md), [ADR-0014](0014-fixed-curvature-ackermann-baseline.md), [ADR-0020](0020-evidence-driven-mvp-simplicity.md), [ADR-0022](0022-opt-in-adaptive-collision-subdivision.md)

## Context

The Hybrid A* search should not normally run a general-purpose differential-equation solver on every expansion. A kinematic model and a separate primitive generator can compute reusable trajectories ahead of search; an instantiated primitive supplies a successor and evaluable motion. Numerical models and articulated chassis benefit from common RK4 generation. The user requests endpoints at a specified traveled arc length and stored points along every generated primitive for visualization and debugging.

## Decision

1. **RK4 is the baseline numerical primitive-generation integrator.** The generator consumes a validated model, initial state, admissible control law and requested geometric extent, and produces a numerically integrated **complete model-state trajectory**. The search-facing interface consumes an already generated/instantiated evaluable primitive; it does not require a public `integrateIncrement()` operation or RK4 calls for each successor when a reusable template is valid. The model/integrator's internal tangent/state-update operations remain implementation-facing OQ-021 details.
2. **Translating primitives terminate at a specified traveled geometric arc length** `L >= 0` (meters), not at straight-line endpoint displacement. With time-domain equations augment RK4 propagation by `ds/dt = hypot(xdot,ydot) >= 0` and integrate the accumulated traveled length. Find the terminal integration parameter for `s = L` to prescribed tolerance; an iterative bracket/refinement of a final step is a simple baseline when distance cannot be converted to integration time directly. For a known constant nonzero translational speed, the corresponding time domain is known directly and need not be root-solved; a validated spatial reparameterization is also acceptable if it produces equivalent geometry. A signed direction/gear is separately recorded; absolute traveled length does not become negative for reverse travel.
3. **Stationary nontranslating primitives**—in-place differential yaw or zero-translation articulation—have `s=0` and instead terminate at a model-specific **angular/configuration extent**, per ADR-0012. No division by translational speed or infinite length-root search when `v=0`. The primitive's native extent and normalized segment progress remain distinct from later execution timing.
4. **Always generate and store ordered intermediate points for every primitive**, including exact designated start and the **same numerically computed endpoint** returned by the primitive. Each stored point includes complete model state and monotonically ordered primitive-local progress (and optionally accumulated traveled length, nominal integration parameter and control metadata). These points are useful for **visualization, debugging, diagnostics and initial collision-check schedules**, with documented generation resolution and numerical tolerances. Keep the point set immutable and reusable; do not copy its samples per search node unless necessary.
5. **The primitive remains continuously evaluable through its retained numerical motion/control-law representation**, not solely an arbitrary polyline through its saved points. `evaluate(lambda)` for `lambda in [0,1]` returns complete model state; a validated dense numerical representation, or localized re-integration from a stored checkpoint, may provide values between stored points. Do **not** claim naive straight-line interpolation of sample states exactly satisfies the kinematic ODE. Ensure `evaluate(0)` and `evaluate(1)` agree with stored start/end and generated successor to the accepted numerical tolerance. For stationary primitives, lambda parameterizes the appropriate native extent rather than distance.
6. **Cache precomputed local primitive catalogs where they are truly reusable**, notably rigid planar models whose complete kinematics are SE(2)-equivariant with matching initial internal state/model config. Instantiating such a template at a search state applies a rigid transform to its full trajectory (and properly transforms any model-specific complete state). **Articulated or state-dependent models** may need per-internal-state catalogs or on-demand generation, with cache keys capturing all initial internal state/control/config dependencies. Do not transform an invalid generic primitive template as if articulation had no effect.
7. **Default collision checking reuses stored primitive points** for the first Ackermann implementation, as approved in [ADR-0025](0025-approved-ackermann-mvp-settings-and-primitive-sample-collision.md): 0.50 m primitives store complete states at 0.10 m arc-length increments (including start/end). The checker tests the complete transformed body geometry at those exact points during search; it **does not create an independent fixed sample schedule**. Only if `adaptiveSubdivision=true` may it request extra intermediate states from the retained numerical trajectory, subject to validity and budget. Both modes remain sampled checks without continuous-collision certification.
8. **Failure handling and numerical verification matter**: detect invalid/degenerate control laws, no progress toward positive traveled length, exceeded iteration/integration budgets, numerical divergence, state/control invalidity and endpoint-length tolerance failures. Never publish an unsuccessful primitive as feasible. Test RK4 length/error convergence, reverse travel, stationary motions and consistency with analytic fixed-curvature arcs. For Ackermann, the *underlying geometric model* still produces straight/constant-curvature circles (ADR-0014); RK4-derived samples and endpoints are numerical approximations, to be checked against analytic oracles, not mislabeled mathematically exact.

## Illustrative generator product (not a frozen C++20 ABI)

```cpp
template<class State>
struct PrimitiveSample {
    double lambda;          // monotonically ordered in [0,1]
    State state;            // full model state; may be template-local
};

template<class State>
struct GeneratedPrimitive {
    State endState;         // equals state returned by evaluate(1)
    double traveledLength;  // nonnegative length for translating motions
    std::vector<PrimitiveSample<State>> samples;
    // Native extent, provenance, control law, numerical evaluation data.
    // evaluate(lambda) -> full state, not merely linear vertex interpolation.
};
```

When a primitive is cached in a local reference frame, the generating start/end/sample states and complete evaluation are transformed upon instantiation to produce world-state results.

## Deferred interface details

- Actual `PrimitiveExtent` and `ControlLaw` C++ types, length-solver bracket/refinement method and error codes (OQ-002/OQ-021). **First Ackermann numerical defaults** (0.50 m primitive, 0.05 s nominal RK4 step at 1 m/s, 0.0001 m endpoint length tolerance, 0.10 m stored sample spacing) are accepted in [ADR-0025](0025-approved-ackermann-mvp-settings-and-primitive-sample-collision.md); they do not freeze a generalized integrator ABI.
- A compact fixed sampling representation versus interpolated/dense numerical checkpoints (benchmark memory and path quality first).
- How native stationary configuration progress is normalized when several coordinates change together and exact API boundary semantics at adjacent path segments (OQ-003).
- Timing and actuator-rate feasibility after geometric trajectory generation, which remain downstream OQ-024.
