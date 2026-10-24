# ADR-0014: Fixed-curvature Ackermann baseline

**Status:** Accepted  
**Date:** 2026-10-24

## Context

The first Ackermann reference model should follow conventional efficient geometric Hybrid A* practice rather than introducing steering angle as an additional search-state coordinate by default.

## Decision

1. The initial Ackermann model is a **three-state geometric bicycle** `q=(x,y,theta)` with steering/curvature supplied by primitive control rather than retained as state.
2. Use a finite, configurable set of **fixed-curvature** forward/reverse primitives: straight (`kappa=0`) and left/right arcs bounded by `|kappa|<=kappa_max`; optional intermediate curvatures may be evaluated. Primitive extent is geometric traveled length per ADR-0012. Exact circular-arc propagation is preferred.
3. Fixed curvature enforces geometric turning-radius bounds, **not steering-angle continuity, steering-rate limits or steering-acceleration limits at joins**. The raw path must preserve maneuver boundaries and record the assurance provided.
4. The optional refiner or downstream trajectory planner may create and validate finite-rate steering transitions, but success is never presumed. Any altered path must be checked against model constraints, endpoint conditions and swept collision geometry before replacing the validated raw path.
5. The generic model framework remains capable of an optional higher-fidelity Ackermann model with steering angle as state, `q=(x,y,theta,delta)`, and steering rate as control. That is a distinct model, not a hidden constraint enforced by the three-state baseline.
6. Analytic Dubins/Reeds–Shepp connections are appropriate only under their stated geometric turning-radius assumptions; they do not certify finite steering-rate transitions.

## Consequences

Retain the small 3D search key for the baseline Ackermann vehicle. Verify exact endpoint integration, reverse arcs, curvature bounds, cusp metadata, collision and post-refinement validation of steering continuity when required. Kinematic-effort transition penalties can discourage difficult curvature switches but **cannot make an infeasible transition feasible**.
