# ADR-0016: Pose-tolerant default goal conditions

**Status:** Accepted  
**Date:** 2026-10-24

## Context

A generic model may have steering, articulation and other configuration variables, while a common navigation request specifies only a target world-frame pose. Goal acceptance must be explicit and must use continuous state instead of a quantized grid index.

## Decision

1. The default goal is a **pose region** in the planning `map` frame, evaluated on the model-defined continuous reference pose `(x,y,theta)`. The candidate must simultaneously satisfy:
   - Euclidean planar distance to target `<= 0.25 m` by default.
   - Absolute wrapped yaw difference `<= 10 degrees` (`pi/18 rad`) by default.
2. Both tolerances are **independently configurable**, including exact-zero tolerance when appropriate. Parameters are expressed in meters and radians. These are library defaults, not guarantees suitable for every robot/environment.
3. For state variables not specified by the request, such as articulated joint angle `gamma`, the default goal imposes **no equality target**, but full state validity, collision and incoming swept-motion constraints still apply. A caller can optionally add model-specific full-state/end-mode constraints and tolerances.
4. A candidate is successful only if its **continuous** endpoint passes all active goal predicates and the full accepted motion has passed its declared validation. Do not accept merely because the discrete key reaches a nominal goal bin.
5. Do not silently move/snap the output endpoint to the specified goal center. Return the actual reached state and report the accepted tolerance.
6. For an admissible goal heuristic, estimate cost to the **entire allowed goal region**. For length-dominated nonnegative costs the Euclidean position bound is `max(0, distance_to_target - xyTolerance)`. A colliding *nominal goal center* does not by itself prove the entire tolerance region invalid; validation must assess whether a valid endpoint exists in the requested region.
7. Distinguish pose-only target validity from optional model-specific joint, steering, mode and exact-pose terminal requirements.

## Consequences

Provide tests for independent position/yaw thresholds, yaw wraparound, exact and loose goals, optional articulation target, invalid endpoints and sweeps, and correct use of the continuous state. Start and goal frame identities must match the map frame or be explicitly transformed before search.
