# ADR-0006: Separate dynamic obstacle trajectory planning

**Status:** Accepted  
**Date:** 2026-10-24

## Context

An occupancy-grid snapshot of moving obstacles lacks predicted obstacle position at robot arrival time. The intended motion controller will use MPC.

## Decision

Hybrid A* uses a **static map snapshot** to compute a model-feasible geometric path and reusable obstacle heuristic. A **separate trajectory planner/MPC** uses time-dependent obstacle predictions, vehicle speed/dynamics, and safety constraints. Persistent blockage may request a new spatial plan. Maintain consistent vehicle geometry and kinematic semantics across modules, but do not couple implementations.

## Alternatives considered

Blend dynamic obstacles into every static heuristic update; add time as a mandatory Hybrid A* state dimension.

## Consequences

Reusable static heuristic and focused planning scope; clear interface needed for replanning signals and segment-level controller references.

## Follow-up

Keep any dynamic map overlays out of baseline heuristic caching unless modeled explicitly. Specify data provenance and blocked-route handling during later integration.
