# ADR-0005: Representation-independent piecewise geometric path

**Status:** Accepted  
**Date:** 2026-10-24

## Context

The downstream MPC may use B-splines, clothoids, Hermite or other parametric curves; the planner also must preserve reversing cusps, in-place rotation, and articulated body states.

## Decision

The planner returns an **ordered piecewise, continuously evaluable geometric motion** with exact boundary conditions, motion direction/maneuver metadata and complete model configuration. Individual segments can be smooth without requiring global differentiability. The representation is not tied to a spline family. Keep exact raw primitive reconstruction available. Optional refinement can replace it only after model/collision validation.

## Alternatives considered

Expose a bare `vector<Pose2D>`, force one global B-spline, or time-parameterize geometry inside Hybrid A*.

## Consequences

Richer contract for the controller, mandatory clear parameter/progress semantics and segment-boundary handling. Ceres may be used for optional constrained refinement but not the core search.

## Follow-up

Define progress for translational vs in-place maneuvers, derivative validity at cusps and exact preservation of start/end full configurations (OQ-003, OQ-013).
