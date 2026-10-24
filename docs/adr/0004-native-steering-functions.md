# ADR-0004: Native Dubins and Reeds–Shepp

**Status:** Accepted  
**Date:** 2026-10-24

## Context

A robust Hybrid A* uses fast model-compatible steering distances and analytic goal connections without requiring a general planning framework at runtime.

## Decision

Implement **both Dubins and Reeds–Shepp natively from scratch**, with deterministic behavior and no runtime OMPL dependency. Use OMPL only as an *optional test oracle* for differential randomized and edge-case comparison. The previously discussed civcode demo repositories are references only, not vendored/copy-pasted source.

## Alternatives considered

Link OMPL directly; vendor solver code; reimplement only Dubins and wrap OMPL Reeds–Shepp.

## Consequences

More development and verification effort, but focused public types, controllable allocations and minimal runtime dependencies.

## Follow-up

Agree numerical tolerance, path-family coverage and separate distance-only benchmarks. Connector availability depends on kinematic assumptions, not name matching.
