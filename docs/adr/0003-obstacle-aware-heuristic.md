# ADR-0003: Static obstacle-aware heuristic

**Status:** Accepted  
**Date:** 2026-10-24

## Context

Pure Euclidean or nonholonomic distance heuristics do not account for detours around walls and narrow passages.

## Decision

Use a reverse shortest-distance (Dijkstra) field on a 2D **static** occupancy map as a core search heuristic, with lazy settlement, optional caching and immutable versioned map snapshots. Combine with model-specific bounds when justified by actual cost units. Search is not restricted to the 2D route corridor by default.

## Alternatives considered

Euclidean-only; mandatory A* reference path/corridor; continuously reconstructing the distance field from moving obstacles.

## Consequences

Fast per-state lookup, reusable across headings and models; preprocessing/cache memory and invalidation obligations.

## Follow-up

A grid-based Dijkstra value is not automatically an admissible lower bound to a continuous geometric path; settle graph, inflation and cost scaling before asserting optimality (OQ-004, OQ-005).
