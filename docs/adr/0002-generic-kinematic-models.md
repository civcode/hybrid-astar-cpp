# ADR-0002: Generic kinematic models and collision frames

**Status:** Accepted  
**Date:** 2026-10-24

## Context

Initial robot types are differential drive and Ackermann; future targets may include articulated steering and multi-section robots.

## Decision

A single Hybrid A* search engine consumes a model-defined state/control type, propagation contract, capabilities and transforms of named rigid collision-body frames. Primitive generation and state indexing are model-aware. Prefer compile-time concepts/templates in hot paths, allowing a thin type-erased interface at application boundaries. Collision geometry is a configuration-dependent union of frame-attached 2D shapes, not one fixed footprint.

## Alternatives considered

A universal `(x,y,theta)` state, or model-specific copies of the search engine. Both would obscure different control and configuration capabilities and cause collision/indexing errors.

## Consequences

More care in state-key/dominance semantics, generic result types, and compilation boundaries; fewer model-specific branches in search.

## Follow-up

Resolve which additional dimensions determine dominance and when steering/articulation state is part of the model (OQ-001, OQ-007, OQ-008).
