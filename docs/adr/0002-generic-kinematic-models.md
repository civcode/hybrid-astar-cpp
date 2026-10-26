# ADR-0002: Generic kinematic models and collision frames

**Status:** Accepted  
**Date:** 2026-10-24

## Context

Initial implementation priorities are differential drive and Ackermann. Ideal mecanum is a third reference model; future targets may include articulated steering and multi-section robots. See [ADR-0008](0008-motion-model-capabilities.md).

## Decision

A single Hybrid A* search engine consumes a model-defined state/control type, controlled differential-equation/derivative and admissibility contract, capabilities and transforms of named rigid collision-body frames. Integration is a separate service, with optional exact model propagation. Primitive generation and state indexing are model-aware. Prefer compile-time concepts/templates in hot paths, allowing a thin type-erased interface at application boundaries. Collision geometry is a configuration-dependent union of frame-attached 2D shapes, not one fixed footprint.

## Alternatives considered

A universal `(x,y,theta)` state, or model-specific copies of the search engine. Both would obscure different control and configuration capabilities and cause collision/indexing errors.

## Consequences

More care in state-key/dominance semantics, generic result types, and compilation boundaries; fewer model-specific branches in search.

## Follow-up

Resolve which additional dimensions determine dominance and when steering/articulation state is part of the model (OQ-001, OQ-007, OQ-008).
