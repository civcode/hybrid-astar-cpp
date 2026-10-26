# ADR-0020: Evidence-driven simplicity for the first viable planner

**Status:** Accepted  
**Date:** 2026-10-24

## Context

The first working version of a generic Hybrid A* library will expose real integration, numerical, collision, geometry, search and performance issues not fully predictable from a paper architecture. It is counterproductive to build speculative optimization and generalized framework machinery before running a reliable vertical slice. Nevertheless, overly permissive approximations or undocumented assumptions make it difficult to diagnose the first failures.

## Decision

1. **Be thorough about the contract and lean about mechanisms.** Explicitly define kinematics, units, frames, invariants, path validity and failure conditions. Implement the smallest readable algorithm that satisfies the currently needed contract. A long list of future extensibility possibilities is not a reason to build them in the MVP.
2. **Develop and test an early end-to-end vertical slice**: validated map + one representative model + primitives + collision checking + search + reconstructable geometric path + structured diagnostics, with deterministic small fixtures. Expand model coverage and optional components progressively against the same contracts. A partially supported model is not advertised as production-ready.
3. **Prefer simple baseline mechanisms already accepted:** one retained arrival per discretized key (ADR-0013); fixed-curvature initial Ackermann (ADR-0014); certified admissible/zero heuristic fallback (ADR-0015); explicit goal region (ADR-0016); immutable full-grid map snapshots, on-demand revisioned caching and full-map strict-unknown validation (ADRs 0017–0019).
4. **Defer speculative complexity** unless a concrete use case and repeatable measurement justify it: multi-label state retention, velocity-augmented search, generalized time/energy optimization during search, incremental heuristic repair, tiled/COW map publication, fine-grained concurrent mutable caches, and sophisticated refinement strategies. Their absence must not contaminate the simple baseline with no-op branches, unused metadata or premature generic abstractions.
5. **Preserve essential correctness and observability:** state/control validity, complete robot geometry and declared swept-collision assurance, no false admissibility/completeness/optimality guarantees, bounded resource/failure statuses, and deterministic tests. When a guarantee is approximate (e.g. discretized collision sampling or merged continuous states), document and report it.
6. **Profile the integrated pipeline before optimizing a component.** Compare realistic scenario success rates, post-refinement path quality where enabled, planning latency, node counts, map/cache overhead and memory. Adopt added sophistication only if it solves a reproduced deficiency with favorable end-to-end cost and a regression test.
7. **Keep interfaces flexible only where demonstrated by reference models.** C++20 concepts and modules remain model-generic, but avoid speculative extension points, forced hierarchy, mandatory virtual dispatch, over-parameterized policies or optional backends in the first running slice. An optional feature need not be implemented just because it has been discussed or specified.
8. **Design Hybrid A* as a feasible-path initializer for downstream path optimization.** The intended pipeline has a search phase that prioritizes finding a collision-checked, kinematically valid route through the environment, followed by a distinct path-optimization/refinement phase. Avoid expensive primitive catalogs, search-stage smoothing or finely tuned curvature/control-effort optimization that would be largely redone downstream, unless evidence shows they improve the **final validated result** or essential search feasibility. Keep necessary full-state primitive trajectories for collision validation, reconstruction and debugging, not extra intermediate search nodes.
9. **Optimization cannot substitute for valid search or restore discarded routes.** Maintain search-level collision/model checks and appropriate topology/maneuver choices; downstream optimization may not recover another homotopy class or reverse maneuver that the search never found. An optimized replacement must pass separate kinematic, geometry/map and boundary validation; on failure retain the original validated geometric path. During early implementation, the separate optimizer may be added after a working planner slice rather than blocking it.

## Explicit non-decisions

- This ADR does **not** weaken existing accepted map, geometry, kinematics, goal or validation contracts and does **not** change previously accepted ADRs.
- The precise minimum **product release gate**, reference robot(s), collision assurance level and benchmarks are still to be selected during implementation planning; this ADR establishes the decision rule, not a fictitious performance target.
- Path refinement, articulated vehicle support, model-specific kinematic effort catalogs and richer search heuristics can be valuable **follow-up increments**, not mandatory prerequisites to obtaining a first end-to-end working planner.
- All documented modules remain candidate architecture; an accepted architectural direction does not mean code exists or that every optional feature must ship in the earliest viable build.

## Verification consequences

- Preserve a tiny fixture suite that runs the entire pipeline as soon as the first model is viable.
- For each nontrivial search optimization or abstraction, document the motivating failure/benchmark and measured **final post-optimization** feasibility/quality and total planning+refinement latency, in addition to search-only time, API costs and rollback path. Do not optimize raw path smoothness as a proxy for end-to-end quality.
- Add regression tests for every bug found during integration and retain tests that demonstrate known baseline limitations.
- Treat debugging clarity, stable output semantics and failure diagnosis as MVP quality requirements, alongside runtime and correctness.
