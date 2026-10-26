> **Current implementation precedence (2026-10-26):** This file preserves the baseline design history/proposal. Where defaults, collision assurance, Ackermann integration, heuristics or RS scheduling differ, follow the [current implementation contract](../architecture/current-implementation.md) and its work-package statuses. The [consolidated external reviews](../architecture/Hybrid_AStar_Consolidated_Specification.md) are proposed design guidance, **not** an already approved ADR or evidence of completed features.

# ADR-0015: Certified admissible heuristics by default

**Status:** Partly superseded for initial Ackermann performance default by [ADR-0027](0027-combined-hybrid-a-star-heuristic-and-guarded-analytic-expansion.md); certified-mode rules preserved  
**Date:** 2026-10-24

## Context

Fast reverse-grid distance estimates and vehicle-specific steering heuristics can materially accelerate Hybrid A*, but careless grid distances and obstacle inflation can overestimate continuous path cost. Search optimality claims must distinguish heuristic admissibility from approximate continuous-state merging.

## Decision

1. The **default heuristic mode** uses only lower bounds that have been certified admissible for the **configured edge cost and goal acceptance region**. Euclidean distance reduced by the goal-position tolerance is an initial safe bound if each translating edge costs at least its reference-point traveled distance and all other costs are nonnegative. Different objective units require an appropriate conversion/proof; zero-valued `h` is always an admissible fallback for nonnegative edge costs.
2. The obstacle-aware reverse-Dijkstra 2D field remains part of the architecture per ADR-0003, but **must not be used as certified lower bound** until its chosen discretization, boundary treatment, unknown-cell policy and obstacle relaxation are proven not to overestimate relevant continuous cost. A heuristic can be computed without being enabled in the strict default mode.
3. Combining lower bounds with `max(h1,h2)` is valid only if each is independently proven admissible for the same cost function and goal region. Inconsistent heuristics require compatible reopen/termination semantics.
4. Provide **explicit opt-in** to weighted search using a certified lower bound and to experimental/non-certified heuristics. Their output reports the heuristic mode, weights, search assumptions, and any lack of solution-quality guarantee.
5. **No false optimization guarantees:** one-node-per-quantized-key Hybrid A* discards potentially nonequivalent continuous arrivals (ADR-0013). Thus a certified `h` **does not establish globally optimal continuous paths**, resolution completeness, or ordinary weighted-A* suboptimality bounds without additional abstraction/proof assumptions.
6. A generated kinematic-effort catalog may have nonnegative transition-dependent edge costs. Certified lower bounds must account for those units and the actual accepted goal region. A model-dependent cost estimate is not automatically an admissible heuristic.

**Update (2026-10-25, ADR-0027):** A combined Reeds–Shepp nonholonomic estimate and 2D obstacle reverse-Dijkstra estimate is **required** for the initial Ackermann implementation. Until corrected/proven for the actual goal *region* and equivalent-meter costs, the two-component combination is an explicitly **non-certified, performance-oriented mode**. A strict certified fallback remains available. The former Euclidean-only initial path is no longer considered enough for the performance-oriented MVP.

## Consequences

Verify heuristic certification on analytical cases, tiny grid maps and goal-region boundaries. Benchmark speed/solution quality in default certified versus explicitly relaxed modes and disclose approximation/termination limitations. The first implementation may use Euclidean-to-goal-region and/or zero heuristic until the obstacle-distance lower-bound proof is established.
