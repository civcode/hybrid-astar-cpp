> **Current implementation precedence (2026-10-26):** This file preserves the baseline design history/proposal. Where defaults, collision assurance, Ackermann integration, heuristics or RS scheduling differ, follow the [current implementation contract](../architecture/current-implementation.md) and its work-package statuses. The [consolidated external reviews](../architecture/Hybrid_AStar_Consolidated_Specification.md) are proposed design guidance, **not** an already approved ADR or evidence of completed features.

# ADR-0013: Single-label discretized Hybrid A* as the baseline

**Status:** Accepted  
**Date:** 2026-10-24  
**Supersedes:** [ADR-0010](0010-compile-time-search-label-policy.md)

## Context

The earlier ADR-0010 accepted compile-time optional multi-label retention. After reviewing conventional Hybrid A* practice and the proposed downstream path-refinement architecture, we prioritize a compact, understandable search kernel. Extra arrival labels may have marginal end-to-end value after refinement and can multiply search memory and computational cost.

## Decision

1. The baseline search retains **one continuous node/arrival per model-defined discretized key**. The key maps to one retained node identifier; no multi-label policy, containers, caps, or eviction logic are required in the initial implementation.
2. Each retained active label preserves its full **continuous model state**, accumulated cost, parent/action, and incoming motion primitive/maneuver metadata. The index has **one currently retained active label reference per key**; historical node records referenced as parents may remain in an append-only pool to preserve correct reconstruction. Do not mutate ancestor state/action records in place when replacing a key's active label. Cost replacement follows the documented lower-`g` rule and appropriate heap stale-entry/reopening behavior.
3. The discrete key is **model-specific**. For the **initial three-state Ackermann implementation**, [ADR-0025](0025-approved-ackermann-mvp-settings-and-primitive-sample-collision.md) specifies `(x-bin,y-bin,yaw-bin,incoming-forward/reverse-mode)` with **0.25 m** XY and **5-degree** yaw bins; the physical ODE state remains `(x,y,yaw)`. Models with articulation or other reachability/collision-relevant configuration variables must index the necessary dimensions. The key is not universally a 3D pose.
4. Incoming primitive/maneuver metadata is kept **on the node**, not generally added to the key. **Approved Ackermann exception (ADR-0025): incoming forward/reverse mode *is* part of the key** because the 0.50 m switch penalty depends on the previous gear. Primitive ID and previous curvature are not part of that key. Remaining history-dependent costs and discretized continuous state still make lower-`g` pruning per key an **approximation**, not proof of true dominance or global optimality.
5. Downstream refinement may improve geometric quality and maneuver smoothness but cannot generally recover a missed homotopy class, unretained arrival, or an intrinsically infeasible discrete transition. Preserve and validate the raw path; report refinement failures.
6. **Do not implement or design public API complexity for optional multi-label retention in the initial milestones.** Revisit only if reproducible scenario benchmarks demonstrate a substantial failure-rate or end-to-end quality gap. Any future extension needs a separate ADR and explicit performance evidence.

## Alternatives considered

- Compile-time optional multi-label policy (ADR-0010): may retain useful arrivals but adds design/testing surface and potentially substantial runtime costs when selected.
- Augment every key with incoming primitive ID: restores more of the history needed for transition-aware costs but increases state count, which is not warranted for the baseline. **Only the coarse incoming forward/reverse mode** is included for the approved Ackermann MVP (ADR-0025).
- Exact dominance across continuous arrivals: not generally available from a shared quantized cell alone.

## Consequences and verification

Benchmark the baseline planner **together with** its proposed optional refiner, as well as planning memory, latency, successful solution rate, path quality, and whether narrow-passage or arrival-direction alternatives are lost. Test key replacement, stale heap records, reopening, model-dependent key dimensions and deterministic tie-breaking. Do not claim global continuous-space optimality or a weighted-search suboptimality factor solely from a certified admissible heuristic.

## Follow-up

OQ-001 is closed for the baseline. The impact of pruning on successful search and refinement outcomes remains a verification obligation, not a reason to introduce multi-label machinery without evidence.
