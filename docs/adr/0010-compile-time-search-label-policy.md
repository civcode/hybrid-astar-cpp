# ADR-0010: Optional multiple labels with a zero-overhead single-label specialization

**Status:** Accepted (policy architecture; retention details still proposed)  
**Date:** 2026-10-24

## Context

Continuous states quantized to one search key can differ in control feasibility, arrival direction, articulation state, or future reachability. Keeping several labels can improve robustness, but the common single-label configuration must not pay multi-label bookkeeping, allocation, per-key memory or conditional-dispatch costs.

## Decision

1. The search engine is parameterized by a **compile-time label-storage/retention policy**, defaulting to `SingleLabelPolicy`. An explicit `MultiLabelPolicy` enables several continuous search labels for one discrete key.
2. The single-label specialization uses a **single node/label ID per key**; do not require a per-key vector, chain head, multi-label container or runtime `if (multilabel)` branch. Do not add fields to baseline search nodes solely for multi-label retention.
3. A multi-label specialization may store several IDs per key. It must expose explicit, configurable retention/pruning (e.g., maximum labels, admissible arrival-state distinctions, quality thresholds), accounting for memory and processing budget.
4. The **state indexer** (which generates quantized keys) and **label policy** (which retains continuous candidates for that key) are separate contracts. Indexing stays model-specific and may include steering, articulation and other relevant dimensions.
5. A planner facade may choose between separately instantiated policy types when a runtime configuration option is desired; the inner search loop remains statically specialized.
6. No optimality, completeness or mathematical dominance guarantee is inferred from cost-only comparison of two continuously different states. Document the approximation and retention conditions separately.
7. Establish a benchmark for the single-label variant against a baseline implementation to verify the no-extra-bookkeeping objective. Template code size/build time may differ; the requirement concerns steady-state search compute, node/key storage and allocations.

Illustrative API sketch (not final):

```cpp
template<class Model, class LabelPolicy = SingleLabelPolicy>
class HybridAStar;

using CompactSearch = HybridAStar<MyModel, SingleLabelPolicy>;
using MultiArrivalSearch = HybridAStar<MyModel, MultiLabelPolicy>;
```

## Alternatives considered

- A runtime `bool enableMultipleLabels` branch in every expansion: risks overhead in the off mode.
- Always maintaining a vector of labels per key: unnecessary allocations and memory for single-label operation.
- One fixed multi-label retention rule: cannot address model-dependent state/arrival equivalence or budget tradeoffs.

## Consequences

Search internals must compile with distinct policy-specific key-value storage and update paths. Parent/edge reconstruction uses stable node indices, including for discarded or stale labels. The public planner may expose a simple configuration choice but dispatch only once per request/facade instance.

## Follow-up / unresolved details

Specify retention criteria, eviction policy, re-open/termination semantics, suitable per-model arrival-distance metrics, and default multi-label budgets. Test that single-label layout/allocations match a dedicated one-label design. See [open questions](../architecture/open-questions.md), OQ-001.
