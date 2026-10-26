# W11 — model-policy refactor of Hybrid A* search

**Implementation branch:** `docs/architecture-baseline` · 2026-10-26.

The native Hybrid A* planner now uses **one typed policy-driven
search kernel** instead of branching on Ackermann or differential-drive
kinematics inside successor expansion. The existing
`ackermann::plan(request)` entry point, result/JSON contracts, and enum
selection remain source-compatible.

## Public planar model contract

The new [model-policy API](../../include/hybrid_astar/planning/model_policy.hpp)
provides the C++20 `planning::PlanarSearchModel` concept. A model specifies:

| Method / type | Responsibility |
| --- | --- |
| `State`, `Key`, `Hash`, `Mode`, `Section`, `Primitive` | Search state, discretization, label-hash, mode and exact-motion types |
| `validate()` and `footprint()` | Validate model-specific options and provide its actual rigid collision geometry |
| `catalog(SampleLimits)` | Precompute reusable local-frame primitives **once per request** |
| `key(State, Mode)` and `keyHash(diagnostics)` | Model-dependent state indexing and hash instrumentation |
| `materialize(State, Primitive)` | Transform a local primitive into an evaluable world-frame section |
| `nextMode(previous, Primitive)` | Update incoming drive mode (retain through stationary spin) |
| `edgeCost(previous, Primitive, Section)` | Positive, finite model-specific effort and switching costs |

`planning::AckermannPolicy` and
`planning::DifferentialDrivePolicy` implement the same concept and
are chosen once at the start of `ackermann::plan()`. That means
`planImpl<Policy>` is compiled with the proper policy; the hot
successor loop contains no `if(model==...)` branch, no direct call
to `buildPrimitiveCatalog`/`differential_drive::buildCatalog`,
and no hardcoded spin-versus-arc edge-cost switch.

The policy also supplies its collision footprint, search-key type and hash, while the kernel
owns queue scheduling, one-label dominance/reopening, strict end
validation, cancellation/work budgets, path reconstruction, and
result provenance. Node storage uses a typed
`planning::SearchNode<State,Key,Mode,Section>`.

## Extending the search without duplicating it

The public `planning::PlanarPolicy` runtime interface and the
`planning::PlanarPolicyAdapter<MyPolicy>` wrapper let users add
another compatible planar rigid-body motion model and call:

```cpp
#include "hybrid_astar/planning/model_policy.hpp"

// CustomPolicy implements the PlanarSearchModel concept and must expose
// rigid-SE2 state, key, section and mode types for the current adapter.
hybrid_astar::planning::PlanarPolicyAdapter<CustomPolicy> model{
    CustomPolicy{request}};
const auto result=hybrid_astar::ackermann::planWithPolicy(request,model);
```

Unlike built-in model selection, this does not require adding a value
to `PlanOptions::Model` or another branch to the search loop.
`model` and any request it references must remain alive through the
synchronous call.

**Demonstrated in Catch2:** `PivotOnlyPolicy` creates two cached
local angular primitives, collapses the irrelevant drive-gear key,
assigns positive spin costs, and searches for a strictly validated 90°
turn through the same engine. It is a third *search policy* rather than
a third complete commercial vehicle model.

## Behavioral contracts that must not regress

- Ackermann remains the default, preserving the six-template search
  order, curvature/gear switch cost, optional Reeds–Shepp connector and
  independently checked refined results.
- Differential drive preserves eight local templates, zero-translation
  rotations with positive angular effort, incoming mode preservation,
  footprint checking at intermediate headings, and rejection of
  incompatible Ackermann analytic/clearance options.
- Map snapshot, frame/source/revision and occupancy policy are request
  pinned. Success requires independently re-evaluated strict sampled
  full-body collision checks; a budget failure cannot masquerade as
  `NoPath`.
- Priority queue order, first/incumbent policy, diagnostics and
  deduplicating search-bin semantics are not changed by this extraction.
- The `SearchNode` representation is typed; model-specific indexing
  is no longer selected through `if` inside the expansion loop.

## Important capability boundary

**This is a model-parametric *planar rigid SE(2)* kernel, not yet an
arbitrary-state articulated/mecanum search.** The public runtime
adapter currently constrains State to `Pose2D`, Key to the planar
quantized key, Section to `MotionSection`, and collision geometry to
the complete rigid rectangle defined by `Footprint`. The generic
template concept and typed search-node storage prepare the interface
for non-planar states, but the goal/heuristic, exact checker, analytic
path contract and output representation still assume three planar
coordinates and a rigid body. True articulated chains require a
model-provided multibody transform/collision contract and dimensional
state keys; ideal mecanum requires lateral-motion primitives and
a matching exact section representation. Neither is claimed here.

## Test/verification plan

New `w11_model_policy_test` Catch2 target:

1. Built-in Ackermann and its adapter-driven invocation must agree on
   status, expanded/generated counts, strict collision evidence,
   path section endpoints, cost and map revision/source/frame.
2. The same equivalence must hold for differential-drive pure rotation.
3. Third `PivotOnlyPolicy` must produce strictly checked
   zero-translation positive-cost rotation, repeat deterministically,
   and preserve the immutable map revision.
4. A deliberately large Ackermann request footprint must **not** override
   a smaller third-party model's declared collision footprint; actual
   independent body checking must use the model geometry.
5. Zero-cost custom transitions, incompatible legacy analytic/clearance
   hooks and a deliberately **forged section endpoint** must be rejected;
   accepted paths must pass exact evaluator-based join and terminal checks.
6. Unsupported differential wheel/rotation configurations must report
   `InvalidRequest` for both policy-based and standard entry points.

Existing W3/W6/W10 scenario, priority-queue, HTTP and OMPL/Catch2
regressions remain registered and unchanged. On a host with Catch2 v3:

```bash
cmake -S . -B build-w11 -DCMAKE_BUILD_TYPE=Release \
  -DHYBRID_ASTAR_BUILD_TESTS=ON -DHYBRID_ASTAR_BUILD_APP=ON
cmake --build build-w11 --parallel
ctest --test-dir build-w11 --output-on-failure
```

**Execution limitation:** The current environment can update and inspect
the remote GitHub repository but cannot clone/fetch the full checkout or
run its installed Catch2/Docker matrix. Newly added W11 C++ tests are
committed but not represented as passed. The exact-checkout build and
sanitizer/OMPL matrix remain release gates.

No PR, merge, or GitHub Actions workflow change is part of W11.
