# W8 bounded geometric shortcut refiner — 2026-10-26

**Status:** implemented opt-in C++20 core prototype; no continuous swept
certificate or actuator feasibility guarantee. Read the
[current implementation contract](current-implementation.md) for defaults
and the [consolidated synthesis](Hybrid_AStar_Consolidated_Specification.md)
for W8 acceptance criteria and alternatives.

## Invocation

```cpp
#include "hybrid_astar/ackermann/refinement.hpp"

auto raw = hybrid_astar::ackermann::plan(request);
if (raw.status == hybrid_astar::ackermann::PlanStatus::Success) {
    auto refined = hybrid_astar::ackermann::refinePath(request, raw);
    // refined.path is populated ONLY after independent strict revalidation
    // of the original, regardless of whether a shortcut was accepted.
}
```

Default proposals are **straight same-heading section-boundary chords**
whose exact steering curvature is zero and whose gear matches signed
longitudinal displacement. They are useful to remove loops, reversals and
unnecessary straight detours but do not smooth arbitrary curved paths.
A `RefinementConnector` callback may instead generate multiple exact
forward/reverse signed `MotionSection` arcs from the proposed window
start to end, e.g. from an application-owned OMPL Reeds–Shepp evaluator.
No OMPL dependency is introduced into core. Calls are synchronous and
connector lifetimes remain the caller's responsibility.

This is deliberately **separate from** `plan()`: default planning
performance and its already validated output are unchanged unless the
application explicitly invokes refinement.

## Acceptance/safety contract

1. Reject invalid request, stale source revision/frame/source/occupancy, or
   a source not marked successfully sampled-validated. Recheck the entire
   original against the request's exact filled-body SAT checker with
   **strict final** step/body-point limits, even if caller provided a forged
   `PlanResult` with cheap cost.
2. Every section must have finite pose, direction, curvature bounded by
   `1/Rmin`, strictly positive progress and exact evaluable endpoint
   continuity (1e-5 position/yaw guard). Normalize the chain via
   `makeSection()` and validate the actual terminal XY/yaw goal tolerance.
3. For every candidate, reconstruct the **whole path** (unchanged prefix,
   proposed replacement, unchanged suffix) and strictly check **all**
   sections with the exact collision checker. A proposed shortcut is never
   accepted merely because its anchor poses are clear. The EDT prefilter
   is **never consulted** here.
4. Independently recompute the configured weighted cost from true arcs,
   incoming reverse gear and all gear switches. When clearance weighting
   is configured, re-integrate the same exact-oriented rectangular-body
   clearance objective used in search. Accept a replacement only when the
   recomputed cost strictly improves beyond the configured positive margin.
5. Count-bounded attempts (default 64), accepted replacements (default 4),
   candidate section cardinality (default 32), and total **exact pose checks
   including original revalidation** (default 100,000) enforce finite
   in-library work. Unbounded time inside a user-supplied callback is NOT
   controlled by these counters.
6. On rejected proposals, post-verification cancellation or work limit, the
   last **strictly revalidated** path is retained. If even the original
   cannot be verified, return `OriginalNotVerified` and no path; do not
   advertise a success or reuse collision evidence on an unverified result.
   `RefinementStatus` states the separate postprocessing outcome.
   Accepted paths set `SolutionQuality::FeasibleAfterRefinement`.
7. Geometry is adaptively **sampled** along true curved sections, not a
   continuously swept-body certificate. `trajectoryActuatorFeasible`
   remains false. There are no steering-rate/curvature continuity claims
   at section joints and no dynamic obstacle or temporal safety check.

## Local test and acceptance evidence

`tests/refinement_test.cpp` covers:

- Shortening a forward–reverse–forward route while removing cusp metadata
  and correctly computing the improved 2.0 equivalent-meter cost.
- A provider-supplied valid shortcut, using the same path acceptance gate.
- Rejecting a malformed original and a mismatched map revision without
  returning a path. Insufficient budget for original validation also cannot
  yield an accepted path.
- Falling back from an endpoint-inconsistent provider proposal and retaining
  a validated path when cancellation occurs in the callback.
- A 4-quarter-arc obstacle detour with matching start/end heading where a
  direct straight shortcut crosses a blocked cell: **reject the shorter
  colliding chord** despite lower nominal cost.
- Attempt-budget exhaustion retaining the strictly revalidated obstacle
  detour, even without any accepted shortcut.
- Nonnegative clearance-weight integration with the correct weighted
  objective and no actuator guarantee.

Local reconstructed core workspace (NOT a full remote repository checkout):

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DHYBRID_ASTAR_ENABLE_OMPL=OFF
cmake --build build --parallel
ctest --test-dir build --output-on-failure

# Separate sanitizer run with clang++:
cmake -S . -B build-asan -DCMAKE_CXX_COMPILER=clang++ \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer" \
  -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address,undefined"
cmake --build build-asan --parallel
ASAN_OPTIONS=detect_leaks=1 ctest --test-dir build-asan --output-on-failure
```

**Observed:** seven out of seven named local core tests pass under GCC 14.2
Release and Clang 17 ASan/UBSan; no new sanitizer issues detected. Tests on
the **exact GitHub checkout** and the real OMPL-enabled adapter remain
outstanding because this environment cannot resolve GitHub from git and has
no OMPL development package installed. No GitHub Actions execution occurred.

## Limitations and next gates

- This is a discrete section-boundary shortcut refiner, **not** a general
  obstacle-aware continuous-curvature numerical smoother, elastic-band
  optimizer, or steering actuator trajectory optimizer.
- Sampling is adaptive but does not certify the complete swept vehicle.
- No optimization or latency improvement is claimed for general maps based
  on one purpose-built regression.
- Independent real-OMPL tests, representative parking/dead-end map
  benchmarks, combined search+refinement p50/p95/p99 latency, callback
  runtime/memory limits and a separately designed bounded strict-validator
  retry policy remain future release gates.
