# W3–W5 follow-up implementation and verification — 2026-10-26

> **Later verification update, 2026-10-26:** This is a historical record
> from before the OMPL source archive was supplied. The actual OMPL 2.0.2
> RS implementation has since been compiled and executed in the focused offline
> CMake harness, including 12,000 signed endpoint cases and sanitizer runs.
> A combined `planAndRefineWithReedsShepp()` adapter and a cumulative strict
> final-validation check budget have also been implemented. See
> [real OMPL source verification](w5-ompl-source-verification.md) for the
> **current** test evidence and remaining full-package build gate.



This is an engineering verification note, not a certification of
continuous swept-body safety or timed actuator feasibility. Read the
[current implementation contract](current-implementation.md) and the
[consolidated specification](Hybrid_AStar_Consolidated_Specification.md).

## W5: signed OMPL Reeds–Shepp integration

- New provider-neutral `SignedSegment` / `SegmentTurn` decoder in
  `steering/signed_sections.{hpp,cpp}`. Signed distances in meters encode
  forward/reverse gear independently of curvature sign. Zero-distance sections
  are removed; a nonzero no-op or nonfinite length is rejected. The decoder
  reconstructs exact `MotionSection` arcs and rejects endpoint disagreement.
- The OMPL adapter converts the library's five normalized signed path elements
  (multiplying by the configured minimum turning radius) through that same
  decoder. It compares independently reconstructed length to OMPL's reported
  metric and rejects disagreements. OMPL's published `getPath()` API is used.
- New explicit `steering::planWithReedsShepp(request, usePerformanceHeuristic)`
  wraps a synchronous planning call with ONE reusable per-request evaluator,
  safely scoped analytic-connector callback and optional exact-goal RS distance
  callback. The default heuristic **remains the certified goal-region
  Euclidean** variant. Selecting the performance flag explicitly turns on the
  noncertified RS-plus-lazy-grid ordering.
- The core planner still independently checks section curvature, real
  endpoints, gear-switch costs, search collision and exact stricter final
  whole-path validation. An invalid/cost-gated connector falls back to ordinary
  six-primitive search. OMPL remains optional and outside the core headers.

**Passed locally without OMPL:** `signed_sections_test` validates exact
forward/reverse composition, two real gear changes, zero sections, reverse
curvature, a deliberately incorrect endpoint, a nonzero no-op, a hand-computed
6.2 equivalent-meter two-cusp connector and fallback after a cost gate.

**Prepared for real OMPL, but NOT executed:** `ompl_smoke` checks nine SE(2)
endpoint configurations (including reverse and heading changes), each chain's
curvature, continuity, length and yaw, at least one real cusp, and the
`planWithReedsShepp` interface both with and without the performance flag.
The OMPL adapter received only a C++20 **syntax check against a small stub
matching published OMPL API shapes**. No real OMPL headers or runtime library
were available; the OMPL-ON build is therefore a required next acceptance gate.

## W3: planner maneuver and map fixtures

A new `scenarios_test` covers:

- Primitive-only reverse parking behind the start, with actual reverse gear.
- Primitive-only two-cusp parking, with strict sampled validation and actual
  goal-region errors.
- Immutable map snapshots with different revisions: the second published
  snapshot blocks a start while the earlier success preserves its provenance.
- Exact filled-body tangency with a closed blocked square and map exterior.
- A wall with a 0.2 m gap for a 1.8 m-wide body: it cannot produce a false
  successful geometric plan under the test's search budget.

The fixture does not prove global search completeness or validate every
parking arrangement.

## W4: final validation resource isolation

`PlanOptions::maxSamplesPerSection` is now specific to **search** sampling.
A separate `maxFinalSamplesPerSection` (default 10000) limits the
independent strict final pass. Reaching the final cap yields
`ValidationIncomplete` with no validated collision-evidence claim.
An explicit regression runs search successfully at a permissive bound, then
checks that final sample cap 3 prevents success, while restoring the 10000
default permits a complete validated path.

Also corrected bookkeeping so an actually accepted strict candidate does
not mark an unrelated strict failure in continued improvement search.

## Local verification environment and limits

Executed, **locally reconstructed core workspace** (not a real clone of the
entire remote repository):

```bash
cmake -S . -B build-w5 -DCMAKE_BUILD_TYPE=Release
cmake --build build-w5 --parallel
ctest --test-dir build-w5 --output-on-failure
# 6/6 named suites pass

cmake -S . -B build-w5-clang -DCMAKE_CXX_COMPILER=clang++ \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer" \
  -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address,undefined"
cmake --build build-w5-clang --parallel
ASAN_OPTIONS=detect_leaks=1 ctest --test-dir build-w5-clang --output-on-failure
# 6/6 named suites pass
```

Required when OMPL is installed in a real checkout:

```bash
cmake -S . -B build-ompl -DCMAKE_BUILD_TYPE=Release \
  -DHYBRID_ASTAR_ENABLE_OMPL=ON -DHYBRID_ASTAR_BUILD_TESTS=ON
cmake --build build-ompl --parallel
ctest --test-dir build-ompl --output-on-failure
```

No GitHub Actions runs were started. The first **real-OMPL** build and full
repository checkout are still unverified; any failure there must be resolved
before declaring W5 complete. Further pending gates: comprehensive random RS
fixtures on real OMPL, wall/U-turn/dead-end comparisons, swept-safety proof
or appropriately bounded risk controls, and timed actuator constraints.
