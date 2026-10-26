# W3 / W5 / W6 — hard-scenario verification and grid search efficiency

**Implemented:** 2026-10-26. **Scope:** static-map Ackermann C++20 planner,
strict sampled full-footprint success evidence, optional OMPL Reeds–Shepp
connector, optional noncertified grid heuristic and bounded shortcut refinement.
No CI or GitHub Actions are configured by this milestone.

## Deterministic scenario battery

Shared typed fixtures live in
[`bench/hard_scenarios.hpp`](../../bench/hard_scenarios.hpp).
The identical map, poses, footprint and expansion/node budgets are used in
each algorithm mode. A missing path under a budget is **not** proof that
a path does not geometrically exist.

| Fixture | Feature exercised | Certified primitive baseline | Performance-grid outcome |
| --- | --- | ---: | ---: |
| `reverse_approach` | Backward path and reverse costs | Success | Success |
| `three_point_turn` | Multiple direction changes | Success | Success |
| `parallel_parking` | Bounded slot near parked objects | Success | Success |
| `narrow_corridor` | Whole-body clearance and heading | Success | Success |
| `dead_end` | U-shaped obstacle / reverse escape (corrected feasible goal) | Success | Success |
| `obstacle_detour` | Route around long wall | BudgetExceeded | BudgetExceeded |
| `sub_vehicle_width_gap` | Impassable 0.25m opening | BudgetExceeded | BudgetExceeded |

The regression deliberately **does not** require every hard case to solve
under a fixed budget. It requires deterministic status and expansion/node
counts, valid termination status, bounded resources, no unverified success,
and preservation of map revision/source/frame, actual end pose and strict
goal tolerances for any returned successful path. A *second independent
refinement-entry strict path check* verifies each such result; optional native
CSC refinement is required never to raise the validated objective.
`impossibleByConstruction` excludes any supposed successful route through
the sub-vehicle-width gap.

An OMPL-enabled build additionally exercises the real signed RS adapter
on four selected maps under the **same** request budgets, checking successful
results independently. This is a conditional test target, not an OMPL
requirement for core-only builds.

## W6 search work and graph relaxation updates

The noncertified goal-grid provider remains **request-local** and
revision-pinned. The certified mode still uses only the Euclidean distance
to the XY goal tolerance region. No graph-only `NoPath` is returned.

1. **Bounded goal-region seeding.** Instead of scanning every cell on every
   request, check only cells whose squares could intersect the closed XY
   tolerance disk (clipped square bounding box). This produces exactly the
   same eligible goal seeds as the previous full scan, including nominal
   centers blocked while tolerance-region fringe cells are free.
2. **Diagonal corner handling.** Default-on relaxed-grid diagonal expansion
   requires both side-adjacent XY cells to be traversable. It avoids an
   artificial diagonal path through the corner of two blocked squares.
   This is **not** a footprint-aware safety check, swept-body guarantee,
   or continuous-space admissibility proof. Set
   `gridPreventDiagonalCornerCutting=false` only for controlled performance
   A/B comparisons.
3. **Bounded total grid work.** `gridMaxSettlementsPerQuery` (default 500)
   now also observes `gridMaxTotalSettlements` (default 20000) over the
   entire request. When the work budget expires, unresolved cells fall back
   to the finite continuous Euclidean (or independent RS if supplied)
   estimate; success still requires the exact path validator.
   A **zero** global grid work allowance is valid and means immediate fallback.
4. **No wasted work on blocked query cells.** Such targets immediately take
   the finite fallback. Previously they could consume many settlements.
   Settled finite cell values are reused even after the heap exhausts.
5. **Diagnostics.** `gridSeedCellsExamined`,
   `gridSeedCellsAccepted`, `gridBlockedQueryFallbacks`, and
   `gridWorkBudgetFallbacks` accompany the existing
   `gridSettlements/gridFallbacks`. These measure work and distinguish
   a bounded fallback from a negative reachability claim.

The pre-existing (x, y, yaw, incoming-gear) single-label dominance check
already prunes noncompetitive arrivals before collision evaluation.
This milestone does **not** silently change its approximation, gear-switch
penalties, or reverse weighted cost. A cross-request heuristic cache is not
introduced without an explicitly verified map/policy/goal cache key.
Settled distances are reused **within** the request only.

## Exploratory local observations

Reconstructed standalone core workspace, Release GCC, **single unpinned run**
per fixture (not a representative benchmark or performance guarantee):

| Scenario | Certified expansions | W6 grid expansions | Grid settled cells |
| --- | ---: | ---: | ---: |
| Reverse approach | 98 | 90 | 800 |
| Three-point turn | 613 | 575 | 870 |
| Parallel parking | 293 | 254 | 1,698 |
| Narrow corridor | 327 | 46 | 5,271 |
| Dead end (corrected) | 671 | 671 | Not rebenchmarked after fixture repair |
| Long obstacle detour | 8,000 | 8,000 | 7,000 (cap) |
| Impassable gap | 1,600 | 1,600 | 7,000 (cap) |

**Correction to the earlier fixture:** the old dead-end goal (31,17) intersected the closed wall. It has been replaced with a reachable reverse escape (24,17)→(16,17), which succeeds in 671 primitive expansions. See [root-cause investigation](w3-w6-failure-investigation.md). The long detour remains budget-limited under the original 7k grid cap, but a 20k grid budget and explicit noncertified 1.2 heuristic weight recover it within 8k expansions. The original table's dead-end failure is superseded.

These data show improvement **only in some maps**: corridor expansions
decreased 327→46 (~86%), but the difficult failures still exhausted
their expansion budgets under the **original** configuration, and extra grid work may increase latency.
Do not enable the noncertified heuristic globally on this evidence.

## Repeatable commands

Core-only Catch2 tests and *separately invoked* benchmark:

```bash
cmake -S . -B build-hard -DCMAKE_BUILD_TYPE=Release \
  -DHYBRID_ASTAR_BUILD_TESTS=ON -DHYBRID_ASTAR_BUILD_BENCHMARKS=ON
cmake --build build-hard --parallel
ctest --test-dir build-hard --output-on-failure
./build-hard/hybrid_astar_hard_scenario_bench 5 > hard-scenarios.csv
```

For OMPL 2.0.2 comparison, use a properly installed OMPL package (the
project's Dockerfile provisions this) and add
`-DHYBRID_ASTAR_ENABLE_OMPL=ON`. The manual benchmark then includes
`ompl_rs`, `ompl_rs_grid` and `ompl_rs_grid_refined` alongside
`primitive`, `primitive_grid`, `primitive_grid_legacy_diagonals` and
`primitive_grid_refined`, `primitive_grid_funded`,
`primitive_grid_weighted_1_2`, and `primitive_grid_weighted_1_5`.
The legacy diagonal row is an explicit
**noncertified controlled A/B mode**, not the default.

CSV provides status, success count, p50/p95/p99 latency, expansions,
generated nodes, exact final checks, grid settlement/seed/fallback counts,
validated geometric cost and length, heuristic certificate flag, work budgets,
and number of strictly evidenced successes. The benchmark is **not**
scheduled automatically and has no threshold assertions on wall-clock time.

**Local checks completed:** GCC Release reconstructed core 8/8 CTest,
Clang ASan/UBSan reconstructed core 8/8 CTest, fourteen repeated
hard-fixture/mode comparisons with independent original-path checking
(including an ASan/UBSan run), and
a blocked-center goal tolerance seeding probe (25 candidates, 19 accepted).
**Still unverified:** running the newly committed Catch2 source on an
installed Catch2 v3 package, the exact remote repository checkout, and the
full official OMPL/Docker matrix on an OMPL-equipped host.

## Next decisions

- Collect multi-seed/representative benchmark distributions and control CPU
  governor, warmups, compiler and host when setting latency targets.
- Follow up on the now-corrected dead-end fixture and recovered detour; investigate harder independently proven feasible narrow-passage scenes (single-label
  approximation, motion primitive reachability and reverse/gear penalties)
  before changing search-state dominance or gear-cost policy.
- Separate swept-path collision proof and actuator-steering-rate feasibility
  from all sampled-geometric path claims.
