# W3/W6 — diagnosis of dead-end and long-wall detour failures

**Investigated:** 2026-10-26. This is a case-specific diagnosis,
not evidence of planner completeness, global optimality or actuator safety.
All numbers below are local C++ Release exploratory runs; elapsed time is
host-dependent, while statuses, work limits, search expansions and weighted
geometric costs are deterministic for the same build.

## 1. Dead-end: invalid benchmark goal, not a search failure

The original `dead_end` fixture had the goal rear axle at **(31, 17, yaw=0)**.
The closed back wall filled **x in [29, 31]** (and y in [11, 24]).
The nominal goal and its 4.6 × 1.8 m vehicle footprint overlap the
occupied back wall. `ExactCollisionChecker::check(goal)` reported
`Colliding` before any search: raising the expansion cap to 30,000 and
halving XY bins never produced a valid solution.

The corrected fixture starts **(24, 17, yaw=0)**, with its nose facing the
closed wall, and ends **(16, 17, yaw=0)**, safely outside the closed end.
The wall remains unchanged. It is now a constrained **reverse escape**,
not a request to drive into a wall.

- Primitive-only certified planner: **Success, 671 expansions**, 8.0 m of
  reverse motion, **8.8 m weighted cost**.
- Real OMPL 2.0.2 source-subset Reeds–Shepp adapter: direct analytic
  connection accepted at the first expansion (strictly sampled).
- Every fixture now has **explicit start and nominal-goal full-footprint
  clearance assertions** to prevent this exact benchmark mistake recurring.

This fix does not establish that *every* pose inside a cul-de-sac is
kinematically reachable.

## 2. Long-wall detour: heuristic work budget was insufficient

The unchanged `obstacle_detour` fixture has start (12,15), goal (34,15),
and a wall at x=[23,23.5], y=[5,24]. Its start and goal are collision-free.
A valid route must navigate around one wall end rather than crossing it.

The original comparison used **8,000 search expansions and only 7,000
grid settlements**. With this grid budget, the grid provider had
exhausted its work before computing useful estimates on the searched side.
Every queried node fell back to Euclidean ordering in that run.
The baseline and performance modes both reached `BudgetExceeded`, even at
30,000 search expansions under the unchanged 7,000 grid budget.

| Configuration (primitive-only) | Grid settlement cap | Search expansions | Result | Weighted cost |
| --- | ---: | ---: | --- | ---: |
| Certified Euclidean, 8k search cap | 0 | 8,000 | BudgetExceeded | — |
| Performance grid, original test cap | 7,000 | 8,000 | BudgetExceeded | — |
| Performance grid, unit weight, 30k search cap | 20,000 | 18,172 | Success | 31.50 m |
| Performance grid, unit weight, 30k search cap | 30,000 | 16,158 | Success | 31.50 m |
| Performance grid, **weight 1.2**, original 8k search cap | 20,000 | **3,530** | **Success** | 34.05 m |
| Performance grid, **weight 1.5**, original 8k search cap | 20,000 | **1,751** | **Success** | 36.05 m |

The 1.2-weighted result is about 2.55 m more expensive than the
unit-weight 31.50 m result (~8.1%). A 1.5 weight trades still more
first-solution quality for fewer expansions. Both remain opt-in.
**None** is a certified bound on solution quality: the grid heuristic is
already noncertified and search uses quantized single-label dominance.

Under an OMPL-enabled source-subset runtime probe, the detour also
succeeded with weight 1.5 and 20,000 grid settlements in **1,137
expansions** (33.1952 m weighted), with an analytic connection accepted
after the search moved to a viable wall-avoiding state. Simply attaching
OMPL without adequate grid work still hit the 8,000-expansion cap.

## 3. Targeted alternative-explanation probes

On the original 7,000-settlement grid budget, I ran **18** detour variants:
XY bins 0.125/0.25/0.5 m × primitive lengths 0.25/0.5/1.0 m × gear
switch cost 0.0/0.5 m. All 18 still hit the 8,000 expansion cap.
Changing XY resolution to 0.125 m or raising the search cap to 30,000
without funding the heuristic also failed.

The one-label search did prune many successors, but these A/B trials
**do not establish whether that approximation prevents some other valid
routes**. They do show it was not necessary to replace search-state
dominance or alter reverse-cost semantics to recover this particular detour.
The U-shaped fixture's prior issue was unequivocally target geometry.

## 4. Narrow, opt-in implementation

`PlanOptions::performanceHeuristicWeight` defaults to **1.0**, with
validated range **[1,3]**. It multiplies the performance-mode search
heuristic *only*. The certified Euclidean mode ignores the setting;
it remains unchanged and admissible according to its existing contract.
The multiplier has an overflow guard for large finite geometric inputs.

A new W6 Catch2 regression ensures a sufficiently funded 1.2-weighted
detour succeeds within the **original 8,000-expansion cap**, repeats
deterministically and passes independent full-path strict revalidation.
It checks that a unit-weight run can also succeed with more search work,
that certified mode is unaffected, and that invalid weights are rejected.

Manual `hybrid_astar_hard_scenario_bench` rows now include
`primitive_grid_funded`, `primitive_grid_weighted_1_2`,
`primitive_grid_weighted_1_5` and
`ompl_rs_grid_weighted_1_2` when OMPL is available.
These rows report both weight and grid work budget explicitly.

## Verification and remaining decisions

Completed locally: reconstructed GCC Release and Clang ASan/UBSan core
CTest suites (8/8 each), corrected dead-end recovery, recovered-detour
strict validation, certified-mode invariance with a nonunit weight,
plus direct execution against the real OMPL 2.0.2 source-subset adapter.

Still pending on a provisioned host: **the new Catch2 fixture tests,
exact GitHub checkout, and official OMPL/Docker CMake package matrix**.
The prior work did not run GitHub Actions; neither does this increment.

The next genuine search investigation should use independently
constructed *feasible* narrow-passage fixtures, measure where single-label
dominance eliminates useful distinct continuous arrivals, and evaluate
alternative pruning with controlled memory/time budgets and a separate
strict-feasibility oracle. No global admissibility, cost-optimality,
continuous swept collision or actuator-feasibility claim is made.
