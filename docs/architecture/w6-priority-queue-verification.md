# W6 — deterministic priority-queue profiling and tuning

**Date:** 2026-10-26. **Scope:** Ackermann Hybrid A* search and its optional
request-local reverse-grid Dijkstra frontier. This is a queue performance
experiment, **not** a new search heuristic, path optimality proof, swept
collision guarantee or actuator-feasible trajectory.

## Implementation

The original `std::priority_queue`-based binary heap remains the **default**.
`PlanOptions::openQueueMode` and `gridQueueMode` independently select:

| Mode | Implementation | Label replacement |
| --- | --- | --- |
| `BinaryLazy` | Original `std::priority_queue`, vector storage | Stale entries discarded when popped |
| `FourAryLazy` | Contiguous four-child heap | Stale entries discarded when popped |
| `IndexedBinary` | Two-child heap with node-id-to-position index | Replace a still-open prior label in place |
| `IndexedFourAry` | Four-child heap with the same index | Replace a still-open prior label in place |

All modes use identical ascending **(f, h, node ID)** deterministic
tie-breaking. The indexed planner queue is given the superseded node ID
**before** overwriting the map entry for its active search label; already
expanded labels have no active queue position and are inserted afresh.
The grid's node IDs are its raster cell indices, so an improved unsettled
cell is updated in place in indexed mode.

Every expansion copies just the immutable snapshot of
`pose/key/gear/g/h/clearanceCost`, instead of copying the entire search
`Node` (including its `MotionSection`). A reference into the growing
node vector would not be safe across reallocation.

New `PlanDiagnostics` counts distinguish planner open and grid:
`openQueuePushes/Pops/Replacements/PeakSize/PeakStorageBytes`;
`gridQueuePushes/Pops/StalePops/PeakSize/PeakStorageBytes`; and the
pre-existing `staleHeapPops`. `PeakStorageBytes` counts the vector
*capacities* of the heap and optional index (not allocator overhead, map,
node storage, or process RSS). Indexed grid mode may allocate a large
sparse node-to-position array even if the live frontier is small.

## Local correctness and sanitizer checks

- Existing standalone reconstructed-core CTest: **8/8 GCC Release** and
  **8/8 Clang ASan/UBSan** pass.
- Deterministic randomized 6,000-operation queue harness exercises all
  four modes with ties, replacing an open label, and re-enqueueing after a
  prior pop; index modes execute thousands of in-place replacements.
- All **seven hard planning fixtures** return the **same status, expansion
  counts, generated counts, geometric cost and collision evidence** when
  either queue is switched independently among all four modes.
- An additional Clang ASan/UBSan run of that harness passed.
- A new Catch2 `tests/priority_queue_test.cpp` is committed and registered
  for the official test build. **The installed Catch2 v3/Docker matrix and
  exact GitHub checkout build have not run in this environment.**

Queue choice affects storage and CPU overhead, not the number of expanded
nodes or the strict path acceptance contract.

## Exploratory end-to-end timing

Seven repeats (plus warmup) per variant, GCC Release, one shared
local execution host, same map/pose/primitive/collision budgets, weighted
noncertified grid with weight 1.2 and total grid cap 20,000. Values in
milliseconds are **exploratory and not hardware-independent guarantees**.
A second machine, build or reversed variant order can change the ranking.

| Scenario | Binary baseline p50 | 4-ary main p50 | Indexed binary main p50 |
| --- | ---: | ---: | ---: |
| Narrow corridor | 1.276 | 1.337 | 1.380 |
| Long obstacle detour | 36.964 | 37.547 | 33.337 |
| Impassable gap | 24.498 | 18.822 | 22.699 |

**Detour queue counts:** original main heap 3,532 pops of which **2**
were stale (0.06%); indexed main removed those 2 and reduced peak
entry count from **4,860 to 4,730**, but increased peak heap-plus-index
storage from **192 KiB to 320 KiB**. Indexed main was faster on this
particular local run but not consistently faster across all fixtures.

The original detour *reverse-grid* queue performed 20,746 pops, of
which **746 were stale** (3.6%), using about **12 KiB** of queue storage.
Indexed grid eliminated stale pops (20,000 pops) but used about **346 KiB**
of heap-plus-index storage and had a slower detour median (39.755 ms
versus the baseline 36.964 ms). On the narrow corridor, indexed grid
used ~202 KiB versus 6 KiB for the original grid queue.

This is why a decrease-key data structure is **not automatically faster**:
the cost of an index array and its scattered writes may exceed the cost
of the relatively few stale entries. Memory and p95 regressions matter.

**Decision:** retain `BinaryLazy` as the production default for **both**
frontiers. Expose the four alternatives as explicit opt-in A/B settings
without silently changing the certified heuristic, geometry, collision
checks, costs, goal tolerances, or resource budgets. Consider promoting an
alternative only if repeatable representative hardware benchmarks show a
meaningful end-to-end improvement within a defined memory budget.

## Reproduction

```bash
cmake -S . -B build-queues -DCMAKE_BUILD_TYPE=Release \
  -DHYBRID_ASTAR_BUILD_TESTS=ON \
  -DHYBRID_ASTAR_BUILD_BENCHMARKS=ON
cmake --build build-queues --parallel
ctest --test-dir build-queues --output-on-failure
./build-queues/hybrid_astar_priority_queue_bench 7 > queue-results.csv
```

The manual CSV includes **p50/p95 latency**, all heap push/pop/stale/
replacement counters, peak heap entries and approximate peak allocated
queue storage. The benchmark checks outcome/cost/expansion equivalence to
the original binary/binary baseline and aborts if a variant differs.
It is never added to CTest or GitHub Actions.

Example controlled A/B:

```cpp
request.options.openQueueMode =
    hybrid_astar::ackermann::OpenQueueMode::FourAryLazy;
request.options.gridQueueMode =
    hybrid_astar::ackermann::OpenQueueMode::BinaryLazy;
```

A grid queue variant only matters when the optional
`PerformanceReedsSheppAndObstacleGrid` heuristic is enabled.

## Remaining opportunities

- Representative CPU governor, warmups, randomized variant order,
  multi-seed p50/p95 latency distributions and RSS/allocator measurements.
- Profile hash-label lookup, geometric collision sampling, OMPL connector
  calls and motion-state allocation separately; they may dominate queues.
- Evaluate cache-friendlier **dense or compact** index representations
  rather than allocating an index slot for every raster cell or node ID.
- Keep stale-open semantics and deterministic tie breaking in regression
  tests if refactoring the search dominance policy.
