# ADR-0017: Explicit unknown-occupancy policy with conservative frontier default

**Status:** Accepted (unknown-cell policy/default and API separation; remaining map details in OQ-006/OQ-023)  
**Date:** 2026-10-24

## Context

In live exploration, the occupancy grid gradually changes as sensors reveal the environment. Unknown cells are not the same as confirmed obstacles or confirmed free space. The library must support frontier exploration without tying the generic Hybrid A* core to a specific SLAM/exploration implementation.

The accepted ROS-compatible semantics in [ADR-0009](0009-ros-map-coordinate-conventions.md) recognize the conventional signed occupancy encoding `-1 = unknown` and `0..100 = occupancy probability` (with `0` known free and `100` occupied).

## Decision

1. **Preserve unknown occupancy as data.** The map representation or its declared adapter conversion distinguishes `-1` from known free and occupied cells. The native internal representation (signed cell or typed classification) is still an OQ-023 detail. Conventional signed occupancy values outside `-1,0..100` must not be silently reinterpreted.
2. **Expose an explicit per-request planning policy**, separate from immutable `MapSnapshot` data:

   ```cpp
   enum class UnknownCellPolicy { Occupied, Free, Reject };

   struct PlanningOptions {
       UnknownCellPolicy unknownCells = UnknownCellPolicy::Occupied;
       uint8_t maxTraversableOccupancy = 0; // Accepted in ADR-0018
   };
   ```

   These are **illustrative** types, not finalized ABI. `PlanRequest` contains or refers to validated options.
3. **Default to `Occupied`**, i.e. unknown cells are treated as blocked for collision and traversability decisions. This supports frontier exploration: an external exploration component selects reachable known-free goals near the observed/unobserved boundary; Hybrid A* plans only over confirmed traversable space for the robot's **complete swept collision geometry**.
4. `Free` treats unknown cells as traversable during planning, explicitly producing an **optimistic/provisional** environmental-validity claim. An unobserved region is never advertised as verified obstacle-free. `Reject` makes unknown occupancy an explicit error instead of implicitly assigning it free/blocked semantics. The exact request-rejection scope (any unknown anywhere vs unknown encountered by a relevant map query) and interactions with global heuristic expansion remain open under OQ-006; avoid aborting frontier exploration on irrelevant unknown cells.
5. The policy is applied consistently across collision checking, inflation/traversability, reachability estimates and heuristic evaluation. If a request changes policy, map-dependent caches derived under another policy cannot be reused without a sound policy-independent derivation. A map snapshot should remain unchanged and usable for multiple policy choices.
6. A planning request reads **one consistent immutable map revision**, while mapping/SLAM may publish newer revisions. Live-map updates or stale-path handling are orchestrated by an application/replanning layer; the planner does not directly mutate its map mid-search.
7. Frontier selection, sensor fusion, exploration progress and online map publishing remain **outside the planner core**; no ROS dependency is introduced.
8. The known-cell threshold is **now accepted** in [ADR-0018](0018-known-free-only-occupancy-threshold.md): `maxTraversableOccupancy=0` by default, and a known cell `c` is traversable iff `c <= max`. Occupancy-storage and owning-handle representation, snapshot-update granularity, cache-invalidation strategy, and detailed `Reject` semantics remain unresolved OQ-006/OQ-023 matters. A concrete [live-map proposal](../architecture/live-map-snapshots-and-cache-proposal.md) records recommendations without declaring them accepted.

## Consequences and tests

- Conventional `int8` adapter tests for `-1`, `0`, intermediate `1..99`, and `100`, with explicit invalid-value rejection.
- Test the three selectable unknown policies against identical snapshots; `Occupied` is the actual default.
- For frontier scenarios, the robot's entire swept footprint remains known free (subject to occupancy threshold), not just its center/reference point.
- Optimistic `Free` mode must distinguish *unknown traversed* from *known-free verified* in result diagnostics/assurance; the later controller/local safety system still responds to new obstacles.
- Test cache separation across occupancy policy, threshold, map revision and frame/geometry.
- Documentation must not claim the map has changed merely because a request chooses another unknown-cell policy.
