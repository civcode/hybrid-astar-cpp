# ADR-0018: Maximum traversable occupancy defaults to known-free only

**Status:** Accepted (threshold semantics/default; map snapshot/cache/strict `Reject` further specified by ADR-0019)  
**Date:** 2026-10-24

## Context

The conventional ROS-compatible occupancy representation preserves signed `-1` for unknown and `0..100` for estimated occupancy probability. In the default frontier-exploration workflow, the robot should travel within verified known-free cells and avoid assuming even slightly uncertain cells are free.

An option named `occupiedThreshold` is ambiguous about whether the boundary itself is traversable.

## Decision

1. Expose a **configurable** integral `maxTraversableOccupancy` with allowed range **0..100** and **default `0`**. For a *known* cell `c in [0,100]`, it is traversable iff `c <= maxTraversableOccupancy`; consequently **known-cell blocking uses strict `c > max`**. At the default, only exactly `0` is traversable.
2. `-1` unknown remains a distinct state and is evaluated **only by the per-request `UnknownCellPolicy`** from [ADR-0017](0017-explicit-unknown-occupancy-policy.md). The default unknown policy is `Occupied`. This interpretation does not alter source map cells.
3. Values outside the declared encoding (signed values other than `-1` and `0..100`), invalid parameter values, and unintended unsigned Nav2 costmap reinterpretation are errors, not silently mapped into traversability.
4. The effective threshold and unknown policy are planning options and part of every derived traversability/inflation/heuristic cache's validity context. Different planning requests may use the **same immutable map snapshot** with different options.
5. A cell's classification is independent of out-of-bounds handling; unrepresented space is not implicitly known free. Apply the classification consistently to the complete swept robot footprint, not just the reference point.
6. The default deliberately rejects known probabilistic cells with values `1..100`. Applications with noisy occupancy estimates may configure a less conservative threshold.

## Illustrative interface (not final ABI)

```cpp
struct OccupancyPolicy {
    uint8_t maxTraversableOccupancy = 0;  // 0..100 inclusive
    UnknownCellPolicy unknownCells = UnknownCellPolicy::Occupied;
};
```

`PlanningOptions` contains this policy; source map contents do not.

## Verification

- For default `max=0`: `0` traversable, `1`, `20` and `100` blocked.
- For `max=20`: `0`, `1`, `20` traversable, `21` and `100` blocked.
- For any threshold, `-1` follows the selected unknown policy, not the numeric comparison.
- Boundary, invalid encoding, invalid threshold, all three unknown options, map immutability and cache-key separation tests.

## Remaining open questions

[ADR-0019](0019-live-map-snapshot-cache-and-reject.md) accepts the baseline snapshot publication/ownership, revision-keyed cache lifecycle and strict full-snapshot `Reject` semantics. OQ-023 still covers native occupancy storage and frame/handle API; OQ-012 covers mutable-cache/planner concurrency.
