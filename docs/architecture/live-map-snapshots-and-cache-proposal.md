# Live-map snapshot, cache, and strict-unknown semantics — accepted design rationale

**Status:** Accepted architectural direction through [ADR-0019](../adr/0019-live-map-snapshot-cache-and-reject.md), supplementing [ADR-0017](../adr/0017-explicit-unknown-occupancy-policy.md) and [ADR-0018](../adr/0018-known-free-only-occupancy-threshold.md). This note gives implementation rationale and examples, not final public API or a decision on OQ-012 concurrency.

## Design goal

Support live frontier exploration with frequent SLAM/map updates, without mutating occupancy under an active Hybrid A* search or requiring a lock or allocation on each collision query. Keep the baseline search, map consumer and cache lifecycle simple; optimize copying/incremental updates only when measured.

## Accepted map ownership and publication principles

- The map producer builds a complete, validated **new immutable revision**, with a unique `mapIdentity` (distinguishes reset/different source), monotonically increasing `revision` within that identity, grid geometry/frame, and ROS-compatible declared occupancy semantics.
- Share the revision as a `shared_ptr<const MapData>` or opaque owning `MapSnapshot` handle. A `MapProvider` may publish the latest handle atomically (`std::atomic<std::shared_ptr<const MapData>>` in C++20); each request obtains one handle at initiation. Only the publication handle swaps, **not** the already published `MapData` buffer.
- The mapping producer may continue accumulating sensor observations in private mutable buffers, then publish at an application-configured *planning update cadence*, not automatically on every LiDAR return. Actual publication and replan triggers are outside the ROS-independent Hybrid A* core.
- One planning request evaluates collision, heuristics and path validity exclusively against its acquired revision. Return `mapIdentity`, `revision`, and validity/unknown-assurance metadata. If the world/map changes afterward, a supervisor/controller determines whether the path must be revalidated, replaced or cancelled.
- **Baseline:** one contiguous grid per revision. This may cost O(N) copying when publishing a full new grid. Start here for clear semantics; evaluate **immutable tiled / copy-on-write** storage only if measured update rate or grid size justifies the complexity. Shared old revisions remain alive while referenced; bound publication/cache retention to control memory.
- Reject changed map origin, resolution, frame, cell semantics or extent as a reusable heuristic-cache match even if integer revision values coincide; map identity/version handling must prevent the ABA/reuse problem.

Illustrative C++20, not a frozen API:

```cpp
struct MapData {
    GridGeometry geometry;
    std::vector<int8_t> occupancy;
    MapIdentity identity;
    uint64_t revision;
};

using MapSnapshot = std::shared_ptr<const MapData>;
// MapProvider holds atomically published MapSnapshot.
struct PlanRequest {
    MapSnapshot map;        // acquired once before the search
    OccupancyPolicy occupancy;
    // start, goal and other options
};
```

## Accepted cache lifecycle principles

Separate immutable **model-dependent** caches (motion primitive catalog, kinematic-effort tables, model-specific templates) from **map-dependent** caches (traversability/inflation, reverse goal-distance field, distance transform).

Each map-dependent cache key identifies **all inputs affecting its data**:
- Map identity and revision (a unique immutable snapshot), including changes to geometry/frame/declared cell encoding.
- Known-cell `maxTraversableOccupancy` and `UnknownCellPolicy`, or an independently proven policy-invariant representation.
- Goal **region**, including XY tolerance, rasterization/multiple goal seeds and appropriate configuration used by that particular field.
- Relevant collision inflation / footprint interpretation / heuristic algorithm and options. Add all other effective dependencies actually read by the producer.

On a new planning request: reuse a cache only for an *exactly compatible* key; otherwise create it on demand. A new map revision does not destroy old caches required by active requests, nor launch automatic recomputation. Keep a bounded LRU (or simpler small per-instance cache) and release entries when idle. A lazily constructed reverse-Dijkstra field returns **only settled** distances, with explicit synchronization or per-instance scratch to avoid races across concurrent requests.

**Initial implementation:** rebuild map-dependent fields on demand after relevant map changes. Do not introduce incremental Dijkstra/D* Lite cache repair or fine-grained tile invalidation before benchmarking frequent-replanning latency, total memory, and map copying. If nearly every replan sees a fresh revision, tune publication cadence first, then investigate incremental cache repair as a separate design extension.

**Admissibility reminder:** cache correctness only means the derived field matches its inputs; it does **not** certify that the field is a lower bound on continuous-space cost. [ADR-0015](../adr/0015-admissible-default-heuristics.md) still governs default search heuristic selection.

## Accepted `UnknownCellPolicy::Reject` semantics

The policy choice changes behavior:
- **`Occupied` (accepted default):** unknown space is blocked; prune a motion candidate if the swept footprint intersects unknown space, continue searching.
- **`Free` (accepted):** unknown space may be traversed for optimistic planning, with provisional/unknown-traversal diagnostics; controller safety remains separate.
- **`Reject` (accepted strict mode):** fail the **request at validation time** with a distinct unknown-occupancy error if any cell in the declared *planning domain* is unknown. Do **not** abort just because the search happens to query an unknown cell first, and do **not** make strict validation dependent on heuristic expansion order.

Initially the **planning domain is the entire supplied snapshot grid**. An optional explicit planning ROI could narrow validation in a later iteration, but its scope must be declared and validated (with relevant swept-footprint margin), not inferred from whichever candidates the algorithm explores. A map publisher may attach a validated `hasUnknown` summary, or the planner may compute/cache it per immutable revision, so repeated strict checks do not incur O(N) scanning each time.

This `Reject` behavior is **accepted by ADR-0019** and is not intended for live frontier maps, which naturally contain unknown cells. Unknown outside the provided grid remains a separate out-of-bounds policy and must not silently become free.

## Validation plan

1. Two concurrent planning requests can read revision R while R+1 publishes, without seeing partial updates or holding a shared map write lock.
2. Replanning with R+1 uses new map-derived fields, but reuses model/primitive caches.
3. Replanning with the same revision/goal/policy reuses eligible cache entries; changing unknown policy, threshold, inflation or goal-region settings invalidates dependent fields only.
4. Cached reverse-Dijkstra distances are settled, and default heuristic admissibility is independently certified.
5. Strict `Reject` result is independent of search order, uses an explicit error status and does not conflate unknown with occupied. Validate the snapshot summary/incremental update assumptions before trusting it.
6. Old snapshot and cache memory is eventually released; benchmark map publication copy costs, peak memory, latency with frequent revisions and local replan cadence.
7. Revalidation on a newer map catches an obstacle newly revealed on a previously accepted path. The planner never claims that an old validated path remains safe after environment updates.

## Remaining implementation-facing questions

- Choose concrete native cell storage, provider/owning-handle type and error API names during interface review (OQ-023).
- Define exact synchronization/scratch ownership for concurrent planner requests and lazy caches (OQ-012).
- Introduce an explicitly declared ROI for strict `Reject` only through a separate future decision; baseline checks the entire supplied snapshot.
- Benchmark publication copy costs, cache memory budget and update cadence before adding tiled snapshots or incremental distance repair.
