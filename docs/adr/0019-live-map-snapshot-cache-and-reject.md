# ADR-0019: Immutable live-map snapshots, revisioned caches and deterministic unknown rejection

**Status:** Accepted  
**Date:** 2026-10-24  
**Related:** [ADR-0009](0009-ros-map-coordinate-conventions.md), [ADR-0015](0015-admissible-default-heuristics.md), [ADR-0017](0017-explicit-unknown-occupancy-policy.md), [ADR-0018](0018-known-free-only-occupancy-threshold.md)

## Context

Live frontier exploration publishes map updates while Hybrid A* searches a grid. A search cannot safely observe occupancy mutation mid-expansion; map-dependent heuristics, inflation and traversability caches cannot be reused blindly after map revisions or occupancy-policy changes. The per-request unknown-cell policy already allows `Occupied` (default), `Free`, and `Reject` (ADR-0017); this ADR fixes the latter's request-validation scope.

## Decision

### Snapshot lifetime and publication

1. **Publish immutable, versioned full-grid snapshots for the initial implementation.** A map producer builds and validates the next map revision privately, then atomically publishes a shared, owning, read-only snapshot handle. The C++20 implementation may use `std::atomic<std::shared_ptr<const MapData>>` at the provider boundary and `std::shared_ptr<const MapData>` in requests; exact public type names remain interface-design details. Merely using `const` is insufficient if any other mutable aliases remain: publication must end producer mutation of the shared backing buffer.
2. Each request **acquires one snapshot once** and holds it throughout search, full-body collision validation, heuristic construction/use and returned-path validation. Publishing a new revision never changes the previous buffer or metadata. Referenced old revisions remain alive until readers release them; publication must not wait for long-running searches.
3. Include a distinct map/source identity and a revision unique/monotonically advancing within that identity, together with validated frame, origin pose, resolution, grid dimensions, cell encoding and other map validity metadata. A map reset/new source receives a different identity; revision-number reuse cannot masquerade as the same map.
4. Initially use a **contiguous owning grid for each revision**; a full rebuild/copy at publication is acceptable for a simple baseline. Immutable tiled/copy-on-write publication is a *future performance extension* only if update-rate/peak-memory benchmarks justify it. The mapping/application layer chooses publication cadence and replanning triggers independently of individual sensor measurements and of the planner core.
5. Return the snapshot identity/revision and relevant occupancy-assurance metadata with the path. A path validated for revision R is **not automatically certified for revision R+1**; map-aware revalidation/replanning and live obstacle response remain application/controller responsibilities.

### Cache lifecycle

6. Separate **model-dependent** immutable data (primitive catalog, model effort lookup tables) from **map-dependent** data (traversability/inflation, goal-distance heuristic, clearance fields). Model-dependent data is reusable across map revisions if its *own* configuration hasn't changed.
7. Map-dependent cache entries have effective-input identities. Include map/source identity and revision, grid geometry/frame/encoding when not subsumed by snapshot identity, goal *region* and rasterization where used, occupancy threshold and unknown policy where relevant, inflation/footprint if relevant, and heuristic settings. A request **reuses only an exactly compatible entry** or an independently proven policy-invariant derivation.
8. If no compatible entry exists, **build on demand** for the request. New map publication neither invalidates data pinned by existing requests nor eagerly recomputes every field. Use bounded retention/eviction (e.g. a small per-instance cache or bounded LRU) so idle entries and old map revisions do not accumulate indefinitely. Do **not** implement incremental Dijkstra/D* Lite repair or tile-granular invalidation in the baseline; reconsider only when measured latency/memory justify it.
9. **Accepted first implementation (OQ-012): one active `plan()` request per planner instance.** A concurrent call on the same instance must **fail fast with a distinct Busy/ConcurrentRequest status**, not race on the mutable cached lazy reverse-Dijkstra queue or implicitly join another request. The instance owns its bounded lazy-field cache and per-request scratch state, and **serial** later requests may resume compatible cached heap+distance/status data. Different planner instances may run concurrently and safely share immutable pinned `MapSnapshot` objects; they must **not share a mutable Dijkstra frontier**. No locks or global concurrent cache are needed for the first implementation; an explicit future concurrency extension may revisit the rule. Valid cache provenance **does not itself certify heuristic admissibility**; ADR-0015 still applies.

### Unknown-cell `Reject` mode

10. **`UnknownCellPolicy::Reject` checks the *entire supplied map snapshot* at request validation time**, before search begins. If **any** map cell is `-1`, return an explicit **unknown-occupancy input/validation failure**, not an ordinary `NoPath` result. Do not wait for a collision query or heuristic expansion to encounter unknown data, because that would make outcomes depend on search order.
11. A snapshot may maintain a validated `hasUnknown` summary, or the planner may compute/cache the property once per immutable revision; its correctness must be tied to the actual snapshot data. The baseline has **no implicit planning ROI exception**. Adding an explicit bounded ROI for strict validation requires a separate reviewed extension.
12. In contrast, `Occupied` treats unknown as blocked and only prunes affected candidate motions; `Free` permits optimistic planning with provisional map-validity diagnostics. Per-request threshold `maxTraversableOccupancy=0` by default applies only to **known** cells (ADR-0018), regardless of unknown policy.

## Consequences and verification

- A plan and every cache entry are provenance-tagged; publication and active search access do not race on mutable occupancy.
- Benchmark **whole-grid publication copy cost**, retained-revision peak memory, revision-change replan latency, repeated same-revision cache hits, and cache eviction.
- Verify identical snapshot data is seen by two concurrent request readers while revision R+1 is published, and that old references free their buffers after release.
- Verify cache reuse only under equal effective inputs (including goal region, threshold, unknown mode, geometry/frame and map identity/revision), and that model-dependent caches survive environment changes.
- Verify `Reject` deterministically fails for `-1` **anywhere** in supplied map including far outside the eventual path, before invoking search; returns a dedicated diagnostic, and uses a reliable validated snapshot summary when available.
- Verify `Occupied` and `Free` do not acquire strict-`Reject` behavior; complete swept-body collision and path-provisional semantics still apply.
- Verify old paths are not represented as valid for a newer revision without a new environment check.

## Outside this decision

Exact native cell storage (signed `int8` vs typed representation), public frame ID type and malformed-encoding adapter conversions remain **OQ-023**. Exact error-enum identifiers and native owning-handle ABI remain implementation-facing questions; **OQ-012's first-implementation planner concurrency rule is accepted as one active request per instance, fail-fast on overlap**. Thread safety for concurrent distinct planner instances sharing immutable maps is required. This ADR does not add SLAM, D* Lite, MPC or moving-obstacle tracking to the ROS-independent core.
