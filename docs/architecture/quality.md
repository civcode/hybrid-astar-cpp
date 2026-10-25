# Nonfunctional requirements and measurement plan — draft v0.1

**No measured results or hardware-specific targets yet.** Choose numerical thresholds only after defining reference hardware, map characteristics and representative vehicles. Each quality requirement has a method to establish its baseline.

| ID | Quality objective | Measurement and intended reporting |
| --- | --- | --- |
| NFR-001 | Predictable planning latency | Wall-clock p50/p95/p99, max and timeouts by scenario, including heuristic preparation, search and refinement separately |
| NFR-002 | Efficient search | Generated / expanded / re-opened nodes, open-set peak, heuristic cache hit ratio |
| NFR-003 | Bounded memory | Peak resident memory, bytes/node, per-request allocations, cache sizes; budget configurable |
| NFR-004 | Robust numerical behavior | Endpoint error, turning-radius scaling, finite/NaN checks, extreme radii, degenerate configurations |
| NFR-005 | Sampled collision at stored primitive/connector poses with **sound conservative hull-mask bypass** and rasterized footprint boundary **plus blocked-cell interior guard**; no continuous swept collision certification; benchmark occupancy/mask/raster time (ADR-0029) |
| NFR-006 | Reproducibility | Fixed seeds, inputs, build flags, map/config hash, machine/OS/compiler metadata |
| NFR-007 | Thread safety / concurrency | Planner-instance ownership, immutable snapshot sharing, cache synchronization; TSAN where possible |
| NFR-008 | Portability | Standalone Linux C++20 and CMake baseline first; validate additional toolchains by matrix |
| NFR-009 | No ROS 2/Ceres/yaml-cpp/nlohmann-json/Catch2/cxxopts in generic public search API; **OMPL permitted in isolated initial runtime steering adapter** (ADR-0028) |
| NFR-010 | Result transparency | Structured status (success, no path, cancelled, budget exceeded, invalid input, internal failure) and metrics |
| NFR-011 | Path quality | Evaluate **final validated path after optional refinement** as the product metric: length, reverse count, steering/curvature, clearance, refinement success rate; also log the raw path for diagnosis (ADR-0020) |
| NFR-012 | Maintainability | Compile-time model contract checks for three reference chassis, focused modules, contract tests and documented decisions |

## Scenario benchmark matrix (proposed)

| Scenario | Purpose |
| --- | --- |
| Empty open field | Baseline propagation/search overhead |
| Wall with distant doorway | Demonstrate benefit of obstacle-aware heuristic |
| Narrow corridor and turn | Footprint precision and curvature feasibility |
| Parking / multi-point turn | Reeds–Shepp and reverse semantics |
| Dead end / unreachable goal | Exhaustion, budget behavior and correctness |
| Rotation-in-place | Differential-drive progress and collision sampling |
| Omnidirectional lateral and combined motion | Mecanum fixed-heading sideways travel, independent yaw, and orientation-sensitive rectangular collision |
| Articulated chassis near obstacles | Full state and body-transform collision validity |
| Replanning same goal on unchanged map | Cache efficiency and lifecycle |
| Replanning after map version change | Safe cache invalidation |
| Large map, sparse obstacles | Memory and scalability |

## Benchmark protocol

1. Record compiler version, build type, CPU model, frequency policy, map dimensions/resolution, kinematic parameters and code commit.
2. Warm up caches, then measure cold-start and warm-cache separately. Use repeatable seeded scenario sets.
3. Isolate heuristic-build, search, collision, steering, path reconstruction, refinement and serialization costs.
4. Report distributions rather than single best runs; preserve failing seeds and inputs.
5. Compare changes against previous baseline; any claimed speedup must include equivalent success-rate and path-feasibility results. When the refiner is available, also report **combined search+refinement runtime** and **final validated path** quality, not only the raw Hybrid A* path.
6. Profile locally before optimizing; avoid running GitHub Actions for local builds or benchmarks.

## Simplicity and evidence gate (ADR-0020)

- Keep the first complete planner pipeline readable and reproducible; count unnecessary abstractions, mutable shared ownership and optional dependencies as maintenance costs, not automatic benefits.
- Every proposed optimization first supplies a motivating reproducible failure or benchmark, then reports end-to-end results (success/feasibility, output quality including optional refinement, latency and memory), not just microbenchmark improvement.
- Preserve robust model/collision validation, clear numerical failure modes and honest reporting of approximate guarantees even when implementation is deliberately minimal.
- Stage optional search policies, map publication optimizations, actuator-effort tables and sophisticated refinement behind evidence rather than requiring them for the first viable product.
- Hybrid A* should be a low-cost feasible-path initializer for the intended downstream optimizer: avoid raw-path micro-optimizations that are not retained in the **final validated solution**. Keep indispensable collision/model validity, topology discovery and original-path fallback.

## Open acceptance thresholds

- [ ] Define reference machines and supported OS/compiler versions.
- [ ] Set per-scenario planning latency budgets (p95/p99), if needed.
- [ ] Set memory budgets and maximum grid/state dimensions.
- [x] Accept sample-pose collision validation using a conservative orientation-hull map lookup and rasterized footprint edges **with an interior guard** (ADR-0029). No continuous swept guarantee; retained RK4 0.10 m sample spacing and optional adaptive subdivision off by default.
- [x] Accept initial Ackermann 0.10 m primitive-generator sample spacing for default collision checks (ADR-0025).
- [ ] Set future stationary-model generation sample extents, optional adaptive threshold/budget and shape intersection numerical tolerances after targeted tests.
- [x] Accept first Ackermann RK4 arc-length endpoint tolerance **0.0001 m** (ADR-0025).
- [ ] Define model-independent numerical tolerances for poses, state constraints and collision shape intersection.
- [ ] Define expected baseline success rate and path-quality comparisons.
