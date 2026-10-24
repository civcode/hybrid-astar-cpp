# Nonfunctional requirements and measurement plan — draft v0.1

**No measured results or hardware-specific targets yet.** Choose numerical thresholds only after defining reference hardware, map characteristics and representative vehicles. Each quality requirement has a method to establish its baseline.

| ID | Quality objective | Measurement and intended reporting |
| --- | --- | --- |
| NFR-001 | Predictable planning latency | Wall-clock p50/p95/p99, max and timeouts by scenario, including heuristic preparation, search and refinement separately |
| NFR-002 | Efficient search | Generated / expanded / re-opened nodes, open-set peak, heuristic cache hit ratio |
| NFR-003 | Bounded memory | Peak resident memory, bytes/node, per-request allocations, cache sizes; budget configurable |
| NFR-004 | Robust numerical behavior | Endpoint error, turning-radius scaling, finite/NaN checks, extreme radii, degenerate configurations |
| NFR-005 | Kinematic and collision correctness | Constraint violation count; complete-trajectory swept-footprint validation with documented resolution/certification |
| NFR-006 | Reproducibility | Fixed seeds, inputs, build flags, map/config hash, machine/OS/compiler metadata |
| NFR-007 | Thread safety / concurrency | Planner-instance ownership, immutable snapshot sharing, cache synchronization; TSAN where possible |
| NFR-008 | Portability | Standalone Linux C++20 and CMake baseline first; validate additional toolchains by matrix |
| NFR-009 | Modularity | No core dependency on ROS 2/OMPL/Ceres/yaml-cpp/nlohmann-json/Catch2/cxxopts |
| NFR-010 | Result transparency | Structured status (success, no path, cancelled, budget exceeded, invalid input, internal failure) and metrics |
| NFR-011 | Path quality | Path length, reverse count, steer variation/curvature, obstacle clearance, refinement success rate |
| NFR-012 | Maintainability | Compile-time model contract checks, focused modules, contract tests and documented decisions |

## Scenario benchmark matrix (proposed)

| Scenario | Purpose |
| --- | --- |
| Empty open field | Baseline propagation/search overhead |
| Wall with distant doorway | Demonstrate benefit of obstacle-aware heuristic |
| Narrow corridor and turn | Footprint precision and curvature feasibility |
| Parking / multi-point turn | Reeds–Shepp and reverse semantics |
| Dead end / unreachable goal | Exhaustion, budget behavior and correctness |
| Rotation-in-place | Differential-drive progress and collision sampling |
| Articulated chassis near obstacles | Full state and body-transform collision validity |
| Replanning same goal on unchanged map | Cache efficiency and lifecycle |
| Replanning after map version change | Safe cache invalidation |
| Large map, sparse obstacles | Memory and scalability |

## Benchmark protocol

1. Record compiler version, build type, CPU model, frequency policy, map dimensions/resolution, kinematic parameters and code commit.
2. Warm up caches, then measure cold-start and warm-cache separately. Use repeatable seeded scenario sets.
3. Isolate heuristic-build, search, collision, steering, path reconstruction, refinement and serialization costs.
4. Report distributions rather than single best runs; preserve failing seeds and inputs.
5. Compare changes against previous baseline; any claimed speedup must include equivalent success-rate and path-feasibility results.
6. Profile locally before optimizing; avoid running GitHub Actions for local builds or benchmarks.

## Open acceptance thresholds

- [ ] Define reference machines and supported OS/compiler versions.
- [ ] Set per-scenario planning latency budgets (p95/p99), if needed.
- [ ] Set memory budgets and maximum grid/state dimensions.
- [ ] Specify collision-validation guarantee level (sampled vs conservative continuous).
- [ ] Define numerical tolerances for pose, arc length and constraint checks.
- [ ] Define expected baseline success rate and path-quality comparisons.
