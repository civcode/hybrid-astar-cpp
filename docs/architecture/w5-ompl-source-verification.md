# W5 / W4 / W8 — real OMPL 2.0.2 source integration evidence

**Recorded:** 2026-10-26. **Status:** the Reeds–Shepp integration has now
actually **compiled, linked, and executed against genuine OMPL 2.0.2 source
files** in a focused standalone CMake harness. This is significantly stronger
evidence than a header-only API test; it is not equivalent to installing/building
the complete upstream OMPL distribution or building a full clone of this
repository against its exported CMake package.

## Offline reproduction without GitHub Actions

Supply an extracted OMPL 2.0.2 source tree and Eigen3 headers. These are
external inputs; no OMPL sources or Eigen header files are vendored.

```bash
cmake -S tests/ompl_source_bridge -B build-source-ompl \
  -DCMAKE_BUILD_TYPE=Release \
  -DOMPL_SOURCE_ROOT=/path/to/ompl-2.0.2 \
  -DEIGEN3_INCLUDE_DIR=/path/to/eigen3
cmake --build build-source-ompl --parallel
ctest --test-dir build-source-ompl --output-on-failure
```

The source bridge compiles the real OMPL StateSpace, SE2, SO2, ReedsShepp
implementation and requisite native support translation units into a
**small static test-only subset**, and links the real adapter and existing
core. It generates only an OMPL 2.0.2 feature/config header for this focused
build. This approach worked without the unavailable Boost.Serialization
binary package because that optional subsystem is not linked.

**Not a production OMPL package:** use the normal
`HYBRID_ASTAR_ENABLE_OMPL=ON` path against a fully installed, official OMPL
CMake package before shipping. The source subset cannot validate the full
distribution's ABI, CMake export, all dependencies, ROS integration or
non-Reeds–Shepp components.

## Tested and observed

- Existing `ompl_smoke`: real short reverse/forward/heading-change paths,
  signed section length, cusp, strict planner connection, default and
  noncertified optional heuristic, zero-length path.
- New `ompl_integration_test`: **12,000 deterministic continuous pose
  pairs** (3000 each at 0.5, 1.0, 4.0 and 9.0 m minimum turning radius).
  OMPL distance nonnegative/symmetric, every signed section finite and
  positive progress, curvature bound, exact chaining, yaw/XY endpoint,
  reconstructed total length matching the OMPL metric. Recorded **10,629,
  10,761, 10,654 and 9,749 sections** and **1,667, 1,886, 2,913 and
  4,261 gear transitions**, respectively, in this seeded sample.
- Full planner: real OMPL reverse connector, explicit noncertified hybrid
  heuristic, bounded validated incumbent search, obstacle wall and
  independent sampled success evidence.
- New synchronous
  `steering::planAndRefineWithReedsShepp(request, refinementOptions, performance)`
  creates **one reusable OMPL evaluator** across both planning and
  independent post-search refinement. The result exposes the unmodified
  validated original separately from optional refiner output. An insufficient
  refiner budget cannot erase the validated original.
- **Real integration found an actual numerical bug:** floating-point
  evaluation of `length * sampleIndex / count` could round the final sample
  slightly *beyond* the closed-form section's domain and cause
  `Invalid Ackermann section evaluation` / `InternalError`.
  All three sampling paths (search collision, oriented clearance integration,
  post-search refiner) now force the final sample to exactly
  `section.lengthMeters`. The triggering improvement-mode regression
  passes against real OMPL.
- **W4 next gate implemented:** `maxTotalFinalPoseChecks` (default 250000)
  now caps cumulative exact strict pose checks across **all** final candidate
  validations. A limit reached before proving any complete path returns
  `ValidationIncomplete`; after at least one verified incumbent,
  `ImproveIncumbent` returns that incumbent with
  `TerminationReason::ValidationBudgetExhausted`. The count never exceeds the
  configured cap and an allowance of zero is an invalid request.

**Verification matrix, local reconstructed core workspace:**

| Configuration | Result |
| --- | --- |
| GCC 14.2, C++20, Release, true OMPL 2.0.2 ReedsShepp subset | **10/10 tests passed** |
| Clang 17, C++20, ASan + UBSan (OMPL subset included) | **10/10 tests passed** |
| Full upstream OMPL CMake package + exact remote repository checkout | **Pending** |

The Eigen3 headers came from an existing local Python dependency, and Boost
headers from system Boost 1.83. No network download, GitHub Actions run or
automatic CI trigger occurred. These are **sampled static-map geometric paths**:
none of this proves continuously swept collision freedom, completeness,
global cost optimality or actuator-feasible timed steering.

## Remaining work after this milestone

- Test the exact GitHub branch checkout and the full official OMPL
  installation's imported CMake targets on the supported product toolchain.
- Expand collision/contact differential fixtures, parking benchmarks and
  documented latency/memory thresholds on product-representative hardware.
- Design proof-backed swept-volume checking or a stated monitoring safety
  contract, and separate steering-rate/acceleration/time parameterization.
- Continue multi-vehicle architecture work only when an actual second
  motion model and its full-state collision representation are implemented.
