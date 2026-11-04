# Refined map-specific label reservations: tier-series v2 and matched timing

## Goal

The first map-specific 32-route calibration measured a peak of **484,534
live labels** on the full Nav2 warehouse with the native differential model
and 70 x 50 cm footprint. At **20% headroom**, the requested capacity is
`ceil(484534 * 1.20) = 581441` labels. The original doubling-only tier
series (version 1) rounded this to **1,048,576 labels**, while a direct
zero-headroom replay proved that **524,288 labels** was enough to avoid
rehashes on those *sampled* routes.

To reduce large upward allocation steps, the calibration now uses **tier
series version 2: four equal integer increments per doubling interval**.

| v2 tier | Reserved labels | Prior v1 tier, if any |
| ---: | ---: | ---: |
| 0 | 0 (growing-table default) | 0 |
| 1 | 65,536 | 1 |
| 2 | 81,920 | — |
| 3 | 98,304 | — |
| 4 | 114,688 | — |
| 5 | 131,072 | 2 |
| 6 | 163,840 | — |
| 7 | 196,608 | — |
| 8 | 229,376 | — |
| 9 | 262,144 | 3 |
| 10 | 327,680 | — |
| 11 | 393,216 | — |
| 12 | 458,752 | — |
| 13 | 524,288 | 4 |
| **14** | **655,360** | — |
| 15 | 786,432 | — |
| 16 | 917,504 | — |
| 17 | 1,048,576 | 5 |

The selected v2 tier for 581,441 requested labels is **14 = 655,360**,
only **12.71% above the headroom-adjusted target**, compared with
**80.33% above target** for the original doubling tier 5. These figures
refer to requested label capacity, **not** actual allocated hash buckets
or measured process RAM.

### Version and persistence

`LabelReserveTierSeriesVersion == 2` and the result field
`tierSeriesVersion` are part of the public calibration API.
`labelReserveForTier(index)` decodes v2; for historical stored
v1 indices use **`labelReserveForTier(index, 1)`**. JSON output now
includes `tier_series_version: 2` alongside both
`recommended_tier_index` and `recommended_reserve_labels`.
Persist all three. An old tier 4 means **524,288** in v1 but
**114,688** in v2; never silently reinterpret historical indices.

The planner still accepts an explicit **label count**, not a tier index,
and `PlanOptions.searchLabelReserve` still defaults to **0**.
Calibration is never triggered implicitly by a planning request.

## Measured performance: 32 seeded warehouse routes (2026-11-04)

**Completed evidence:** `nav2-label-refined-tiers-32.csv` contains
**128 successful rows**: 32 seeded routes × 4 reservation tiers,
five timed search repetitions after a per-variant warmup.
All 128 rows have **zero rehashes** and **identical search-work results**
for each matched route; all report tier-series v2, map fingerprint
`16467401262430778714`, seed `20261104`, differential model and
large (70 × 50 cm) footprint. The CSV analyzer verified the entire
four-way matrix with profiling disabled.

| Tier | Reserved labels | Median search p50 | Median paired change versus tier 13 | Wins versus tier 13 | Median bucket-array MiB |
| ---: | ---: | ---: | ---: | ---: | ---: |
| **13** | **524,288** | **176.562 ms** | baseline | — | **4.29** |
| **14** | **655,360** | **176.708 ms** | **~0.00%** | **16/32** | **5.03** |
| 15 | 786,432 | 177.667 ms | **0.41% slower** | 10/32 | 6.36 |
| 17 | 1,048,576 | 178.551 ms | **0.72% slower** | 5/32 | 8.06 |

**Tier 13 and tier 14 have no consistent measured latency winner.**
Tier 14 is faster on 16/32 pairs, tier 13 on the other 16;
the median paired tier-14-versus-tier-13 difference is only
**+0.001% slower**, well below a useful actionable difference in
this run. Their *independently calculated medians* differ slightly
(176.562 vs 176.708 ms); do not substitute that difference for the
median of paired percentages.

- Tier 14 needs **+0.73 MiB of estimated bucket pointers** versus
  tier 13. It retains **170,826 labels** of spare nominal capacity
  at the hardest observed peak of 484,534 (35.3% above that peak).
  Tier 13 has only **39,754 labels** spare (8.2% above the peak).
  Neither margin proves anything about unobserved requests.
- Tier 17 uses **+3.03 MiB** estimated buckets compared with tier 14
  and has worse search p50 on **27/32 matched routes**.
  Its median paired slowdown versus tier 14 is **0.68%**.
- Tier 15 also offers no observed advantage: versus tier 14 its
  median paired slowdown is **0.38%** and it wins just **6/32**.
- The five most demanding sample peaks exceed the older 262,144-label
  setting. Sample 24 reached **484,534 labels** and **478,906
  expansions**; its per-tier search medians were
  983.884 / 986.787 / 980.121 / 982.702 ms. This one hard route
  does **not** override the paired 32-route result.
- Memory values are **bucket arrays only**, excluding node allocations,
  temporary rehash allocations, allocator overhead and peak process RSS.
  The benchmark does not establish which tier minimizes total RAM.

**Recommendation for this calibration profile:** retain **v2 tier 14
(655,360)** as the **20%-headroom-adjusted**, sampled zero-rehash
recommendation: its performance is effectively level with tier 13,
and its incremental bucket allocation is just ~0.73 MiB.
Use **tier 13 (524,288)** when minimizing the measured bucket memory
matters more than retaining the configured headroom; it also avoided
rehashes on all these sampled routes. Do **not** select tiers 15/17
for this workload without new supporting evidence.

This selection is **map-, footprint-, model- and budget-specific**.
The smaller inflated geometry, other seeds, smaller maps, higher budgets
and real peak-RSS effects are **not** validated by this comparison.
The library default remains `searchLabelReserve=0`; calibration
and use of a tier are explicit.

## Manual matched speed experiment

A new executable
`hybrid_astar_label_reserve_refined_tiers_benchmark` tests **tiers
13 / 14 / 15 / 17**, i.e. **524,288 / 655,360 / 786,432 / 1,048,576**
labels. It uses exactly the seeded spatial sampling implementation
from `calibrateLabelReserve` with the original 32-route parameters.
The `--samples 32 --seed 20261104` mode should reproduce the
positions from `nav2-label-calibration-large-32.csv` if the map,
geometry and planner settings remain unchanged.

Unlike the calibration CSV, this benchmark **times searches**.
It warms each tier separately, executes multiple unprofiled timed
repeats per route, rotates/reverses variant execution order, and
enforces identical search work, paths and strict collision diagnostics.
It rejects a rehash in any tier; the purpose is to distinguish
**speed/load-factor effects** after all tiers already avoid rehashes.

All map sampling/discovery is excluded from timed variant samples.
Timing includes each search's initial hash-table allocation. Bucket
memory is reported as `labelPeakBuckets * sizeof(void*)`, a bucket
array estimate only: not label-node allocations, allocator overhead,
transient peak process RAM or RSS.

### Build and fast functional check

```bash
git switch feature/nav2-equivalent-envelope-footprint-benchmark
git pull --ff-only
export BUILD_JOBS=28 TEST_JOBS=4 PROFILE_HOTPATHS=OFF
docker compose build test

docker compose --profile test run --rm test \
  ctest --test-dir build-docker --output-on-failure -j 4 \
  -R '^(label_reserve_calibration_test|hotpath_test)$'
```

The two-route smoke test below checks the new benchmark machinery but
is **not** a subset of the original 32-route sample: `--samples 2`
changes the automatically chosen spatial strata. Use `--samples 32`
for the exact original route-generation configuration.

```bash
docker compose --profile test run --rm test \
  ./build-docker/hybrid_astar_label_reserve_refined_tiers_benchmark \
  --model differential --geometry large --fast-empty-aabb \
  --map apps/demo/maps/nav2_warehouse_full.yaml \
  --samples 2 --seed 20261104 --repeats 1 \
  --max-expansions 500000 --max-generated 2000000 \
  --grid-settlements 100000 \
  > nav2-label-refined-tiers-smoke.csv

python3 tools/analyze_label_reserve_refined_tiers.py \
  nav2-label-refined-tiers-smoke.csv --expected-routes 2
```

Expected smoke CSV: **8 rows** = 2 randomized routes x 4 tiers.

### Full 32-route experiment

```bash
docker compose --profile test run --rm test \
  ./build-docker/hybrid_astar_label_reserve_refined_tiers_benchmark \
  --model differential --geometry large --fast-empty-aabb \
  --map apps/demo/maps/nav2_warehouse_full.yaml \
  --samples 32 --seed 20261104 --repeats 5 \
  --max-expansions 500000 --max-generated 2000000 \
  --grid-settlements 100000 \
  > nav2-label-refined-tiers-32.csv

python3 tools/analyze_label_reserve_refined_tiers.py \
  nav2-label-refined-tiers-32.csv
```

Expected full CSV: **128 matched rows**, 32 routes x 4 tiers,
each with five timed repeats plus a warmup. The generator also
runs an **unreserved** baseline for route discovery and invariance
checks, but those discovery runs are **not** speed measurements.

The analyzer verifies tier-series v2, capacities, success, zero
rehashes, matching route and map identifiers, work counters and all
four variants per route. It also **rejects profiled builds** (the CSV
`profile_enabled` column must be zero; compile with
`PROFILE_HOTPATHS=OFF`). It reports matched median search times,
paired speedups, wins, bucket-memory estimates and hard-route outliers.

## Decision criteria

The earlier fixed 10-route benchmark showed 262,144 to be fastest
for those 10 easier warehouse routes. The much harder randomized
32-route workload exceeded 262,144 labels on five routes. It is
**not justified** to carry the original latency results over to the
larger tiers, nor to assume that zero rehashes automatically gives
the fastest configuration.

- Compare **tier 14** with tier 13 and tier 17 on the *same* route.
  A median speed improvement must be weighed against memory cost.
- Retain the proven **0-rehash** and identical-search-work requirements.
- Evaluate both body geometries and additional seeds before changing
  any deployment-wide settings. Higher search budgets can demand
  even more label capacity.
- A real peak-RSS comparison, if required, needs a separate memory
  profiling run; this CSV only estimates bucket-array memory.

**Status:** The 32-route large-footprint benchmark is complete
and summarized above; the manual command remains available for exact
reproduction and other geometries/seeds. No GitHub Actions workflow
was created or executed; the executable is manual-only and not
registered with CTest.
