# Nav2 warehouse: measured EDT clearance and A* label-reservation results

**Evidence date:** 2026-11-04. All numbers below come from manually run
full-resolution Nav2 warehouse replay CSVs, not estimates from profiled
code or predictions of future improvements.

**Benchmark fixtures:** `apps/demo/maps/nav2_warehouse_full.yaml` and
`bench/fixtures/nav2_full_10_seed20261029.csv`. The 70 x 50 cm rectangle
uses zero geometric inflation; the 50 x 30 cm rectangle uses a 10 cm
inflation margin. The prefix-sum empty-region shortcut is ON; the
experimental conservative fast AABB is ON for the benchmarks reported
here. Model: native differential drive.

## 1. Fixed-size A* label-table pre-reservation: measured benefit

**Input files:** `nav2-label-reserve-ab.csv` (40 rows, profiling OFF,
5 repeats per variant) and `nav2-label-lookup-profile.csv` (40 rows,
profiling ON, 3 repeats per variant). Each covers 10 replayed routes, two
geometries, and reservations of 0 (baseline) versus 262,144 labels.
The experiment uses exactly the same search, collision and refinement
logic on both sides.

| Uninstrumented metric (20 matched comparisons) | Baseline: reserve 0 | Reserve 262,144 |
| --- | ---: | ---: |
| Median search p50 | 231.42 ms | 220.28 ms |
| Median total pipeline p50 | 252.15 ms | 241.18 ms |
| Median label-table rehashes | 14 | 0 |
| Successful paired searches | 20 | 20 |

- **Median paired search-time saving: 4.85% (10.40 ms).**
- **Wins: 20/20** warehouse route/geometry comparisons.
- Median paired total-pipeline saving: **4.60%**.
- Median estimated *additional bucket-array* memory: **0.80 MiB**.
  That number does **not** measure heap-allocated label nodes, allocator
  metadata, transient peak memory, or the full process resident set.
  Four comparisons actually used fewer buckets with the reservation,
  because the baseline hash table grew beyond the reserved capacity.
- Expanded nodes, generated nodes, collision calls, label find/write
  counts, pruning and final paths did **not** change.
- Median paired savings were **4.85%** for 70 x 50 cm, **4.99%** for
  50 x 30 cm + 10 cm inflation.

The **instrumented** label profile further explains the result:

| Median profiled cost | Reserve 0 | Reserve 262,144 |
| --- | ---: | ---: |
| State-label lookup time | 94.24 ms | 86.69 ms |
| State-label write time | 21.09 ms | 13.00 ms |
| Hash time | 33.58 ms | 26.61 ms |
| Exact collision time | 208.51 ms | 208.11 ms |

Median hash calls dropped by approximately **416,264** per search.
In the baseline profiled runs, label lookup time was about **65.8%
successor hits**, **20.9% successor misses**, **13.2% open-queue hits**,
and essentially **0% open-queue misses**. About **81.2%** of successor
lookups hit existing keys. Profiling overhead is material: **do not add
nested hash timing to label lookup/write timing** or use profiling-build
search times to determine production speedup.

### Decision

**Retain the existing opt-in `PlanOptions.searchLabelReserve=262144`**
for warehouse-sized planning workloads. The original default remains
`0`, meaning no pre-reserve. A universal fixed allocation is not yet
justified for small maps, low node counts or short searches.

The **0 / 65,536 / 131,072 / 262,144** follow-up is now complete
on the same full map (results immediately below). Repeat it on
smaller/shorter workloads before choosing a possibly adaptive reserve
policy. A cache-friendlier
hash table or faster key hash is a separate future experiment, not
assumed beneficial without a measured comparison.

See the manual four-way instructions in
[`label_reserve_sweep_benchmark.md`](label_reserve_sweep_benchmark.md).

### 2026-11-04 follow-up: four-way reservation sweep COMPLETED

**New input files:** `nav2-label-reserve-sweep.csv` (80 rows, 10 routes x
2 footprints x 4 sizes, 5 timed repetitions per variant,
`PROFILE_HOTPATHS=OFF`) and `nav2-label-reserve-sweep-smoke.csv`
(16 rows, first 2 routes, 1 repetition). The new full run preserved
**Success**, path geometry, exact collision calls, expansion counts,
label/queue decisions and refinement results for all 20 matched
four-way comparisons. The experiment isolates reservation with the
fast empty-AABB shortcut enabled and clearance scoring disabled.

| Reserved labels | Median search p50 | Median paired search saving vs 0 | Wins vs 0 | Median rehashes | Median bucket estimate |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | 231.995 ms | baseline | — | 14.0 | 1.32 MiB |
| 65,536 | 228.479 ms | **2.06%** | 20/20 | 1.5 | 1.58 MiB |
| 131,072 | 226.000 ms | **3.54%** | 20/20 | 0.5 | 1.58 MiB |
| **262,144** | **221.622 ms** | **4.86%** | **20/20** | **0** | **2.12 MiB** |

**262,144 is faster than 131,072 in 20/20**, with **1.32% median
paired additional speedup** (median **2.99 ms/search**). It is also
faster than 65,536 and the baseline in all 20 matched routes. The
original 262,144-versus-zero result (**4.85%**) is replicated at
**4.86%** in the new independent full sweep.

- Median **paired** estimated bucket-array difference versus baseline:
  65,536 **-0.14 MiB**, 131,072 **+0.39 MiB**, 262,144 **+0.80 MiB**.
  These are medians of *paired differences* and need not equal the
  differences between median bucket footprints in the table.
- Median bucket-array cost of 262,144 versus 131,072: **+0.54 MiB**.
  Ten of twenty larger runs grew the 131,072 table to the *same*
  277,261 buckets as 262,144, but incurred one rehash to get there.
  Smaller runs avoided that rehash yet 262,144 still edged out 131,072,
  consistent with a lower table load factor improving hot-path lookup
  costs. This explanation is plausible, not a separately isolated
  causal benchmark.
- 131,072 had **0 rehashes in 10/20** cases and **1 in 10/20**.
  65,536 had 0 in 4/20, 1 in 6/20 and 2 in 10/20.
  The 262,144 setting had **0 in all 20**.
- Peak distinct state labels ranged from **42,700 to 179,519**.
  In the 10 smaller comparisons with at most 131,072 labels, the
  262,144-versus-131,072 median saving shrank to about **0.64 ms**
  (0.79% paired). In the 10 larger comparisons it was about
  **4.62 ms** (1.64% paired). A fixed large reservation therefore
  has a smaller payoff on shorter/light searches.

**Updated decision:** For the tested large warehouse, **262,144 is
the clearly fastest measured pre-reserve** and remains a good
*explicit* configuration. Retain `searchLabelReserve=0` as the
default pending short-route/small-map memory-and-latency tests;
do **not** claim a universal or validated adaptive reserve rule
from this single warehouse. Further options include an adaptive
rule informed by earlier runs or actual estimated search complexity,
and a separately benchmarked cache-friendly hash table or hash
function. Do not silently enable an automatic rule.

See [the four-way sweep protocol](label_reserve_sweep_benchmark.md)
and `tools/analyze_nav2_label_reserve_sweep.py` for reproducibility.

The subsequent [offline, map-stratified label reservation calibration](map_specific_label_reserve_calibration.md)
provides a seeded C++ function and manual CLI for estimating the smallest
no-rehash tier on **sampled routes for a particular map and configuration**.
It returns both an integer tier index and label capacity. It does not
supersede the measurements above or guarantee no rehash on unseen routes.

**Follow-up calibration result (32 routes, large differential footprint):**
`nav2-label-calibration-large-32.csv` identified 484,534 peak labels
and selected **tier 5 (1,048,576 labels)** with 20% requested
headroom. A direct replay with `--headroom 0` in
`nav2-label-calibration-large-32-tier4.csv` verified **tier 4
(524,288 labels)** with zero rehashes in **all 32/32 samples**,
and identical recorded results between the two CSVs. Thus **524,288 labels** is the smallest directly verified sampled
no-rehash tier (v1 tier 4 / v2 tier 13). Under the original v1 series,
**1,048,576 labels** was the 20%-headroom recommendation (v1 tier 5).
The refined **v2** series now has four capacity steps per doubling,
so the same 581,441-label headroom target selects **v2 tier 14 =
655,360 labels**. The completed **128-row, five-repeat-per-tier
matched benchmark** then verified zero rehashes and identical work at
**all four tested v2 tiers** (13/14/15/17). Tier 13 and tier 14
had essentially equal paired speed (16/32 wins each; ~0.00%
median paired delta), at **4.29 versus 5.03 MiB** estimated
bucket-array memory. Tier 17 was slower than tier 14 on 27/32
routes while using about **3.03 MiB more buckets**. Retain
**v2 tier 14 (655,360)** as the 20%-headroom recommendation for
the sampled large-footprint warehouse workload; tier 13 is an
acceptable tighter-memory option with less spare capacity. No claim
is made for other geometries, unseen routes or process RSS.
See the [calibration report](map_specific_label_reserve_calibration.md) and
[completed matched tier benchmark](refined_label_reserve_tiers_benchmark.md).

## 2. EDT-assisted capped clearance: route quality versus computational cost

**Input file:** `nav2-clearance-edt-smoke.csv` (12 rows; 2 routes x 2
geometries x 3 variants; **1 timed repetition each**). The earlier
`nav2-clearance-edt.csv` (60 rows) was **INVALID** as a performance
measurement: clearance-enabled native model requests were rejected
before planning. That policy/refinement restriction has since been
fixed. Use only the corrected smoke results here.

The three smoke variants were:
- `off`: `clearanceWeight=0` (normal search).
- `exact`: penalty weight **0.15**, cutoff radius **0.40 m**, exact
  orientation-aware, inflation-aware clearance calculation.
- `edt`: the exact same positive cost objective; an exact Euclidean
  distance transform proves the penalty is zero far from obstacles,
  while the ambiguous/nearby samples fall back to exact geometry.

The bounded penalty is `lambda * max(0,1 - d/R)^2`, integrated along
a path. Its zero-cost radius is **0.40 m of inflated-footprint
clearance**, not 0.40 m from the robot's reference point. The EDT
does not change the exact collision checker.

| Route / footprint | OFF search | Exact clearance search | EDT clearance search |
| --- | ---: | ---: | ---: |
| 0 / 70 x 50 cm | 352.06 ms | 1,886.78 ms | 1,255.42 ms |
| 0 / 50 x 30 cm + 10 cm | 316.50 ms | 1,772.11 ms | 1,159.22 ms |
| 1 / 70 x 50 cm | 312.47 ms | 1,933.14 ms | 1,441.47 ms |
| 1 / 50 x 30 cm + 10 cm | 278.87 ms | 1,846.80 ms | 1,348.35 ms |

- EDT versus **exact clearance**: **30.2% median paired search saving**,
  ranging from **25.4% to 34.6%** over four comparisons.
- EDT versus **clearance disabled**: still **3.57x–4.84x slower**.
- **3,108,113 of 3,893,592** EDT clearance samples (**79.8%**)
  skipped expensive exact clearance scans, with 785,479 fallbacks.
  These are *clearance scans*, not collision checks.
- EDT construction: **10.25–13.61 ms per search**, already included
  in search timings; resident EDT grid **13,472,352 bytes (12.85 MiB)**.
  The field is built per request in this experiment, not shared
  between search requests.
- Positive clearance cost yielded **3.3%–4.2% more A* expansions**
  and **13.9%–15.2% more exact collision-check calls** than OFF.
  It did **not** reduce the search/collision workload in this sample.
- All 12 rows succeeded. Exact and EDT variants had **identical**
  search work, geometry, raw weighted costs, clearance costs,
  refinement behavior and collision-call counts.

The independently sampled minimum and 10th-percentile inflated
footprint clearances improved, but depended strongly on the route:

| Route / geometry | Min clearance OFF -> EDT | P10 clearance OFF -> EDT |
| --- | ---: | ---: |
| 0 / 70 x 50 cm | 0.001 -> 0.128 m | 0.094 -> 0.259 m |
| 0 / 50 x 30 cm + 10 cm | 0.018 -> 0.107 m | 0.103 -> 0.308 m |
| 1 / 70 x 50 cm | 0.005 -> 0.005 m | 0.131 -> 0.136 m |
| 1 / 50 x 30 cm + 10 cm | 0.004 -> 0.005 m | 0.107 -> 0.144 m |

These are 0.2 m sampled indicators, **not continuous swept-footprint
minimum-clearance certificates**. The penalty does not preempt collision
validation: a generated motion is collision checked **before** its
clearance cost is evaluated. A different exploration order can therefore
increase, decrease, or leave unchanged the total collision calls.

### Decision

**Keep both clearance scoring and its EDT accelerator opt-in.** The
current evidence supports path-clearance improvements and a substantial
EDT reduction in exact clearance-scan cost, but **not a net planning
speedup versus clearance OFF**. Do not globally enable EDT or the
clearance cost from a two-route, one-repeat smoke test.

Next validation: all 10 warehouse routes with 3+ timed repetitions,
plus a penalty-weight and radius sensitivity study. Possible later
performance improvements include a cheaper near-obstacle exact-distance
fallback, a cached per-map EDT, or an explicitly approximate EDT-derived
cost with independent exact collision checking and separate path-quality
acceptance criteria.

See [`clearance_edt_benchmark.md`](clearance_edt_benchmark.md) for
implementation semantics and manual benchmark commands.

## Reproducibility and limits

- Benchmark output is **not** a claim of globally optimal paths,
  continuous swept-body collision safety or real-robot feasibility.
- Label reservation only changes allocation/performance; benchmark
  contracts require identical results and work.
- The EDT experiment deliberately changes the objective between OFF and
  positive weight. Only **exact versus EDT** must preserve path equivalence.
- Timing was measured on one host, with limited repetitions. Warmup,
  CPU scheduling, clock frequency and map workload may affect results.
- The repository's normal CI workflows were neither enabled nor run;
  all performance targets require explicit manual execution.
