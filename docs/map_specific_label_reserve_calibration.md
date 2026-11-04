# Map-specific A* state-label reservation calibration

## Purpose and output

`calibrateLabelReserve(prototype, settings)` is a **public, offline,
opt-in C++ function** in
`include/hybrid_astar/ackermann/label_reserve_calibration.hpp`.
It measures how many **simultaneously active state labels** Hybrid A* actually
stores while exploring deterministic, spatially stratified, randomly jittered
start/goal pairs on an immutable occupancy grid. It returns **both**:

- `recommendedTierIndex`: a stable index into the reservation series;
- `recommendedReserveLabels`: the exact
  `PlanOptions.searchLabelReserve` value to pass to future requests.

| Tier index | Reserved labels |
| ---: | ---: |
| 0 | 0 — original growing-table baseline, **not** a zero-rehash certificate |
| 1 | 65,536 |
| 2 | 131,072 |
| 3 | 262,144 |
| 4 | 524,288 |
| 5 | 1,048,576 |
| 6 | 2,097,152 |
| ... | Doubling thereafter, within machine/budget limits |

The function selects the **smallest nonzero tier** above the largest
`diagnostics.labelPeakSize` observed, including a configurable
`headroomFraction` (default **20%**). The tier must be no larger than
`PlanOptions.maxGeneratedNodes` because the planner rejects larger
explicit pre-reserves. Otherwise calibration **fails clearly**, rather
than silently using zero reserve or exceeding the work budget.

Example for consuming the result:

```cpp
using namespace hybrid_astar::ackermann;
PlanRequest templateRequest = /* map, model, footprint, options */;
LabelReserveCalibrationOptions settings;
settings.routeSamples = 32;
settings.seed = 20261104;
settings.headroomFraction = .20;
const auto calibration = calibrateLabelReserve(templateRequest, settings);
if (!calibration.sampleTargetMet || !calibration.verificationPassed) {
  throw std::runtime_error("Calibration is incomplete or not verified");
}
std::cout << "tier=" << calibration.recommendedTierIndex
          << " reserve=" << calibration.recommendedReserveLabels << "\n";

// Store with map fingerprint and the SAME model/options configuration.
// Apply only to future requests using the same map and search settings:
futureRequest.options.searchLabelReserve =
    calibration.recommendedReserveLabels;
```

**The planner default remains `searchLabelReserve=0`.** Nothing in
`plan()` selects the tier automatically, and no background task/CI job
calibrates maps on deployment.

## Measured 32-route Nav2 warehouse calibration (2026-11-04)

The manually executed `nav2-label-calibration-large-32.csv` covers
**32 differential-drive, large (70 x 50 cm) footprint routes** with
seed `20261104`. The earlier four-route smoke CSV is not a sufficient
capacity estimate for the same map.

- **32/32 Success**, no budget-limited rows; **32/32 sampled replay
  rehash counts = 0** at the selected tier.
- **32 distinct start strata, 28 distinct goal strata, 60 distinct
  start-or-goal strata**; these are distinct endpoint regions, **not**
  proof of traversability/coverage of every free map cell.
- Observed peak live-label counts: **6,368 min, 100,553 median,
  347,005 p90, 411,561 p95, 484,534 maximum**. The hardest
  case was **sample 24**: 478,906 expansions, 653,630 generated
  nodes and 484,534 peak labels. With a 500,000-expansion budget,
  this case already used **95.8%** of the permitted expansions.
- With **reserve 0**, the median label-table rehash count was **14**
  (range 10–16). On the verification replays, every reported
  `verified_rehashes` value was **0**.
- **Tier 4 = 524,288** is the smallest tier able to fit all
  **observed** peaks, because none exceed 524,288.
  **Direct verification is now complete:** the second file,
  `nav2-label-calibration-large-32-tier4.csv`, reran the same
  32 routes with `--headroom 0`. All 32 reported `Success`,
  every verification had **zero rehashes**, and **all recorded CSV
  fields matched** the earlier tier-5 run (including poses, status,
  expansions, generated nodes and peak labels).
- With the default **20% headroom**, the selection target becomes
  `ceil(484534 * 1.20) = 581441`, so the calibration returns
  **tier 5 = 1,048,576** reserved labels. Two of 32 samples
  (indices **3** and **24**) exceed
  `floor(524288 / 1.20) = 436906`, pushing the headroom
  target beyond tier 4.
- The four-route smoke maximum had been only **367,653** live
  labels (tier 4 with 20% headroom); the larger sample found
  a **32% higher** maximum and thus selected a higher tier.

**Interpretation:** **Tier 4 is now the smallest directly
verified zero-rehash tier** for these 32 sampled routes. At the
484,534-label peak it leaves **39,754 unused reserved entries**,
about **7.58% of the tier-4 capacity** (8.20% relative to observed
peak). Tier 5 remains the **20%-headroom policy recommendation**,
not the smallest observed no-rehash requirement. Both verified tiers
preserved identical work and outcomes on all 32 sampled requests.

This establishes **neither tier's runtime superiority**: the calibration
CSV contains no timed search measurements or peak RSS. Tier 5 reserves
524,288 more labels up front than tier 4; compare their measured
latency and memory consumption under the *same 32 sampled routes*
before choosing a deployment setting. The discrete doubling sequence
can allocate much more capacity than the headroom target; an optional
**non-tier reserve of 581,441** is another possible future timing and
memory experiment, not a tested recommendation.

The result applies only to the tested large footprint and 500k/2M
work budgets. Run the corresponding 32-route calibration for
`--geometry small-inflated` as well; and consider broader seeds or
harder routes, particularly because sample 24 approached the expansion
budget. No universal zero-rehash guarantee is established.

## Spatially randomized but reproducible route coverage

The generator works in the **map-local metric frame**, so translated
and rotated ROS map origins are supported:

1. Partition the map into a roughly aspect-ratio-aware grid of about
   twice as many strata as requested routes, capped at 4,096 strata.
   `strataX` and `strataY` may override this for thin/complex maps.
2. Inside every stratum, sample jittered positions and random headings
   using a seeded SplitMix64 stream. Accept only poses whose **whole
   inflated/rotated footprint** passes the normal exact collision checker
   and occupancy/unknown-cell policy. Try at most 48 poses per stratum
   by default; occupied/too-narrow strata remain marked unavailable.
3. Deterministically shuffle the valid strata, rotate start positions,
   and select goals from geographically separated strata. Reject pairs
   closer than the configured minimum (by default 15% of map diagonal)
   and duplicate **ordered** stratum pairs.
4. Run real native `plan()` with reserve **0** for each accepted pair.
   Include successful routes and hard, unsuccessful, budget-limited
   routes in the observed peak-label maximum. Log the status,
   expansions, generated nodes, peak labels and natural rehashes.
5. Select the smallest tier above `ceil(maxPeak * (1 + headroom))`.
   By default, **replay every identical start/goal with that tier** and
   require **zero rehashes** plus unchanged path, expanded/generated
   work, collision calls and label operations. A verification mismatch
   aborts the calibration.

The result reports `validStrata`, `coveredStartStrata`,
`coveredGoalStrata`, `coveredEitherStrata` (unique map regions reached
by at least one start or goal), `successfulRoutes`,
`budgetLimitedRoutes`, `maxObservedPeakLabels`,
`sampleTargetMet`, and `verificationPassed`.
A stable FNV-1a `mapFingerprint` of geometry + occupancy values is
also returned; the map's revision alone need not be globally unique.

The generator fails if it cannot produce the requested number of
**distinct collision-free stratum pairs**. It never quietly downgrades
a requested 32-pair calibration to a three-pair trial.
Many consecutive failed searches or calibration runs that mostly hit the
search budget warrant separate analysis: **peak labels may grow further
when that budget is raised**.

### Important limitations

This is an **empirical map- and configuration-specific calibration**,
not a proof that every possible future start/goal path can never rehash.
Even with complete stratification, sample size and orientation coverage
are finite. Recalibrate if map cells/resolution/origin, occupancy threshold,
unknown-cell policy, robot geometry/inflation, motion primitives, cost model,
search configuration, heuristic or work budgets change.

Choosing a larger reserve eliminates rehashing only on routes whose
actual active-label count fits the capacity. It also allocates buckets
up front, increasing memory and sometimes slowing tiny searches.
The observed zero-rehash property is verified on the *sampled* routes,
not on arbitrary future routes. The calibration itself runs every route
twice by default; it is intentionally **offline and potentially
expensive**, never a hidden latency cost of routine planning.

The earlier 10-route warehouse sweep found 262,144 fastest (4.86% median
paired saving versus no reserve, 20/20 wins). This new calibration is a
way to estimate an appropriate tier for a **different map**;
it is **not** a replacement for a latency-versus-memory sweep.

## Manually run on the warehouse map (28 compilation jobs)

Use the **same budgets as real production requests** when estimating
reservations. The previous Nav2 warehouse experiment allowed
500,000 expansions and 2,000,000 generated nodes; reducing those
budgets can artificially cap peak labels and produce an unsafe
underestimate for more demanding future plans.

From the repository root:

```bash
git switch feature/nav2-equivalent-envelope-footprint-benchmark
git pull --ff-only
export BUILD_JOBS=28 TEST_JOBS=4 PROFILE_HOTPATHS=OFF
docker compose build test

docker compose --profile test run --rm test \
  ctest --test-dir build-docker --output-on-failure -j 4 \
  -R '^(label_reserve_calibration_test|hotpath_test|w10_differential_search_test|w12_mecanum_test)$'
```

Short functional smoke calibration (four routes, each planned twice):

```bash
docker compose --profile test run --rm test \
  ./build-docker/hybrid_astar_label_reserve_map_calibration_benchmark \
  --map apps/demo/maps/nav2_warehouse_full.yaml \
  --model differential --geometry large \
  --samples 4 --seed 20261104 --fast-empty-aabb \
  --max-expansions 500000 --max-generated 2000000 \
  > nav2-label-calibration-smoke.csv
```

For broader map coverage, sample **32 routes** and preserve the
seed, model, geometry, budgets and search options for later comparison:

```bash
docker compose --profile test run --rm test \
  ./build-docker/hybrid_astar_label_reserve_map_calibration_benchmark \
  --map apps/demo/maps/nav2_warehouse_full.yaml \
  --model differential --geometry large \
  --samples 32 --seed 20261104 --headroom 0.20 \
  --max-expansions 500000 --max-generated 2000000 \
  --grid-settlements 100000 --fast-empty-aabb \
  > nav2-label-calibration-large.csv

docker compose --profile test run --rm test \
  ./build-docker/hybrid_astar_label_reserve_map_calibration_benchmark \
  --map apps/demo/maps/nav2_warehouse_full.yaml \
  --model differential --geometry small-inflated \
  --samples 32 --seed 20261104 --headroom 0.20 \
  --max-expansions 500000 --max-generated 2000000 \
  --grid-settlements 100000 --fast-empty-aabb \
  > nav2-label-calibration-small.csv
```

The CLI writes a per-route CSV to **stdout** (redirected above to the
host); it reports the selected tier, reserved labels, coverage,
peak labels, map fingerprint and verification status to **stderr**.
To obtain a JSON summary on the host, mount a writable destination
because a normal `docker compose run --rm` container is deleted:

```bash
mkdir -p bench/results
docker compose --profile test run --rm \
  -v "$PWD/bench/results:/results" test \
  ./build-docker/hybrid_astar_label_reserve_map_calibration_benchmark \
  --map apps/demo/maps/nav2_warehouse_full.yaml \
  --model differential --geometry large \
  --samples 32 --fast-empty-aabb \
  --summary-json /results/label-calibration-large.json \
  > nav2-label-calibration-large.csv
```

The resulting file at `bench/results/label-calibration-large.json`
contains the tier, label count, seed, map fingerprint, geometry, headroom,
work budgets, coverage and verification flag. To calibrate a different
ROS map, pass its YAML to `--map`; `--geometry large|small-inflated`
selects the two benchmark chassis profiles. For custom geometry or
policy settings, use the **C++ function directly** with a custom
`PlanRequest`. The suggested reserve for a fleet that uses both
profiles must cover the **maximum** required tier across the profiles.

## Follow-up validation

Run the earlier four-way throughput benchmark on short searches and
small maps; measure whether pre-reserving the inferred tier actually
improves latency after counting up-front hash-bucket allocation.
Use `PROFILE_HOTPATHS=OFF` for these comparisons. Do not change the
planner's global default without those results.

No GitHub Actions workflow is triggered, dispatched or modified by
this feature. The map calibration CLI is deliberately NOT registered
with CTest.
