# Representative passenger-car fixture (first Ackermann tests)

**Status:** Accepted as a repeatable **test fixture**, not a general vehicle default or a claim of exact model identification.

## Grounding

Manufacturer-published turning circles for ordinary compact cars are approximately 11 m **diameter**. Example: Volkswagen Golf owner's manual lists a 10.9 m turning-circle diameter, 2,637 mm wheelbase and 4,255 mm length (https://www.vwgolf.org/dimensions-47.html). Toyota's 2025 Corolla comparison lists a 35.6 ft turning circle (about 10.85 m), 106.3 in wheelbase (about 2.70 m), 182.5 in length (about 4.64 m), and 70.1 in width (about 1.78 m): https://www.toyota.com/corolla/2025/compare/.

**Crucial distinction:** the published turning circle describes an **outer tire/curb-to-curb vehicle sweep**, not the minimum turning radius of the rear-axle centerline used by our kinematic bicycle model. Its half-diameter is therefore **not automatically the numerical `R_min`**. Using a front-wheel track of about **1.55 m** and wheelbase around **2.7 m**, simple turn-center geometry gives **3.9–4.0 m** for the rear-axle center radius when outer front-wheel turning diameter is 10.85–10.9 m: `R_rear ≈ sqrt((D_turn/2)^2 - wheelbase^2) - track/2`. The approximation neglects tire width, slip and differences in manufacturer's turning-circle measurement methods. We therefore select an illustrative **rear-axle-centerline `R_min=4.0 m`**, rather than halving the published turning-circle diameter.

## First test fixture (meters)

| Quantity | Fixture value |
| --- | --- |
| Model | Geometric 3-state Ackermann bicycle, rear axle at local origin, heading +x |
| Rear-axle minimum turning radius | **4.0 m** |
| Wheelbase | **2.7 m** |
| Body footprint | **4.6 m × 1.8 m**, rigid rectangle |
| Local body shape bounds | x ∈ **[-1.0, +3.6] m**, y ∈ **[-0.9, +0.9] m** relative to rear-axle center |
| Map resolution | **0.20 m** sample occupancy map |
| Primitive distance / samples | 0.50 m / 0.10 m, six poses |
| Search key | 0.25 m XY / **5° yaw (72 bins)** / incoming gear |
| Goal | XY≤0.25 m; yaw≤10° (independent from search bins) |
| Analytic attempt | OMPL RS on selected nodes within **20.0 m** XY; **no maximum path-length cutoff** |

Derived illustrative check: at maximal curvature, a 0.50 m primitive changes yaw by `L/R_min=0.125 rad ≈ 7.16°`, exceeding one 5° bin width. Equivalent max front steering angle in the bicycle convention is `atan(wheelbase/R_min)≈34.0°`. Vehicle physical dimensions/turning performance are only approximate test parameters. The rectangle extends 1.0 m rearward from the rear axle and 3.6 m forward.

The broad-phase all-heading circular envelope for this footprint has radius `sqrt(3.6² + 0.9²) ≈ 3.71 m`; this may flag a large fraction of an occupied map, so profile the broad-phase effectiveness. Changing the physical footprint, wheelbase or R_min must not silently preserve incompatible primitive/geometry caches.

## Verification

Test right/left/straight and forward/reverse RK4 primitive endpoints, 72 heading-bin alias rates over subcell starts, full rectangle placement and grid rastering, near-goal RS continuity and collisions, and collision-candidate EDT flagging with the 3.71 m envelope. Re-run with smaller/larger turning radii for sensitivity; these are fixtures, not presets for all clients.
