# Hybrid A* Planning Lab — local demo application

This is a **local interactive browser application backed by the real C++20
Hybrid A* library**. It is not a browser-only pathfinding simulation.

The demo uses no UI framework, Node server or ROS installation.
For a fully provisioned Docker build including **Catch2 v3 and OMPL 2.0.2**, see [Docker setup](../../docs/docker.md). The native
executable handles planning, collision validation, optional refinement and
JSON serialization; the bundled HTML/CSS/JavaScript renders the map, route,
reverse motion, orientation and animated rectangular vehicle footprint.

## Build and run (OMPL optional)

From the repository root:

```bash
cmake -S . -B build-demo -DCMAKE_BUILD_TYPE=Release \
  -DHYBRID_ASTAR_BUILD_APP=ON -DHYBRID_ASTAR_ENABLE_OMPL=OFF
cmake --build build-demo --parallel --target hybrid_astar_demo
./build-demo/hybrid_astar_demo
```

Open **http://127.0.0.1:8765** in a local browser. The server binds only to
the IPv4 loopback interface; it does not expose a network service to other
machines. Stop it with **Ctrl+C**. You can choose a different local port:

```bash
./build-demo/hybrid_astar_demo --port 9000
```

With a **complete installed OMPL CMake package**, set
`-DHYBRID_ASTAR_ENABLE_OMPL=ON` alongside
`-DHYBRID_ASTAR_BUILD_APP=ON`. The UI then enables the
real OMPL Reeds–Shepp connector and optional OMPL-assisted refinement.
OMPL-disabled builds continue to use the exact six Ackermann primitives.

The focused [OMPL source bridge](../../tests/ompl_source_bridge/CMakeLists.txt)
tests the adapter but is **not an official OMPL package installation**.
It is not a substitute for the complete OMPL-ON CMake package build.

## Features

- Five reproducible static 50 × 40 m occupancy-map scenarios: open field,
  straight reverse, parking approach, warehouse obstacles, and a fully
  separated **blocked** map.
- Set start/goal world x, y (metres) and yaw (degrees); click on the map for
  the **start**, Shift-click for the **goal**. Angles are editable fields.
- Default obstacle-grid heuristic (noncertified performance ordering);
  uncheck the box to use the certified Euclidean heuristic,
- Strict optional post-path refinement for **Ackermann and differential drive**.
  Differential mode tries straight shortcuts and optional wheel-rate-feasible
  CSC arcs when the curved checkbox is enabled. Refinement never inserts
  new in-place rotations, avoiding unnecessary full stops.
  The entire candidate is independently checked against the rectangular body
  and swept stationary turns; if it does not strictly lower weighted cost, the
  validated original is retained. Mecanum refinement remains unsupported.
  bounded validated incumbent improvement, post-search refinement, and native
  curvature-bounded CSC shortcuts.
- Optional Reeds–Shepp analytic connections and combined refinement when
  OMPL support is compiled.
- Visualize exact analytic section trajectories in green for forward and
  amber for reverse. The animation shows the actual **4.6 × 1.8 m filled
  vehicle footprint** at a sampled position along the returned path.
- Overlay the original route when postprocessing returns a verified path.
  Show weighted cost, length, reverse length, expanded nodes, time, validation
  status and refinement outcome.
- Default poses are scenario-specific; custom poses replace the defaults
  until a new scenario is selected.

**Important:** The animation's display samples are *not* the collision
certification samples. Only success returned by the library's independent
strict adaptive-sampled validator can be labeled validated. There is **no
continuous swept guarantee or steering-rate/actuator feasibility** claim.
A budget-limited response is not proof of a geometric NoPath.

## Continuous metric footprint inflation

The Planning Lab now has **Geometric footprint safety margin** (checkbox,
off by default) and an **Inflation distance (m)** input. Enter any finite
nonnegative distance up to the demo's 10 m safety limit, such as `0.05`
or `0.1`. Unchecking inflation sets the effective margin to **zero**
without discarding the typed value. Click **Plan path** to apply changes.

The parameter is a **Euclidean offset** around the physical filled robot
rectangle (rounded corners). For every sampled robot pose the exact checker
tests continuous rectangle-to-occupied-square distance, including rotated
map origins and finite map boundaries. The occupied grid is **not dilated**,
and the margin does not round to whole map cells. The original robot dimensions
remain unchanged for animation, and an amber halo visualizes the extra zone
around its body.

The same margin applies to the start pose, search collision checks, strictly
rechecked goal paths, optional post-path refinement, and the conservative disc
prefilter. The prefilter's enclosing discs increase in radius by the margin,
so they cannot bypass inflated collisions. When the parameter is zero the
original SAT-only intersection fast path is retained. Higher margin values
can make narrow passages unreachable or increase search expansions.

HTTP GET/POST `/api/plan` accepts `inflate=1&inflationMeters=0.05`;
use `inflate=0` to turn off the feature without resetting the configured
value. JSON exports include the **effective** `footprintInflationMeters`.
The equivalent one-shot CLI flag is `--inflate 0.05`, for example:

```bash
./build-demo/hybrid_astar_demo --once --scenario warehouse \
  --model differential --inflate 0.05
```

The margin is **independent of** optional soft `clearanceWeight`: the former
is a hard sampled geometric rejection rule, the latter is a path-quality
penalty computed from the physical footprint. Exact metric offsets are
implemented for the repository's **current rectangular collision model**.
The reusable polygon-to-square distance routine admits future convex-body
collision implementations, but circles, concave bodies and multi-body
geometries are **not yet active** in the current planner. As before,
adaptive checking samples whole paths; it is not a mathematically continuous
swept-volume certificate.

## Choose the vehicle model — Ackermann, differential drive or mecanum

Planning Lab's **Kinematic model** selector chooses the existing
Ackermann rear-axle planner, ideal differential-drive chassis, or an ideal
four-wheel **mecanum** chassis. Mecanum has a compact center-referenced
rectangle, 14 cached cardinal/diagonal/combined-motion templates and
independent body-lateral travel even when heading is unchanged. Its
holonomic route segments are blue; spin markers remain violet.
Unsupported Ackermann-only Reeds–Shepp and shortcut options are disabled
for differential and mecanum models.
Differential mode uses a center-referenced **0.70 m × 0.50 m** rectangle,
eight precomputed local trajectories, and real **in-place rotations**.
Rotations have zero traveled metres but positive angular effort cost;
the browser animates their heading samples and displays a violet turn
marker at the fixed reference position.

```bash
./build-demo/hybrid_astar_demo --once --model differential \
  --scenario open --start 10 15 0 --goal 10 15 90
```

HTTP planning accepts `model=differential` or `model=ackermann` in
GET/POST, and export JSON includes `model`, `footprint` and the
`sections[].kind`/`sections[].turnRadians` fields. Sampled poses
remain map-frame world coordinates, including for rotated YAML origins.
Changing models invalidates the old path and triggers replanning.

Differential mode deliberately rejects Ackermann-only
`ompl`/`refine`/`curved` options and is not a ROS robot controller.
Both models independently validate pose-sampled full-footprint geometry
after search. **No continuous swept collision or actuator feasibility
certificate is supplied.** See
[W10 search integration](../../docs/architecture/w10-differential-search.md).

## Interactive obstacle editing (immutable map revisions)

Planning Lab can now edit the occupancy map **in the browser**, without
changing your original PGM/YAML file or built-in scenario:

1. Choose **Paint obstacle**, **Paint free**, **Paint unknown**, or
   **Restore original cell** in *Map interaction*.
2. Choose a 1-, 3-, 5- or 7-cell brush diameter and drag on the map.
   Drawing is in map-local cells even when the loaded YAML map is rotated.
3. The displayed old route is **immediately hidden** when a cell changes.
   On pointer release, the page sends the changed cells to the C++ planner
   and displays a path only after a fresh planning/strict-validation result.
4. **Reset map edits** restores the complete original map and requests a
   fresh plan with a new revision. Switch scenarios to clear pending edits.

The editor stores at most **4096 changed cells** (relative to the loaded
base snapshot). Requests use `POST /api/plan` with a maximum 64 KiB
URL-encoded body. The `edits` field is a canonical, ascending, unique list
`cellIndex:occupancy` (e.g., `12040:100,12041:-1`).
Permitted values are 100 blocked, 0 free, and -1 unknown;
`editRevision` is a monotonically increasing browser generation.
Even a full reset sends a new generation with an empty edit list.

Each request copies the original map into a **new immutable MapSnapshot**,
retaining frame/source and assigning `baseRevision + editRevision`.
The planner, its optional Reeds–Shepp adapter, strict final validation and
the refiner all receive that *same* snapshot. Earlier results remain pinned
to their old revisions; the browser never reuses their collision evidence
after a map change. The server does not persist edited maps between reloads.
The revision counter is request-supplied in this local demo, not a
multiuser transactional map database.

The prior **GET /api/plan** endpoint remains supported for older clients
and unedited examples. POST is only accepted for the planning endpoint with
`application/x-www-form-urlencoded` and an explicit bounded Content-Length.
Unsupported methods, duplicate/sortedness errors, out-of-bounds indices and
invalid occupancy values are rejected.

**Collision evidence remains adaptive sampled full-footprint geometry, not
a continuous swept-body or actuator feasibility certificate.**

## Import a PGM map and export a validated planning result

An application-provided **PGM P2 or P5** grayscale occupancy map can be loaded
at startup. This is a static, pinned C++ `MapSnapshot`; there is no browser
file-upload endpoint. The map is read once, rather than accepting arbitrary
filenames or data from an HTTP query.

```bash
./build-demo/hybrid_astar_demo --map apps/demo/maps/sample.pgm --resolution 0.5
# Browse http://127.0.0.1:8765, then select Imported map

./build-demo/hybrid_astar_demo --once \
  --map apps/demo/maps/sample.pgm --resolution 0.5 \
  --start 8 15 0 --goal 32 15 0 \
  --output /tmp/hybrid-plan.json
```

The browser exposes **Export path JSON** after a plan has completed; it
downloads the full output including original and optionally refined exact
path sections, gear, sampled visualization points, cost, planning diagnostics,
effective start/goal and map occupancy runs. This is *not* a timed trajectory
or a ROS Path publication.

**PGM semantics:** P2 (ASCII) and P5 (8-bit binary), grayscale maxval 1–255,
image **black = blocked** and **white = free** with a 50% threshold. For
maxval=255, the shade 205 is interpreted as **unknown** (-1), matching
common ROS map files. By default unknown cells are treated as occupied by
the planner. The first PGM row is at the **top** of the image; the loader
flips it to the map's bottom-left positive-Y convention. Resolution defaults
to 0.25 metres/pixel and is set with `--resolution`. Bare PGM imports use an origin of `(0,0,0)`; use `--map-yaml` for
translated or rotated map origins. Files are limited to 1024×1024 dimensions, at most **262,144 pixels**
and finite 0–2 metres/pixel resolution. The loader rejects malformed headers,
truncated data, excess pixels, and unsupported formats.

Imported map data is immutable for each request. Start and goal initial
positions are chosen at 20% and 80% of map width along its vertical midpoint;
they must be adjusted for maps where either footprint is blocked. Collision
checking is still done by the core filled-footprint checker.

### ROS-style YAML map metadata with rotated origins

The new `--map-yaml` option loads a map-server-style YAML description and
its referenced P2/P5 PGM raster. Relative `image:` paths resolve relative to
the YAML file, not the process's working directory:

```bash
./build-demo/hybrid_astar_demo \
  --map-yaml apps/demo/maps/sample.yaml

./build-demo/hybrid_astar_demo --once \
  --map-yaml apps/demo/maps/sample.yaml \
  --output /tmp/rotated-plan.json
```

The bundled sample uses a **90° map yaw**, a nonzero world translation and
`mode: trinary`. The C++ planner now initializes custom start/goal poses in
the **rotated world frame**, and the browser translates world-to-map
coordinates for occupancy rendering, path samples and click-to-place.
All pose fields remain world x/y in metres and yaw in **degrees** on the UI
and CLI. YAML `origin: [x, y, yaw]` instead uses **radians**, as in ROS.

Supported metadata: `image`, `resolution`, `origin`, `negate`,
`occupied_thresh`, `free_thresh`, and `mode: trinary`. The probability
model is `negate ? pixel/maxval : 1 - pixel/maxval`; values above
`occupied_thresh` become blocked, below `free_thresh` become free, and
intermediate values remain unknown/occupied by default. The legacy bare
`--map` PGM behavior remains unchanged.

This is a **restricted flat map YAML schema**, not a complete YAML parser:
multiline YAML, compressed/PNG raster formats, `scale`/`raw` modes and
external frame metadata are not accepted. A YAML map supplies its own
resolution, so `--resolution` cannot be combined with `--map-yaml`.
The user supplies the file at application startup; the HTTP API cannot
open an arbitrary server-side YAML path.

## CLI / simple JSON API

To calculate a path and print a JSON document without starting the server:

```bash
./build-demo/hybrid_astar_demo --once --scenario reverse
./build-demo/hybrid_astar_demo --once --scenario open \
  --start 10 15 0 --goal 19 16 20 --refine --curved
./build-demo/hybrid_astar_demo --once --scenario warehouse
./build-demo/hybrid_astar_demo --once --scenario warehouse --certified
./build-demo/hybrid_astar_demo --once --scenario warehouse --budget 60000
```

The web UI calls these **loopback-only** HTTP endpoints:

| Endpoint | Response |
| --- | --- |
| `/` | HTML application |
| `/style.css`, `/app.js`, `/map_geometry.js`, `/map_editor.js` | Bundled assets |
| `/api/capabilities` | JSON bool indicating OMPL availability |
| `GET /api/plan?scenario=reverse` | Backward-compatible request without edits |
| `POST /api/plan` | Bounded form-encoded plan with optional cell overrides and map revision |

Supported query fields: `scenario`, `startx`,
`starty`, `startyaw`, `goalx`,
`goaly`, `goalyaw`, `ompl`,
`refine`, `curved`, `performance`,
`improve`, `budget`, `edits`, `editRevision`. Boolean options accept `0` or
`1`. Yaw values in the HTTP and CLI interfaces are in
**degrees**. The actual C++ request is populated in radians.

This small developer server accepts GET and POST only for planning (POST
limited to 64 KiB form bodies, GET URLs to 2048 bytes), serves only its fixed
bundled assets and binds to
`127.0.0.1` by default. The explicit `--bind 0.0.0.0` option is for
container networking; publish the Docker host port to **127.0.0.1 only**. It has no authentication or production-grade
multiuser request scheduling and should **not** be internet-exposed.

## Raise the search budget

`BudgetExceeded` means the search stopped after spending its work limit. It is
not a geometric `NoPath` proof, so the first diagnostic is the limit itself:
every `/api/plan` and `--once` response reports the effective
`limits.maxExpansions`, `limits.maxGeneratedNodes` and
`limits.maxFinalPoseChecks`, and the web UI shows them as **Expansion limit**
next to **Expansions**.

The **default is now 1,000,000 expanded nodes** for all scenarios and for
the library API, with an independent ceiling of 4,000,000 generated nodes.
This permits long detours without requiring manual per-scenario overrides.
It is a work ceiling, not a promise of a path or a runtime limit. You can
lower the budget for interactive iteration:

- **Browser:** *Search budget* initially reads `1000000`; edit to any integer
  from `100` to `1000000`. Clearing the field keeps the library default.
- **CLI:** `--budget EXPANSED_NODES` (same `100..1000000` range).
- **HTTP:** `GET /api/plan?scenario=warehouse&budget=60000` or the same field in
  a `POST /api/plan` form body.

`maxGeneratedNodes` scales to at least `4x` the expanded-node budget.
The default heuristic is now obstacle-grid with weight 1.0 for all three
chassis models. The Planning Lab checkbox starts checked. Uncheck it, pass
`performance=0` through the HTTP API or pass `--certified` via CLI to
select certified Euclidean guidance. The historical `--performance` /
`performance=1` inputs remain supported.
High expansion ceilings can consume significant CPU and RAM, especially
on impossible maps. Prior performance measurements on older versions do
not characterize the new holonomic terminal connector or sampling policy.
Out-of-range or non-integer budgets are rejected with `400` / exit code 2.

For differential drive, `refine=1` enables cost-reducing straight chords,
without inserting stationary rotations; `curved=1` also tries native CSC
arcs using a 0.75 m candidate
radius (not an Ackermann minimum turning radius). The ideal differential
drive model may time-scale each feasible curvature to respect wheel speed
limits; acceleration and dynamic smoothness are not certified. The refiner
independently checks the full route, including swept in-place turns, with a
bounded check budget (200,000 poses by default in Planning Lab), and always
preserves the original strictly validated route when it cannot improve it.
The browser checkbox settings apply equally to Ackermann and differential
drive; Reeds–Shepp and refinement remain unsupported for mecanum.

Other quality/effort levers are orthogonal: `improve=1` keeps searching
for a better incumbent after the first validated path, and `refine`/`curved`
post-process an already validated path. The obstacle-grid heuristic changes
search ordering, not full-footprint validation or any safety certification.

## Manual verification

```bash
cmake -S . -B build-demo -DCMAKE_BUILD_TYPE=Release -DHYBRID_ASTAR_BUILD_APP=ON
cmake --build build-demo --parallel
ctest --test-dir build-demo --output-on-failure
```

The named C++ `demo_test` covers forward/reverse results,
strict evidence, optional refinement, invalid scenarios, blocked routes and
machine-readable errors. If Python 3 is available on POSIX systems,
`demo_http_test` also starts the actual localhost server
on an ephemeral port, checks static resources and headers, plans a reverse
and warehouse route, rejects malformed requests and tests HTTP method
enforcement. No GitHub Actions triggers are added.

## Code

- `apps/demo/map_io.{hpp,cpp}`, `apps/demo/map_yaml.{hpp,cpp}`: PGM raster and restricted map-server YAML import
- `apps/demo/demo.hpp`, `apps/demo/demo.cpp`: scenario
  creation, synchronous planner/refiner orchestration and JSON output
- `apps/demo/main.cpp`: native command line and loopback HTTP server
- `apps/demo/web/`: static dependency-free HTML/CSS/JavaScript
- `tests/demo_test.cpp` and `tests/demo_http_test.py`:
  local end-to-end regression checks

The demo ships built-in maps and local PGM/YAML imports. Reading arbitrary
ROS OccupancyGrid messages or publishing ROS paths remains future work.
