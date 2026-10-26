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
- Optional obstacle-grid heuristic (noncertified performance ordering),
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

## Import a PGM map and export a validated planning result

An application-provided **PGM P2 or P5** grayscale occupancy map can be loaded
at startup. This is a static, pinned C++ `MapSnapshot`; there is no browser
file-upload endpoint. The map is read once, rather than accepting arbitrary
filenames or data from an HTTP query.

```bash
./build-demo/hybrid_astar_demo --map apps/demo/maps/sample.pgm --resolution 0.5
# Browse http://127.0.0.1:8765, then select Imported PGM map

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
to 0.25 metres/pixel and is set with `--resolution`. Origin and yaw are
currently fixed to `(0,0,0)`; support for ROS map YAML metadata is future
work. Files are limited to 1024×1024 dimensions, at most **262,144 pixels**
and finite 0–2 metres/pixel resolution. The loader rejects malformed headers,
truncated data, excess pixels, and unsupported formats.

Imported map data is immutable for each request. Start and goal initial
positions are chosen at 20% and 80% of map width along its vertical midpoint;
they must be adjusted for maps where either footprint is blocked. Collision
checking is still done by the core filled-footprint checker.

## CLI / simple JSON API

To calculate a path and print a JSON document without starting the server:

```bash
./build-demo/hybrid_astar_demo --once --scenario reverse
./build-demo/hybrid_astar_demo --once --scenario open \
  --start 10 15 0 --goal 19 16 20 --refine --curved
./build-demo/hybrid_astar_demo --once --scenario warehouse --performance
```

The web UI calls these **loopback-only** GET endpoints:

| Endpoint | Response |
| --- | --- |
| `/` | HTML application |
| `/style.css` and `/app.js` | Bundled assets |
| `/api/capabilities` | JSON bool indicating OMPL availability |
| `/api/plan?scenario=reverse` | Original result, refinement if requested, and compact occupancy runs |

Supported query fields: `scenario`, `startx`,
`starty`, `startyaw`, `goalx`,
`goaly`, `goalyaw`, `ompl`,
`refine`, `curved`, `performance`,
`improve`. Boolean options accept `0` or
`1`. Yaw values in the HTTP and CLI interfaces are in
**degrees**. The actual C++ request is populated in radians.

This small developer server intentionally accepts only GET, permits a maximum
URL of 2048 bytes, serves only three fixed page assets and binds to
`127.0.0.1` by default. The explicit `--bind 0.0.0.0` option is for
container networking; publish the Docker host port to **127.0.0.1 only**. It has no authentication or production-grade
multiuser request scheduling and should **not** be internet-exposed.

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

- `apps/demo/demo.hpp`, `apps/demo/demo.cpp`: scenario
  creation, synchronous planner/refiner orchestration and JSON output
- `apps/demo/main.cpp`: native command line and loopback HTTP server
- `apps/demo/web/`: static dependency-free HTML/CSS/JavaScript
- `tests/demo_test.cpp` and `tests/demo_http_test.py`:
  local end-to-end regression checks

The current demo ships built-in maps only; reading arbitrary PGM/ROS
OccupancyGrid files or writing ROS paths is a later application milestone.
