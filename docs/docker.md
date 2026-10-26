# Docker development, testing and Planning Lab demo

The root [Dockerfile](../Dockerfile) provides a reproducible Ubuntu 24.04
environment for **building, testing and running** the real C++20 Hybrid A*
planner. Dependencies include GCC, Git, CMake, Ninja, Python 3,
Eigen3, Boost, FLANN, ODE and yaml-cpp. **Catch2 v3.16.1 is built by
CMake FetchContent by default**, not installed from Ubuntu's packages.
It builds **OMPL 2.0.2 from its
official tagged source**, with VAMP and Python bindings disabled; this avoids
accidentally linking an older distro OMPL version.

Docker and network access to the upstream source tarball are required on the
host for the initial build. The OMPL archive previously supplied for local
debugging is **not** copied into the repository or Docker context.

## Start the demo (local browser)

```bash
docker compose up --build demo
```

Open **http://127.0.0.1:8765**. Docker Compose explicitly publishes the host
port only on **127.0.0.1**. The executable inside the container must listen
on its container interface, so the Docker image uses
`hybrid_astar_demo --bind 0.0.0.0` (an explicit opt-in to binding beyond
loopback). On bare metal, the default remains 127.0.0.1.

Stop with Ctrl+C, then `docker compose down` if desired.

Without Compose:

```bash
docker build --target demo -t hybrid-astar:demo .
docker run --rm -p 127.0.0.1:8765:8765 hybrid-astar:demo
```

**Do not** publish `-p 8765:8765` on a public interface. The demo server
is a single-user development tool with no authentication. It is not a
production web service or robot motion controller.

## Demo with an imported PGM map

The Docker image includes a [sample PGM occupancy map](../apps/demo/maps/sample.pgm),
and the browser can display any P2/P5 map loaded at server startup:

```bash
docker build --target demo -t hybrid-astar:demo .
docker run --rm -p 127.0.0.1:8765:8765 hybrid-astar:demo \
  ./build-docker/hybrid_astar_demo --bind 0.0.0.0 \
  --map apps/demo/maps/sample.pgm --resolution 0.5
```

To use your own file without rebuilding the image, mount a trusted directory
read-only and pass its **container path**:

```bash
docker run --rm -p 127.0.0.1:8765:8765 \
  -v "$PWD/maps:/imported:ro" hybrid-astar:demo \
  ./build-docker/hybrid_astar_demo --bind 0.0.0.0 \
  --map /imported/my-map.pgm --resolution 0.25
```

The browser automatically selects **Imported map** when available. Its
**Export path JSON** button downloads the actual core planner results to your
browser, not a file written inside the container. CLI JSON export remains
available with `--once --output FILE.json`, where FILE refers to the
container filesystem unless a writeable host volume is mounted.

PGM maps use top-row-first raster order, 0=obstacle, white=free and the
grayscale value 205 as unknown for maxval=255. The default occupancy policy
treats unknown as occupied. No remote map-upload endpoint is enabled.

### Load a ROS map YAML descriptor

The image includes [`sample.yaml`](../apps/demo/maps/sample.yaml), which
references the sibling `sample.pgm` file and uses a 90° rotated world origin.

```bash
docker run --rm -p 127.0.0.1:8765:8765 hybrid-astar:demo \
  ./build-docker/hybrid_astar_demo --bind 0.0.0.0 \
  --map-yaml apps/demo/maps/sample.yaml
```

When mounting your own map files, mount the **YAML and PGM together** so
relative image paths resolve within the container. The browser map, vehicle
headings and click placement use the YAML translation/yaw. The Docker image
also includes Node.js for an optional coordinate round-trip CTest.
This restricted map YAML reader supports P2/P5 and `mode: trinary` only.

## Run Catch2 + CTest (explicit manual action)

```bash
docker compose run --build --rm test
```

Or:

```bash
docker build --target test -t hybrid-astar:test .
docker run --rm hybrid-astar:test
```

This executes the C++ test suites under **Catch2 v3** (including actual
`TEST_CASE`, `SECTION`, `REQUIRE` assertions) via CTest, the native HTTP
demo smoke (Python 3), and OMPL-enabled signed-path/integration tests.
Tests are **not automatically run on Docker image build**; the `test`
service is under a Compose profile and has to be requested explicitly.
The optional [GitHub Actions verification workflow](ci/manual-verification.md)
uses `workflow_dispatch` only; no workflow run is dispatched automatically.

To run the planner directly in command-line JSON mode:

```bash
docker run --rm hybrid-astar:demo \
  ./build-docker/hybrid_astar_demo --once --scenario reverse --ompl
```

Optional native benchmark executables are also built in the image:

```bash
docker run --rm hybrid-astar:demo \
  ./build-docker/hybrid_astar_planner_bench 3
docker run --rm hybrid-astar:demo \
  ./build-docker/hybrid_astar_refinement_bench 3
```

Benchmarks are manual; they are not stable product latency promises.

## Build without Docker

Catch2 **v3.16.1 is fetched automatically by CMake** when tests are enabled
(the default). The initial configuration requires Git and HTTPS access to
`github.com/catchorg/Catch2`; later configurations reuse the downloaded
FetchContent source tree. An installed Catch2 package is not required.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DHYBRID_ASTAR_BUILD_TESTS=ON \
  -DHYBRID_ASTAR_BUILD_APP=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

To use an already installed Catch2 v3 CMake package instead (for offline
or distribution builds):

```bash
cmake -S . -B build-system-catch2 \
  -DHYBRID_ASTAR_USE_SYSTEM_CATCH2=ON
cmake --build build-system-catch2 --parallel
ctest --test-dir build-system-catch2 --output-on-failure
```

Alternatively, keep FetchContent enabled and supply an **existing** unpacked
Catch2 v3.16.1 source checkout with
`-DFETCHCONTENT_SOURCE_DIR_CATCH2=/absolute/path/to/Catch2`. This is
useful for offline builds and does not change the default behavior.

For a dependency-minimal **core + app only** build, disable tests explicitly
(`-DHYBRID_ASTAR_BUILD_TESTS=OFF`); this avoids requiring Catch2. Likewise,
`-DHYBRID_ASTAR_ENABLE_OMPL=OFF` omits OMPL integration. Those two options
do not alter the project's geometric safety/verification semantics.

## Catch2 migration notes

Earlier executable regression tests used C++ `main()` with `throw` or
numeric error codes. That was useful for rapidly bootstrapping a
no-third-party-dependency vertical slice, **but it deviated from the documented
Catch2 v3 contract**, and the documentation had not clearly labeled this
as a temporary compromise.

All existing C++ tests now run under `Catch2::Catch2WithMain`; their
pre-existing assertions and coverage are retained in registered test cases.
New `tests/catch2_contract_test.cpp` directly uses
`TEST_CASE`/`SECTION`/`REQUIRE` to exercise exact arc and strict validation
properties. Migrating individual legacy assertions to more granular
expression-level Catch2 messages is worthwhile ongoing test cleanup, not a
precondition for using the real Catch2 runner.

## Verification limitations

These Docker paths have not been executed from this assistant environment
because the Docker daemon and HTTPS access needed for the default Catch2
FetchContent download are unavailable here. They require a first on-machine
Docker build/test to confirm OMPL's
full package installation and all imported CMake targets. The earlier local
OMPL 2.0.2 source-subset integration and sanitizer test evidence remain
separate and do not substitute for that Docker verification.

Even a passing Docker build does not certify continuous swept collision
freedom, actuator feasibility, optimality or safety for a real vehicle.
