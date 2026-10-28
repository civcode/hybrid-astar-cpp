# syntax=docker/dockerfile:1
# Reproducible Linux development/test/demo environment. No GitHub Actions.
# Catch2 v3.16.1 is downloaded by CMake FetchContent by default; OMPL is
# built from the exact 2.0.2 upstream tag for signed-path integration.
FROM ubuntu:24.04 AS dependencies
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build pkg-config git \
    ca-certificates curl tar python3 nodejs \
    libeigen3-dev libboost-all-dev \
    libflann-dev libode-dev libyaml-cpp-dev \
  && rm -rf /var/lib/apt/lists/*

ARG OMPL_VERSION=2.0.2
# Maximum parallel compiler processes for both OMPL and Hybrid A* builds.
ARG BUILD_JOBS=2
RUN curl --fail --location --retry 3 \
      "https://github.com/ompl/ompl/archive/refs/tags/${OMPL_VERSION}.tar.gz" \
      -o /tmp/ompl.tar.gz \
  && mkdir -p /tmp/ompl-src \
  && tar -xzf /tmp/ompl.tar.gz -C /tmp/ompl-src --strip-components=1 \
  && cmake -S /tmp/ompl-src -B /tmp/ompl-build -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DOMPL_BUILD_VAMP=OFF \
      -DOMPL_BUILD_PYTHON_BINDINGS=OFF \
  && cmake --build /tmp/ompl-build --parallel "${BUILD_JOBS}" \
  && cmake --install /tmp/ompl-build \
  && ldconfig \
  && rm -rf /tmp/ompl.tar.gz /tmp/ompl-src /tmp/ompl-build

FROM dependencies AS build
# Redeclare the inherited build argument in this build stage.
ARG BUILD_JOBS
# Profiling is opt-in because per-operation clocks change runtimes.
ARG PROFILE_HOTPATHS=OFF
# Opt-in only: normally keep the expensive 100-path benchmarks out of CTest.
ARG REGISTER_REFINEMENT_BENCHMARK_TESTS=OFF
WORKDIR /workspace
COPY . /workspace
RUN cmake -S . -B build-docker -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DHYBRID_ASTAR_ENABLE_OMPL=ON \
      -DHYBRID_ASTAR_BUILD_TESTS=ON \
      -DHYBRID_ASTAR_BUILD_APP=ON \
      -DHYBRID_ASTAR_BUILD_BENCHMARKS=ON \
      -DHYBRID_ASTAR_REGISTER_REFINEMENT_BENCHMARK_TESTS="${REGISTER_REFINEMENT_BENCHMARK_TESTS}" \
      -DHYBRID_ASTAR_PROFILE_HOTPATHS="${PROFILE_HOTPATHS}" \
  && cmake --build build-docker --parallel "${BUILD_JOBS}"

# Build with --target test to execute CTest / Catch2 (including OMPL tests).
FROM build AS test
ENV CTEST_OUTPUT_ON_FAILURE=1
CMD ["ctest", "--test-dir", "build-docker", "--output-on-failure"]

# Default image starts the interactive planning app.
# Only the container interface is open; publish the HOST port to loopback.
FROM build AS demo
RUN useradd --system --uid 10001 --home-dir /workspace --no-create-home planner
USER planner
EXPOSE 8765
CMD ["./build-docker/hybrid_astar_demo", "--bind", "0.0.0.0", "--port", "8765"]
