# ADR-0007: Scoped dependencies and verification-first development

**Status:** Accepted  
**Date:** 2026-10-24

## Context

The project prioritizes high-performance standalone C++ and independent validation, with specific supporting-library preferences.

## Decision

Prefer **Catch2 v3** for tests, **cxxopts** for CLIs, **nlohmann/json** and **yaml-cpp** for external configuration, **Eigen3** where beneficial, and **Ceres Solver** for optional nonlinear least-squares refinement. Do not expose testing, config, CLI, optimization or OMPL symbols in core search APIs. Implement all project-specific motion-planning numerical methods natively. Design test/benchmark contracts alongside architecture, run them locally, and keep any future GitHub Actions workflows **manual-only** unless the user explicitly requests automated triggers.

## Alternatives considered

A monolithic dependency graph or outsourcing hot-path path-planning math to external libraries.

## Consequences

More carefully factored CMake targets and adapters; maintain a slim search runtime and reproducible local validation.

## Follow-up

Decide supported dependency versions, package discovery strategy and target portability matrix (OQ-015, OQ-016).
