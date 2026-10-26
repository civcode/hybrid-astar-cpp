# ADR-0001: ROS 2-independent core

**Status:** Accepted  
**Date:** 2026-10-24

## Context

The planner will eventually run in a ROS 2 environment but must also support standalone simulations, examples, and tests.

## Decision

Implement the planning kernel as a standard C++20/CMake library **without ROS 2 headers, message types, TF2 calls, ROS parameters, or lifecycle dependencies**. Use ROS-friendly practices (SI units, named frames, YAML configuration, structured diagnostics), but keep ROS 2 and Nav2 functionality in optional external adapters. Adapter dependencies always point into core, never the reverse.

## Alternatives considered

Direct ROS 2/Nav2 package: faster initial integration but couples core algorithm, build and deployment to ROS. Generic ROS messages in public types: familiar but prevents standalone use.

## Consequences

Additional serialization/type conversion code at boundaries; easy local benchmarking and direct reuse outside ROS. Search never resolves hypothetical body poses via TF2.

## Follow-up

Define native path-to-ROS message mapping without discarding reverse/rotation/articulation semantics (OQ-017).
