# ADR-0009: ROS-compatible map coordinates and occupancy grids

**Status:** Accepted
**Date:** 2026-10-24

## Context

The planner is a standalone C++20 library but needs direct, predictable interoperation with ROS 2/Nav2 maps, transforms, and robot frames. Ambiguity between image-row coordinates, Cartesian grid coordinates, frame conventions, and cell centers can cause mirrored maps and incorrect collision results.

## Decision

Adopt the **semantics** of ROS REPs 103 and 105 plus `nav_msgs/msg/OccupancyGrid` and `nav_msgs/msg/MapMetaData`, **without bringing ROS headers into the core**:

1. Use right-handed, planar XY with Z up, meters and radians. Body-frame +X is forward, +Y left; positive yaw increases counterclockwise about +Z.
2. The planning world frame defaults to `map`, a globally consistent, world-fixed frame. `odom` denotes a locally continuous frame, and `base_link` is a robot-attached frame. The core treats these as *named, typed metadata* and only uses already-transformed data; TF2 lookups happen in adapters.
3. The map coordinate system is **not automatically geographic ENU**. For geographically referenced maps use ENU (X east, Y north, Z up) by default. Indoor maps may align with building axes as permitted by REP-105; coordinate orientation and origin must be documented.
4. Grid indices `(i,j)` grow with map-local +X and +Y. Store cells in **row-major order, X fastest** (`index = j*width+i`). Do not flip Y as for top-left-origin raster image coordinates.
5. Store map resolution in meters/cell, dimensions in cells, origin as a full planar SE(2) pose in the world frame, and a frame identifier. The **lower-left corner** of cell `(0,0)` is the declared grid origin (the cell-center offset is `(+0.5,+0.5)` cells), following common Nav2 costmap world/grid conversion semantics. For a rotated origin, apply the full inverse/forward SE(2) transform; no assumption that yaw is zero.
6. The ROS OccupancyGrid adapter accepts `int8` occupancy values; under the conventional occupancy-probability encoding, **`-1` means unknown and `0..100` means increasing occupied probability**. ROS's message allows application-specific interpretations, so make the expected encoding explicit. Configurable occupied threshold and unknown policy are required. Nav2 8-bit costmap costs are a *different encoding* and must be converted explicitly.
7. All starts, goals, paths, robot body transforms and obstacle map snapshots must be expressed in a **single known planning frame** for one request. A mismatched/unresolved frame is an error, not an implicit identity transform.
8. Preserve immutable map snapshot identity/version separately from ROS timestamps. Frame identity, origin, yaw, resolution, dimensions and cell semantic policy are part of heuristic cache validity.

## Alternatives considered

- Screen/image coordinates with Y increasing downward: convenient for images but introduces axis inversion in the planning kernel.
- World frame implicitly geographic ENU: invalid for many indoor maps.
- Require zero-yaw grid origins: reduces transform cost but loses valid rotated occupancy maps.
- Use ROS messages/TF2 throughout core: breaks the standalone requirement.
- Accept raw occupancy/costmap bytes without a declared encoding: silently confuses unknown, free and lethal cells.

## Consequences

The environment module defines deterministic world/grid transformations and explicit validation. ROS-facing adapters map native types to REP-103/105 conventions without duplicating semantics in the search kernel. Frame consistency and map conversion become executable Catch2 contracts.

## References

- REP-103: https://www.ros.org/reps/rep-0103.html
- REP-105: https://www.ros.org/reps/rep-0105.html
- nav_msgs OccupancyGrid: https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/OccupancyGrid.msg
- nav_msgs MapMetaData: https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/MapMetaData.msg
- Nav2 costmap conversions: https://github.com/ros-navigation/navigation2/blob/main/nav2_costmap_2d/src/costmap_2d.cpp

## Follow-up / unresolved details

Exact native occupancy storage/semantic representation (signed `int8` vs explicit classified/cost data), treatment of invalid occupancy values, occupancy threshold defaults, frame-id representation, and map-snapshot owning handle remain detailed-interface review matters. See [coordinate specification](../architecture/map-coordinates.md) and OQ-006/OQ-023. All interfaces must preserve unknown semantics and explicit transformations.
