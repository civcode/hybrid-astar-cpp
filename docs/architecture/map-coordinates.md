# Map frames, grid coordinates and occupancy semantics — v0.1

**Accepted convention:** ROS 2 REP-103, REP-105 and nav_msgs OccupancyGrid-compatible map coordinates, with no ROS 2 build dependency in the core. See [ADR-0009](../adr/0009-ros-map-coordinate-conventions.md).

## Frames and axes

- Plan in a single named **world-fixed** frame, usually `map`. `odom` is the locally continuous frame; `base_link` is attached to the robot, and is not automatically identical to a model's axle center or rear-chassis reference point.
- Right-handed coordinates. Body +X forward, +Y left, +Z up. Yaw \(\theta=0\) faces world +X and positive yaw turns counterclockwise (toward +Y).
- Units: meters, radians, seconds. A 2D point and heading are a pose in **a declared frame**; numeric coordinates without a frame must not silently cross frame boundaries.
- **ENU** (X east, Y north, Z up) is the ROS convention for local geographic Cartesian coordinates. A map frame for an indoor building may be oriented with building walls; it need not be geographic ENU.
- The ROS 2 adapter resolves any `map <- odom <- base_link` and collision-frame transforms before a planning request; the search never queries TF2. A map-frame discontinuity in localization between requests is an adapter concern, not a reason to mutate a live map snapshot.

## Grid origin and memory layout

The occupancy grid has `width` columns, `height` rows, resolution `r > 0` meters/cell, origin world pose `(o_x,o_y,psi)`, and `frameId`. Our convention places the **map-local lower-left corner** of cell `(0,0)` at the origin; cell centers add a half-cell offset, as in Nav2's `mapToWorld` conversion.

For in-range integer map index `(i,j)`:
- `0 <= i < width`; `0 <= j < height`.
- Memory location: `index = j*width+i`, with X changing fastest.
- Row `j=0` is the **lowest-Y row in map-local coordinates**, *not* the first row of a top-left-origin image that has been flipped.
- Transform from **cell center** to world point:

\[
\begin{bmatrix}x\\y\end{bmatrix}=
\begin{bmatrix}o_x\\o_y\end{bmatrix}
+R(\psi)\,r\,\begin{bmatrix}i+\frac12\\j+\frac12\end{bmatrix},
\quad
R(\psi)=\begin{bmatrix}\cos\psi&-\sin\psi\\\sin\psi&\cos\psi\end{bmatrix}.
\]

For world point `(x,y)`, apply the **inverse** grid origin transform and scale:

\[
\begin{bmatrix}\xi\\\eta\end{bmatrix}
=\frac1r R(-\psi)\left(
\begin{bmatrix}x\\y\end{bmatrix}-
\begin{bmatrix}o_x\\o_y\end{bmatrix}
\right),\qquad
i=\lfloor\xi\rfloor,\quad j=\lfloor\eta\rfloor.
\]

Check both continuous `(xi,eta)` coordinates against the half-open domain `[0,width) x [0,height)` **before** unsigned conversion, and reject NaN/Inf. Use `floor`, never truncation toward zero for negative coordinates. Boundary points on the maximum edge are outside. Grid origin yaw may be nonzero: neglecting it produces wrong collision and heuristic lookups. 

World/map frame heading composition is `theta_world = normalize(theta_grid + psi)` when transforming a pose. Occupancy-grid orientation is the map *origin* pose, not a statement that the map `frameId` itself changed.

## Occupancy representation

The `nav_msgs/OccupancyGrid` message uses signed `int8` cells. The common occupancy encoding is:

| Raw value | Conventional interpretation |
| --- | --- |
| `-1` | Unknown |
| `0` | Free |
| `1..99` | Intermediate occupancy probability |
| `100` | Occupied |

The ROS message documents application-dependent occupancy values; **the adapter must declare and validate its chosen interpretation**. A policy specifies an occupied threshold (`0..100`), treatment of unknown cells (`Free`, `Occupied` or `Reject`), and invalid value behavior. Nav2 unsigned costmap encoding (`0..255` costs) is **not** an OccupancyGrid probability array and must not be blindly reinterpreted.

Internally, choose either signed occupancy values **with explicit validation**, or a typed cell-state/probability representation. Do **not** put raw `uint8_t` occupancy into a public contract while interpreting `-1` as unknown without defining the conversion. Prefer immutable backing storage and immutable map snapshots.

Collision checking and obstacle heuristics must use explicit, mutually consistent map semantics. A deliberately relaxed map may be used for a lower bound only when its validity conditions are documented.

## Suggested native API sketch

```cpp
struct Pose2D { double x, y, yaw; };

struct GridGeometry {
    double resolutionMeters;
    Pose2D originInFrame;          // lower-left corner of (0,0)
    uint32_t width, height;
    std::string frameId = "map";  // defaults only; no TF lookup
};

struct GridIndex { int32_t x, y; };
// Occupancy representation and owning handle remain under review.
// worldToCell(worldPoint, geometry) -> optional<GridIndex>
// cellCenterToWorld(GridIndex, geometry) -> Point2D
// linearIndex(i,j,width) -> size_t, after bounds checks
// validatePlanFrames(map.frameId, start.frameId, goal.frameId)
```

A ROS adapter converts quaternion yaw to planar `originInFrame.yaw` after checking that the map origin orientation is valid and planar. The native API should not require quaternion or ROS message headers.

## Catch2 contract tests

- [ ] Grid indexing: for width 3, (`0,0`) -> 0, (`1,0`) -> 1, (`0,1`) -> 3.
- [ ] Map origin translation and nonzero map origin yaw (`pi/2`, other non-cardinal rotations).
- [ ] World->grid->cell-center round-trip; define comparison as cell membership, not exact original world point.
- [ ] Border tests: origin included, upper bounds excluded, negative local coordinates rejected (not truncated to zero).
- [ ] Identity map (yaw zero) agrees with Nav2 `worldToMap` and `mapToWorld` behavior.
- [ ] Body-forward and yaw test: `theta=0` points +X; `theta=pi/2` points +Y.
- [ ] Raster import does not silently flip rows; test distinct corner-cell obstacles.
- [ ] Conventional `int8` occupancy handling for `-1,0,50,100`, threshold boundary, unknown policy and malformed values.
- [ ] Separate Nav2 costmap translation: unsigned 8-bit costs aren't occupancy probabilities.
- [ ] Non-planar origin orientation rejected or handled via explicit adapter policy.
- [ ] Frame mismatch between start/goal/map rejected; optional ROS adapter transforms to `map` first.
- [ ] Map version/origin/resolution/frame/semantic policy changes invalidate relevant heuristic caches.

## External references

- https://www.ros.org/reps/rep-0103.html
- https://www.ros.org/reps/rep-0105.html
- https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/OccupancyGrid.msg
- https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/MapMetaData.msg
- https://github.com/ros-navigation/navigation2/blob/main/nav2_costmap_2d/src/costmap_2d.cpp
