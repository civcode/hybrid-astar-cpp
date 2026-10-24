# Algorithms and invariants — draft v0.1

These are **behavioral specifications**, not proof of completeness, optimality, or hard real-time guarantees. Exact tie-breaking, node-label policy, numeric precision and sampling/certification require review.

## Hybrid A* request flow

```mermaid
sequenceDiagram
  participant Client
  participant Planner
  participant Heuristic
  participant Search
  participant Validator
  participant Connector
  participant Path
  Client->>Planner: plan(start, goal, model, map snapshot, config)
  Planner->>Planner: validate inputs, setup per-request state
  Planner->>Heuristic: obtain/reuse goal distance field
  Planner->>Search: initialize start label (g=0)
  loop until valid goal, exhausted, cancelled, or budget reached
    Search->>Search: pop lowest f=g+h, discard stale entry
    Search->>Connector: optional model-compatible goal connection
    Connector-->>Search: possible analytic motion
    Search->>Validator: validate complete analytic motion
    Search->>Search: generate primitive successors
    Search->>Validator: validate swept motion of each successor
    Search->>Heuristic: evaluate h for accepted successors
    Search->>Search: relax eligible state labels, push open queue
  end
  Search->>Path: parent chain + full primitive / analytic segments
  Path->>Validator: final path validation
  Path-->>Planner: validated raw path
  opt refinement configured
    Planner->>Path: constrained refinement & revalidation
  end
  Planner-->>Client: status + path or reason + diagnostics
```

## Core search pseudocode

```text
validate request/map/model/start/goal
initialize or fetch model-specific primitive catalog and heuristic cache
check full start and goal collision state
insert start label: continuous state, key, g=0, parent=none
while open not empty:
    if cancellation or budget reached: return explicit failure
    pop best f; skip stale labels
    if goal condition met: record valid candidate / apply termination policy
    optionally try analytic connector:
        if model-valid and entire motion collision-free:
            evaluate its actual configured cost; record candidate
    for each primitive candidate from current model state:
        propagate complete motion and intermediate states
        reject invalid controls, infeasible states, or swept collisions
        compute successor key and edge cost (must be nonnegative)
        relax or retain label per documented discretized-state policy
        compute heuristic (or pending lazy field completion)
        enqueue eligible successor
    terminate only when stopping rule justifies chosen quality guarantee
reconstruct exact state/control segments from stable parent records
validate reconstructed trajectory against request's map/model
optionally refine, accepting only separately validated result
return status, path and diagnostics
```

### Critical search invariants

- `g` represents the cost of actual accepted transitions; always nonnegative and finite. In-place rotations have **positive cost** even with zero translation.
- Parent/action records reconstruct the *exact* accepted motion (not straight lines between discretized cell centers).
- A popped node is not automatically globally optimal if heuristic is inconsistent, weighted, or state keys merge nonequivalent states. Reopening/termination assumptions must be explicit.
- Analytic expansion cannot bypass feasibility or collision checks. First-found analytic connection may be suboptimal; do not assert optimality without a justified termination rule.
- The discretized search may lose reachability at finite resolution; report actual guarantees, not idealized continuous-space completeness.

## Obstacle-aware heuristic

1. Use immutable static grid and a **goal-rooted, nonnegative-weight reverse graph search** (Dijkstra); do not just run point-to-point A*.
2. Optionally lazy-expand until the queried cell is **settled**; incomplete frontier values cannot be returned as final distances.
3. Use compact contiguous 2D storage for distances and visited/settled state; permit cache reuse only with matching map version, frame, resolution, goal index, occupancy/unknown policy, and algorithm settings.
4. Keep obstacle-aware and kinematic lower bounds separate. If both are admissible in cost units, `h=max(h_obstacle,h_kinematic)` is admissible.
5. **Admissibility caveat:** a grid's 4-/8-connected shortest-path cost is generally an *upper* approximation to shortest unconstrained continuous distance; conservative edge graph definitions/inflation and boundary offsets must be proved safe before claiming an admissible bound. Heuristic mode and guarantee metadata must expose this.
6. Treat unreachable (infinite) heuristic cells carefully: map-level disconnection can prune safely only if the static relaxed map models traversability conservatively; otherwise retain search freedom.

## Kinematic steering and primitive integration

- Primitive providers generate model-specific **admissible controls** from the complete state; precompute local templates only for state-invariant models. Holonomic/nonholonomic classification metadata never substitutes for model validity checks.
- The model's controlled differential equation is the source of truth; a separate integration strategy produces full intermediate state trajectories. For constant-curvature Ackermann, exact arcs are preferred. Use native RK4 or other validated integration for more complex equations; RK4 is not embedded in the search kernel.
- Differential-drive in-place rotations must be modeled and collision-checked for full swept angular motion. Ideal mecanum must also support sideways translation with unchanged yaw and simultaneous translation/rotation without treating yaw as the tangent heading.
- Native Dubins (forward-only, bounded curvature) and Reeds–Shepp (forward/reverse, bounded curvature) are separate optional connectors. Reversing cusps preserve gear transitions. No steering-rate or articulation assumption is silently inferred.
- Numeric cost terms (reverse, steering changes, clearance) must remain consistent across normal primitives, analytic expansions and lower-bound heuristics.

## Swept collision checking

Check every robot body `T_i(q) P_i` across the transition. Broad phase may use conservative bounding circles/AABBs/distance bounds; narrow phase uses transformed convex shapes. Adaptive sampling must consider maximum displacement of **any** body point due to translation, rotation and articulation, not just reference-point distance. Sampled validity is approximate; continuous guarantees require conservative interpolation bounds or continuous collision detection and must be identified separately in results.

## Reconstruction, refinement and output

- Follow stable node parent indices back to start; reverse sequence, append any validated analytic connection, reconstruct intermediate model states.
- Keep exact original motion path, including cusp and rotate-in-place events and its start/end conditions.
- Optional shortcutting or curve optimization only accepts replacements with verified endpoints, model constraints, collision clearance and cost policy.
- For clamped spatial curves, endpoint heading can be constrained through endpoint tangents, but curvature, curvature rate and articulated feasibility require extra constraints; a smooth spline is **not** automatically feasible.
- Output is a piecewise path with monotone progress, independently evaluable segments and full model-state semantics. Holonomic travel direction and body yaw may differ. The separate MPC owns final timing, speeds, acceleration and moving-obstacle prediction; rate-limited path claims must state the timing assumptions on which they rely.
