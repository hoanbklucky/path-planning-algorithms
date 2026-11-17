# Robot path planning basics

A planner finds a route from a start to a goal while respecting obstacles and allowed motion. A **path** describes geometry; a **trajectory** adds timing, velocities, and accelerations. A controller uses feedback to follow that trajectory.

## Represent the robot before choosing the planner

For a point-like mobile robot, use (x,y). For a car, heading and steering constraints matter. For an arm, use joint angles q=(q1,...,qn); the corresponding **configuration space** describes all joint configurations. A configuration is invalid if any robot link collides with an obstacle or with another link.

The provided simulator uses a point robot in a known 2D map. Walls represent forbidden robot-center positions, so footprint inflation is assumed already done. For a circular robot, inflate obstacles by its radius plus a safety margin. A real robot also needs motion feasibility, timing, and feedback control.

## Five planners in one picture

| Planner | Representation | How exploration proceeds | Meaning of its result |
|---|---|---|---|
| Dijkstra | Weighted graph or grid | Expand the smallest accumulated cost g | Minimum cost on that graph with nonnegative edge costs |
| A\* | Weighted graph or grid | Expand minimum f=g+h | Minimum graph cost under appropriate heuristic and implementation conditions |
| RRT | A growing configuration tree | Extend toward random samples | Feasible route; generally not optimal |
| RRT\* | A growing tree with costs | Select lower-cost parents and rewire | Improves an incumbent route; asymptotic optimality under standard assumptions |
| PRM | A sampled roadmap graph | Build connections, then run graph search | Best route on the constructed finite roadmap |

**PRM** means Probabilistic Roadmap. “RPM” in the original request is interpreted as PRM here.

A\* and Dijkstra can search a PRM roadmap too. “Graph search” and “sampling” are not mutually exclusive: PRM samples a graph, then searches it.

## Collision checking applies to edges

Two free endpoints do not imply a free connecting segment. The simulator checks entire segments using exact intersection with closed, axis-aligned rectangles. Touching an obstacle boundary counts as a collision. This checker is specific to the simulator's geometry; it does not check a robot arm or enforce turning limits.

## Compare the right quantities

The grid planners use four-connected, unit-cost moves. Sampling planners use continuous straight-line motions and Euclidean length. A continuous path can be shorter than the grid optimum because its allowed motions differ.

Expanded grid cells, RRT iterations, PRM edge tests, and animation seconds measure different things. Report path quality, success rate over multiple seeds, and actual planner computation separately. A single run does not rank algorithms generally.

## Learning sequence

Read [Dijkstra and A\*](01_dijkstra_astar.md), then [RRT](02_rrt.md), [RRT\*](03_rrt_star.md), and [PRM](04_prm.md). Run the [guided activities](05_simulation_activities.md). The [resource list](06_resources.md) links optional browser simulators, videos, MATLAB demonstrations, and primary references.
