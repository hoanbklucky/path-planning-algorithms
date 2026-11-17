# Optional simulators and videos

Use these resources if you want another explanation or a larger demonstration. The local Python simulator is enough for the guided exercises.

## Try in your browser

| Resource | What to try |
|---|---|
| [MySimulator path planning simulator](https://www.mysimulator.uk/algorithms/path-planning-algorithm/sim.html) | Another browser visualization recommended by the instructor; explore its available controls and compare search behavior |
| [PhysSandbox A* and Dijkstra grid](https://physandbox.com/engineering/astar-dijkstra-grid) | Change walls, connectivity, and heuristic; step through the search and compare frontier growth |
| [PhysSandbox engineering catalog](https://physandbox.com/engineering) | Find its RRT Path Planner and other engineering demonstrations |

For either site, record which motion model, heuristic, weights, and map you used. Results from different settings are not directly comparable. These are external websites; their controls and availability can change.

## Watch

| Video | Why watch it |
|---|---|
| [Path Planning with A* and RRT — Autonomous Navigation Part 4](https://www.youtube.com/watch?v=QR3U1dgc5RE&list=PLaH6cYVpQeoM&index=1) | A broader introduction linking occupancy maps, graphs, search, and sampling |
| [Sebastian Lague — A* Pathfinding E01 algorithm explanation](https://www.youtube.com/watch?v=-L-WgKMFuhE) | A visual explanation of A*; pause and predict the next selection |
| [Motion Planning Algorithms RRT RRT* PRM — MIT 6.881 final project](https://www.youtube.com/watch?v=gP6MRe_IHFo) | Compare tree growth, rewiring, and roadmap behavior |

The videos are linked as optional learning resources. This repository's code and numerical examples were developed independently.

## Try a larger MATLAB project

[ruddrho/matlab-multi-algorithm-robot-navigation](https://github.com/ruddrho/matlab-multi-algorithm-robot-navigation) compares A*, Dijkstra, RRT, RRT*, and PRM in a shared simulated navigation setup, with a local DWA controller, simulated LiDAR, occupancy mapping, animations, and example results.

At the time of review, its README recommends MATLAB R2020a or newer and a desktop session, with no third-party packages. Follow that repository's current requirements and startup instructions. Its example outputs describe individual runs, not a universal planner ranking.

Suggested task: watch the stored planning animation first. Then run it if MATLAB is available. Explain why a shortest global path need not give the shortest navigation completion time.

## Original A* video source

[Sebastian Lague's Pathfinding repository](https://github.com/SebLague/Pathfinding) contains episode-organized source material for the video series. This is a separate project; use its own environment and instructions.

## Primary theory references

- [Steven M. LaValle — Planning Algorithms](https://planning.cs.uiuc.edu/): graph search, configuration space, roadmaps, and sampling-based planning.
- [Karaman and Frazzoli — Sampling-based Algorithms for Optimal Motion Planning, 2011](https://arxiv.org/abs/1105.1186): asymptotic properties of RRT*, PRM*, and related planners.

Use these for deeper theory. No infinite-sample guarantee implies that one short classroom run is already optimal.
