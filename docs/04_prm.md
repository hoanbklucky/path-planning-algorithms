# Probabilistic Roadmap

PRM samples **free configurations** and connects nearby samples with valid local motions. The resulting graph is a roadmap. Attach the start and goal, then use a graph algorithm such as Dijkstra or A* to find a route through the roadmap.

## Two conceptual stages

**Build:** Sample collision-free configurations, choose nearby neighbors, and validate every proposed edge. Store the accepted edges and their costs.

**Query:** Connect the start and goal to the roadmap and search it. For many queries in an unchanged environment, the main roadmap can be reused. Recheck or rebuild when obstacles change.

Our compact simulator rebuilds for each run and includes start and goal while constructing the neighbor candidates; it demonstrates both stages but does not expose a reusable multi-query cache.

## Worked example with two corridors

S=(2,5), G=(8,5). A closed obstacle occupies 4≤x≤6 and 3≤y≤7. Use these free roadmap samples and a connection radius of 4.1:

| Top corridor | Coordinate | Bottom corridor | Coordinate |
|---|---|---|---|
| A | (2,8) | D | (2,1) |
| B | (5,8) | E | (5,1) |
| C | (8,8) | F | (8,1) |

Connect every pair no farther apart than 4.1 when its entire segment is valid. This produces two routes:

| Route | Edge lengths | Total |
|---|---|---:|
| S → A → B → C → G | 3+3+3+3 | **12** |
| S → D → E → F → G | 4+3+3+4 | 14 |

Dijkstra on this roadmap returns the upper route of cost 12. This does **not** establish the shortest route in continuous free space; it is the shortest route using the available vertices and edges.

S→G has length 6, so it is outside the current connection radius. If the radius is increased to include it, it still must be rejected because it crosses the obstacle. Distance alone is not an edge-validity test.

If B is removed while all other settings are unchanged, the top corridor disconnects and the bottom route of cost 14 remains. If E is also removed, the roadmap has no S-to-G path even though the continuous environment still has routes. More samples or different connections may solve the problem.

## What the simulator shows

Select **PRM**:

1. **Sample:** Free sample points appear.
2. **Connect:** Each candidate edge is displayed as accepted or rejected.
3. **Query:** Dijkstra searches the roadmap. Selection and neighbor inspection appear in separate phases.
4. **Finish:** The orange path is the shortest route on the constructed roadmap.

The simulator defaults to 140 free random samples and k=10 nearest-neighbor candidates per vertex. An undirected candidate pair is checked once. The hand example uses a radius instead of k-nearest connections to keep its arithmetic transparent.

## PRM versus RRT

| Question | PRM | RRT |
|---|---|---|
| What grows? | A graph across free space | A tree rooted at the start |
| Is a new sample attached to just one parent? | No, multiple valid edges are allowed | Yes |
| Typical motivation | Repeated queries in a static environment | Single-query exploration |
| How is the final route found? | Graph search | Trace parent pointers after connecting the goal |

Basic PRM is not automatically asymptotically optimal. PRM* uses connection rules designed for asymptotic optimality. Probabilistic completeness also depends on appropriate sampling and connection assumptions; finite budgets can fail.

References: [LaValle, Planning Algorithms](https://planning.cs.uiuc.edu/) and [Karaman and Frazzoli](https://arxiv.org/abs/1105.1186).
