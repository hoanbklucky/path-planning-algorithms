# RRT star

RRT\* adds **cost-aware parent selection** and **rewiring** to RRT. It can find a feasible path first and improve it as more samples are processed. Do not stop the animation at the first orange path.

## One iteration

1. Sample q_rand and find q_near, as in RRT.
2. Steer by at most step length η to create q_new. Reject a colliding extension.
3. Collect nearby existing vertices.
4. For every valid parent candidate v, compare g(v)+distance(v,q_new). Pick the smallest.
5. Add q_new and its selected parent edge.
6. For nearby vertices v, test whether going through q_new reduces g(v). If so, change v's parent after validating the edge.
7. Update the costs of every affected descendant.
8. Re-evaluate valid goal connections and display the cheapest current path.

The nearest geometric node need not provide the cheapest route from the start.

## Worked example with a cheaper parent and two rewires

Consider a local obstacle-free patch containing A=(2,2), B=(3,4), C=(4,2), and a new point Q=(3,2). The existing tree has g(A)=9, g(B)=5, and g(C)=10. These are accumulated path costs from S=(0,0); earlier detours outside the patch explain why some are larger than straight-line distance. B's cost 5 equals its direct distance from S.

The nearest-node tie is resolved in favor of A. Suppose all three nodes are in the neighbor set and each local connection is collision-free.

| Parent candidate | Existing g | Distance to Q | Candidate g(Q) |
|---|---:|---:|---:|
| A | 9 | 1 | 10 |
| B | 5 | 2 | **7** |
| C | 10 | 1 | 11 |

Choose **B**, even though A is nearer. Now consider rewiring:

| Node | Old cost | Route through Q | Decision |
|---|---:|---:|---|
| A | 9 | 7+1=8 | Reparent A to Q |
| C | 10 | 7+1=8 | Reparent C to Q |
| D=(5,2), a child of C | 11 | New g(C)+1=9 | Update descendant cost |

Rewiring is more than drawing a shorter edge. If C's cost changes and D's stored cost remains 11, subsequent parent comparisons can be wrong. The implementation recomputes descendant costs after every successful rewire.

The stored g values are inputs from an existing tree with earlier detours. This local example isolates the parent comparisons and descendant updates; it is not a full prescribed sampling trace.

## What the simulator shows

Select **RRT\***. Blue edges form the current tree; a red dashed edge marks an old connection replaced in this iteration, and a purple line marks the rewired connection. Orange shows the current best start-to-goal route. The status reports iteration, cumulative rewires, and incumbent length.

The defaults use 800 iterations, step=1.5, goal bias=0.1, and seed=7. Unlike basic RRT, this run continues after finding a goal connection. A newly drawn edge need not be part of the orange path.

The neighbor radius is r(n)=min(η, γ sqrt(log(n)/n)) in 2D, with γ=18 in this teaching implementation. The nearest valid extension remains a fallback parent. The goal is a separately validated connection to a tree node within one step length.

## Guarantees and limits

RRT\* is asymptotically optimal under the standard sampling, clearance, local-planner, and connection-parameter assumptions. This is an infinite-sample property, not a claim that an 800-iteration run finds the exact optimum or outperforms RRT on every trial. Neighborhood rules and constants matter.

This teaching code uses linear nearest-neighbor scans and explicit descendant updates for clarity, rather than an optimized spatial index. It supports 2D straight-line geometry, not full robot dynamics.

Primary reference: [Karaman and Frazzoli, Sampling-based Algorithms for Optimal Motion Planning](https://arxiv.org/abs/1105.1186).
