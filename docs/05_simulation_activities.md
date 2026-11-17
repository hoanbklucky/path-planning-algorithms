# Guided simulation activities

Run from the repository root:

```bash
python -m pip install -r simulators/requirements.txt
python simulators/path_planning_sim.py
```

Use `py` on Windows or `python3` on macOS/Linux if needed. Install and run with the same interpreter. Use a desktop Python session with a graphical display, not an inline notebook backend.

## Controls

| Control | Behavior |
|---|---|
| Algorithm selector | Dijkstra, A\*, RRT, RRT\*, or PRM; changing it resets the run |
| Select next node | Highlight the chosen grid node before changing neighbor costs |
| Inspect neighbors | Reveal new or improved costs on the following click |
| Reveal path | Finish when the selected node is the goal |
| Step | One sampling or edge-check event; PRM query uses the two-phase search |
| Play / Pause | Animate one visible event at a time |
| Reset | Repeat the selected algorithm with the same seed |
| New random seed | Increment the seed and reset the selected planner |
| Show g+h=f | Display grid-search cost labels |
| Events / sec | Playback speed, default 2; use higher values for roadmap construction |

Manual stepping pauses playback. Costs discovered on earlier expansions remain visible during selection; new costs appear only during inspection. Yellow means selected, blue means expanded or a tree/roadmap edge, green means frontier or accepted edge, and orange outlines mark grid cost updates. Purple lines show RRT\* rewires; orange lines show the current best solution.

RRT\* continues until its budget is exhausted. A visible solution is not an instruction to stop.

## Predict, step, explain

1. **Dijkstra:** Select a node. Predict its neighbors' costs before revealing them.
2. **A\*:** Verify f=g+h. At S=(1,2), h=20. Explain why a wall can force motion away from the goal.
3. **RRT:** Pause on a rejected extension. Are its endpoints free? Does the segment intersect a wall?
4. **RRT\*:** Watch the first solution, then continue. Find a rewire and explain which descendant costs must change.
5. **PRM:** Separate roadmap construction from query search. Does a dense-looking roadmap necessarily connect S and G?

## Reproducible default results

| Planner | Outcome | Reported work |
|---|---|---|
| Dijkstra | Cost 28 | 145 expanded cells, including goal |
| A\* | Cost 28 | 97 expanded cells, including goal |
| RRT seed 7 | Length 27.57 | 246 iterations |
| RRT\* seed 7 | Best length 21.10 | 800 iterations, 558 tree nodes, 426 rewires |
| PRM seed 7 | Roadmap route length 20.92 | 140 free samples plus endpoints; 114 query expansions |

The grid and continuous planners allow different motions, so their costs are not a direct optimality contest. Counts depend on the specified tie rules and parameters. These are deterministic examples, not statistical comparisons.

## Change one condition

- Repeat RRT and RRT\* with several random seeds.
- Increase RRT\*'s iteration budget. Plot best-so-far cost against iterations.
- Reduce PRM's samples or k. Observe disconnected roadmaps.
- Set A\*'s heuristic to zero and compare with Dijkstra.
- Change one wall in `OBSTACLES`; check that start and goal are still free.
- Keep the same start, goal, map, and cost definition when making comparisons.

Record success, route length, collision validity, and planner-specific work. If comparing runtime, measure actual planner computation separately from drawing, GUI waits, and playback.

## A 50 minute lesson

| Time | Activity |
|---|---|
| 0–7 minutes | States, edges, costs, collision checking |
| 7–17 | Dijkstra and A\* worked graph; two-phase stepping |
| 17–27 | Grid detours; explain g, h, f |
| 27–38 | RRT and RRT\*; rejection, parent choice, rewiring |
| 38–45 | PRM construction and graph query |
| 45–50 | Explain planner choice using one observed result |

## Verify the code

```bash
python -m unittest discover -s tests -v
python simulators/path_planning_sim.py --check
```

Tests cover phase separation, path validity, descendant cost propagation, monotone RRT\* incumbents, tree reachability, finite-roadmap optimality against Bellman-Ford, and a disconnected PRM case.
