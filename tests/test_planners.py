"""Meaningful planner and teaching-animation invariants; standard library unittest."""
import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'simulators'))
import path_planning_sim as base
from sampling_planners import rrt_star,prm,propagate_costs

class Planners(unittest.TestCase):
    def test_grid_selection_precedes_cost_changes(self):
        for astar,expected in [(False,145),(True,97)]:
            previous_g={base.START:0};previous_closed=set();selected=None
            for frame in base.grid_search(astar):
                if frame.phase=='select':
                    self.assertEqual(frame.g,previous_g)
                    self.assertEqual(frame.closed,previous_closed)
                    self.assertNotIn(frame.current,frame.closed)
                    selected=frame.current
                else:
                    if frame.phase=='expand':self.assertEqual(frame.current,selected)
                    previous_g=frame.g.copy();previous_closed=frame.closed.copy()
            self.assertEqual(frame.count,expected)
            self.assertEqual(len(frame.path)-1,28)
    def test_segment_collision(self):
        self.assertFalse(base.segment_free((4,2),(6,2))) # free endpoints, blocked edge
        self.assertFalse(base.segment_free((4.5,1),(4.5,3))) # boundary
        self.assertTrue(base.segment_free((4,8),(6,8)))
    def test_descendant_cost_propagation(self):
        nodes=[(0,0),(0,2),(2,2),(3,2),(4,2)]
        # Original path to node 2 is through node 1; rewire 2 directly to root.
        parents=[None,0,0,2,3]
        costs=propagate_costs(nodes,parents)
        self.assertAlmostEqual(costs[4],math.sqrt(8)+2)
    def test_rrt_star_tree_and_improving_incumbent(self):
        previous=math.inf;rewires=0
        for f in rrt_star(seed=7):
            rewires+=len(f.rewired)
            if f.path:
                self.assertLessEqual(f.best_cost,previous+1e-9);previous=f.best_cost
                self.assertTrue(all(base.segment_free(a,b) for a,b in zip(f.path,f.path[1:])))
                self.assertAlmostEqual(sum(math.dist(a,b) for a,b in zip(f.path,f.path[1:])),f.best_cost)
        self.assertEqual(f.phase,'done');self.assertTrue(f.path);self.assertGreater(rewires,0)
        # A tree has N-1 edges and every vertex is reachable from the root.
        self.assertEqual(len(f.edges),len(f.nodes)-1)
        reach={base.START}
        for _ in f.nodes:
            before=len(reach)
            for a,b in f.edges:
                if a in reach:reach.add(b)
            if len(reach)==before:break
        self.assertEqual(reach,set(f.nodes))
        self.assertTrue(all(base.segment_free(a,b) for a,b in f.edges))
        print(f.note)
    def test_prm_valid_graph_and_shortest_query(self):
        selected_g=None
        for f in prm(seed=7):
            if f.phase=='select':selected_g=f.g.copy()
            if f.phase=='expand':self.assertIsNotNone(selected_g)
        self.assertTrue(f.path)
        self.assertTrue(all(base.segment_free(a,b) for a,b in f.edges))
        # Bellman-Ford independently validates the optimal cost on the finite roadmap.
        distance={n:math.inf for n in f.nodes};distance[base.START]=0
        for _ in range(len(f.nodes)-1):
            changed=False
            for a,b in f.edges:
                w=math.dist(a,b)
                if distance[a]+w<distance[b]:distance[b]=distance[a]+w;changed=True
                if distance[b]+w<distance[a]:distance[a]=distance[b]+w;changed=True
            if not changed:break
        length=sum(math.dist(a,b) for a,b in zip(f.path,f.path[1:]))
        self.assertAlmostEqual(length,distance[base.GOAL])
        self.assertTrue(all(base.segment_free(a,b) for a,b in zip(f.path,f.path[1:])))
        print(f.note)
    def test_prm_disconnected_budget(self):
        f=list(prm(seed=7,samples=0,k=0))[-1]
        self.assertFalse(f.path);self.assertEqual(f.phase,'done')



class TeachingDetails(unittest.TestCase):
    def test_next_grid_node_matches_actual_selection(self):
        for astar in [False,True]:
            expected=None
            for f in base.grid_search(astar):
                if f.phase=='select':
                    if expected is not None:self.assertEqual(f.current,expected)
                    self.assertIn('WHY:',f.note)
                    self.assertIn('smallest frontier f' if astar else 'smallest frontier g',f.note)
                if f.phase=='expand':expected=f.next_node
    def test_rrt_preview_and_accumulated_costs(self):
        old_nodes=[base.START];old_g={base.START:0.}
        final=None
        for f in base.rrt(seed=7):
            if f.phase=='preview':
                self.assertEqual(f.nodes,old_nodes);self.assertEqual(f.g,old_g)
                self.assertIsNone(f.attempt[2])
                a,b,_=f.attempt
                self.assertAlmostEqual(f.candidate_cost,f.g[a]+math.dist(a,b))
            else:
                for a,b in f.edges:
                    self.assertAlmostEqual(f.g[b],f.g[a]+math.dist(a,b))
                old_nodes=f.nodes.copy();old_g=f.g.copy()
            final=f
        self.assertEqual(final.count,246)
        self.assertAlmostEqual(final.g[base.GOAL],sum(math.dist(a,b) for a,b in zip(final.path,final.path[1:])))
    def test_rrt_star_parent_preview_and_rewired_node_costs(self):
        last_nodes=[base.START];last_g={base.START:0.};parent_frames=0
        for f in rrt_star(seed=7,budget=150):
            if f.phase in ['preview','parent']:
                self.assertEqual(f.nodes,last_nodes);self.assertEqual(f.g,last_g)
            if f.phase=='parent':
                parent_frames+=1
                self.assertNotIn(f.proposal,f.g)
                self.assertAlmostEqual(f.candidate_cost,f.g[f.parent_node]+math.dist(f.parent_node,f.proposal))
                self.assertIn('minimum g(parent)+edge',f.note)
            if f.phase in ['grow','done']:
                for a,b in f.edges:self.assertAlmostEqual(f.g[b],f.g[a]+math.dist(a,b))
                last_nodes=f.nodes.copy();last_g=f.g.copy()
        self.assertGreater(parent_frames,0)
    def test_prm_preview_does_not_add_unchecked_edge(self):
        last_edges=[]
        for f in prm(seed=7,samples=12,k=3):
            if f.phase=='edge_preview':
                self.assertEqual(f.edges,last_edges)
                self.assertIsNone(f.attempt[2]);self.assertIn('WHY:',f.note)
            elif f.phase=='connect':last_edges=f.edges.copy()

if __name__=='__main__':unittest.main()
