"""RRT* and PRM teaching planners; geometry is shared with the original simulator."""
import heapq
import math
import random
from path_planning_sim import Frame,START,GOAL,WIDTH,HEIGHT,point_free,segment_free

def trace(nodes,parents,i):
    path=[]
    while i is not None:
        path.append(nodes[i]);i=parents[i]
    return path[::-1]

def propagate_costs(nodes,parents):
    """Recompute every descendant cost after reparenting (small teaching trees)."""
    costs=[None]*len(nodes);costs[0]=0.
    def cost(i):
        if costs[i] is None:costs[i]=cost(parents[i])+math.dist(nodes[i],nodes[parents[i]])
        return costs[i]
    for i in range(len(nodes)):cost(i)
    return costs

def rrt_star(seed=7,step=1.5,goal_bias=.1,budget=800,gamma=18.0):
    rng=random.Random(seed);nodes=[START];parents=[None];costs=[0.];goal_links=set()
    best=math.inf;best_path=[];rewires_total=0
    for iteration in range(1,budget+1):
        sample=GOAL if rng.random()<goal_bias else (rng.uniform(-.5,WIDTH-.5),rng.uniform(-.5,HEIGHT-.5))
        nearest=min(range(len(nodes)),key=lambda i:math.dist(nodes[i],sample))
        near=nodes[nearest];distance=math.dist(near,sample)
        if distance<1e-10:continue
        ratio=min(1.,step/distance)
        new=tuple(near[j]+ratio*(sample[j]-near[j]) for j in (0,1))
        accepted=segment_free(near,new)
        rewired=[]
        if accepted and all(math.dist(q,new)>1e-9 for q in nodes):
            # Standard shrinking 2D neighborhood; nearest remains a fallback parent.
            n=len(nodes)+1
            radius=min(step,gamma*math.sqrt(math.log(n)/n))
            neighbors=[i for i,q in enumerate(nodes) if math.dist(q,new)<=radius]
            candidates=set(neighbors+[nearest])
            valid=[i for i in candidates if segment_free(nodes[i],new)]
            parent=min(valid,key=lambda i:costs[i]+math.dist(nodes[i],new))
            idx=len(nodes);nodes.append(new);parents.append(parent)
            costs.append(costs[parent]+math.dist(nodes[parent],new))
            # Positive-length costs prevent improving any ancestor through its child.
            for i in neighbors:
                if i==0 or i==parent:continue
                candidate=costs[idx]+math.dist(new,nodes[i])
                if candidate+1e-10<costs[i] and segment_free(new,nodes[i]):
                    old=parents[i];parents[i]=idx
                    costs=propagate_costs(nodes,parents)
                    rewired.append((nodes[old],nodes[i],new));rewires_total+=1
            if math.dist(new,GOAL)<=step and segment_free(new,GOAL):goal_links.add(idx)
            if goal_links:
                goal_parent=min(goal_links,key=lambda i:costs[i]+math.dist(nodes[i],GOAL))
                best=costs[goal_parent]+math.dist(nodes[goal_parent],GOAL)
                best_path=trace(nodes,parents,goal_parent)+[GOAL]
        edges=[(nodes[p],nodes[i]) for i,p in enumerate(parents) if p is not None]
        note=f'RRT* iteration {iteration}/{budget}: {"added node" if accepted else "rejected edge"}; rewires={rewires_total}.'
        note+='\nBest path length: '+(f'{best:.2f}. Continuing to improve.' if best_path else 'not found yet.')
        yield Frame('rrtstar',nodes=nodes.copy(),edges=edges,sample=sample,attempt=(near,new,accepted),
                    path=best_path.copy(),note=note,count=iteration,phase='grow',
                    rewired=rewired,best_cost=best)
    yield Frame('rrtstar',nodes=nodes.copy(),edges=[(nodes[p],nodes[i]) for i,p in enumerate(parents) if p is not None],
                path=best_path,note=f'RRT* budget complete. Nodes={len(nodes)}; rewires={rewires_total}.\n'
                +(f'Best path length={best:.2f}.' if best_path else 'No path found; increase budget or change seed.'),
                count=budget,phase='done',best_cost=best)

def prm(seed=7,samples=140,k=10):
    """Basic k-nearest PRM, followed by Dijkstra on the constructed roadmap."""
    rng=random.Random(seed);nodes=[];edges=[]
    while len(nodes)<samples:
        q=(rng.uniform(-.5,WIDTH-.5),rng.uniform(-.5,HEIGHT-.5))
        if not point_free(q):continue
        nodes.append(q)
        yield Frame('prm',nodes=nodes.copy(),sample=q,phase='sample',note=f'PRM build: free sample {len(nodes)}/{samples}.')
    # Indices 0..samples-1 are reusable roadmap vertices; endpoints are query vertices.
    nodes.extend([START,GOAL]);start_idx=samples;goal_idx=samples+1
    adj=[[] for _ in nodes];pairs=set()
    for i,q in enumerate(nodes):
        nearest=sorted((j for j in range(len(nodes)) if j!=i),key=lambda j:math.dist(q,nodes[j]))[:k]
        pairs.update(tuple(sorted((i,j))) for j in nearest)
    for index,(i,j) in enumerate(sorted(pairs),1):
        a,b=nodes[i],nodes[j];accepted=segment_free(a,b)
        if accepted:
            weight=math.dist(a,b);adj[i].append((j,weight));adj[j].append((i,weight));edges.append((a,b))
        yield Frame('prm',nodes=nodes.copy(),edges=edges.copy(),attempt=(a,b,accepted),phase='connect',
                    note=f'PRM connect: candidate edge {index}/{len(pairs)} — {"accept" if accepted else "reject collision"}.\n'
                         f'Undirected roadmap edges={len(edges)}.')
    g={start_idx:0.};parents={};closed=set();front={start_idx};queue=[(0.,start_idx)]
    expansions=0
    def snap(phase,current,note,path=None):
        return Frame('prm',current=nodes[current] if current is not None else None,
                     nodes=nodes.copy(),edges=edges.copy(),
                     closed={nodes[i] for i in closed},frontier={nodes[i] for i in front},
                     g={nodes[i]:v for i,v in g.items()},h={nodes[i]:0 for i in g},
                     phase=phase,note=note,path=path or [],count=expansions)
    while queue:
        gu,u=heapq.heappop(queue)
        if gu!=g[u] or u in closed:continue
        front.discard(u)
        yield snap('select',u,f'PRM query SELECT vertex {u}: g={gu:.2f}.\nNext: inspect its roadmap neighbors.')
        closed.add(u);expansions+=1
        if u==goal_idx:
            ids=[u]
            while u in parents:u=parents[u];ids.append(u)
            yield snap('done',None,f'PRM goal reached. Roadmap path length={gu:.2f}; expanded={expansions}.\n'
                       'Optimal on this finite roadmap, not necessarily in continuous free space.',
                       [nodes[i] for i in ids[::-1]])
            return
        changes=0
        for v,c in sorted(adj[u]):
            ng=gu+c
            if ng<g.get(v,math.inf):
                g[v]=ng;parents[v]=u;front.add(v);heapq.heappush(queue,(ng,v));changes+=1
        yield snap('expand',u,f'PRM query INSPECT neighbors: {changes} cost update(s).\nNext: select the next roadmap vertex.')
    yield snap('done',None,'PRM roadmap has no start-to-goal connection.\nIncrease samples or neighbors; this does not prove the continuous problem is impossible.')
