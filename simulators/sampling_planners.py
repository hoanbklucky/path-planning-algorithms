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
    def snapshot(phase,iteration,note,sample=None,near=None,new=None,parent=None,
                 attempt=None,focus=None,rewired=None,candidate=math.inf):
        return Frame('rrtstar',current=near,nodes=nodes.copy(),
                     edges=[(nodes[p],nodes[i]) for i,p in enumerate(parents) if p is not None],
                     g=dict(zip(nodes,costs)),sample=sample,proposal=new,parent_node=parent,
                     attempt=attempt,focus=focus or set(),path=best_path.copy(),count=iteration,
                     phase=phase,note=note,rewired=rewired or [],best_cost=best,candidate_cost=candidate)
    for iteration in range(1,budget+1):
        biased=rng.random()<goal_bias
        sample=GOAL if biased else (rng.uniform(-.5,WIDTH-.5),rng.uniform(-.5,HEIGHT-.5))
        nearest=min(range(len(nodes)),key=lambda i:math.dist(nodes[i],sample))
        near=nodes[nearest];distance=math.dist(near,sample)
        if distance<1e-10:continue
        ratio=min(1.,step/distance)
        new=tuple(near[j]+ratio*(sample[j]-near[j]) for j in (0,1))
        intro=(f'Iteration {iteration}/{budget}: {"goal-biased" if biased else "uniform random"} sample=({sample[0]:.2f}, {sample[1]:.2f}).\n'
               f'Nearest is n{nearest}=({near[0]:.2f}, {near[1]:.2f}), distance={distance:.2f}. '
               f'Steer at most {step:.2f} to ({new[0]:.2f}, {new[1]:.2f}).')
        yield snapshot('preview',iteration,intro+'\nNEXT: check the extension, then compare collision-free nearby parents by g(parent)+edge length.',
                       sample,near,new,attempt=(near,new,None),focus={near})
        accepted=segment_free(near,new)
        rewired=[];parent=None;candidate=math.inf;summary=''
        if accepted and all(math.dist(q,new)>1e-9 for q in nodes):
            n=len(nodes)+1
            radius=min(step,gamma*math.sqrt(math.log(n)/n))
            neighbors=[i for i,q in enumerate(nodes) if math.dist(q,new)<=radius]
            candidates=set(neighbors+[nearest])
            valid=[i for i in candidates if segment_free(nodes[i],new)]
            ranked=sorted(valid,key=lambda i:(costs[i]+math.dist(nodes[i],new),i))
            parent=ranked[0];candidate=costs[parent]+math.dist(nodes[parent],new)
            comparison='; '.join(f'n{i}: {costs[i]:.2f}+{math.dist(nodes[i],new):.2f}={costs[i]+math.dist(nodes[i],new):.2f}'
                                 for i in ranked[:3])
            summary=(f'Parent candidates: {len(valid)} collision-free; radius={radius:.2f}. Best totals: {comparison}.\n'
                     f'CHOOSE n{parent}: minimum g(parent)+edge={candidate:.2f}, not necessarily the nearest node.')
            yield snapshot('parent',iteration,intro+'\n'+summary+'\nNEXT: add this parent edge, then test cheaper routes through the new node to its neighbors.',
                           sample,near,new,nodes[parent],(nodes[parent],new,True),
                           focus={nodes[i] for i in ranked[:3]},candidate=candidate)
            idx=len(nodes);nodes.append(new);parents.append(parent);costs.append(candidate)
            for i in neighbors:
                if i==0 or i==parent:continue
                alternative=costs[idx]+math.dist(new,nodes[i])
                if alternative+1e-10<costs[i] and segment_free(new,nodes[i]):
                    old=parents[i];parents[i]=idx;costs=propagate_costs(nodes,parents)
                    rewired.append((nodes[old],nodes[i],new));rewires_total+=1
            if math.dist(new,GOAL)<=step and segment_free(new,GOAL):goal_links.add(idx)
            if goal_links:
                goal_parent=min(goal_links,key=lambda i:costs[i]+math.dist(nodes[i],GOAL))
                best=costs[goal_parent]+math.dist(nodes[goal_parent],GOAL)
                best_path=trace(nodes,parents,goal_parent)+[GOAL]
            result=(f'ADD n{idx}: g={costs[idx]:.2f}; selected parent=n{parent}. '
                    f'Rewires this iteration={len(rewired)}, cumulative={rewires_total}.\n'
                    'Rewired descendants now have updated g values. NEXT: sample again.')
        else:
            result='REJECT: colliding edge or duplicate node. Tree and costs unchanged. NEXT: sample again.'
        incumbent=f'Best path cost={best:.2f}; continuing to improve.' if best_path else 'No complete goal path yet.'
        yield snapshot('grow',iteration,intro+'\n'+result+'\n'+incumbent,
                       sample,new if accepted else near,new,nodes[parent] if parent is not None else None,
                       (nodes[parent] if parent is not None else near,new,accepted),
                       focus={near,new}|{child for old,child,newparent in rewired},
                       rewired=rewired,candidate=candidate)
    yield snapshot('done',budget,f'RRT* budget complete. Nodes={len(nodes)}; rewires={rewires_total}.\n'
                   +(f'Best path cost={best:.2f}.' if best_path else 'No path found; increase budget or change seed.'))

def prm(seed=7,samples=140,k=10):
    """Basic k-nearest PRM, followed by Dijkstra on the constructed roadmap."""
    rng=random.Random(seed);nodes=[];edges=[]
    while len(nodes)<samples:
        q=(rng.uniform(-.5,WIDTH-.5),rng.uniform(-.5,HEIGHT-.5))
        yield Frame('prm',nodes=nodes.copy(),sample=q,proposal=q,phase='sample_preview',
                    note=f'PRM uniform sample=({q[0]:.2f}, {q[1]:.2f}). Free nodes so far={len(nodes)}/{samples}.\n'
                         'NEXT: test this point for collision. During construction, costs from S are not assigned yet.')
        if not point_free(q):
            yield Frame('prm',nodes=nodes.copy(),sample=q,phase='sample',
                        note='REJECT sampled point: it lies in an obstacle. NEXT: draw another sample.')
            continue
        nodes.append(q)
        yield Frame('prm',nodes=nodes.copy(),sample=q,current=q,phase='sample',
                    note=f'ACCEPT: roadmap vertex n{len(nodes)-1}=({q[0]:.2f}, {q[1]:.2f}).\n'
                         f'NEXT: sample until {samples} free vertices exist, then connect nearby pairs (k={k}).')
    # Indices 0..samples-1 are reusable roadmap vertices; endpoints are query vertices.
    nodes.extend([START,GOAL]);start_idx=samples;goal_idx=samples+1
    adj=[[] for _ in nodes];pairs=set()
    for i,q in enumerate(nodes):
        nearest=sorted((j for j in range(len(nodes)) if j!=i),key=lambda j:math.dist(q,nodes[j]))[:k]
        pairs.update(tuple(sorted((i,j))) for j in nearest)
    for index,(i,j) in enumerate(sorted(pairs),1):
        a,b=nodes[i],nodes[j]
        distance=math.dist(a,b)
        yield Frame('prm',nodes=nodes.copy(),edges=edges.copy(),current=a,proposal=b,
                    attempt=(a,b,None),phase='edge_preview',
                    note=f'PRM candidate {index}/{len(pairs)}: n{i} to n{j}, edge length={distance:.2f}.\n'
                         f'WHY: one endpoint ranks the other among its {k} nearest neighbors.\n'
                         'NEXT: collision-check the entire segment; keep it only if valid.')
        accepted=segment_free(a,b)
        if accepted:
            weight=math.dist(a,b);adj[i].append((j,weight));adj[j].append((i,weight));edges.append((a,b))
        yield Frame('prm',nodes=nodes.copy(),edges=edges.copy(),attempt=(a,b,accepted),phase='connect',
                    note=f'PRM edge n{i} to n{j}: {"ACCEPT" if accepted else "REJECT collision"}, length={distance:.2f}.\n'
                         f'Undirected edges={len(edges)}. NEXT: check the next nearest-neighbor candidate pair.')
    g={start_idx:0.};parents={};closed=set();front={start_idx};queue=[(0.,start_idx)]
    expansions=0
    def snap(phase,current,note,path=None,next_idx=None):
        return Frame('prm',current=nodes[current] if current is not None else None,
                     nodes=nodes.copy(),edges=edges.copy(),
                     closed={nodes[i] for i in closed},frontier={nodes[i] for i in front},
                     g={nodes[i]:v for i,v in g.items()},h={nodes[i]:0 for i in g},
                     phase=phase,note=note,path=path or [],count=expansions,
                     next_node=nodes[next_idx] if next_idx is not None else None)
    while queue:
        gu,u=heapq.heappop(queue)
        if gu!=g[u] or u in closed:continue
        front.discard(u)
        yield snap('select',u,f'PRM query SELECT n{u}=({nodes[u][0]:.2f}, {nodes[u][1]:.2f}): g={gu:.2f}.\n'
                   'WHY: smallest frontier g (Dijkstra); equal g uses the lower vertex index.\n'
                   'NEXT: inspect roadmap neighbors using candidate g = current g + Euclidean edge length.',next_idx=u)
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
        live=[(cost,i) for cost,i in queue if cost==g.get(i) and i not in closed]
        nxt=min(live)[1] if live else None
        why=(f'NEXT: n{nxt}=({nodes[nxt][0]:.2f}, {nodes[nxt][1]:.2f}), g={g[nxt]:.2f}; minimum frontier g.'
             if nxt is not None else 'NEXT: frontier is empty.')
        yield snap('expand',u,f'PRM query: {changes} cost update(s).\n'+why,next_idx=nxt)
    yield snap('done',None,'PRM roadmap has no start-to-goal connection.\nIncrease samples or neighbors; this does not prove the continuous problem is impossible.')

