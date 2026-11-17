"""Teaching simulator: Dijkstra, A*, RRT, RRT*, and PRM. Python 3 + matplotlib."""
import argparse
import heapq
import itertools
import math
import random
import time
import textwrap
from dataclasses import dataclass, field

WIDTH, HEIGHT = 16, 12
START, GOAL = (1, 2), (14, 9)
# Closed rectangles; coordinates denote blocked cell boundaries.
OBSTACLES = [(4.5, -.5, 5.5, 7.5), (8.5, 4.5, 9.5, 11.5),
             (11.5, -.5, 12.5, 6.5)]
EDGES = [('S','A',4),('S','B',2),('S','C',6),('A','D',5),
         ('A','E',6),('B','D',4),('B','F',7),('C','F',2),
         ('D','E',1),('D','F',3),('E','H',3),('F','H',2),
         ('F','I',5),('H','G',4),('I','G',3)]
HEURISTIC = dict(S=10,A=8,B=9,C=6,D=6,E=5,F=4,H=3,I=2,G=0)

def graph_trace(astar=False):
    adj = {}
    for a,b,c in EDGES:
        adj.setdefault(a,[]).append((b,c)); adj.setdefault(b,[]).append((a,c))
    g={'S':0}; parent={}; pq=[(HEURISTIC['S'] if astar else 0, 'S', 0)]
    rows=[]; closed=set()
    while pq:
        priority,u,gu=heapq.heappop(pq)
        if gu != g[u] or u in closed: continue
        closed.add(u); updates=[]
        if u=='G':
            path=[u]
            while u in parent: u=parent[u];path.append(u)
            rows.append((path[0],gu,priority,'Stop'))
            return rows, list(reversed(path)),gu
        for v,c in sorted(adj[u]):
            cand=gu+c
            if cand<g.get(v,math.inf):
                old=g.get(v,math.inf);g[v]=cand;parent[v]=u
                heapq.heappush(pq,(cand+(HEURISTIC[v] if astar else 0),v,cand))
                updates.append(f'{v}: {old if old<math.inf else "∞"} → {cand}')
        rows.append((u,gu,priority,'; '.join(updates) or 'No improvement'))

def point_free(p):
    x,y=p
    return (-.5 <= x <= WIDTH-.5 and -.5 <= y <= HEIGHT-.5
            and not any(x0 <= x <= x1 and y0 <= y <= y1 for x0,y0,x1,y1 in OBSTACLES))

def segment_free(a,b):
    """Exact slab intersection with closed axis-aligned rectangles."""
    if not point_free(a) or not point_free(b): return False
    for x0,y0,x1,y1 in OBSTACLES:
        lo,hi=0.,1.
        for p,delta,mn,mx in [(a[0],b[0]-a[0],x0,x1),(a[1],b[1]-a[1],y0,y1)]:
            if abs(delta)<1e-12:
                if p<mn or p>mx: lo,hi=1.,0.;break
            else:
                enter,leave=sorted(((mn-p)/delta,(mx-p)/delta))
                lo=max(lo,enter);hi=min(hi,leave)
                if lo>hi:break
        if lo<=hi:return False
    return True

@dataclass
class Frame:
    kind: str
    current: object=None
    closed: set=field(default_factory=set)
    frontier: set=field(default_factory=set)
    g: dict=field(default_factory=dict)
    h: dict=field(default_factory=dict)
    nodes: list=field(default_factory=list)
    edges: list=field(default_factory=list)
    sample: object=None
    attempt: object=None
    path: list=field(default_factory=list)
    note: str=''
    count: int=0
    phase: str=''
    updated: set=field(default_factory=set)
    rewired: list=field(default_factory=list)
    best_cost: float=math.inf
    candidate_cost: float=math.inf
    next_node: object=None
    proposal: object=None
    parent_node: object=None
    focus: set=field(default_factory=set)

def grid_search(astar=False):
    count=itertools.count()
    def heur(n):return abs(n[0]-GOAL[0])+abs(n[1]-GOAL[1]) if astar else 0
    g={START:0};parent={};closed=set();front={START}
    pq=[(heur(START),heur(START),next(count),START,0)]
    while pq:
        _,_,_,u,gu=heapq.heappop(pq)
        if gu!=g[u] or u in closed:continue
        front.discard(u)
        # Pause before relaxation: no neighbor costs or parents have changed yet.
        yield Frame('grid',current=u,closed=closed.copy(),frontier=front.copy(),
                    g=g.copy(),h={n:heur(n) for n in g},count=len(closed),phase='select',next_node=u,
                    note=f'SELECT {u}: g={gu}, h={heur(u)}, f={gu+heur(u)}. Completed expansions={len(closed)}.\n'
                         + ('WHY: smallest frontier f=g+h; ties prefer smaller h, then insertion order.\n'
                            if astar else 'WHY: smallest frontier g; equal costs use insertion order.\n')
                         + ('The goal is selected. Next: finish and reveal the path.'
                            if u==GOAL else 'Next: inspect its neighbors and reveal any new or improved costs.'))
        closed.add(u)
        if u==GOAL:
            path=[u]
            while u in parent:u=parent[u];path.append(u)
            yield Frame('grid',closed=closed.copy(),g=g.copy(),h={n:heur(n) for n in g},
                        path=path[::-1],count=len(closed),phase='done',
                        note=f'Goal reached. Cost = {gu}; expanded = {len(closed)} (includes goal).')
            return
        updated=set()
        for dx,dy in [(1,0),(0,1),(-1,0),(0,-1)]:
            v=(u[0]+dx,u[1]+dy)
            if not point_free(v) or not segment_free(u,v):continue
            ng=gu+1
            if ng<g.get(v,math.inf):
                g[v]=ng;parent[v]=u;front.add(v);updated.add(v)
                heapq.heappush(pq,(ng+heur(v),heur(v),next(count),v,ng))
        live=[entry for entry in pq if entry[4]==g.get(entry[3]) and entry[3] not in closed]
        nxt=min(live)[3] if live else None
        why=(f'NEXT: {nxt}, g={g[nxt]}, h={heur(nxt)}, f={g[nxt]+heur(nxt)}. '
             + ('Minimum frontier f; ties use lower h, then insertion order.' if astar
                else 'Minimum frontier g; ties use insertion order.')) if nxt is not None else 'NEXT: frontier empty.'
        yield Frame('grid',current=u,closed=closed.copy(),frontier=front.copy(),
                    g=g.copy(),h={n:heur(n) for n in g},count=len(closed),phase='expand',next_node=nxt,
                    updated=updated,
                    note=f'INSPECT neighbors of {u}: {len(updated)} cost update(s). Completed expansions={len(closed)}.\n'
                         'Orange outlines mark new or improved costs.\n'+why)
    yield Frame('grid',closed=closed,count=len(closed),note='No path exists on this grid.')

def rrt(seed=7,step=1.5,goal_bias=.1,budget=5000):
    rng=random.Random(seed);nodes=[START];parents=[None];edges=[];costs=[0.]
    for k in range(1,budget+1):
        biased=rng.random()<goal_bias
        sample=GOAL if biased else (rng.uniform(-.5,WIDTH-.5),rng.uniform(-.5,HEIGHT-.5))
        i=min(range(len(nodes)),key=lambda i:math.dist(nodes[i],sample));near=nodes[i]
        dist=math.dist(near,sample)
        if dist<1e-12:continue
        scale=min(1.,step/dist);new=tuple(near[j]+scale*(sample[j]-near[j]) for j in (0,1))
        length=math.dist(near,new);candidate=costs[i]+length
        intro=(f'Iteration {k}: {"goal-biased" if biased else "uniform random"} sample=({sample[0]:.2f}, {sample[1]:.2f}).\n'
               f'Nearest is n{i}=({near[0]:.2f}, {near[1]:.2f}); distance to sample={dist:.2f}.\n'
               f'Steer at most {step:.2f}: proposed node=({new[0]:.2f}, {new[1]:.2f}), edge length={length:.2f}.\n'
               f'Candidate g(new)=g(n{i})+edge={costs[i]:.2f}+{length:.2f}={candidate:.2f}.')
        yield Frame('rrt',current=near,nodes=nodes.copy(),edges=edges.copy(),sample=sample,
                    proposal=new,parent_node=near,attempt=(near,new,None),g=dict(zip(nodes,costs)),
                    focus={near},count=k,phase='preview',candidate_cost=candidate,
                    note=intro+'\nNEXT: check the entire proposed edge for collision; add it only if valid.')
        ok=segment_free(near,new)
        if ok:
            nodes.append(new);parents.append(i);edges.append((near,new));costs.append(candidate)
            if math.dist(new,GOAL)<=step and segment_free(new,GOAL):
                path=[GOAL];idx=len(nodes)-1
                while idx is not None:path.append(nodes[idx]);idx=parents[idx]
                path.reverse();cost=sum(math.dist(a,b) for a,b in zip(path,path[1:]))
                g=dict(zip(nodes,costs));g[GOAL]=cost
                yield Frame('rrt',current=new,parent_node=near,nodes=nodes.copy(),edges=edges.copy(),
                            g=g,path=path,count=k,phase='done',best_cost=cost,
                            note=f'Goal reached. Path cost={cost:.2f}; iterations={k}; tree nodes={len(nodes)}.\n'
                                 'Every displayed g is distance from S along the tree, not distance from the random sample.')
                return
        yield Frame('rrt',current=new if ok else near,nodes=nodes.copy(),edges=edges.copy(),
                    g=dict(zip(nodes,costs)),sample=sample,proposal=new,parent_node=near,
                    attempt=(near,new,ok),focus={near,new},count=k,phase='grow',
                    note=intro+('\nACCEPT: add the disk and its parent edge. NEXT: draw a new random sample.'
                                if ok else '\nREJECT: edge touches an obstacle. Tree and costs stay unchanged. NEXT: sample again.'))
    yield Frame('rrt',nodes=nodes,edges=edges,g=dict(zip(nodes,costs)),count=budget,phase='done',
                note='Budget exhausted. This does not prove no path exists.')

def draw(ax,frame,labels=False,all_costs=False):
    from matplotlib.patches import Rectangle
    from matplotlib.collections import LineCollection
    ax.clear();ax.set_aspect('equal');ax.set_xlim(-.5,WIDTH-.5);ax.set_ylim(-.5,HEIGHT-.5)
    ax.set_xticks(range(WIDTH));ax.set_yticks(range(HEIGHT));ax.tick_params(labelsize=8)
    ax.set_xlabel('x');ax.set_ylabel('y')
    for x0,y0,x1,y1 in OBSTACLES:ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,color='#303b48',zorder=3))
    if frame.kind=='grid':
        for pts,color in [(frame.closed,'#b5cde0'),(frame.frontier,'#a7dda8')]:
            for x,y in pts:ax.add_patch(Rectangle((x-.46,y-.46),.92,.92,color=color,zorder=1))
        if frame.current is not None:
            x,y=frame.current;ax.add_patch(Rectangle((x-.46,y-.46),.92,.92,color='#ffcc55',zorder=2))
        for x,y in frame.updated:
            ax.add_patch(Rectangle((x-.46,y-.46),.92,.92,fill=False,
                                   edgecolor='#e37c1c',linewidth=2,zorder=4))
        if frame.next_node is not None and frame.next_node!=frame.current:
            ax.scatter(*frame.next_node,s=220,facecolors='none',edgecolors='#b54db2',linewidths=2,zorder=6)
        if labels:
            for (x,y),g in frame.g.items():
                h=frame.h[(x,y)]
                ax.text(x,y,f'{g}+{h}\n={g+h}',ha='center',va='center',fontsize=5,zorder=5)
    else:
        if frame.edges:ax.add_collection(LineCollection(frame.edges,colors='#5596bf',linewidths=.8))
        if frame.nodes:
            ax.scatter(*zip(*frame.nodes),s=32,color='#5596bf',edgecolors='white',linewidths=.5,zorder=4)
        if frame.current is not None:
            ax.scatter(*frame.current,s=100,color='#ffcc55',edgecolors='#9b751c',zorder=6)
        if frame.parent_node is not None:
            ax.scatter(*frame.parent_node,s=165,facecolors='none',edgecolors='#119c91',linewidths=2,zorder=6)
        if frame.proposal is not None:
            ax.scatter(*frame.proposal,s=90,facecolors='none',edgecolors='#b54db2',linewidths=2,zorder=6)
        if labels and frame.proposal is not None and math.isfinite(frame.candidate_cost) and frame.proposal not in frame.g:
            ax.annotate(f'tentative g={frame.candidate_cost:.2f}',frame.proposal,
                        xytext=(5,-15),textcoords='offset points',fontsize=8,color='#8a3387',zorder=9)
        if frame.kind=='prm':
            for pts,color in [(frame.closed,'#395a80'),(frame.frontier,'#43ad60')]:
                if pts:ax.scatter(*zip(*pts),s=24,color=color,zorder=4)
            if frame.current is not None:ax.scatter(*frame.current,s=100,color='#ffcc55',zorder=5)
        for old,child,new in frame.rewired:
            ax.plot(*zip(old,child),'--',color='#cc4444',lw=2,zorder=4)
            ax.plot(*zip(new,child),color='#a13cc0',lw=2.5,zorder=5)
        if frame.sample is not None:ax.scatter(*frame.sample,marker='x',c='#bb55aa',s=50,zorder=5)
        if frame.attempt:
            a,b,ok=frame.attempt
            ax.plot([a[0],b[0]],[a[1],b[1]],'--',c='#7b7b7b' if ok is None else '#22aa55' if ok else '#dd3333',lw=2,zorder=5)
        if frame.next_node is not None and frame.next_node!=frame.current:
            ax.scatter(*frame.next_node,s=150,facecolors='none',edgecolors='#b54db2',linewidths=2,zorder=6)
        if labels:
            visible=set(frame.g) if all_costs or len(frame.g)<=35 else set(frame.nodes[-12:])
            visible.update(frame.focus)
            visible.update(q for q in [frame.current,frame.parent_node] if q is not None)
            if frame.kind=='prm' and not all_costs:
                visible=frame.focus | {q for q in [frame.current,frame.next_node] if q is not None}
            for q in visible:
                if q not in frame.g:continue
                ax.annotate(f'g={frame.g[q]:.2f}',q,xytext=(5,6),textcoords='offset points',
                            fontsize=7,color='#203c50',zorder=9,
                            bbox=dict(facecolor='white',edgecolor='none',alpha=.80,pad=.6))
    if frame.path:
        ax.plot(*zip(*frame.path),color='#e37c1c',lw=2.7,zorder=6)
    ax.scatter(*START,c='#218944',s=70,edgecolors='white',zorder=7)
    ax.scatter(*GOAL,c='#c33030',marker='*',s=150,edgecolors='white',zorder=7)
    ax.text(START[0]-.6,START[1]-.7,'S',weight='bold')
    ax.text(GOAL[0]+.3,GOAL[1]+.2,'G',weight='bold')

def launch(seed=7):
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button,RadioButtons,CheckButtons,Slider
    fig=plt.figure(figsize=(14,9));ax=fig.add_axes([.05,.33,.68,.60])
    explanation=fig.add_axes([.05,.095,.68,.205]);explanation.axis('off')
    status=explanation.text(0,1,'',va='top',fontsize=10)
    fig.text(.78,.95,'Planner',weight='bold')
    radio=RadioButtons(fig.add_axes([.78,.70,.20,.22]),['Dijkstra','A*','RRT','RRT*','PRM'])
    checks=CheckButtons(fig.add_axes([.78,.565,.20,.10]),['Show costs','All node costs'],[True,False])
    btns={}
    for label,y in [('Step',.49),('Play / Pause',.42),('Reset',.35),('New random seed',.28)]:
        btns[label]=Button(fig.add_axes([.78,y,.20,.05]),label)
    speed=Slider(fig.add_axes([.15,.035,.49,.025]),'Events / sec',1,20,valinit=2,valstep=1)
    fig.text(.78,.075,'Disk: tree / roadmap node\nYellow: selected / nearest node\nTeal ring: selected parent\nPurple ring: proposed / NEXT node\nGray dash: unchecked proposal\nGreen / red dash: accepted / rejected\nPurple edge: RRT* rewire\nOrange: best path / grid cost update\ng: cost from S along current edges',fontsize=8.5)
    state=dict(planner='Dijkstra',seed=seed,running=False,labels=True,all_costs=False,
               done=False,frame=Frame('grid'))
    def reset(event=None):
        state['running']=False;state['done']=False;state['last_tick']=time.monotonic()
        from sampling_planners import rrt_star,prm
        factories={'Dijkstra':lambda:grid_search(False),'A*':lambda:grid_search(True),
                   'RRT':lambda:rrt(state['seed']),'RRT*':lambda:rrt_star(state['seed']),
                   'PRM':lambda:prm(state['seed'])}
        state['gen']=factories[state['planner']]()
        state['frame']=Frame({'RRT':'rrt','RRT*':'rrtstar','PRM':'prm'}.get(state['planner'],'grid'));refresh()
    def refresh():
        draw(ax,state['frame'],state['labels'],state['all_costs'])
        phase=state['frame'].phase
        phase_title={'select':'Selected node: predict its neighbor costs',
                     'expand':'Neighbors inspected: next node is ringed',
                     'preview':'Preview: predict the connection',
                     'edge_preview':'Preview: check a proposed roadmap edge',
                     'sample_preview':'Preview: test a sampled configuration',
                     'grow':'Tree updated',
                     'parent':'Compare parents before adding the node',
                     'done':'Run complete'}.get(phase,'Shared obstacle geometry')
        ax.set_title(state['planner']+' | '+phase_title,fontsize=12)
        step_label=('Reveal path' if phase=='select' and state['frame'].current==GOAL
                    else 'Inspect neighbors' if phase=='select'
                    else 'Check / connect' if phase in ['preview','parent','edge_preview','sample_preview']
                    else 'Select next node' if state['planner'] in ['Dijkstra','A*'] or phase=='expand'
                    else 'Next sample / edge')
        btns['Step'].label.set_text(step_label)
        note=state['frame'].note or ('NEXT: select the minimum-cost frontier node, then inspect its neighbors.'
                 if state['planner'] in ['Dijkstra','A*'] else
                 'NEXT: draw a sample; the preview will explain which node or edge is considered and why.')
        status.set_text('\n'.join(textwrap.fill(line,width=105) for line in note.splitlines()))
        fig.canvas.draw_idle()
    def advance(event=None):
        if state['done']:return
        try:state['frame']=next(state['gen'])
        except StopIteration:state['done']=True;state['running']=False
        if state['frame'].phase=='done' or (state['frame'].path and state['planner']=='RRT'):
            state['done']=True;state['running']=False
        refresh()
    def manual_step(event):
        state['running']=False;advance()
    def play(event):
        state['running']=not state['running'] if not state['done'] else False
        state['last_tick']=time.monotonic()
    def choose(label):state['planner']=label;reset()
    def labels(label):
        key='labels' if label=='Show costs' else 'all_costs'
        state[key]=not state[key];refresh()
    def reseed(event):state['seed']+=1;reset()
    def tick():
        if state['running'] and time.monotonic()-state['last_tick']>=1/float(speed.val):
            state['last_tick']=time.monotonic();advance()
    radio.on_clicked(choose);checks.on_clicked(labels)
    btns['Step'].on_clicked(manual_step);btns['Play / Pause'].on_clicked(play)
    btns['Reset'].on_clicked(reset);btns['New random seed'].on_clicked(reseed)
    timer=fig.canvas.new_timer(interval=50);timer.add_callback(tick);timer.start()
    reset();plt.show()

def verify():
    # Costs independently checked by Bellman-Ford on the worked graph.
    dist={n:math.inf for n in HEURISTIC};dist['S']=0
    for _ in range(len(dist)-1):
        for a,b,c in EDGES:dist[b]=min(dist[b],dist[a]+c);dist[a]=min(dist[a],dist[b]+c)
    for a,b,c in EDGES:assert abs(HEURISTIC[a]-HEURISTIC[b])<=c
    for astar in [False,True]:
        rows,path,cost=graph_trace(astar);assert cost==dist['G']==14
        print('Graph', 'A*' if astar else 'Dijkstra',rows,path,cost)
    results=[list(grid_search(a))[-1] for a in [False,True]]
    assert len(results[0].path)==len(results[1].path)>1
    for f in results:
        assert all(segment_free(a,b) for a,b in zip(f.path,f.path[1:]))
        print(f.note)
    assert not segment_free((4,2),(6,2))
    assert segment_free((4,8),(6,8))
    for seed in [7,8,9]:
        last=None
        for last in rrt(seed):pass
        assert last.path, f'No solution for seed {seed}'
        assert all(segment_free(a,b) for a,b in zip(last.path,last.path[1:]))
        print('RRT seed',seed,last.note)
    print('Verification passed.')
if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--check',action='store_true');parser.add_argument('--seed',type=int,default=7)
    args=parser.parse_args()
    if args.check:verify()
    else:launch(args.seed)

