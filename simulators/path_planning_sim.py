"""Teaching simulator: Dijkstra, A*, and basic RRT. Python 3 + matplotlib."""
import argparse
import heapq
import itertools
import math
import random
import time
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
                    g=g.copy(),h={n:heur(n) for n in g},count=len(closed),phase='select',
                    note=f'SELECT {u}: g={gu}, h={heur(u)}, f={gu+heur(u)}. Completed expansions={len(closed)}.\n'
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
        yield Frame('grid',current=u,closed=closed.copy(),frontier=front.copy(),
                    g=g.copy(),h={n:heur(n) for n in g},count=len(closed),phase='expand',
                    updated=updated,
                    note=f'INSPECT neighbors of {u}: {len(updated)} cost update(s). Completed expansions={len(closed)}.\n'
                         'Orange outlines mark new or improved costs. Next: select the next node.')
    yield Frame('grid',closed=closed,count=len(closed),note='No path exists on this grid.')

def rrt(seed=7,step=1.5,goal_bias=.1,budget=5000):
    rng=random.Random(seed);nodes=[START];parents=[None];edges=[]
    for k in range(1,budget+1):
        sample=GOAL if rng.random()<goal_bias else (rng.uniform(-.5,WIDTH-.5),rng.uniform(-.5,HEIGHT-.5))
        i=min(range(len(nodes)),key=lambda i:math.dist(nodes[i],sample));near=nodes[i]
        dist=math.dist(near,sample)
        if dist<1e-12:continue
        scale=min(1.,step/dist);new=tuple(near[j]+scale*(sample[j]-near[j]) for j in (0,1))
        ok=segment_free(near,new)
        if ok:
            nodes.append(new);parents.append(i);edges.append((near,new))
            if math.dist(new,GOAL)<=step and segment_free(new,GOAL):
                path=[GOAL];idx=len(nodes)-1
                while idx is not None:path.append(nodes[idx]);idx=parents[idx]
                path.reverse();cost=sum(math.dist(a,b) for a,b in zip(path,path[1:]))
                yield Frame('rrt',nodes=nodes.copy(),edges=edges.copy(),path=path,count=k,
                            note=f'Goal reached. Length={cost:.2f}; iterations={k}; tree nodes={len(nodes)}.')
                return
        yield Frame('rrt',current=new,nodes=nodes.copy(),edges=edges.copy(),sample=sample,
                    attempt=(near,new,ok),count=k,
                    note=f'Iteration {k}: {"accept" if ok else "reject collision"}; tree nodes={len(nodes)}; seed={seed}.')
    yield Frame('rrt',nodes=nodes,edges=edges,count=budget,note='Budget exhausted. This does not prove no path exists.')

def draw(ax,frame,labels=False):
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
        if labels:
            for (x,y),g in frame.g.items():
                h=frame.h[(x,y)]
                ax.text(x,y,f'{g}+{h}\n={g+h}',ha='center',va='center',fontsize=5,zorder=5)
    else:
        if frame.edges:ax.add_collection(LineCollection(frame.edges,colors='#5596bf',linewidths=.8))
        if frame.nodes:
            ax.scatter(*zip(*frame.nodes),s=7,color='#5596bf',zorder=2)
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
            ax.plot([a[0],b[0]],[a[1],b[1]],'--',c='#22aa55' if ok else '#dd3333',lw=2,zorder=5)
    if frame.path:
        ax.plot(*zip(*frame.path),color='#e37c1c',lw=2.7,zorder=6)
    ax.scatter(*START,c='#218944',s=70,edgecolors='white',zorder=7)
    ax.scatter(*GOAL,c='#c33030',marker='*',s=150,edgecolors='white',zorder=7)
    ax.text(START[0]-.6,START[1]-.7,'S',weight='bold')
    ax.text(GOAL[0]+.3,GOAL[1]+.2,'G',weight='bold')

def launch(seed=7):
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button,RadioButtons,CheckButtons,Slider
    fig=plt.figure(figsize=(12,8));ax=fig.add_axes([.07,.20,.68,.73])
    status=fig.text(.07,.1,'',fontsize=10)
    fig.text(.79,.91,'Planner',weight='bold')
    radio=RadioButtons(fig.add_axes([.79,.70,.18,.20]),['Dijkstra','A*','RRT','RRT*','PRM'])
    checks=CheckButtons(fig.add_axes([.79,.60,.18,.07]),['Show g+h=f'],[False])
    btns={}
    for label,y in [('Step',.53),('Play / Pause',.45),('Reset',.37),('New RRT seed',.29)]:
        btns[label]=Button(fig.add_axes([.79,y,.18,.055]),label)
    speed=Slider(fig.add_axes([.15,.035,.5,.025]),'Events / sec',1,20,valinit=2,valstep=1)
    fig.text(.79,.12,'Blue: expanded / graph\nGreen: frontier / accepted\nYellow: selected node\nOrange outline: cost update\nRed dashed: rejected / old edge\nPurple line: rewired edge\nPurple ×: random sample\nOrange line: current best path',fontsize=8)
    state=dict(planner='Dijkstra',seed=seed,running=False,labels=False,done=False,frame=Frame('grid'))
    def reset(event=None):
        state['running']=False;state['done']=False;state['last_tick']=time.monotonic()
        from sampling_planners import rrt_star,prm
        factories={'Dijkstra':lambda:grid_search(False),'A*':lambda:grid_search(True),
                   'RRT':lambda:rrt(state['seed']),'RRT*':lambda:rrt_star(state['seed']),
                   'PRM':lambda:prm(state['seed'])}
        state['gen']=factories[state['planner']]()
        state['frame']=Frame({'RRT':'rrt','RRT*':'rrtstar','PRM':'prm'}.get(state['planner'],'grid'));refresh()
    def refresh():
        draw(ax,state['frame'],state['labels'])
        phase=state['frame'].phase
        phase_title={'select':'1 / 2  Selected node — predict its neighbor costs',
                     'expand':'2 / 2  Neighbors inspected — costs revealed'}.get(phase,'Shared obstacle geometry')
        ax.set_title(state['planner']+'   |   '+phase_title,fontsize=12)
        btns['Step'].label.set_text('Reveal path' if phase=='select' and state['frame'].current==GOAL
                                   else 'Inspect neighbors' if phase=='select'
                                   else 'Select next node' if state['planner'] in ['Dijkstra','A*'] or phase=='expand' else 'Step')
        btns['New RRT seed'].label.set_text('New random seed')
        status.set_text(state['frame'].note or 'Press Select next node or Play.\nGrid search alternates selection and neighbor inspection.')
        fig.canvas.draw_idle()
    def advance(event=None):
        if state['done']:return
        try:state['frame']=next(state['gen'])
        except StopIteration:state['done']=True;state['running']=False
        if state['frame'].phase=='done' or (state['frame'].path and state['planner']=='RRT'):
            state['done']=True;state['running']=False
        refresh()
    def manual_step(event):
        state['running']=False
        advance()
    def play(event):
        state['running']=not state['running'] if not state['done'] else False
        state['last_tick']=time.monotonic()
    def choose(label):state['planner']=label;reset()
    def labels(label):state['labels']=not state['labels'];refresh()
    def reseed(event):state['seed']+=1;reset()
    def tick():
        if state['running'] and time.monotonic()-state['last_tick']>=1/float(speed.val):
            state['last_tick']=time.monotonic()
            # Exactly one visible event per tick; never batch past selection.
            advance()
    radio.on_clicked(choose);checks.on_clicked(labels)
    btns['Step'].on_clicked(manual_step);btns['Play / Pause'].on_clicked(play)
    btns['Reset'].on_clicked(reset);btns['New RRT seed'].on_clicked(reseed)
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
