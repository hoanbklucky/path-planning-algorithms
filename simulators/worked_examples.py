"""Print and verify the numerical examples in docs without opening a GUI."""
import heapq
import math
from path_planning_sim import graph_trace

def worked_prm(remove=()):
    vertices={'S':(2,5),'A':(2,8),'B':(5,8),'C':(8,8),
              'D':(2,1),'E':(5,1),'F':(8,1),'G':(8,5)}
    vertices={n:q for n,q in vertices.items() if n not in remove}
    # This example's candidate edges lie entirely above, below, left, or right of its rectangle.
    # Validate segment/rectangle intersection explicitly for all radius candidates.
    def free(a,b):
        lo,hi=0.,1.
        for p,delta,mn,mx in [(a[0],b[0]-a[0],4,6),(a[1],b[1]-a[1],3,7)]:
            if abs(delta)<1e-12:
                if not mn<=p<=mx:return True
            else:
                enter,leave=sorted(((mn-p)/delta,(mx-p)/delta))
                lo=max(lo,enter);hi=min(hi,leave)
                if lo>hi:return True
        return False
    adj={n:[] for n in vertices}
    for a,qa in vertices.items():
        for b,qb in vertices.items():
            cost=math.dist(qa,qb)
            if a!=b and cost<=4.1 and free(qa,qb):adj[a].append((b,cost))
    g={'S':0};parent={};queue=[(0,'S')]
    while queue:
        gu,u=heapq.heappop(queue)
        if gu!=g[u]:continue
        if u=='G':
            path=[u]
            while u in parent:u=parent[u];path.append(u)
            return path[::-1],gu
        for v,c in adj[u]:
            if gu+c<g.get(v,math.inf):g[v]=gu+c;parent[v]=u;heapq.heappush(queue,(gu+c,v))
    return [],math.inf

def main():
    for a in [False,True]:
        rows,path,cost=graph_trace(a)
        print('\nA*' if a else '\nDijkstra')
        for row in rows:print(row)
        print(path,cost)
        assert cost==14
    print('\nRRT*: parent candidate costs for Q')
    q=(3,2);candidates={'A':((2,2),9),'B':((3,4),5),'C':((4,2),10)}
    costs={n:g+math.dist(v,q) for n,(v,g) in candidates.items()}
    assert costs=={'A':10.,'B':7.,'C':11.}
    print(costs,'choose B; rewire A to cost 8, C to cost 8; D becomes 9.')
    for remove,expected in [((),12),(('B',),14),(('B','E'),math.inf)]:
        path,cost=worked_prm(remove);assert cost==expected
        print('PRM removed',remove,'path',path,'cost',cost)
if __name__=='__main__':main()
