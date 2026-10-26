"""Deterministic occupancy grid and forward-only Hybrid A* for Ackermann steering.
Unknown space may be planned through, but only short sensor-checked segments execute.
"""
import math
import time
import numpy as np
from shared.kinematics import steering_angle


class GoalBlocked(RuntimeError):
    """No goal region is currently free; reversing is not a remedy."""


class PlanningTimeout(RuntimeError):
    """Search budget exhausted; this does not establish a physical blockage."""


def advance(pose, distance, steer, wheelbase):
    x,y,t = pose
    turn = distance*math.tan(math.radians(steer))/wheelbase
    if abs(turn) < 1e-8:
        return (x+distance*math.cos(t), y+distance*math.sin(t), t)
    r=distance/turn
    return (x+r*(math.sin(t+turn)-math.sin(t)), y-r*(math.cos(t+turn)-math.cos(t)), (t+turn+math.pi)%(2*math.pi)-math.pi)


class Grid:
    def __init__(self, cfg):
        self.cfg=cfg
        self.xmin,self.xmax,self.ymin,self.ymax=cfg['bounds']
        self.cell=cfg['grid_cm']
        self.data=np.zeros((math.ceil((self.xmax-self.xmin)/self.cell), math.ceil((self.ymax-self.ymin)/self.cell)),dtype=np.uint8)
        self.observed=np.zeros_like(self.data)
        self.obstacle_age=np.zeros_like(self.data)
        self.inflated=np.zeros_like(self.data)
        self.cost=np.ones_like(self.data,dtype=float)
        self.scans=0
        self.obstacle_points=np.empty((0,2));self.obstacle_tree=None

    def index(self,x,y):
        return (int(math.floor((x-self.xmin)/self.cell)), int(math.floor((y-self.ymin)/self.cell)))

    def inside(self,idx):
        return 0<=idx[0]<self.data.shape[0] and 0<=idx[1]<self.data.shape[1]

    def world(self,idx):
        return (self.xmin+(idx[0]+.5)*self.cell, self.ymin+(idx[1]+.5)*self.cell)

    def line(self,a,b):
        steps=max(abs(b[0]-a[0]),abs(b[1]-a[1]),1)
        return list(dict.fromkeys((round(a[0]+(b[0]-a[0])*i/steps),round(a[1]+(b[1]-a[1])*i/steps)) for i in range(steps+1)))

    def update(self,pose,readings):
        # Keep a short rolling obstacle memory: glancing sonar misses must not
        # erase a box before the rear of the car has passed it. Expiration avoids
        # the permanent ghost walls seen with unbounded dead-reckoning maps.
        self.obstacle_age=np.maximum(self.obstacle_age.astype(np.int16)-1,0).astype(np.uint8)
        self.data[:]=self.obstacle_age>0
        x,y,t=pose
        offset=self.cfg['sensor_offset_cm']
        sx,sy=x+offset*math.cos(t),y+offset*math.sin(t)
        origin=self.index(sx,sy)
        hits=[]
        previous=None
        for reading in readings:
            d=reading['distance']
            if d is None:
                previous=None
                continue
            angle=t+math.radians(reading['angle'])
            ray=min(d,self.cfg['scan_max_cm'])
            endpoint=self.index(sx+ray*math.cos(angle),sy+ray*math.sin(angle))
            cells=self.line(origin,endpoint)
            for c in cells[:-1]:
                if self.inside(c):
                    self.observed[c]=1
            if d < self.cfg['scan_max_cm']:
                hits.append(endpoint)
                if previous is not None and abs(d-previous[0])<18 and abs(reading['angle']-previous[2])<=11:
                    hits.extend(self.line(previous[1],endpoint))
                previous=(d,endpoint,reading['angle'])
            else:
                previous=None
        # Apply occupied endpoints after freespace rays, so same-scan rays cannot erase walls.
        for c in hits:
            if self.inside(c):
                self.data[c]=1; self.observed[c]=1; self.obstacle_age[c]=self.cfg.get('obstacle_memory_scans',4)
        self.inflate()
        self.scans+=1

    def inflate(self):
        from scipy.ndimage import distance_transform_edt
        if not np.any(self.data):
            self.inflated[:]=0
            self.cost[:]=1.
        else:
            distances=distance_transform_edt(1-self.data)*self.cell
            self.inflated[:]=(distances<=self.minimum_radius()+self.cell*.5)
            # Soft obstacle cost favors roomier paths; hard footprint checks remain.
            self.cost=1.+self.cfg.get('obstacle_cost_weight',2.)*np.exp(-np.maximum(distances-self.minimum_radius(),0.)/12.)

        from scipy.spatial import cKDTree
        indices=np.argwhere(self.data)
        self.obstacle_points=np.array([self.world(tuple(i)) for i in indices]).reshape(-1,2)
        self.obstacle_tree=cKDTree(self.obstacle_points) if len(indices) else None

    def minimum_radius(self):
        if 'body_width_cm' not in self.cfg:return self.cfg['radius_cm']
        return min(self.cfg['front_bumper_offset_cm'],self.cfg['rear_overhang_cm'],self.cfg['body_width_cm']/2)+self.cfg['clearance_cm']

    def free(self,pose):
        x,y=pose[:2]
        if 'body_width_cm' not in self.cfg or len(pose)<3:
            # Relaxed circular clearance for the 2-D Dijkstra guide; full
            # heading-dependent rectangle checks validate every actual arc.
            r=self.minimum_radius()
            if not self.xmin+r<=x<=self.xmax-r or not self.ymin+r<=y<=self.ymax-r:return False
            cell=self.index(x,y)
            return self.inside(cell) and not self.inflated[cell]
        theta=pose[2];c=math.cos(theta);s=math.sin(theta)
        front=self.cfg['front_bumper_offset_cm'];rear=self.cfg['rear_overhang_cm']
        margin=self.cfg['clearance_cm'];a=(front+rear)/2+margin;b=self.cfg['body_width_cm']/2+margin
        center=np.array([x+(front-rear)/2*c,y+(front-rear)/2*s])
        extent_x=a*abs(c)+b*abs(s);extent_y=a*abs(s)+b*abs(c)
        if not(self.xmin<=center[0]-extent_x and center[0]+extent_x<=self.xmax and self.ymin<=center[1]-extent_y and center[1]+extent_y<=self.ymax):return False
        if self.obstacle_tree is None:return True
        half_cell=self.cell/2
        candidates=self.obstacle_tree.query_ball_point(center,math.hypot(a,b)+half_cell*math.sqrt(2))
        if not candidates:return True
        delta=self.obstacle_points[candidates]-center
        # Separating-axis rectangle/square intersection: occupied cells have
        # their full area, and the measured car has a clearance margin.
        local_x=delta[:,0]*c+delta[:,1]*s
        local_y=-delta[:,0]*s+delta[:,1]*c
        projection=half_cell*(abs(c)+abs(s))
        overlap=(np.abs(local_x)<=a+projection)&(np.abs(local_y)<=b+projection)&(np.abs(delta[:,0])<=extent_x+half_cell)&(np.abs(delta[:,1])<=extent_y+half_cell)
        return not np.any(overlap)

    def export(self):
        return {'cell_cm':self.cell,'bounds':[self.xmin,self.xmax,self.ymin,self.ymax],
                'occupied':self.data.tolist(),'inflated':self.inflated.tolist(),
                'observed':self.observed.tolist(),'scans':self.scans}


def hybrid_astar(grid,start,goal,timeout=5.):
    """Use jrialland/astar's search engine with car-specific motion primitives."""
    from astar import AStar
    if not grid.free(start):
        raise RuntimeError('Estimated car footprint overlaps obstacle clearance; inspect map/reset pose')
    if not (grid.xmin<=goal[0]<=grid.xmax and grid.ymin<=goal[1]<=grid.ymax):
        raise RuntimeError('Goal is outside map bounds')
    deadline=time.monotonic()+timeout
    # SciPy computes a 2-D obstacle-aware distance field. This guides the packaged
    # A* around walls instead of exploring thousands of equally near headings.
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import dijkstra
    shape=grid.data.shape; total=grid.data.size
    rows=[];cols=[];weights=[];goals=[]
    free=np.zeros(shape,dtype=bool)
    for ix in range(shape[0]):
        for iy in range(shape[1]):
            p=grid.world((ix,iy)); free[ix,iy]=grid.free(p)
            if free[ix,iy] and math.hypot(p[0]-goal[0],p[1]-goal[1])<=grid.cfg['goal_tolerance_cm']:
                goals.append(ix*shape[1]+iy)
    for ix,iy in np.argwhere(free):
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            nx,ny=ix+dx,iy+dy
            if not(0<=nx<shape[0] and 0<=ny<shape[1]) or not free[nx,ny]: continue
            if dx and dy and not(free[ix+dx,iy] and free[ix,iy+dy]):continue
            rows.append(ix*shape[1]+iy);cols.append(nx*shape[1]+ny);weights.append(grid.cell*math.hypot(dx,dy)*(grid.cost[ix,iy]+grid.cost[nx,ny])/2)
    if not goals: raise GoalBlocked('No free goal cells')
    graph=coo_matrix((weights,(rows,cols)),shape=(total,total)).tocsr()
    distances=dijkstra(graph,directed=False,indices=goals,min_only=True).reshape(shape)
    def key(p):
        return (round(p[0]/2.5),round(p[1]/2.5),round((p[2]%(2*math.pi))/math.radians(10))%36)
    initial=key(start)
    poses={initial:tuple(start)}
    actions={}
    class CarSearch(AStar):
        def heuristic_cost_estimate(self,node,unused):
            p=poses[node]
            direct=max(0.,math.hypot(p[0]-goal[0],p[1]-goal[1])-grid.cfg['goal_tolerance_cm'])
            around=distances[grid.index(*p[:2])]
            return 1.15*max(direct,around-grid.cell*1.5)  # Weighted Hybrid A*: practical speed, not an optimality claim.
        def distance_between(self,a,b):
            return 5.*(grid.cost[grid.index(*poses[a][:2])]+grid.cost[grid.index(*poses[b][:2])])+abs(actions[a,b])*.012
        def is_goal_reached(self,node,unused):
            p=poses[node]
            return math.hypot(p[0]-goal[0],p[1]-goal[1])<=grid.cfg['goal_tolerance_cm']
        def neighbors(self,node):
            if time.monotonic()>deadline:
                raise PlanningTimeout('Planning time limit; inspect map or choose a clearer goal')
            found=[]
            for steer in (0.,-30.,30.,-15.,15.):
                samples=[advance(poses[node],d,steering_angle(steer,grid.cfg),grid.cfg['wheelbase_cm']) for d in range(1,11)]
                if not all(grid.free(p) for p in samples): continue
                dest=key(samples[-1])
                if dest==node: continue
                # Each key has a fixed representative pose, avoiding moving graph nodes.
                if dest not in poses: poses[dest]=samples[-1]
                if not grid.free(poses[dest]): continue
                if not math.isfinite(distances[grid.index(*poses[dest][:2])]):continue
                if (node,dest) not in actions: actions[node,dest]=steer
                if dest not in found: found.append(dest)
            return found
    path=CarSearch().astar(initial,('goal',))
    if path is None: raise RuntimeError('No collision-free forward route found')
    nodes=list(path)
    return [{'pose':poses[b],'steer':actions[a,b],'distance':10.} for a,b in zip(nodes,nodes[1:])]


def approach_blocked_goal(grid,start,goal):
    """Approach a distant blocked goal for a new viewpoint, without clearing hits.

    Every candidate uses the same footprint-aware A* and current occupancy.
    Near a blocked destination we stop instead of substituting an arrival point.
    """
    remaining=math.hypot(goal[0]-start[0],goal[1]-start[1])
    if remaining<=80:return None
    bearing=math.atan2(goal[1]-start[1],goal[0]-start[0])
    deadline=time.monotonic()+5.
    for offset in (0,20,-20,40,-40):
        angle=bearing+math.radians(offset)
        waypoint=(start[0]+50*math.cos(angle),start[1]+50*math.sin(angle))
        if not grid.free(waypoint):continue
        budget=deadline-time.monotonic()
        if budget<=0:break
        try:
            route=hybrid_astar(grid,start,waypoint,timeout=min(1.5,budget))
        except RuntimeError:
            continue
        if route:return waypoint,route
    return None
