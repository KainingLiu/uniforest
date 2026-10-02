"""Free configuration-space search, with no prescribed passage coordinates.

The grid is a numerical search discretization, never a set of mandatory robot
waypoints. Endpoint connections, visibility shortcuts and continuous smoothing
remove it from the executable reference whenever clearance permits.
"""
import heapq
import math
import numpy as np
from .planner import Pose, NavigationError, wrap


class PoseSearch:
    def __init__(self,planner):
        self.p=planner
        b=planner.field['playable_bounds_mm']
        self.xs=np.arange(b['left']+100,b['right'],100.)
        self.ys=np.arange(b['top']+100,b['bottom'],100.)
        self.nx,self.ny=len(self.xs),len(self.ys); self.nh=12
        xy=np.array([(x,y) for y in self.ys for x in self.xs])
        self.xy=xy
        self.poses=np.array([(x,y,h*30) for x,y in xy for h in range(12)])
        self.clear=planner.clearance(self.poses)
        self.valid=(self.clear>=75)&self.terrain(self.poses)
        self.rotate=planner.centre_clearance(xy)>=planner.radius+75
        self.directions=((1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1))
        self.edges={}
        self.moving_rotation={}
        for k,(dx,dy) in enumerate(self.directions):
            samples=xy[:,None,:]+np.linspace(0,1,7)[None,:,None]*np.array([dx*100,dy*100])
            free=planner.centre_clearance(samples.reshape(-1,2)).reshape(-1,7).min(axis=1)
            off_ramp=((samples[:,:,1]<=4100)|(samples[:,:,1]>=5100)).all(axis=1)
            self.moving_rotation[k]=(free>=planner.radius+75)&off_ramp
        # Seven samples plus >=75 mm clearance bound every constant-heading
        # 100/sqrt(2)*100 mm grid edge, including thin/corner obstacle crossings.
        for h in range(12):
            for k,(dx,dy) in enumerate(self.directions):
                starts=np.c_[xy,np.full(len(xy),h*30.)]
                samples=starts[:,None,:]+np.linspace(0,1,7)[None,:,None]*np.array([dx*100,dy*100,0])
                clearance=planner.clearance(samples.reshape(-1,3)).reshape(-1,7).min(axis=1)
                self.edges[h,k]=(clearance>=75)

    def terrain(self,poses):
        q=np.atleast_2d(poses)
        # Infer ramp bounds from geometry, not navigation anchors. Keep heading
        # parallel to its ascent/descent axis while on the ramp, including a
        # 100 mm transition band. Either forward or reverse travel is legal.
        ramp=next(s for s in self.p.field['shapes'] if s['id']=='ramp_blue')
        r=np.array(ramp['polygon']); lo,hi=r.min(axis=0),r.max(axis=0)
        on=((q[:,0]>lo[0])&(q[:,0]<hi[0])&(q[:,1]>lo[1]-100)&(q[:,1]<hi[1]+100))
        aligned=np.abs((q[:,2]-90+90)%180-90)<2
        return (~on)|aligned

    def line(self,a,b,step=12):
        d=np.array(b)-np.array(a)
        count=max(2,int(max(np.linalg.norm(d[:2])/step,abs(d[2])/1.0))+2)
        return np.array(a)+np.linspace(0,1,count)[:,None]*d

    def segment(self,a,b):
        q=self.line(a,b)
        c=self.p.clearance(q)
        if c.min() < -1e-6 or not self.terrain(q).all(): return None
        speed=self.p.motion.clearance_speed(c)
        ds=np.linalg.norm(np.diff(q[:,:2],axis=0),axis=1)
        t=float(np.sum(ds/np.minimum(speed[:-1],speed[1:])))
        return max(t,abs(b[2]-a[2])/self.p.motion.yaw_deg_s)

    def attachments(self,pose):
        a=np.array([pose.x,pose.y,pose.yaw])
        dist=np.linalg.norm(self.poses[:,:2]-a[:2],axis=1)
        angle=np.abs((self.poses[:,2]-a[2]+180)%360-180)
        ids=np.flatnonzero(self.valid & (dist<=230)&(angle<=30.001))
        ids=sorted(ids,key=lambda i:dist[i]+angle[i]*6)
        result=[]
        for i in ids[:36]:
            b=self.poses[i].copy(); b[2]=a[2]+wrap(b[2]-a[2])
            cost=self.segment(a,b)
            if cost is not None: result.append((int(i),cost))
        if not result: raise NavigationError('operation pose has no collision-free configuration-space connection')
        return result

    def find(self,start,goal):
        starts=self.attachments(start); goals=dict(self.attachments(goal))
        costs={}; previous={}; heap=[]; counter=0
        def heuristic(i):
            p=self.poses[i]
            return max(math.hypot(p[0]-goal.x,p[1]-goal.y)/self.p.motion.cruise_mm_s,abs(wrap(p[2]-goal.yaw))/self.p.motion.yaw_deg_s)
        for i,c in starts:
            costs[i]=c; previous[i]=None; heapq.heappush(heap,(c+heuristic(i),counter,i,c));counter+=1
        winner=None; best=float('inf')
        while heap:
            priority,_,i,cost=heapq.heappop(heap)
            if cost!=costs.get(i): continue
            if priority>=best: break
            if i in goals and cost+goals[i]<best:
                winner=i;best=cost+goals[i]
            cell,h=divmod(i,12); y,x=divmod(cell,self.nx)
            successors=[]
            for k,(dx,dy) in enumerate(self.directions):
                xx,yy=x+dx,y+dy
                if not(0<=xx<self.nx and 0<=yy<self.ny) or not self.edges[h,k][cell]: continue
                j=(yy*self.nx+xx)*12+h
                if not self.valid[j]: continue
                clearance=min(self.clear[i],self.clear[j])
                speed=float(self.p.motion.clearance_speed(clearance))
                if 4100<self.poses[i,1]<5100: speed=min(speed,self.p.motion.ramp_mm_s)
                successors.append((j,100*math.hypot(dx,dy)/speed))
                if self.moving_rotation[k][cell]:
                    for sign in (-1,1):
                        coupled=(yy*self.nx+xx)*12+(h+sign)%12
                        if self.valid[coupled]:
                            successors.append((coupled,max(100*math.hypot(dx,dy)/speed,30/self.p.motion.yaw_deg_s)))
            if self.rotate[cell]:
                for sign in (-1,1):
                    j=cell*12+(h+sign)%12
                    if self.valid[j]: successors.append((j,30/self.p.motion.yaw_deg_s))
            for j,step in successors:
                new=cost+step
                if new<costs.get(j,float('inf')):
                    costs[j]=new; previous[j]=i
                    heapq.heappush(heap,(new+heuristic(j),counter,j,new));counter+=1
        if winner is None: raise NavigationError('no connected free-space path between operation poses')
        ids=[]; i=winner
        while i is not None: ids.append(i);i=previous[i]
        values=[np.array([start.x,start.y,start.yaw])]
        for i in reversed(ids):
            q=self.poses[i].copy();q[2]=values[-1][2]+wrap(q[2]-values[-1][2])
            if np.linalg.norm(q-values[-1])>.001: values.append(q)
        end=np.array([goal.x,goal.y,values[-1][2]+wrap(goal.yaw-values[-1][2])])
        if np.linalg.norm(end-values[-1])>.001: values.append(end)
        return self.compress(values)

    @staticmethod
    def compress(values):
        out=[]
        for q in values:
            if out and np.linalg.norm(q-out[-1])<.001: continue
            while len(out)>=2:
                a,b=out[-1]-out[-2],q-out[-1]
                if np.dot(a,b)>.99999*np.linalg.norm(a)*np.linalg.norm(b): out.pop()
                else: break
            out.append(q)
        return out

    def shortcut(self,values):
        result=[values[0]];i=0
        while i<len(values)-1:
            # Compare feasible traversal cost; do not force a slow wall-hugging
            # line when a smooth clearance excursion is materially faster.
            best=i+1; original=0.
            for j in range(i+1,len(values)):
                old=self.segment(values[j-1],values[j])
                original+=old if old is not None else 1e9
                direct=self.segment(values[i],values[j])
                if direct is not None and direct<=original*1.02: best=j
            result.append(values[best]); i=best
        return self.compress(result)
