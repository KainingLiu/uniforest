"""Parameterized operation positions and free-space, full-pose route planning.

World units: mm, clockwise degrees, x right/y down. Geometry is supplied by the
caller; this module never imports simulation or assumes that CAD is surveyed.
Each endpoint is a robot operation pose, not the point on a material wall.
"""
from dataclasses import dataclass
import math
import numpy as np
from .motion import CompetitionMotion, route_timing
from .tracking import TrackingGains
from control.chassis import Chassis, LATERAL_DISTANCE_SCALE


class NavigationError(RuntimeError):
    pass


def wrap(value):
    return (value + 180) % 360 - 180


def smooth(u):
    return u*u*u*(10 + u*(-15 + 6*u))


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float

    def __post_init__(self):
        if any(isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v)
               for v in (self.x, self.y, self.yaw)):
            raise ValueError('pose must contain finite numeric coordinates')


@dataclass(frozen=True)
class Location:
    family: str
    along: float = .5
    gap_mm: float = 20.0

    def __post_init__(self):
        for v in (self.along, self.gap_mm):
            if isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v):
                raise ValueError('location parameters must be finite numbers')
        if not 0 <= self.along <= 1 or not 0 <= self.gap_mm <= 120:
            raise ValueError('along must be within [0,1], gap within [0,120] mm')


@dataclass
class Route:
    start: Pose
    goal: Pose
    source: Location
    target: Location
    samples: np.ndarray  # x,y,yaw,arc length, conservative body clearance,speed cap
    via: tuple
    needs_tag6: bool
    nominal_length_mm: float
    blend_radius_mm: float = 0.0
    candidate_count: int = 1
    motion: CompetitionMotion | None = None
    tracking: TrackingGains | None = None

    def at(self, distance):
        s = self.samples[:, 3]
        return np.array([np.interp(distance, s, self.samples[:, i]) for i in (0, 1, 2, 4, 5)])


class FieldPlanner:
    """Small interface: resolve an operation position and plan one directed pair.

    Position families describe goals; free-space search and continuous candidate
    optimization generate motion per request. Invalid endpoints are rejected,
    never silently snapped to a convenient demonstration point.
    """
    LABELS = {'start': '启动区墙角', 'orange_ground': '地面橙块取块墙',
              'orange_highland': '高台橙块取块墙', 'purple': '紫块取块墙',
              'building': '搭建区操作位置', 'tag2': 'Tag2 观察入口',
              'tag3': 'Tag3 观察入口', 'tag4': 'Tag4 观察入口',
              'tag5': 'Tag5 观察入口', 'tag6': 'Tag6 观察入口'}

    def __init__(self, field, robot, *, motion=None, tracking=None):
        self.motion = motion or CompetitionMotion.competition()
        self.tracking = tracking or TrackingGains()
        self.field, self.robot = field, robot
        self.front = float(robot['front_extent_mm'])
        self.rear = float(robot['rear_extent_mm'])
        self.half_width = float(robot['width_mm'])/2
        if min(self.front, self.rear, self.half_width) <= 0:
            raise ValueError('positive robot envelope required')
        self.radius = math.hypot(max(self.front, self.rear), self.half_width)
        rects = []
        for shape in field['shapes']:
            if shape.get('collision') or shape['id'] in ('platform_red', 'ramp_red'):
                p = np.asarray(shape['polygon'], dtype=float)
                lo, hi = p.min(axis=0), p.max(axis=0)
                if len(p) != 4 or any(not (x in (lo[0], hi[0]) and y in (lo[1], hi[1])) for x,y in p):
                    raise ValueError('this field adapter requires rectangular geometry')
                rects.append((*(.5*(lo+hi)), *(.5*(hi-lo))))
        self.rects = np.asarray(rects)
        if not len(self.rects):
            raise ValueError('obstacle geometry is required')
        bounds = field['playable_bounds_mm']
        if [bounds[k] for k in ('left','top','right','bottom')] != [400,400,4400,6800]:
            raise ValueError('field topology must be reviewed for changed bounds')

    def clearance(self, poses):
        """SAT separation lower bound for the entire asymmetric OBB.

        Positive means disjoint; negative means penetration. Includes forbidden
        opponent plateau/ramp, and all CAD collision rectangles. Broadcasting
        makes dense trajectory checks inexpensive without centre-point proxies.
        """
        a = np.atleast_2d(np.asarray(poses, dtype=float))
        theta = np.radians(a[:,2]); c, s = np.cos(theta)[:,None], np.sin(theta)[:,None]
        shift = (self.front-self.rear)/2
        dx = self.rects[None,:,0] - a[:,0,None] - shift*c
        dy = self.rects[None,:,1] - a[:,1,None] - shift*s
        hx, hy = self.rects[None,:,2], self.rects[None,:,3]
        length, width = (self.front+self.rear)/2, self.half_width
        gaps = np.stack((np.abs(dx)-(length*np.abs(c)+width*np.abs(s)+hx),
                         np.abs(dy)-(length*np.abs(s)+width*np.abs(c)+hy),
                         np.abs(dx*c+dy*s)-(length+hx*np.abs(c)+hy*np.abs(s)),
                         np.abs(-dx*s+dy*c)-(width+hx*np.abs(s)+hy*np.abs(c))))
        return gaps.max(axis=0).min(axis=1)

    def centre_clearance(self, xy):
        a = np.asarray(xy)
        delta = np.maximum(np.abs(a[:,None,:]-self.rects[None,:,:2])-self.rects[None,:,2:],0)
        return np.linalg.norm(delta, axis=2).min(axis=1)

    def resolve(self, location):
        f, u, gap = location.family, location.along, location.gap_mm
        w, reach = self.half_width, self.front
        if f not in self.LABELS:
            raise ValueError(f'unknown location family: {f}')
        # End-face width must fit on the accessible wall. The small lateral
        # reserve keeps endpoint yaw error from touching a perpendicular wall.
        reserve = 35
        if f == 'start':
            pose = Pose(400+w+gap, 6800-self.rear-gap, 270)
        elif f == 'orange_ground':
            pose = Pose(400+w+reserve+u*(1800-2*w-2*reserve), 5000+reach+gap,270)
        elif f == 'orange_highland':
            pose = Pose(2600+w+reserve+u*(1800-2*w-2*reserve),2500+reach+gap,270)
        elif f == 'purple':
            # Only the level platform face is a valid collection pose; the
            # ramp side is not silently declared a flat pickup station.
            pose = Pose(2600+reach+gap,2500+w+reserve+u*(1700-2*w-2*reserve),180)
        elif f == 'building':
            pose = Pose(1600+w+reserve+u*(2800-2*w-2*reserve),6200-reach-gap,90)
        else:
            pose = {'tag2':Pose(1300,5600,270), 'tag3':Pose(3500,3600,180),
                    'tag4':Pose(3500,3200,270), 'tag5':Pose(3700,5600,0),
                    'tag6':Pose(3500,5600,90)}[f]
        if self.clearance([(pose.x,pose.y,pose.yaw)])[0] < -1e-6:
            raise NavigationError('operation pose intersects an obstacle')
        return pose

    def catalog(self):
        return [dict(id=f,label=label,variable=f in ('orange_ground','orange_highland','purple','building'),
                     start=self.resolve(Location(f,0)).__dict__,end=self.resolve(Location(f,1)).__dict__)
                for f,label in self.LABELS.items()]

    def locate(self,family,pose):
        """Recover along-wall parameters from a measured pickup end pose."""
        if family not in ('orange_ground','orange_highland','purple','building'):
            raise ValueError('this location does not have a continuous wall coordinate')
        a=self.resolve(Location(family,0,0)); b=self.resolve(Location(family,1,0))
        axis=np.array([b.x-a.x,b.y-a.y]); delta=np.array([pose.x-a.x,pose.y-a.y])
        along=float(np.dot(delta,axis)/np.dot(axis,axis))
        normal=np.array([-math.cos(math.radians(a.yaw)),-math.sin(math.radians(a.yaw))])
        gap=float(np.dot(delta,normal))
        if abs(wrap(pose.yaw-a.yaw))>8:
            raise NavigationError('measured pickup heading outside operation contract')
        try:
            return Location(family,along,gap)
        except ValueError as exc:
            raise NavigationError('measured pose outside accessible wall interval') from exc

    @staticmethod
    def _rounded(vertices, radius):
        """Lines and quintic Bezier fillets with zero endpoint curvature."""
        pts = [np.array(vertices[0],float)]
        for v in vertices[1:]:
            v=np.array(v,float)
            if np.linalg.norm(v-pts[-1])>.01: pts.append(v)
        if len(pts)==1: return np.array(pts)
        samples=[]
        def line(a,b):
            for t in np.linspace(0,1,max(2,int(np.linalg.norm(b-a)/8)+2)):
                samples.append(a+t*(b-a))
        cursor=pts[0]
        for i in range(1,len(pts)-1):
            a,b,d=pts[i-1],pts[i],pts[i+1]
            before,after=b-a,d-b
            n1,n2=np.linalg.norm(before),np.linalg.norm(after)
            v1,v2=before/n1,after/n2
            if np.dot(v1,v2)>.9999:
                continue
            if np.dot(v1,v2)<-.95:
                # A reversal needs a different corridor rather than a cusp.
                raise NavigationError('corridor contains a reversing cusp')
            r=min(radius,.44*n1,.44*n2)
            p0,p5=b-r*v1,b+r*v2
            line(cursor,p0)
            h=.45*r
            controls=np.array([p0,p0+h*v1,p0+2*h*v1,p5-2*h*v2,p5-h*v2,p5])
            for t in np.linspace(0,1,max(12,int(2*r/8))):
                weights=np.array([math.comb(5,j)*t**j*(1-t)**(5-j) for j in range(6)])
                samples.append(weights@controls)
            cursor=p5
        line(cursor,pts[-1])
        a=np.array(samples)
        return a[np.r_[True,np.linalg.norm(np.diff(a,axis=0),axis=1)>.0001]]

    def _spread_heading(self,xy,start,goal):
        """Distribute rotation along free motion, without an intermediate pose.

        The SE(2) search supplies a feasible fallback. This candidate moves yaw
        changes away from its discrete in-place rotations into available space.
        Every resulting pose is checked again against the full body geometry.
        """
        arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))]
        free=self.centre_clearance(xy)>=self.radius+60
        yaw=np.full(len(xy),float(start.yaw))
        def rotate(left,right,before,after):
            delta=wrap(after-before)
            ids=np.flatnonzero(free & (arc>=left)&(arc<=right))
            if abs(delta)<.001:
                yaw[arc>=left]=before; return before
            if len(ids)<2: raise NavigationError('no interval for moving orientation change')
            runs=np.split(ids,np.where(np.diff(ids)>1)[0]+1)
            run=max(runs,key=lambda r:arc[r[-1]]-arc[r[0]])
            a,b=arc[run[0]],arc[run[-1]]
            if b-a<2: raise NavigationError('moving orientation interval too short')
            mask=arc>=left
            yaw[mask]=before+delta*smooth(np.clip((arc[mask]-a)/(b-a),0,1))
            return before+delta
        r=next(s for s in self.field['shapes'] if s['id']=='ramp_blue')
        rect=np.array(r['polygon']);lo,hi=rect.min(axis=0),rect.max(axis=0)
        ramp=np.flatnonzero((xy[:,0]>lo[0])&(xy[:,0]<hi[0])&(xy[:,1]>lo[1]-100)&(xy[:,1]<hi[1]+100))
        if len(ramp):
            options=(90,270)
            middle=min(options,key=lambda y:(abs(wrap(y-start.yaw))+abs(wrap(goal.yaw-y)),abs(wrap(y-start.yaw))))
            reached=rotate(0,arc[ramp[0]],start.yaw,middle)
            rotate(arc[ramp[-1]],arc[-1],reached,goal.yaw)
        else:
            rotate(0,arc[-1],start.yaw,goal.yaw)
        return yaw

    def plan(self, source, target, *, actual_start=None):
        nominal=self.resolve(source); goal=self.resolve(target)
        start=actual_start or nominal
        if actual_start is not None and source.family in ('orange_ground','orange_highland','purple','building'):
            source=self.locate(source.family,actual_start)
            nominal=self.resolve(source)
        if math.hypot(start.x-nominal.x,start.y-nominal.y)>150 or abs(wrap(start.yaw-nominal.yaw))>8:
            raise NavigationError('start estimate outside the selected operation region; relocalize')
        return self._plan_poses(start, goal, source, target)

    def plan_between(self, start, goal):
        """Plan a competition route between measured and recipe-derived poses.

        This preserves a recipe's endpoint while allowing free-space search to
        choose its path. Contact and visual alignment remain separate actions.
        """
        if not isinstance(start, Pose) or not isinstance(goal, Pose):
            raise TypeError('competition navigation requires two finite Pose values')
        return self._plan_poses(start, goal, Location('route_start'), Location('route_goal'))

    def _plan_poses(self, start, goal, source, target):
        if self.clearance([(start.x,start.y,start.yaw)])[0]<-1e-6:
            raise NavigationError('actual start is in collision')
        if self.clearance([(goal.x,goal.y,goal.yaw)])[0]<-1e-6:
            raise NavigationError('route goal is in collision; check map and route calibration')
        if math.hypot(start.x-goal.x,start.y-goal.y)<.1 and abs(wrap(start.yaw-goal.yaw))<.01:
            row=np.array([[start.x,start.y,start.yaw,0,self.clearance([(start.x,start.y,start.yaw)])[0],0]])
            return Route(start,goal,source,target,row,(),target.family=='building',0,motion=self.motion,tracking=self.tracking)
        arrival=goal
        if target.gap_mm<3 and not target.family.startswith('tag'):
            reserve=3-target.gap_mm
            if target.family=='start':
                arrival=Pose(goal.x+.8*reserve,goal.y-.8*reserve,goal.yaw)
            else:
                a=math.radians(goal.yaw)
                arrival=Pose(goal.x-math.cos(a)*reserve,goal.y-math.sin(a)*reserve,goal.yaw)
        from .search import PoseSearch
        search=getattr(self,'_search',None)
        if search is None:
            search=self._search=PoseSearch(self)
        raw=search.find(start,arrival)
        direct=[np.array([start.x,start.y,start.yaw]),
                np.array([arrival.x,arrival.y,start.yaw+wrap(arrival.yaw-start.yaw)])]
        candidates=[direct,search.shortcut(raw),raw]
        failure=''; feasible=[]
        for path,rounding,spread in ((p,r,m) for p in candidates for r in (850,450,260,140,60,20,8) for m in (True,False)):
            try:
                if spread:
                    positions=search.compress([q[:2] for q in path])
                    if len(positions)<2: raise NavigationError('no translation to distribute orientation')
                    xy=self._rounded(positions,rounding)
                    yaw=self._spread_heading(xy,start,arrival)
                    rounded=np.c_[xy,yaw*4]
                else:
                    scaled=[np.array([q[0],q[1],q[2]*4]) for q in path]
                    rounded=self._rounded(scaled,rounding)
                    xy=rounded[:,:2]; yaw=rounded[:,2]/4
                arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(rounded,axis=0),axis=1))]
                poses=np.c_[xy,yaw]
                if not search.terrain(poses).all():
                    raise NavigationError('curve violates ramp heading envelope')
                clearance=self.clearance(poses)
                if clearance.min() < -1e-5:
                    raise NavigationError('swept body intersects obstacle')
                endpoint_distance=np.minimum(np.linalg.norm(xy-np.array([start.x,start.y]),axis=1),
                                             np.linalg.norm(xy-np.array([goal.x,goal.y]),axis=1))
                if np.any((endpoint_distance>300)&(clearance<60)):
                    raise NavigationError('transit curve has insufficient localization reserve')
                # Between-sample translation + corner sweep bound. Near-wall
                # terminal straight segments are separately low-speed guarded.
                travel=np.linalg.norm(np.diff(xy,axis=0),axis=1)
                sweep=travel+self.radius*np.abs(np.radians(np.diff(yaw)))
                conservative=np.minimum(clearance[:-1],clearance[1:])-sweep/2
                if np.any((conservative<0)&(np.minimum(clearance[:-1],clearance[1:])>8)):
                    raise NavigationError('trajectory sampling cannot certify clearance')
                speed=self.motion.clearance_speed(clearance)
                # Ramp speed is explicitly lower than open-floor cruise.
                speed[(xy[:,1]>=3650)&(xy[:,1]<=5500)&(xy[:,0]>2800)]=np.minimum(
                    speed[(xy[:,1]>=3650)&(xy[:,1]<=5500)&(xy[:,0]>2800)],self.motion.ramp_mm_s+self.motion.correction_limit(self.motion.ramp_mm_s))
                # Reference changes in yaw constrain translational progress.
                if len(arc)>2:
                    dyaw=np.gradient(yaw,arc)
                    speed=np.minimum(speed,min(self.motion.yaw_deg_s,self.motion.curve_yaw_deg_s)/np.maximum(np.abs(dyaw),1e-6))
                    yaw_curvature=np.abs(np.gradient(dyaw,arc))
                    speed=np.minimum(speed,np.sqrt(.5*self.motion.yaw_acceleration/np.maximum(yaw_curvature,1e-6)))
                    tang=np.gradient(xy,arc,axis=0)
                    curvature=np.linalg.norm(np.gradient(tang,arc,axis=0),axis=1)
                    speed=np.minimum(speed,np.sqrt(self.motion.curve_accel_mm_s2/np.maximum(curvature,1e-6)))
                    angle=np.radians(yaw); c,s=np.cos(angle),np.sin(angle)
                    bx=(c*tang[:,0]+s*tang[:,1])/10
                    by=(-s*tang[:,0]+c*tang[:,1])*LATERAL_DISTANCE_SCALE/10
                    wheel_factor=np.array([max(map(abs,Chassis.mecanum_rpm(x,y,-w))) for x,y,w in zip(bx,by,dyaw)])
                    speed=np.minimum(speed,self.motion.wheel_rpm/np.maximum(wheel_factor,1e-6))
                candidate=Route(start,goal,source,target,np.c_[xy,yaw,arc,clearance,speed],
                                tuple(tuple(float(v) for v in q[:2]) for q in path),target.family=='building',float(arc[-1]),rounding)
                candidate.motion=self.motion
                candidate.tracking=self.tracking
                duration=float(route_timing(candidate.samples,self.motion)[0][-1])
                feasible.append((duration,candidate.nominal_length_mm,candidate))
            except NavigationError as exc:
                failure=str(exc)
        if feasible:
            chosen=min(feasible,key=lambda item:item[:2])[2]
            chosen.candidate_count=len(feasible)
            return chosen
        raise NavigationError(f'no feasible free-space curve: {failure}')
