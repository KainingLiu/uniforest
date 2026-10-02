"""Run the production goal navigator against the existing four-wheel plant."""
from collections import deque
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from Strategy.navigation import FieldPlanner, Location, Pose, NavigationError
from Strategy.navigation.localization import Observation, compose
from Strategy.navigation.control import MotionFeedback, execute_route
from .core import MecanumPlant, SimulationSettings

HERE=Path(__file__).resolve().parent


def engine_revision():
    files=sorted((HERE.parent/'Strategy/navigation').glob('*.py'))+[HERE/'navigation.py',HERE/'core.py',
        HERE.parent/'control/chassis.py',HERE.parent/'control/motion_law.py',HERE.parent/'Strategy/settings.py']
    return hashlib.sha256(b''.join(p.read_bytes() for p in files)).hexdigest()[:12]


def load_planner():
    field=json.loads((HERE/'field_model.json').read_text(encoding='utf-8'))
    robot=json.loads((HERE/'robot_model.json').read_text(encoding='utf-8'))
    return FieldPlanner(field,robot)


def simulate(source,target,*,planner=None,vision='normal',delay_s=.12,drift_mm=0,
             wheel_response_s=.08,max_points=1600):
    if vision not in ('normal','dropout','missing','outlier','no_building'):
        raise ValueError('unknown visual scenario')
    if not 0<=delay_s<=1 or not -60<=drift_mm<=60 or not .02<=wheel_response_s<=.2:
        raise ValueError('scenario outside supported verification envelope')
    planner=planner or load_planner(); route=planner.plan(source,target)
    plant=MecanumPlant(SimulationSettings(wheel_response_s=wheel_response_s))
    queue=deque(); next_frame=0.; trace=[]; injected=False

    def truth():
        return compose(route.start,Pose(plant.x,plant.y,plant.yaw))

    def feedback():
        pose=compose(route.start,Pose(plant.ox,plant.oy,plant.oyaw))
        # Gradual encoder bias, distinct from true position. No truth is given
        # to the controller except through explicitly simulated camera frames.
        bias=drift_mm*min(1,plant.now/8)
        return MotionFeedback(Pose(pose.x+bias,pose.y+.3*bias,pose.yaw),plant.measured_velocity(),plant.now)

    def observations():
        nonlocal next_frame,injected
        now=plant.now; p=truth()
        if now>=next_frame:
            next_frame=now+.05
            dx,dy=3500-p.x,6800-p.y
            bearing=math.degrees(math.atan2(dy,dx))
            angle=abs((bearing-p.yaw+180)%360-180)
            visible=(p.y>5100 and math.hypot(dx,dy)<2700 and angle<62.5)
            hidden=vision=='missing' or (vision=='dropout' and 1<int(now)%7<3)
            if visible and not hidden:
                noisy=Pose(p.x+.5*math.sin(now*9),p.y+.5*math.cos(now*11),p.yaw+.025*math.sin(now*7))
                if vision=='outlier' and not injected:
                    noisy=Pose(p.x+600,p.y,p.yaw); injected=True
                queue.append((now+delay_s,Observation(noisy,now)))
            if (route.needs_tag6 and vision!='no_building'
                    and math.hypot(p.x-route.goal.x,p.y-route.goal.y)<500
                    and abs((p.yaw-route.goal.yaw+180)%360-180)<25):
                queue.append((now+delay_s,Observation(Pose(p.x,p.y,p.yaw),now,'building',2)))
        ready=[]
        while queue and queue[0][0]<=now+1e-8:
            ready.append(queue.popleft()[1])
        return ready

    def check():
        plant.guard(); p=truth()
        if planner.clearance([(p.x,p.y,p.yaw)])[0]<-1e-6:
            plant.emergency_stop()
            raise NavigationError('physical simulation envelope collided; run stopped')

    def stop():
        return plant.send_velocity(type(plant.measured_velocity())())

    def record(row):
        p=truth(); row.update(x=p.x,y=p.y,yaw=p.yaw,
                              estimated_x=row['x'],estimated_y=row['y'],
                              true_clearance=float(planner.clearance([(p.x,p.y,p.yaw)])[0]))
        trace.append(row)
    try:
        result=execute_route(planner,route,read_feedback=feedback,read_observations=observations,
                             send_velocity=plant.send_velocity,check=check,stop=stop,
                             clock=lambda:plant.now,sleep=plant.sleep,trace=record)
    except Exception as exc:
        result=dict(status='stopped',reason=str(exc),build_ready=False,elapsed_s=plant.now)
        row=dict(trace[-1]) if trace else dict(t=0,x=0,y=0,yaw=0,progress=0,tag_locked=False,building_ready=False)
        v=plant.measured_velocity()
        row.update(t=plant.now,vx=v.vx_mm_s,vy=v.vy_mm_s,wz=v.yaw_deg_s)
        record(row)  # Preserve the failing physical pose in collision metrics.
    p=truth()
    points=np.array([[r['x'],r['y'],r['yaw']] for r in trace] or [[p.x,p.y,p.yaw]])
    clear=planner.clearance(points)
    speeds=np.array([math.hypot(r['vx'],r['vy']) for r in trace] or [0.])
    fast=speeds>300
    v=np.array([[r['vx'],r['vy'],r['wz']] for r in trace])
    a=np.diff(v,axis=0)/.02 if len(v)>1 else np.zeros((1,3))
    internal_stops=0; quiet_since=None; counted=False
    for row,speed in zip(trace,speeds):
        quiet=(speed<5 and abs(row['wz'])<1 and .02<row['progress']<.98)
        if not quiet:
            quiet_since=None; counted=False
        else:
            quiet_since=row['t'] if quiet_since is None else quiet_since
            if not counted and row['t']-quiet_since>=.12:
                internal_stops+=1; counted=True
    error=math.hypot(p.x-route.goal.x,p.y-route.goal.y)
    result.update(simulation_only=True,field_validated=False,source=asdict(source),target=asdict(target),
                  motion_profile=planner.motion.describe(),tracking_gains=asdict(planner.tracking),
                  start=asdict(route.start),goal=asdict(route.goal),via=route.via,
                  candidate_count=route.candidate_count,blend_radius_mm=route.blend_radius_mm,
                  reference=[dict(x=float(r[0]),y=float(r[1]),yaw=float(r[2])) for r in route.samples[::max(1,len(route.samples)//500)]],
                  metrics=dict(elapsed_s=plant.now,length_mm=plant.path_length,position_error_mm=error,
                               yaw_error_deg=abs((p.yaw-route.goal.yaw+180)%360-180),
                               min_clearance_mm=float(clear.min()),
                               high_speed_min_clearance_mm=float(clear[fast].min()) if fast.any() else None,
                               peak_speed_mm_s=float(speeds.max()),
                               peak_wheel_rpm=plant.max_wheel,
                               peak_accel_mm_s2=float(np.linalg.norm(a[:,:2],axis=1).max()),
                               collisions=int(np.any(clear<-1e-6)),
                               internal_stops=internal_stops))
    indices=np.linspace(0,len(trace)-1,min(max_points,len(trace))).astype(int) if trace else []
    result['points']=[{k:round(v,4) if isinstance(v,float) else v for k,v in trace[i].items()} for i in indices]
    return result


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser()
    parser.add_argument('--source',default='start'); parser.add_argument('--target',default='orange_highland')
    parser.add_argument('--vision',default='normal'); parser.add_argument('--drift',type=float,default=0)
    args=parser.parse_args()
    out=simulate(Location(args.source),Location(args.target),vision=args.vision,drift_mm=args.drift)
    print(json.dumps({k:v for k,v in out.items() if k not in ('points','reference','visual_events')},ensure_ascii=True,indent=2))
