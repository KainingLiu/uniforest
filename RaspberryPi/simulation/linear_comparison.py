"""Classic Chassis._move_linear and goal follower on the SAME four-wheel plant.

Open floor, no vision, no obstacles. Reports differences, never real-world
performance. Legacy PID/arrival and continuous-path arrival are kept intact.
"""
from dataclasses import replace
import json
import math
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from control.chassis import Chassis, COUNTS_PER_CM, LATERAL_DISTANCE_SCALE, MECANUM_RPM_PER_CM_S
from protocol.commands import TelemBatch, MotorFeedback
from Strategy.navigation import Location, Pose
from Strategy.navigation.planner import Route
from Strategy.navigation.control import MotionFeedback, execute_route
from Strategy.navigation.motion import CompetitionMotion
from .core import MecanumPlant


def compare(distance=2000, direction='forward', *, motion=None):
    motion=motion or CompetitionMotion.competition()
    axes={'forward':(1,0),'backward':(-1,0),'right':(0,1),'left':(0,-1)}
    dx,dy=axes[direction]
    speed=motion.short_mm_s if distance<motion.short_distance_mm else motion.cruise_mm_s
    accel=motion.short_accel_s if distance<motion.short_distance_mm else motion.accel_s
    old=MecanumPlant()
    class Link:
        connected=True
        emergency_stop_generation=0
        def set_chassis_speed(self,rpms):
            old.targets=list(rpms); return True
        def emergency_stop(self):
            self.emergency_stop_generation+=1; old.emergency_stop()
    link=Link(); chassis=Chassis(link)
    def feedback():
        chassis.update_telem(TelemBatch(motors=[MotorFeedback(cumulative_pos=round(c),speed_rpm=round(r))
                for c,r in zip(old.counts,old.rpms)],yaw_deg=-old.yaw))
        return chassis.telem
    def advance(t): old.sleep(t); feedback()
    signs={'forward':(-1,1,1,-1),'backward':(1,-1,-1,1),'right':(1,1,-1,-1),'left':(-1,-1,1,1)}[direction]
    scale=LATERAL_DISTANCE_SCALE if dy else 1
    with patch('control.chassis.time.monotonic',side_effect=lambda:old.now):
        feedback()
        old_result=chassis._move_linear(int(distance*scale*COUNTS_PER_CM/10),list(signs),
                chassis._mm_s_to_rpm(speed*scale),link,feedback,distance,
                sleep_fn=advance,hold_ms=0,accel_ms=round(accel*1000),route_mode=True,distance_scale=scale)
    new=MecanumPlant()
    arc=np.linspace(0,distance,max(3,int(distance/5)+1))
    new_speed=motion.short_mm_s if distance<motion.short_distance_mm else motion.travel_mm_s
    cap=min(new_speed+motion.correction_limit(new_speed),motion.wheel_rpm*10/(MECANUM_RPM_PER_CM_S*scale))
    points=np.c_[dx*arc,dy*arc,np.zeros(len(arc)),arc,np.full(len(arc),2000),np.full(len(arc),cap)]
    route=Route(Pose(0,0,0),Pose(dx*distance,dy*distance,0),Location('start'),Location('tag2'),points,(),False,distance,motion=motion)
    floor=SimpleNamespace(clearance=lambda poses:np.full(len(poses),2000.),radius=520)
    def new_feedback():
        return MotionFeedback(Pose(new.ox,new.oy,new.oyaw),new.measured_velocity(),new.now)
    result=execute_route(floor,route,read_feedback=new_feedback,read_observations=lambda:[],
            send_velocity=new.send_velocity,check=new.guard,stop=lambda:new.send_velocity(type(new.measured_velocity())()),
            clock=lambda:new.now,sleep=new.sleep)
    def report(plant):
        trace=plant.export_trace(maximum=1500)
        return dict(elapsed_s=round(plant.now,3),peak_speed_mm_s=round(plant.max_speed,3),
                    error_mm=round(math.hypot(plant.x-dx*distance,plant.y-dy*distance),3),points=trace)
    return dict(distance_mm=distance,direction=direction,simulation_only=True,field_validated=False,
                motion=motion.describe(),classic=dict(**report(old),timed_out=old_result.timed_out),
                optimized=dict(**report(new),status=result['status']))


if __name__=='__main__':
    from pathlib import Path
    results=[compare(d,direction) for d in (300,1200,2500) for direction in ('forward','right')]
    out=Path(__file__).with_name('output')/'linear-comparison.json'
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
    for r in results:
        print(json.dumps({k:({i:v for i,v in value.items() if i!='points'} if k in ('classic','optimized') else value)
                          for k,value in r.items() if k!='motion'}))
