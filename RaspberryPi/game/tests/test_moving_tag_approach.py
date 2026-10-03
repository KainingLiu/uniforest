"""Moving-camera/encoder replay through the production local trajectory loop."""
from dataclasses import replace
import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from control.trajectory import Waypoint, BodyVelocity
from Strategy.optimizations.tag_approach import TagApproach, TagApproachProfile
from Strategy.settings import PROFILES
from tests.test_continuous_trajectory import Plant, profile


class TagReplay:
    def __init__(self, *, visible=True, delay=.06, shift=(30.,40.), offset=100., yaw=0.):
        self.plant=Plant()
        self.visible=visible
        self.delay=delay
        self.freeze=False
        self.frozen=None
        self.steps=[]
        angle=math.radians(yaw)
        px,py=1200*math.cos(angle),1200*math.sin(angle)
        self.points=(Waypoint(0,0,0),Waypoint(px*250/1200,py*250/1200,yaw),Waypoint(px,py,yaw),
                     Waypoint(px-math.sin(angle)*offset,py+math.cos(angle)*offset,yaw))
        self.goal=(self.points[-1].x_mm+shift[0],self.points[-1].y_mm+shift[1])
        self.offset,self.yaw=offset,yaw
        self.control=SimpleNamespace(config=PROFILES['building-1'],robot=self,
                                     _heading_error=lambda target:-yaw)
        self.guide=TagApproach(self.control,self.points[-1],2,offset,'build',TagApproachProfile(building=True))

    @property
    def field_pose(self):
        p=self.plant
        self.steps.append((p.now,p.x,p.y,self.guide.active))
        if not self.visible:
            return None
        if self.freeze and self.frozen is not None:
            return self.frozen
        stamp=math.floor((p.now-self.delay)*20)/20
        # Replay the physical pose at capture time, independently of odometry.
        index=max(0,min(len(p.poses)-1,round((stamp-100)/.02)-1))
        x,y=(p.poses[index][:2] if p.poses else (0.,0.))
        a=math.radians(self.yaw)
        dx,dy=self.goal[0]-x,self.goal[1]-y
        tag=SimpleNamespace(tag_id=6,distance_m=(425+math.cos(a)*dx+math.sin(a)*dy)/1000,
                            lateral_m=(-math.sin(a)*dx+math.cos(a)*dy-self.offset)/1000,score=.5)
        self.frozen=SimpleNamespace(valid=True,calibrated=False,captured_monotonic=stamp,tag_solutions=[tag])
        return self.frozen

    def run(self):
        return self.plant.run(self.points,profile(route_derived=True,max_speed_mm_s=800.,
            max_accel_mm_s2=1500.,max_wheel_rpm=4000.,max_yaw_speed_deg_s=120.,
            max_yaw_accel_deg_s2=300.,timeout_s=35.),guidance=self.guide)


class MovingTagApproachTests(unittest.TestCase):
    def test_camera_delay_and_offset_converge_without_stopping_at_old_tag_point(self):
        r=TagReplay()
        r.run()
        self.assertTrue(r.guide.completed)
        self.assertLess(math.hypot(r.plant.x-r.goal[0],r.plant.y-r.goal[1]),8.)
        self.assertGreaterEqual(r.guide.accepted_frames,4)
        self.assertTrue(any(active and x<1100 for _,x,_,active in r.steps))
        self.assertEqual(r.plant.stops,0)
        # There is no intermediate zero command once the robot starts moving.
        moving=[v for _,v in r.plant.commands]
        first=next(i for i,v in enumerate(moving) if v!=BodyVelocity())
        last=max(i for i,v in enumerate(moving) if v!=BodyVelocity())
        self.assertNotIn(BodyVelocity(),moving[first:last])

    def test_missing_tag_stops_at_original_end_without_executing_offset(self):
        r=TagReplay(visible=False)
        r.run()
        self.assertFalse(r.guide.completed)
        self.assertLess(math.hypot(r.plant.x-1200,r.plant.y),8.)
        self.assertEqual(r.plant.stops,0)

    def test_implausible_tag_target_keeps_original_endpoint(self):
        r=TagReplay(shift=(0,900))
        r.run()
        self.assertFalse(r.guide.active)
        self.assertFalse(r.guide.completed)
        self.assertLess(math.hypot(r.plant.x-1200,r.plant.y),8.)

    def test_tag_loss_after_handoff_stops_without_replaying_route(self):
        r=TagReplay()
        r.plant.fault=lambda:setattr(r,'freeze',r.guide.active)
        with self.assertRaisesRegex(RuntimeError,'Tag6 lost'):
            r.run()
        self.assertFalse(r.guide.completed)
        self.assertEqual(r.plant.stops,1)

    def test_heading_convention_and_left_offset_are_preserved(self):
        r=TagReplay(offset=-100,yaw=20,shift=(10,-20))
        r.run()
        self.assertTrue(r.guide.completed)
        self.assertLess(math.hypot(r.plant.x-r.goal[0],r.plant.y-r.goal[1]),8.)
        self.assertLess(abs(r.plant.yaw-20),1.)

    def test_three_building_offsets_at_half_turn_heading(self):
        for offset in (100.,400.,-500.):
            with self.subTest(offset=offset):
                r=TagReplay(offset=offset,yaw=180,shift=(20.,-15.))
                r.run()
                self.assertTrue(r.guide.completed)
                self.assertLess(math.hypot(r.plant.x-r.goal[0],r.plant.y-r.goal[1]),8.)
                self.assertLess(abs(r.plant.yaw-180),1.)

    def test_real_chassis_adapter_forwards_guide(self):
        r=TagReplay(visible=False)
        p=r.plant
        with patch('control.chassis.time.monotonic',side_effect=lambda:p.now), \
             patch('control.chassis.time.sleep',side_effect=p.sleep):
            p.publish()
            result=p.chassis.follow_trajectory(r.points,profile(route_derived=True),
                check=lambda:None,guidance=r.guide)
        self.assertFalse(r.guide.completed)
        self.assertLess(abs(result.final_pose.x_mm-1200),8.)


if __name__=='__main__': unittest.main()
