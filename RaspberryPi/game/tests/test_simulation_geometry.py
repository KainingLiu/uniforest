"""Geometric counterexamples for planar replay, independent of route execution."""

import math
import unittest

from simulation.geometry import assess_trace, polygon_clearance, polygons_overlap, robot_polygon


def rectangle(x0,y0,x1,y1):
    return ((x0,y0),(x1,y0),(x1,y1),(x0,y1))


def field(*shapes):
    return dict(width_mm=2000,height_mm=2000,shapes=list(shapes))


def obstacle(polygon, **values):
    return dict(id='wall',label='Wall',polygon=polygon,collision=True,**values)


class GeometryTests(unittest.TestCase):
    def test_clockwise_body_rotation_and_front_extension(self):
        body = robot_polygon(100,200,90,100,40,30)
        self.assertAlmostEqual(min(p[0] for p in body),80)
        self.assertAlmostEqual(max(p[0] for p in body),120)
        self.assertAlmostEqual(min(p[1] for p in body),150)
        self.assertAlmostEqual(max(p[1] for p in body),280)

    def test_edges_cross_even_when_no_vertex_is_inside(self):
        horizontal = rectangle(-10,-1,10,1)
        vertical = rectangle(-1,-10,1,10)
        self.assertTrue(polygons_overlap(horizontal,vertical))

    def test_containment_contact_and_concave_notch(self):
        concave = ((0,0),(8,0),(8,2),(2,2),(2,8),(0,8))
        self.assertFalse(polygons_overlap(concave,rectangle(3,3,4,4)))
        self.assertTrue(polygons_overlap(concave,rectangle(.5,3,1.5,4)))
        self.assertTrue(polygons_overlap(rectangle(0,0,2,2),rectangle(2,1,4,3)))
        self.assertTrue(polygons_overlap(rectangle(0,0,10,10),rectangle(2,2,3,3)))

    def test_clearance_is_edge_distance_not_centre_distance(self):
        a,b = rectangle(0,0,2,2),rectangle(5,6,8,8)
        self.assertAlmostEqual(polygon_clearance(a,b),5)
        self.assertAlmostEqual(polygon_clearance(b,a),5)
        self.assertEqual(polygon_clearance(a,rectangle(2,0,4,2)),0)

    def test_rotating_corner_strikes_obstacle_while_centre_is_clear(self):
        # At 0 and 90 degrees neither footprint touches the obstacle near the
        # diagonal front corner. At 45 degrees the long chassis crosses it.
        shape = obstacle(rectangle(568,568,574,574))
        report = assess_trace([dict(x=0,y=0,yaw=0,t=0),dict(x=0,y=0,yaw=90,t=1)],
                              dict(x_mm=500,y_mm=500,yaw_deg=0),field(shape),
                              dict(length_mm=220,width_mm=20))
        self.assertGreater(report['collision_count'],0)
        self.assertEqual(report['collisions'][0]['shape_id'],'wall')
        self.assertGreater(report['samples_checked'],2)
        self.assertEqual(report['min_clearance_mm'],0)

    def test_thin_obstacle_between_sparse_samples_cannot_be_tunnelled(self):
        shape = obstacle(rectangle(550,498,551,502))
        report = assess_trace([dict(x=0,y=0,yaw=0),dict(x=100,y=0,yaw=0)],
                              dict(x=500,y=500,yaw=0),field(shape),
                              dict(length_mm=2,width_mm=2),max_step_mm=100)
        self.assertGreater(report['collision_count'],0)
        self.assertEqual(report['collisions'][0]['evidence'],'swept_envelope')

    def test_true_arc_is_checked_instead_of_endpoint_chord(self):
        # A half-circle centred at (100,0) passes local (100,-100); its chord
        # stays on y=0. The obstacle only intersects the arc.
        trace = [dict(x=0,y=0,yaw=0,t=0),
                 dict(x=200,y=0,yaw=180,t=1,
                      arc=dict(center_x_mm=100,center_y_mm=0,sweep_deg=180))]
        shape = obstacle(rectangle(598,398,602,402))
        report = assess_trace(trace,dict(x=500,y=500,yaw=0),field(shape),
                              dict(length_mm=4,width_mm=4))
        self.assertGreater(report['collision_count'],0)
        without_arc = [trace[0],{k:v for k,v in trace[1].items() if k != 'arc'}]
        self.assertEqual(assess_trace(without_arc,dict(x=500,y=500,yaw=0),field(shape),
                                     dict(length_mm=4,width_mm=4))['collision_count'],0)

    def test_world_mapping_uses_rotated_local_right_axis(self):
        report = assess_trace([dict(x=0,y=0,yaw=0),dict(x=100,y=50,yaw=0)],
                              dict(x_mm=500,y_mm=500,yaw_deg=90),
                              field(obstacle(rectangle(447,597,453,603))),
                              dict(length_mm=10,width_mm=10))
        self.assertGreater(report['collision_count'],0)
        self.assertLess(report['collisions'][0]['x_mm'],500)
        self.assertGreater(report['collisions'][0]['y_mm'],500)

    def test_footprint_outside_field_with_centre_inside_is_reported(self):
        report = assess_trace([dict(x=0,y=0,yaw=45)],dict(x=10,y=10,yaw=0),
                              field(),dict(length_mm=100,width_mm=50))
        self.assertEqual(report['collisions'][0]['shape_id'],'__boundary__')

    def test_height_does_not_make_ramp_or_platform_a_wall(self):
        ramp = dict(id='ramp',label='Ramp',kind='ramp',height_mm=1000,
                    collision=False,polygon=rectangle(400,400,600,600))
        report = assess_trace([dict(x=0,y=0,yaw=0)],dict(x=500,y=500,yaw=0),
                              field(ramp),dict(length_mm=100,width_mm=100))
        self.assertEqual(report['collision_count'],0)
        self.assertGreater(report['min_clearance_mm'],0)
        self.assertTrue(any('离散' in warning for warning in report['warnings']))

    def test_asymmetric_cad_body_is_referenced_to_wheel_centre(self):
        # Wheel centre stays at (500,500); front reaches y=800 at 90 degrees,
        # rear only reaches y=400. A symmetric 600 mm box would invent a rear hit.
        robot = dict(front_extent_mm=300,rear_extent_mm=100,width_mm=40)
        rear = obstacle(rectangle(490,240,510,260))
        front = dict(obstacle(rectangle(490,790,510,810)),id='front')
        report = assess_trace([dict(x=0,y=0,yaw=90)],dict(x=500,y=500,yaw=0),
                              field(rear,front),robot)
        self.assertEqual({item['shape_id'] for item in report['collisions']},{'front'})

    def test_invalid_geometry_is_rejected_instead_of_ignored(self):
        with self.assertRaises(ValueError):
            robot_polygon(0,0,0,-1,20)
        with self.assertRaises(ValueError):
            polygons_overlap(((0,0),(1,0),(2,0)),rectangle(0,0,1,1))
        with self.assertRaises(ValueError):
            assess_trace([dict(x=math.nan,y=0,yaw=0)],dict(x=500,y=500,yaw=0),
                         field(),dict(length_mm=10,width_mm=10))


if __name__ == '__main__':
    unittest.main()
