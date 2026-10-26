import math
import os
from pathlib import Path
import tempfile
import unittest
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from policies import StallWindow, inspect_pickup, model_collision_boxes, distance_from_rotated_box

SDF=Path(os.environ.get('TEAM_CAFE_SDF',str(Path.home()/'team_serving_robot/ros_ws/src/serving_robot_gazebo/worlds/cafe.world.sdf')))

class GeometryTests(unittest.TestCase):
    def test_actual_team_counter(self):
        # May be unavailable on development runner; fixture tests also check independent logic.
        if not SDF.is_file():self.skipTest('Project SDF not mounted in test build host')
        r=inspect_pickup(SDF,'cafe_counter',{'x':-2.3,'y':1.45,'yaw':math.pi/2},.35,.10,25.)
        self.assertTrue(r['pass_static']);self.assertAlmostEqual(r['body_clearance_estimate_m'],.25,places=3)
        self.assertFalse(r['is_actual_service_position_approved'])
    def test_fixture_box(self):
        with tempfile.TemporaryDirectory() as d:
            s=Path(d)/'world.sdf'
            s.write_text('''<sdf version="1.10"><world name="cafe"><model name="cafe_counter"><pose>-2.3 2.4 0 0 0 0</pose><link name="base"><collision name="c"><geometry><box><size>2.5 0.7 1</size></box></geometry></collision></link></model></world></sdf>''')
            a=inspect_pickup(s,'cafe_counter',{'x':-2.3,'y':1.45,'yaw':math.pi/2},.35,.10,25.)
            self.assertTrue(a['pass_static'])
            self.assertAlmostEqual(a['distance_from_counter_collision_m'],.60)
            self.assertAlmostEqual(a['body_clearance_estimate_m'],.25)
            b=inspect_pickup(s,'cafe_counter',{'x':-2.3,'y':1.45,'yaw':-math.pi/2},.35,.10,25.)
            self.assertFalse(b['pass_static'])
            b=inspect_pickup(s,'cafe_counter',{'x':-2.3,'y':2.0,'yaw':math.pi/2},.35,.10,25.)
            self.assertFalse(b['pass_static'])

class StallTests(unittest.TestCase):
    def test_idle_stationary_triggers_after_window(self):
        s=StallWindow(30.,.08,.15);r=None
        for i in range(121):r=s.observe(i*.25,1.,1.)
        self.assertTrue(r['stalled'])
    def test_normal_move_does_not_trigger(self):
        s=StallWindow(30.,.08,.15);r=None
        for i in range(121):r=s.observe(i*.25,i*.01,0)
        self.assertFalse(r['stalled'])
    def test_back_and_forth_not_false_stall(self):
        s=StallWindow(30.,.08,.15);r=None
        for i in range(121):r=s.observe(i*.25,math.sin(i*.25)*.10,0)
        self.assertFalse(r['stalled'])
    def test_reset_on_reordered_time(self):
        s=StallWindow(30.,.08,.15)
        for i in range(120):s.observe(i*.25,0,0)
        self.assertIsNone(s.observe(2,0,0))
    def test_reset_on_stale_samples(self):
        s=StallWindow(30.,.08,.15)
        for i in range(25):s.observe(i*.25,0,0)
        self.assertIsNone(s.observe(100,0,0))
    def test_invalid_measurements_not_trusted(self):
        s=StallWindow(30.,.08,.15)
        self.assertIsNone(s.observe(1,float('nan'),0));self.assertIsNone(s.observe(1,0,0))
    def test_threshold_validation(self):
        with self.assertRaises(ValueError):StallWindow(2,0.08,0.15)

if __name__=='__main__':unittest.main()
