import unittest, math, xml.etree.ElementTree as ET
from pathlib import Path
from cafe_autonomy.waypoints import read_waypoints,choose_waypoint
from cafe_autonomy.policy import FINAL_PHYSICAL_M,GOAL_NAV_XY_M

ROOT=Path(__file__).resolve().parents[1]
class Assets(unittest.TestCase):
    def test_model_uniqueness(self):
        for rid in ('robot_01','robot_02'):
            root=ET.parse(ROOT/'models'/f'{rid}.sdf').getroot()
            self.assertEqual(root.find('./model').attrib['name'],rid)
            self.assertEqual(len(root.findall('.//sensor')),8)
            self.assertEqual(root.find('.//plugin[@name="gz::sim::systems::DiffDrive"]/topic').text,f'/model/{rid}/cmd_vel')
    def test_world_no_robot_autospawn(self):
        root=ET.parse(ROOT/'worlds/cafe.sdf').getroot()
        names={m.attrib['name'] for m in root.findall('.//model')}
        self.assertNotIn('robot_01',names)
        self.assertNotIn('robot_02',names)
        self.assertIn('cafe_counter',names)
    def test_waypoint_consistency(self):
        poses=read_waypoints(ROOT/'config/waypoints.yaml')
        self.assertEqual(set(['kitchen','robot_01_HOME','robot_02_HOME']).intersection(poses.keys()),{'kitchen','robot_01_HOME','robot_02_HOME'})
        for i in range(1,5):
            p=choose_waypoint(poses,f'table_{i:02}',xy=(1.,-2.))
            self.assertTrue(math.isfinite(p['yaw']))
    def test_precision_constants_preserved(self):
        self.assertAlmostEqual(FINAL_PHYSICAL_M,0.10)
        self.assertAlmostEqual(GOAL_NAV_XY_M,0.10)
    def test_table_facing_90_degrees(self):
        poses=read_waypoints(ROOT/'config/waypoints.yaml')
        for i in range(1,5):
            approaches=poses[f'table_{i:02}']['approaches']
            self.assertAlmostEqual(approaches['top']['yaw'],-math.pi/2)
            self.assertAlmostEqual(approaches['bottom']['yaw'],math.pi/2)
    def test_map_dimensions(self):
        pgm=(ROOT/'maps/cafe.pgm').read_bytes()
        self.assertTrue(pgm.startswith(b'P5\n240 200\n255\n'))
        self.assertEqual(len(pgm.split(b'255\n',1)[1]),240*200)
if __name__=='__main__':unittest.main()
