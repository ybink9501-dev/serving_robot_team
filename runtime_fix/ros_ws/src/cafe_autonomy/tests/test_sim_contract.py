import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from cafe_autonomy.policy import ZoneArbiter
from cafe_autonomy.runtime_config import load_limits
ROOT=Path(__file__).resolve().parents[1]

class Contracts(unittest.TestCase):
    def test_physical_scenario_entities_have_collisions(self):
        for model in ('person','movable_obstacle'):
            root=ET.parse(ROOT/'models'/f'{model}.sdf').getroot()
            self.assertTrue(root.findall('.//collision'))
    def test_contact_sensors_present_on_each_robot(self):
        for r in ('robot_01','robot_02'):
            root=ET.parse(ROOT/'models'/f'{r}.sdf').getroot()
            self.assertEqual(len(root.findall('.//sensor[@type="contact"]')),1)
    def test_zone_owner_keeps_inside_precedence(self):
        z=ZoneArbiter()
        # Robot 02 enters first; Robot 01 must wait when entering approach zone.
        holds=z.update({'robot_02':(-2.3,1.4),'robot_01':(-3.8,1.4)})
        self.assertEqual(z.owners['kitchen_access'],'robot_02')
        self.assertIn('robot_01',holds)
        holds=z.update({'robot_02':(-2.3,1.4),'robot_01':(-3.3,1.4)})
        self.assertIn('robot_01',holds)
        holds=z.update({'robot_02':(2.4,-2.3),'robot_01':(-3.3,1.4)})
        self.assertEqual(z.owners['kitchen_access'],'robot_01')
        self.assertNotIn('robot_01',holds)
    def test_sim_truth_not_used_as_detector(self):
        safety=(ROOT/'cafe_autonomy/safety.py').read_text()
        self.assertIn("'/cafe/people_detected'",safety)
        self.assertNotIn("'/cafe/people'",safety)
        self.assertIn('STATIONARY_PERSON_REPLAN',safety)
    def test_runtime_config_is_single_source_for_both_robots(self):
        import yaml
        cfg=load_limits(ROOT/'config/runtime_limits.yaml')
        for r in ('robot_01','robot_02'):
            d=yaml.safe_load((ROOT/'config'/f'nav2_{r}.yaml').read_text())
            self.assertNotIn('&id', (ROOT/'config'/f'nav2_{r}.yaml').read_text())
            self.assertEqual(d[f'/{r}/controller_server']['ros__parameters']['general_goal_checker']['xy_goal_tolerance'],cfg['navigation']['goal_xy_tolerance_m'])
            self.assertEqual(d[f'/{r}/controller_server']['ros__parameters']['FollowPath']['vx_min'],0.0)
    def test_fleet_uses_map_not_comparing_odom(self):
        src=(ROOT/'cafe_autonomy/fleet.py').read_text()
        self.assertIn('on_amcl',src)
        self.assertIn("if msg.header.frame_id!='map'",src)
        self.assertIn('self.odom_wall[robot]=time.monotonic()',src)
    def test_metrics_and_acceptance_present(self):
        assert (ROOT/'cafe_autonomy/metrics.py').exists()
        assert (ROOT.parents[2]/'tools/acceptance.py').exists()
