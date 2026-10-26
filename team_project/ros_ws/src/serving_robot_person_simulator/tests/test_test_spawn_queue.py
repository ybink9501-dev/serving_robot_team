import sys
import time
from pathlib import Path
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from serving_robot_person_simulator.manager import Manager


class FakeManager:
    def __init__(self):
        self.cfg = {'test_spawn': {'x': 1.25, 'y': -3.0, 'yaw': 1.5707963267948966,
                                   'wait_timeout_s': 60.0},
                    'person_release_radius_m': 1.0,
                    'robot_release_radius_m': 1.05,
                    'lanes': [{'start': [0.0, -1.7], 'end': [0.0, 2.3]}]}
        self.people = {}
        self.positions = {}
        self.robot_names = []
        self.main_paused = False
        self.last_error = None
        self.events = []
        self.test_spawn_pending = {'requested': 3, 'created': 0,
                                   'deadline': time.monotonic() + 60.0}
        self.test_spawn_result = None

    def fresh_pose(self, name):
        return self.positions.get(name)

    def _spawn_one(self, ident, start, end, guest_mode=False, spawn_at=None, yaw=0.0):
        name = f'test_person_{ident:02d}'
        self.people[ident] = SimpleNamespace(name=name, target_end=True, paused=True)
        self.positions[name] = spawn_at
        self.events.append(('spawn', ident, spawn_at, yaw))

    def record(self, event, **details):
        self.events.append((event, details))

    def publish_status(self, error=None):
        pass


class TestModeQueueTests(unittest.TestCase):
    def test_sequential_clearance_and_exact_origin(self):
        mgr = FakeManager()
        Manager._advance_test_spawn(mgr)
        self.assertEqual(mgr.test_spawn_result['created'], 1)
        self.assertEqual(mgr.events[0][2], (1.25, -3.0))
        self.assertEqual(mgr.events[0][3], 1.5707963267948966)
        self.assertFalse(mgr.people[1].paused)
        Manager._advance_test_spawn(mgr)
        self.assertEqual(mgr.test_spawn_result['created'], 1)
        mgr.positions['test_person_01'] = (1.25, -2.01)
        Manager._advance_test_spawn(mgr)
        self.assertEqual(mgr.test_spawn_result['created'], 1)
        mgr.positions['test_person_01'] = (1.25, -2.0)
        Manager._advance_test_spawn(mgr)
        self.assertEqual(mgr.test_spawn_result['created'], 2)
        mgr.positions['test_person_02'] = (1.25, -1.9)
        Manager._advance_test_spawn(mgr)
        self.assertEqual(mgr.test_spawn_result['state'], 'complete')
        self.assertEqual(mgr.test_spawn_result['created'], 3)
        self.assertEqual(len([e for e in mgr.events if e[0] == 'spawn']), 3)

    def test_timeout_keeps_admitted_person_and_reports_partial(self):
        mgr = FakeManager()
        Manager._advance_test_spawn(mgr)
        mgr.test_spawn_pending['deadline'] = time.monotonic() - 1
        Manager._advance_test_spawn(mgr)
        self.assertEqual(mgr.test_spawn_result['state'], 'timeout')
        self.assertEqual(mgr.test_spawn_result['created'], 1)
        self.assertEqual(len(mgr.people), 1)
        self.assertIsNone(mgr.test_spawn_pending)


if __name__ == '__main__':
    unittest.main()
