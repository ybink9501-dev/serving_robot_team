import unittest
from cafe_autonomy.policy import (FleetCore, next_after_nav, may_confirm,
    velocity_limit,person_factor,norm_angle, distance)
from cafe_autonomy.waypoints import choose_waypoint

class Policies(unittest.TestCase):
    def test_queue_and_duplicate(self):
        f=FleetCore(); a,new=f.submit('r1','table_01','robot_01')
        self.assertTrue(new); self.assertFalse(f.submit('r1','table_01','robot_01')[1])
        self.assertEqual(len(f.queue),1)
        with self.assertRaises(ValueError):f.submit('r1','table_02','robot_01')
        self.assertEqual(f.dispatch_next({'robot_01'}).request_id,'r1')
        self.assertIsNone(f.dispatch_next({'robot_01'}))
        f.update('robot_01','COMPLETED','r1')
        self.assertEqual(f.tasks['r1'].state,'SUCCEEDED')
    def test_kitchen_gate(self):
        f=FleetCore(); f.submit('a','table_01','robot_01')
        self.assertIsNone(f.dispatch_next({'robot_01'},kitchen_free=False))
    def test_wrong_previous_request_cannot_complete_new_task(self):
        f=FleetCore();f.submit("old","table_01","robot_01")
        f.dispatch_next({"robot_01"});f.update("robot_01","COMPLETED","old")
        f.submit("new","table_02","robot_01");f.dispatch_next({"robot_01"})
        f.update("robot_01","COMPLETED","old")
        self.assertEqual(f.tasks["new"].state,"ASSIGNED")

    def test_modes(self):
        self.assertFalse(may_confirm('LOAD','GOING_PICKUP','KITCHEN_PICKUP'))
        self.assertTrue(may_confirm('LOAD','WAIT_LOAD','KITCHEN_PICKUP'))
        self.assertTrue(may_confirm('COMPLETE','WAIT_HANDOVER','DELIVER_DIRECT'))
        self.assertEqual(next_after_nav('RETURNING_HOME','DELIVER_DIRECT'),'COMPLETED')
        self.assertEqual(next_after_nav('GOING_PICKUP','KITCHEN_PICKUP'),'WAIT_LOAD')
    def test_limits(self):
        self.assertEqual(velocity_limit(-0.9,2.0),(0.0,0.8))
        self.assertEqual(person_factor(0.79),0.0)
        self.assertEqual(person_factor(2.0),1.0)
        self.assertAlmostEqual(person_factor(1.15),0.5)
        self.assertLess(abs(norm_angle(3.5)),3.15)
        self.assertAlmostEqual(distance((0,0),(3,4)),5.)
    def test_table_common(self):
        poses={'t':{'approaches':{'top':{'x':0.,'y':2.,'yaw':0.},'bottom':{'x':0.,'y':-2.,'yaw':3.}}}}
        self.assertEqual(choose_waypoint(poses,'t',(0,-3))['y'],-2.)
        self.assertEqual(choose_waypoint(poses,'t',(0,-3),'top')['y'],2.)
if __name__=='__main__':unittest.main()
