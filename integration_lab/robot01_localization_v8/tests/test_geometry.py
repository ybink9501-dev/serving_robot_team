import math,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from geometry import parse_gz_model_pose,angle_delta,pose_difference,map_cell,footprint_free,allowed_state
from types import SimpleNamespace

class TestGeometry(unittest.TestCase):
    def test_pose_parses_gz(self):
        s='''Requesting state for world [cafe]...\nWarning [Utils.cc:132] bad sdf\nModel: [74]\n - Name: robot_01\n - Pose [ XYZ (m) ] [ RPY (rad) ]:\n   [-3.564360 1.569150 -0.000000]\n   [-0.000000 0.000000 1.882740]'''
        self.assertEqual(parse_gz_model_pose(s),(-3.56436,1.56915,1.88274))
    def test_missing_gz(self):
        with self.assertRaises(ValueError):parse_gz_model_pose('not here')
    def test_angle_wrap(self):
        self.assertAlmostEqual(abs(math.degrees(angle_delta(math.radians(179),math.radians(-179)))),2,places=6)
    def test_difference(self):
        d,a=pose_difference((-2.075,1.972,2.079),(-3.564,1.569,1.883))
        self.assertAlmostEqual(d,1.542,places=2);self.assertAlmostEqual(a,11.23,places=1)
    def test_gating(self):
        self.assertTrue(allowed_state({'robot_id':'robot_01','state':'CANCELED','map_valid':True}))
        self.assertFalse(allowed_state({'robot_id':'robot_01','state':'GOING_PICKUP','map_valid':True}))
        self.assertFalse(allowed_state({'robot_id':'robot_02','state':'CANCELED','map_valid':True}))
    def test_map_occupancy(self):
        o=SimpleNamespace(position=SimpleNamespace(x=-5.,y=-4.),orientation=SimpleNamespace(w=1.,z=0.,x=0.,y=0.))
        info=SimpleNamespace(width=200,height=200,resolution=.05,origin=o)
        m=SimpleNamespace(info=info,data=[0]*40000)
        self.assertTrue(footprint_free(m,-3.564,1.569))
        self.assertFalse(footprint_free(m,-6.,1.569))
        ix,iy=map_cell(-3.564,1.569,info);m.data[iy*info.width+ix]=100
        self.assertFalse(footprint_free(m,-3.564,1.569))

if __name__=='__main__':unittest.main()
