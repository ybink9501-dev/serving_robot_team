import unittest,sys,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from policy import SpinWindow

class PolicyTests(unittest.TestCase):
    def p(self,s=12,progress=.08,angle=math.pi):return SpinWindow(s,progress,angle)
    def test_detects_reported_spin(self):
        p=self.p();r=None
        for i in range(121):r=p.observe(i*.1,.04*i/120,0,3.44*i/120)
        self.assertTrue(r['triggered'])
    def test_allows_normal_progress(self):
        p=self.p();r=None
        for i in range(121):r=p.observe(i*.1,.3*i/120,0,3.44*i/120)
        self.assertFalse(r['triggered'])
    def test_allows_less_than_spin_limit(self):
        p=self.p();r=None
        for i in range(121):r=p.observe(i*.1,0,0,2.8*i/120)
        self.assertFalse(r['triggered'])
    def test_catches_back_and_forth_turns(self):
        p=self.p();r=None
        for i in range(121):r=p.observe(i*.1,0,0,.09*(i%2))
        self.assertTrue(r['triggered'])
    def test_avoids_before_window(self):
        p=self.p();r=None
        for i in range(40):r=p.observe(i*.1,0,0,.2*i)
        self.assertFalse(r['triggered'])
    def test_clear_resets(self):
        p=self.p();p.observe(0,0,0,0);p.observe(12,0,0,4)
        p.clear();self.assertFalse(p.observe(30,0,0,0)['triggered'])
    def test_nonfinite_rejected(self):
        p=self.p()
        with self.assertRaises(ValueError):p.observe(0,math.nan,0,0)
    def test_backwards_clock_resets(self):
        p=self.p();p.observe(2,0,0,1);r=p.observe(1,0,0,1)
        self.assertFalse(r['triggered'])

if __name__=='__main__':unittest.main()
