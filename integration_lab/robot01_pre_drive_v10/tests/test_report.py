import json
from pathlib import Path
import tempfile
import unittest
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from report import report

class Reports(unittest.TestCase):
    def test_missing_is_not_success(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'empty.jsonl';p.write_text('')
            self.assertEqual(report([p])['verdict'],'NO_EVIDENCE')
    def test_pickup_only(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'data.jsonl'
            p.write_text('\n'.join(json.dumps({'channel':'serving_status','data':{'state':state,'reason':state}}) for state in ('GOING_PICKUP','WAIT_LOAD'))+'\n')
            self.assertEqual(report([p])['verdict'],'PICKUP_WAIT_LOAD_OBSERVED')
    def test_cancel_is_failure(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'data.jsonl'
            p.write_text('\n'.join(json.dumps({'channel':'serving_status','data':{'state':state,'reason':state}}) for state in ('GOING_PICKUP','CANCELED'))+'\n')
            self.assertEqual(report([p])['verdict'],'FAILED_OR_INTERRUPTED')
    def test_accepted_goal_is_not_completion(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'data.jsonl';p.write_text(json.dumps({'channel':'nav_event','data':{'event':'NAV_GOAL_ACCEPTED'}})+'\n')
            self.assertEqual(report([p])['verdict'],'NO_EVIDENCE')
if __name__=='__main__':unittest.main()
