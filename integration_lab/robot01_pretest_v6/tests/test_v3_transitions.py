"""Execute the installed V3 Supervisor's real callback methods WITHOUT ROS startup.
This is not a live driving test, and it does not publish anything to ROS.
"""
import ast
import json
import os
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace


def source_file():
    if os.environ.get('LAB_V3_SOURCE'):
        return Path(os.environ['LAB_V3_SOURCE'])
    return Path.home()/'team_robot_lab/serving_v3/supervisor_v3.py'


def callbacks():
    path=source_file()
    if not path.is_file():raise FileNotFoundError(f'Installed V3 supervisor missing: {path}')
    src=ast.parse(path.read_text(encoding='utf8'),filename=str(path))
    cls=next(n for n in src.body if isinstance(n,ast.ClassDef) and n.name=='Supervisor')
    # Compile only the class definition; no ROS imports, node init, ROS topics or motion.
    import time
    env={'Node':object,'json':json,'time':time,
         'json_msg':lambda obj:SimpleNamespace(data=json.dumps(obj))}
    exec(compile(ast.Module(body=[cls],type_ignores=[]),str(path),'exec'),env)
    return env['Supervisor']


class StateTransitionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.Supervisor=callbacks()
    def setUp(self):
        self.n=object.__new__(self.Supervisor)
        self.n.robot='robot_01';self.n.revision=2;self.n.state='IDLE'
        self.n.table=None;self.n.side=None;self.n.target=None
        self.n.options={'table_01':{'top':{'enabled':True,'id':'table_01_top'},
                                    'left':{'enabled':False,'id':'table_01_left'}}}
        self.n.commands=[];self.n.states=[];self.n.feedback=[];self.n.planning=None
        self.n.pub=SimpleNamespace(publish=lambda m:self.n.commands.append(json.loads(m.data)),
                                   get_subscription_count=lambda:1)
        self.n.ready=lambda:None
        self.n.set_state=lambda s,r:(setattr(self.n,'state',s),self.n.states.append((s,r)))
        self.n.status=lambda reason=None:self.n.feedback.append(reason)
        self.n.go=lambda target:(setattr(self.n,'target',target),self.n.commands.append({'command':'GO','destination_id':target}))
        self.n.choose_table=lambda:(setattr(self.n,'state','GOING_TABLE'),setattr(self.n,'target','table_01_top'))
    def send(self,method,command):
        getattr(self.n,method)(SimpleNamespace(data=json.dumps(command)))
    def robot(self,destination,state='SERVING'):
        self.send('on_robot_state',{'robot_id':'robot_01','destination_id':destination,'state':state})
    def test_full_manual_serving_sequence(self):
        self.send('on_request',{'command':'START','table':'table_01','side':'top'})
        self.assertEqual(self.n.state,'GOING_PICKUP')
        self.assertEqual(self.n.target,'counter_pickup')
        self.robot('counter_pickup')
        self.assertEqual(self.n.state,'WAIT_LOAD')
        self.send('on_complete',{'command':'CONFIRM_LOAD'})
        self.assertEqual(self.n.state,'GOING_TABLE')
        self.robot('table_01_top')
        self.assertEqual(self.n.state,'WAIT_HANDOVER')
        self.send('on_complete',{'command':'COMPLETE'})
        self.assertEqual(self.n.state,'RETURNING_PICKUP')
        self.assertEqual(self.n.target,'counter_pickup')
        self.robot('counter_pickup')
        self.assertEqual(self.n.state,'COMPLETED')
    def test_unconfirmed_load_cannot_leave_kitchen(self):
        self.send('on_request',{'command':'START','table':'table_01','side':'top'})
        self.robot('counter_pickup')
        self.send('on_complete',{'command':'COMPLETE'})
        self.assertEqual(self.n.state,'WAIT_LOAD')
        self.assertEqual(self.n.target,'counter_pickup')
    def test_no_complete_before_handover(self):
        self.send('on_request',{'command':'START','table':'table_01','side':'top'})
        self.send('on_complete',{'command':'COMPLETE'})
        self.assertEqual(self.n.state,'GOING_PICKUP')
    def test_reject_chair_approach(self):
        self.send('on_request',{'command':'START','table':'table_01','side':'left'})
        self.assertEqual(self.n.state,'IDLE')
        self.assertEqual(self.n.commands,[])
    def test_cancel_does_not_issue_goal(self):
        self.send('on_request',{'command':'START','table':'table_01','side':'top'})
        self.send('on_request',{'command':'CANCEL'})
        self.assertEqual(self.n.state,'CANCELED')
        self.assertEqual(self.n.commands[-1],{'command':'CANCEL'})
    def test_wrong_robot_state_does_not_advance(self):
        self.send('on_request',{'command':'START','table':'table_01','side':'top'})
        self.robot('table_01_top')
        self.assertEqual(self.n.state,'GOING_PICKUP')

if __name__=='__main__':unittest.main()
