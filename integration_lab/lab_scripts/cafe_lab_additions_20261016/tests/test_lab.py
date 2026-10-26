import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from serving_flow import ServingFlow
from build_cafe_map import combine, hit, make_map, candidate_points


def test_kitchen_table_manual_return():
    flow=ServingFlow(mode='manual')
    assert flow.start('table_01')[0] == ('GO','kitchen')
    assert flow.robot_state('NAVIGATING','kitchen') == []
    assert flow.robot_state('SERVING','kitchen')[0] == ('GO','table_01')
    assert flow.robot_state('SERVING','table_01')[0][1] == 'WAIT_HANDOVER'
    assert flow.complete()[0] == ('GO','kitchen')
    assert flow.robot_state('SERVING','kitchen')[-1][1]=='COMPLETED'


def test_auto_no_premature_completion():
    t=[100.0]
    flow=ServingFlow(mode='auto',auto_wait_s=12,clock=lambda:t[0])
    flow.start('table_01')
    flow.robot_state('SERVING','kitchen')
    assert flow.tick()==[]
    flow.robot_state('SERVING','table_01')
    t[0]+=11
    assert flow.tick()==[]
    t[0]+=1
    assert flow.tick()[0]==('GO','kitchen')


def test_not_succeeded_not_serving():
    flow=ServingFlow()
    flow.start('table_01')
    assert flow.robot_state('NAVIGATING','kitchen')==[]
    assert flow.robot_state('SERVING','table_01')==[]  # wrong phase
    assert flow.state=='GOING_KITCHEN'
    assert flow.robot_state('ERROR','kitchen')[-1][1]=='ERROR'


def test_geom_map():
    s=dict(kind='box',x=-2,y=1,yaw=0,sx=1,sy=1,source='box')
    img,w,h,x,y=make_map([s],[dict(name='test',x=-2,y=1)],0.05,0.7)
    assert w>30 and h>30
    assert img.count(0)>0
    assert hit(-2,1,s)
    assert not hit(0,0,s)
    assert combine((1,2,0,0),(3,4,0,0))==(4,6,0,0)


def test_candidates_not_auto_approved():
    m=[dict(name='table_set_01',x=1,y=1,yaw=0,collisions=1)]
    s=[dict(kind='circle',x=1,y=1,yaw=0,radius=.3,source='foo')]
    results=candidate_points(m,s,(-5,-5,5,5))
    assert results['table_01']['candidate_poses']
    assert all(not p['verified'] for p in results['table_01']['candidate_poses'])
