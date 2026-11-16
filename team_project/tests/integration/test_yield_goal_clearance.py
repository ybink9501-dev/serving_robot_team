from types import SimpleNamespace as NS
from cafe_autonomy.yield_policy import yield_goal_clear

def grid():
    return NS(info=NS(resolution=.1,width=30,height=30,origin=NS(position=NS(x=0.,y=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.))),data=[0]*900)

def test_point_free_but_body_hits_furniture_is_rejected():
    g=grid();g.data[15*30+18]=100
    assert not yield_goal_clear(g,(1.5,1.5),.48)
    assert yield_goal_clear(g,(1.0,1.5),.48)

def test_unknown_and_map_boundary_do_not_offer_yield_space():
    g=grid();g.data[15*30+15]=-1
    assert not yield_goal_clear(g,(1.5,1.5),.48)
    assert not yield_goal_clear(g,(.2,1.5),.48)
    assert not yield_goal_clear(None,(1.5,1.5),.48)
