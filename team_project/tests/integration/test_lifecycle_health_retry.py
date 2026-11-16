"""A lost lifecycle response must not permanently suppress readiness checks."""
from concurrent.futures import Future
from types import SimpleNamespace
import cafe_autonomy.mission as module
from cafe_autonomy.mission import Mission


class Client:
    def __init__(self):
        self.requests=[]
        self.removed=[]
    def service_is_ready(self):return True
    def call_async(self,request):
        future=Future()
        self.requests.append(future)
        return future
    def remove_pending_request(self,future):self.removed.append(future)


def test_lost_response_is_cancelled_and_retried(monkeypatch):
    clock=[0.]
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    client=Client()
    mission=SimpleNamespace(health_clients={'controller_server':client},health_pending={},nav_states={})
    Mission.check_health(mission)
    first=client.requests[0]
    clock[0]=1.
    Mission.check_health(mission)
    assert len(client.requests)==1
    clock[0]=3.
    Mission.check_health(mission)
    assert first.cancelled() and client.removed==[first]
    assert len(client.requests)==2
    assert not mission.nav_states
    client.requests[-1].set_result(SimpleNamespace(current_state=SimpleNamespace(id=3)))
    assert mission.nav_states['controller_server']==(3,3.)
    assert not mission.health_pending
