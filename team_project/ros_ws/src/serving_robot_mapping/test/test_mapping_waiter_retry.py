from pathlib import Path
from types import SimpleNamespace
import importlib.util

path=Path(__file__).resolve().parents[2]/'serving_robot_bringup/scripts/wait_for_ready.py'
spec=importlib.util.spec_from_file_location('mapping_waiter_retry_test',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class Future:
    def __init__(self,done=False,state=2):self.complete=done;self.state=state
    def done(self):return self.complete
    def result(self):return SimpleNamespace(current_state=SimpleNamespace(id=self.state))


class Client:
    def __init__(self):self.removed=[];self.requests=[]
    def service_is_ready(self):return True
    def remove_pending_request(self,future):self.removed.append(future)
    def call_async(self,request):
        future=Future();self.requests.append(future);return future


def state(timeout,future):
    client=Client()
    return SimpleNamespace(args=SimpleNamespace(lifecycle_request_timeout=timeout),
        lifecycle_active=set(),state_clients={'bt_navigator':client},
        lifecycle_pending={'bt_navigator':future},lifecycle_sent={'bt_navigator':0.}),client


def test_lost_response_is_retried_without_fabricating_active(monkeypatch):
    monkeypatch.setattr(module.time,'monotonic',lambda:10.)
    old=Future();node,client=state(3.,old)
    module.Waiter._check_lifecycle(node)
    assert client.removed==[old] and len(client.requests)==1
    assert not node.lifecycle_active
    client.requests[0].complete=True;client.requests[0].state=3
    module.Waiter._check_lifecycle(node)
    assert node.lifecycle_active=={'bt_navigator'}


def test_default_operation_behavior_is_unchanged(monkeypatch):
    monkeypatch.setattr(module.time,'monotonic',lambda:1000.)
    old=Future();node,client=state(0.,old)
    module.Waiter._check_lifecycle(node)
    assert not client.removed and not client.requests
    assert not node.lifecycle_active


def test_inactive_reply_never_passes_readiness(monkeypatch):
    monkeypatch.setattr(module.time,'monotonic',lambda:10.)
    node,client=state(3.,Future(done=True,state=2))
    module.Waiter._check_lifecycle(node)
    assert not node.lifecycle_active
