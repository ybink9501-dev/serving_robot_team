from threading import Event
import time
from serving_robot_mapping.mapping_visibility_worker import VisibilityWorker


def test_one_readonly_job_does_not_block_or_queue_new_map_revisions():
    entered=Event();release=Event();seen=[]
    def evaluate(grid,*args):
        seen.append(grid);entered.set();release.wait(2)
        return [],{'snapshot':True}
    worker=VisibilityWorker(evaluate);data=[0]*9;grid=(data,3,3,.05,0.,0.,0.)
    try:
        assert worker.poll('old',grid,(0,0),[],.52,.5,12.) is None
        assert entered.wait(1)
        data[0]=100
        start=time.monotonic()
        assert worker.poll('new',grid,(1,1),[],.52,.5,12.) is None
        assert time.monotonic()-start<.1 and len(seen)==1
        assert seen[0][0][0]==0
        release.set();worker.future.result(timeout=1)
        key,result=worker.poll('new',grid,(1,1),[],.52,.5,12.)
        assert key=='old' and result[1]['snapshot']
    finally:release.set();worker.close()
