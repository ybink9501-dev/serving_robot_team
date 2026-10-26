#!/usr/bin/env python3
"""Safety regression: remote Action uncertainty must block follow-up GO."""
import sys
from pathlib import Path
import test_mock_ros as T
from test_mock_ros import Sim

T.install_fakes()
path=str(Path(sys.argv[1]).resolve())
checks=[]
def check(name, condition):
    checks.append((name, bool(condition)))
    print(('PASS' if condition else 'FAIL')+' '+name)

# A send_goal_async future that never resolves. Timeout must not permit new GO.
s=Sim(path, navigation_timeout_s=10.0)
s.go('table_03');s.run(11.0)
check('timeout enters ERROR',s.states[-1]=='ERROR')
s.go('table_01')
check('new GO blocked with unresolved remote goal',s.goals==1 and s.events[-1]=='REJECTED')
h=s.accept(0)
check('late acceptance requests cancel',h.cancelled)
s.go('table_01')
check('new GO blocked until result acknowledgment',s.goals==1)
s.cancel_result(h)
s.go('table_01')
check('GO allowed after terminal result',s.goals==2)
s.close()
# Result after accepted+timeout clears block too.
s=Sim(path,navigation_timeout_s=10.0)
s.go('table_01');h=s.accept(0);s.run(11.0)
check('accepted goal canceled on timeout',h.cancelled)
s.go('table_03')
check('accepted but unresolved goal blocks GO',s.goals==1)
s.cancel_result(h);s.go('table_03')
check('accepted terminal result unlocks GO',s.goals==2)
s.close()
print('PASS' if all(x[1] for x in checks) else 'FAIL',sum(v for _,v in checks),'/',len(checks))
sys.exit(0 if all(x[1] for x in checks) else 1)
