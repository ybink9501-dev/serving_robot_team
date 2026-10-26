#!/usr/bin/env python3
"""V3 Goal-duplication / cancel safety regression, run against the V4 server.
test_safe_gate.py's fakes lack geometry_msgs.Twist (V4 imports it), so add it here."""
import runpy, sys
from pathlib import Path
here = Path(__file__).resolve().parent
sys.path.insert(0, str(here))
import test_mock_ros as T
_orig = T.install_fakes
def _install():
    _orig(); sys.modules['geometry_msgs.msg'].Twist = lambda: T.Msg()
T.install_fakes = _install
sys.argv = ['test_safe_gate.py', str(here / 'destination_server_lab_v4.py')]
runpy.run_path(str(here / 'test_safe_gate.py'), run_name='__main__')
