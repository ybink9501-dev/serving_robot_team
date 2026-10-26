#!/usr/bin/env python3
"""Offline regression: deploy script shall not write Nav2 param/YAML values."""
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace
import ast
import activate_v4 as A

source = Path(A.__file__).read_text()
tree = ast.parse(source)
# Inspect constant argument expressions in subprocess calls to guard against unsafe regression.
assert not any(
    isinstance(node, (ast.List, ast.Tuple))
    and len(node.elts) >= 3
    and all(isinstance(x, ast.Constant) for x in node.elts[:3])
    and [x.value for x in node.elts[:3]] == ['ros2', 'param', 'set']
    for node in ast.walk(tree)
), 'Nav2 param set call is present in installer'
assert "shutil.copy2" not in source and "LAB_NAV2" not in source, 'Nav2 config overwrite is present'
assert 'Unsupported flag' in source and "arg != '--check-only'" in source

calls=[]
def readonly_run(cmd, **kwargs):
    calls.append(cmd)
    assert cmd[:3] == ['ros2','param','get'], cmd
    n=cmd[-1]
    v=A.EXPECTED_NAV2[n]
    p='String' if 'plugin' in n else 'Boolean' if n.endswith('stateful') else 'Double'
    return SimpleNamespace(stdout=f'{p} value is: {v}\n', stderr='', returncode=0)
with patch.object(A.subprocess, 'run', side_effect=readonly_run):
    A.check_nav2_readonly()
assert len(calls) == len(A.EXPECTED_NAV2)
print('PASS: 6 Nav2 values verified read-only and no YAML/param writes')

def mismatched(cmd, **kwargs):
    n=cmd[-1]
    v='15.0' if n == 'progress_checker.movement_time_allowance' else A.EXPECTED_NAV2[n]
    return SimpleNamespace(stdout=f'String value is: {v}\n', stderr='', returncode=0)
with patch.object(A.subprocess, 'run', side_effect=mismatched):
    try:
        A.check_nav2_readonly()
    except RuntimeError as e:
        assert 'mismatch' in str(e)
    else:
        raise AssertionError('Mismatched runtime config not rejected')
print('PASS: changed 15.0s runtime value is rejected, not overwritten')
