"""Check generated Robot01 LAB waypoint snapshot after prepare.py, no ROS movement."""
import json
from pathlib import Path
import pytest
import yaml


def test_generated_semantic_snapshot():
    root=Path(__file__).resolve().parents[1]
    manifest=root/'generated/manifest.json'
    if not manifest.exists():
        pytest.skip('Run prepare.py before snapshot audit')
    d=yaml.safe_load((root/'generated/active_waypoints.yaml').read_text())
    r=json.loads(manifest.read_text())
    ids={a['entity_id'] for a in d['waypoints']}
    assert len(ids)==len(d['waypoints'])==14
    assert 'counter_pickup' in ids and 'kitchen' in ids
    for i in range(1,5):
        name=f'table_{i:02d}'
        assert all(k in ids for k in (name,f'{name}_top',f'{name}_bottom'))
        assert {v['side'] for v in r['choices'][name] if v['enabled']}=={'top','bottom'}
        assert {v['side'] for v in r['choices'][name] if not v['enabled']}=={'left','right'}
    assert d['map_revision']==r['map_revision']
    assert r['map_info']['width']>0 and r['map_info']['height']>0
