"""Start Mapping -> review artifact -> validation -> separately approved adoption.
Works on a copy of maps/ and config/, with the fake runner instead of ROS."""
import os
import shutil
import sys
import time
import json

import pytest
import yaml
from fastapi.testclient import TestClient

from .test_api import make

REAL = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
RUNNER = os.path.join(os.path.dirname(__file__), 'fake_mapping_runner.py')


@pytest.fixture()
def repo(tmp_path):
    r = tmp_path / 'repo'
    for rel in ('config', 'maps', 'ros_ws/src/serving_robot_navigation/serving_robot_navigation',
                'tools/site_builder/site_builder'):
        shutil.copytree(os.path.join(REAL, rel), r / rel, ignore=shutil.ignore_patterns('__pycache__', 'out'))
    return r


def build(tmp_path, repo, mode='finish', **kw):
    os.environ['FAKE_MAP_MODE'] = mode
    app = make(repo=str(repo), data_dir=str(tmp_path / 'data'), mapping_enabled=True,
               mapping_runner=RUNNER, mapping_ros_python=sys.executable,
               mapping_require_ros_env=False, **kw)
    app.state.mapping.cfg.ready_check = lambda: None
    return app


@pytest.fixture()
def client(tmp_path, repo):
    with TestClient(build(tmp_path, repo)) as c:
        c.repo = repo
        yield c


def wait_ready(c, secs=15):
    end = time.time() + secs
    while time.time() < end:
        st = c.get('/api/mapping/status').json()
        if st['phase'] == 'stopped':
            return st
        time.sleep(0.2)
    raise AssertionError(st)


def mark_autonomy_handoff_validated(save_result):
    """Test-only stand-in for later destination and Nav2 route validation."""
    path = os.path.join(save_result['review_result']['output_dir'], 'navigation_handoff.json')
    handoff = json.loads(open(path, encoding='utf-8').read())
    handoff['destination_profile'] = {'verified': True, 'profile_id': 'test-site'}
    handoff['nav2_route_validation'] = {'verified': True, 'routes_checked': 1}
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(handoff, f)


def runner_up(c, secs=10):
    """The hanging fake runner has written its log, so its Stop handler is installed."""
    end = time.time() + secs
    while time.time() < end:
        if c.get('/api/mapping/status').json()['progress']['goals_sent'] >= 4:
            return
        time.sleep(0.05)
    raise AssertionError('runner did not come up')


def test_status_contract_when_idle(client):
    st = client.get('/api/mapping/status').json()
    assert st['phase'] == 'idle' and st['store_id'] == 'demo_store'
    assert {'started_at', 'last_saved_revision', 'draft_ready', 'stage', 'progress'} <= set(st)


def test_disabled_backend_answers_503(tmp_path, repo):
    app = make(repo=str(repo), data_dir=str(tmp_path / 'd'))
    with TestClient(app) as c:
        r = c.post('/api/mapping/start')
        assert r.status_code == 503 and r.json()['code'] == 'MAPPING_DISABLED'


def test_full_flow_replaces_the_map(client):
    mdir = client.repo / 'maps' / 'stores' / 'demo_store'
    old_pgm = (mdir / 'map.pgm').read_bytes()
    old_rev = client.get('/api/stores/demo_store/map').json()['map_revision']
    # a saved editor draft exists before
    sem = client.get('/api/stores/demo_store/semantics').json()
    assert client.put('/api/stores/demo_store/semantics', json=sem).status_code == 200

    assert client.post('/api/mapping/start').status_code == 200
    st = wait_ready(client)
    assert st['draft_ready'] and st['error'] is None
    info = client.get('/api/stores/demo_store/map-review').json()
    assert info['width'] == 40 and client.get(info['image_url']).headers['content-type'] == 'image/png'

    # Save Map without the final confirmation is refused, nothing changes
    r = client.post('/api/mapping/save', json={'source_hash': info['source_hash'], 'strokes': []})
    assert r.status_code == 422
    assert (mdir / 'map.pgm').read_bytes() == old_pgm

    r = client.post('/api/mapping/save', json={'source_hash': info['source_hash'],
                                              'confirmed': True, 'strokes': []})
    assert r.status_code == 200, r.text
    out = r.json()
    assert out['adopted'] is False
    assert out['review_result']['navigation_validated'] is True
    handoff = out['review_result']['navigation_handoff']
    assert handoff['map_frame'] == 'map'
    assert handoff['completion_validation']['reason'] == 'CLOSED_FREE_REGION_VERIFIED'
    assert handoff['destination_profile'] == 'PENDING_AUTONOMY_INTEGRATION'
    assert handoff['nav2_route_validation'] == 'PENDING_AUTONOMY_INTEGRATION'
    assert handoff['operation_map_authorized'] is False
    assert client.get('/api/stores/demo_store/map').json()['map_revision'] == old_rev
    assert client.post('/api/mapping/adopt', json={}).status_code == 422
    pending = client.post('/api/mapping/adopt', json={'confirmed': True})
    assert pending.status_code == 422
    assert pending.json()['code'] == 'SAVED_MAP_NOT_APPROVED'
    assert client.get('/api/stores/demo_store/map').json()['map_revision'] == old_rev

    # The later autonomy phase must validate the destination profile and Nav2
    # routes before an operator can promote this candidate map.
    handoff_path = os.path.join(out['review_result']['output_dir'], 'navigation_handoff.json')
    saved_handoff = json.loads(open(handoff_path, encoding='utf-8').read())
    saved_handoff['destination_profile'] = {'verified': True, 'profile_id': 'test-site'}
    saved_handoff['nav2_route_validation'] = {'verified': True, 'routes_checked': 1}
    with open(handoff_path, 'w', encoding='utf-8') as f:
        json.dump(saved_handoff, f)
    adopted = client.post('/api/mapping/adopt', json={'confirmed': True})
    assert adopted.status_code == 200, adopted.text
    out = adopted.json()
    assert out['adopted'] is True and out['needs_robot_restart'] is True
    assert '로봇을 재시작' in out['message']
    saved_handoff = json.loads(open(handoff_path, encoding='utf-8').read())
    assert saved_handoff['operation_map_authorized'] is True
    assert saved_handoff['state'] == 'OPERATION_MAP_ADOPTED'
    new_map = client.get('/api/stores/demo_store/map').json()
    assert new_map['width'] == 40 and new_map['map_revision'] == old_rev + 1
    assert yaml.safe_load((mdir / 'map.yaml').read_text())['origin'][:2] == [0.0, 0.0]
    assert (mdir / 'map.pgm').read_bytes() != old_pgm
    # the old map is kept, with the editor's draft
    b = client.get('/api/stores/demo_store/map/backups').json()
    assert len(b) == 1 and b[0]['has_semantics']
    bdir = os.path.join(str(client.repo.parent), 'data', 'map_backups', 'demo_store', b[0]['id'])
    assert (open(os.path.join(bdir, 'map', 'map.pgm'), 'rb').read() == old_pgm)
    assert os.path.isfile(os.path.join(bdir, 'semantics.json'))
    # the editor starts empty on the new map, old drafts for the old revision are refused
    ed = client.get('/api/stores/demo_store/semantics').json()
    assert ed['entities'] == [] and ed['map_revision'] == new_map['map_revision']
    r = client.put('/api/stores/demo_store/semantics', json={**sem})
    assert r.status_code == 409
    # back to idle and a new mapping can start
    assert client.get('/api/mapping/status').json()['phase'] == 'idle'
    assert client.post('/api/mapping/start').status_code == 200
    wait_ready(client)


def test_failed_edited_map_validation_cannot_be_adopted(tmp_path, repo):
    with TestClient(build(tmp_path, repo, mode='unknown_contact')) as c:
        c.post('/api/mapping/start')
        wait_ready(c)
        info = c.get('/api/stores/demo_store/map-review').json()
        original = c.get('/api/stores/demo_store/map').json()
        result = c.post('/api/mapping/save', json={
            'source_hash': info['source_hash'], 'confirmed': True, 'strokes': []})
        assert result.status_code == 200
        assert result.json()['adopted'] is False
        assert result.json()['review_result']['navigation_validated'] is False
        assert result.json()['review_result']['navigation_validation']['reason'] == 'FREE_SPACE_TOUCHES_UNKNOWN'
        refused = c.post('/api/mapping/adopt', json={'confirmed': True})
        assert refused.status_code == 422
        assert refused.json()['code'] == 'SAVED_MAP_NOT_APPROVED'
        assert c.get('/api/stores/demo_store/map').json() == original


def test_unfinished_exploration_is_reviewable_but_not_auto_verified(tmp_path, repo):
    with TestClient(build(tmp_path, repo, mode='unverified_exit')) as c:
        c.post('/api/mapping/start')
        status = wait_ready(c)
        assert status['review_available'] is True
        assert status['draft_ready'] is True
        assert status['auto_map_verified'] is False
        assert status['map_complete'] is False
        assert status['completion_reason'] == 'EXPLORER_EXITED'
        review = c.get('/api/stores/demo_store/map-review').json()
        assert review['review_available'] is True
        assert review['auto_map_verified'] is False
        assert review['explorer_terminal_status'] == 'EXPLORER_EXITED'


def test_review_artifact_records_slam_and_manual_edit_provenance(tmp_path, repo):
    with TestClient(build(tmp_path, repo)) as c:
        assert c.post('/api/mapping/start').status_code == 200
        wait_ready(c)
        info = c.get('/api/stores/demo_store/map-review').json()
        original_map = c.get('/api/stores/demo_store/map').json()
        result = c.post('/api/mapping/save', json={
            'source_hash': info['source_hash'], 'confirmed': True,
            'strokes': [{'x1': 10, 'y1': 10, 'x2': 12, 'y2': 10, 'value': 100}],
        }).json()['review_result']
        with open(os.path.join(result['output_dir'], 'review_record.json')) as file:
            record = json.load(file)
        assert record['provenance']['base_map']['source'] == 'slam_saved_map_from_lidar_pipeline'
        assert record['provenance']['manual_edits'][0]['source'] == 'operator_manual_edit'
        assert record['provenance']['manual_edits'][0]['value'] == 100
        assert record['provenance']['manual_edits'][0]['points_top_left_pixels'][0] == {'x': 10, 'y': 10}
        assert result['navigation_handoff']['operation_map_authorized'] is False
        assert c.get('/api/stores/demo_store/map').json() == original_map


def test_origin_shift_is_applied(tmp_path, repo):
    with TestClient(build(tmp_path, repo, mapping_shift_origin=[-3.2, -2.4])) as c:
        c.post('/api/mapping/start')
        info = None
        wait_ready(c)
        info = c.get('/api/stores/demo_store/map-review').json()
        r = c.post('/api/mapping/save', json={'source_hash': info['source_hash'], 'confirmed': True, 'strokes': []})
        assert not r.json()['adopted']
        mark_autonomy_handoff_validated(r.json())
        assert c.post('/api/mapping/adopt', json={'confirmed': True}).json()['adopted']
        o = c.get('/api/stores/demo_store/map').json()['origin']
        assert (round(o['x'], 3), round(o['y'], 3)) == (-3.2, -2.4)


def test_no_orders_and_no_idle_home_while_mapping(tmp_path, repo):
    with TestClient(build(tmp_path, repo, mode='hang')) as c:
        assert c.post('/api/mapping/start').status_code == 200
        r = c.post('/api/tasks', json={'type': 'SERVE', 'destination_id': 'table_01'})
        assert r.status_code == 409 and r.json()['code'] == 'MAPPING_BUSY'
        assert c.get('/api/tasks').json() == []
        runner_up(c)
        c.post('/api/mapping/stop')
        wait_ready(c)
        r = c.post('/api/tasks', json={'type': 'SERVE', 'destination_id': 'table_01'})
        assert r.status_code in (200, 201)


def test_start_refused_while_orders_active(tmp_path, repo):
    with TestClient(build(tmp_path, repo)) as c:
        assert c.post('/api/tasks', json={'type': 'SERVE', 'destination_id': 'table_01'}).status_code in (200, 201)
        r = c.post('/api/mapping/start')
        assert r.status_code == 409 and r.json()['code'] == 'ORDERS_ACTIVE'


def test_adopt_refused_while_orders_active_then_retry(client):
    client.post('/api/mapping/start')
    info = (wait_ready(client), client.get('/api/stores/demo_store/map-review').json())[1]
    fleet = client.app.state.fleet
    fleet.queue.append('fake')                       # an order is waiting
    saved = client.post('/api/mapping/save', json={'source_hash': info['source_hash'],
                                                  'confirmed': True, 'strokes': []})
    assert saved.status_code == 200
    assert saved.json()['adopted'] is False
    r = client.post('/api/mapping/adopt', json={'confirmed': True})
    assert r.status_code == 409 and r.json()['code'] == 'ORDERS_ACTIVE'
    assert client.get('/api/stores/demo_store/map').json()['width'] == 180        # untouched
    fleet.queue.clear()
    mark_autonomy_handoff_validated(saved.json())
    r = client.post('/api/mapping/adopt', json={'confirmed': True})
    assert r.status_code == 200 and r.json()['adopted']
    assert client.get('/api/stores/demo_store/map').json()['width'] == 40


def test_review_and_save_without_a_draft(client):
    assert client.get('/api/stores/demo_store/map-review').status_code == 409
    r = client.post('/api/mapping/save', json={'confirmed': True, 'strokes': []})
    assert r.status_code == 409 and r.json()['code'] == 'NO_DRAFT'
    assert client.post('/api/mapping/adopt', json={'confirmed': True}).status_code == 409
    assert client.post('/api/mapping/stop').status_code == 409
    assert client.get('/api/stores/other/map-review').status_code == 404


def test_pending_draft_blocks_start_until_discarded(client):
    client.post('/api/mapping/start')
    wait_ready(client)
    r = client.post('/api/mapping/start')
    assert r.status_code == 409 and r.json()['code'] == 'DRAFT_PENDING'
    assert client.post('/api/mapping/discard').status_code == 200
    assert client.post('/api/mapping/start').status_code == 200
    wait_ready(client)


def test_map_revision_survives_a_restart_of_the_backend(tmp_path, repo):
    with TestClient(build(tmp_path, repo)) as c:
        c.post('/api/mapping/start')
        info = (wait_ready(c), c.get('/api/stores/demo_store/map-review').json())[1]
        r = c.post('/api/mapping/save', json={'source_hash': info['source_hash'],
                                              'confirmed': True, 'strokes': []})
        assert r.json()['adopted'] is False
        blocked = c.post('/api/mapping/adopt', json={'confirmed': True})
        assert blocked.status_code == 422
        assert blocked.json()['code'] == 'SAVED_MAP_NOT_APPROVED'
        rev = c.get('/api/stores/demo_store/map').json()['map_revision']
        assert rev == 1
    with TestClient(build(tmp_path, repo)) as c2:                     # backend started again
        assert c2.get('/api/stores/demo_store/map').json()['map_revision'] == rev
        assert c2.get('/api/stores/demo_store/semantics').json()['map_revision'] == rev
