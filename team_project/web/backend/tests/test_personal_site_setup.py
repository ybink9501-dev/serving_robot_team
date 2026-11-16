"""Private drafts/bundles and mode isolation; simulated Fleet, never Gazebo."""
import copy
import hashlib
import json
from pathlib import Path
from unittest.mock import patch,PropertyMock
import pytest
from fastapi.testclient import TestClient
from tests.test_personal_gateway import pair, order
from app.personal import create_personal_app
from app.personal_site_setup import seed_semantics


@pytest.fixture
def client(pair):
    g,s,cfg=pair
    g.close()
    app=create_personal_app(cfg,transport=s)
    g=app.state.gateway
    with TestClient(app) as c:
        yield c,app.state.site_setup,g,s


def base(client):
    c,setup,g,s=client
    return seed_semantics(g,setup.settings.store_id)


def test_existing_frontend_draft_contract_and_seed_constraints(client):
    c,setup,g,s=client;doc=base(client);sid=setup.settings.store_id
    assert len([p for p in doc['waypoints'] if p['entity_id']=='table_01'])==2
    assert any(z['type']=='exclusive' for z in doc['zones'])
    doc['zones'].append(dict(id='indoor',type='operating',polygon=[[-1,-1],[8,-1],[8,6],[-1,6]]))
    r=c.put(f'/api/stores/{sid}/semantics',json=doc)
    assert r.status_code==200,r.text
    assert c.get(f'/api/stores/{sid}/semantics').json()==r.json()
    assert c.get(f'/api/stores/{sid}/semantics/status').status_code==200
    assert not s.calls


def test_operating_boundary_required_before_provisional_export(client):
    c,setup,g,s=client;sid=setup.settings.store_id
    assert c.put(f'/api/stores/{sid}/semantics',json=base(client)).status_code==200
    r=c.post(f'/api/stores/{sid}/semantics/apply')
    assert r.status_code==422 and r.json()['code']=='OPERATING_REGION_REQUIRED'
    assert not s.calls


def test_promotion_and_adoption_require_separate_validation(client):
    c,setup,g,s=client;sid=setup.settings.store_id
    assert c.post(f'/api/stores/{sid}/semantics/promote',json={}).status_code==409
    assert c.post('/api/mapping/adopt',json={}).status_code==409
    assert not s.calls


def test_mapping_mode_blocks_serving_without_dispatch(client):
    c,setup,g,s=client
    with patch.object(type(setup.mapping),'active',new_callable=PropertyMock,return_value=True):
        r=c.post('/api/tasks',json=order())
    assert r.status_code==409,r.text
    assert r.json()['code']=='ROBOT_IN_MAPPING_MODE'
    assert not s.calls


def valid_doc(client):
    doc=base(client)
    for e in doc['entities']:
        if e['type']=='table':e['top_size_m']=dict(x=.8,y=.8)
    doc['zones'].append(dict(id='indoor',type='operating',polygon=[[-2,-2],[9,-2],[9,7],[-2,7]]))
    return doc


def test_export_preserves_all_approaches_and_never_activates_bundle(client):
    c,setup,g,s=client;sid=setup.settings.store_id;doc=valid_doc(client)
    files=[Path(g.profile[k]) for k in ('map_yaml','image_path','waypoints_file','zones_file')]
    before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    assert c.put(f'/api/stores/{sid}/semantics',json=doc).status_code==200
    r=c.post(f'/api/stores/{sid}/semantics/apply')
    assert r.status_code==200,r.text
    folder=Path(r.json()['output_dir']);bundle=json.loads((folder/'bundle.json').read_text())
    assert len(bundle['destinations']['table_01']['approaches'])==2
    assert bundle['destinations']['robot_01_HOME']==g.destinations['robot_01_HOME']
    assert bundle['coordinate_transform']=='identity'
    assert not bundle['navigation_authorized'] and bundle['gazebo_validation']=='NOT_RUN'
    assert before=={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    assert not s.calls


def test_table_heading_and_unsupported_zone_shape_rejected(client):
    c,setup,g,s=client;sid=setup.settings.store_id;doc=valid_doc(client)
    next(p for p in doc['waypoints'] if p['entity_id']=='table_01')['pose']['yaw']=0.
    assert c.put(f'/api/stores/{sid}/semantics',json=doc).status_code==200
    r=c.post(f'/api/stores/{sid}/semantics/apply')
    assert r.status_code==422 and r.json()['code']=='TABLE_NOT_PERPENDICULAR',r.text
    doc=valid_doc(client)
    doc['zones'].append(dict(id='triangle',type='no_go',polygon=[[7,6],[8,6],[8,7]]))
    assert c.put(f'/api/stores/{sid}/semantics',json=doc).status_code==200
    r=c.post(f'/api/stores/{sid}/semantics/apply')
    assert r.status_code==422 and r.json()['code']=='NON_RECTANGULAR_RUNTIME_ZONE',r.text


def test_operation_mode_cannot_start_mapping(client):
    c,setup,g,s=client
    with patch('app.personal_site_setup.runtime_mode',return_value='operation'):
        r=c.post('/api/mapping/start',json={})
    assert r.status_code==409 and r.json()['code']=='MAPPING_MODE_REQUIRED'
    assert not s.calls


def test_in_motion_geometry_checks_block_but_draft_save_allowed(client):
    c,setup,g,s=client;sid=setup.settings.store_id
    with patch('app.personal_site_setup.mapping_motion_active',return_value=True):
        assert c.put(f'/api/stores/{sid}/semantics',json=base(client)).status_code==200
        assert c.get(f'/api/stores/{sid}/semantics/graph').status_code==409
        assert c.post(f'/api/stores/{sid}/semantics/apply').status_code==409
    assert not s.calls
