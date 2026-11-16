import yaml
import pytest
from PIL import Image
from serving_robot_mapping.mapping_domain import load_domain,apply_domain,derive_candidate
from serving_robot_mapping.saved_map_verification import verify_saved_map
from serving_robot_mapping.mapping_path_risk import MappingPathRiskGuard


def contract(tmp_path):
    p=tmp_path/'domain.yaml'
    p.write_text(yaml.safe_dump(dict(schema_version=1,frame_id='map',site_id='test',
        operator_approved=True,approval_basis='explicit test approval',
        closed_portals=[{'id':'door'}],interior_polygon=[[.5,.5],[4.5,.5],[4.5,4.5],[.5,4.5]])))
    return p


def raw_map(tmp_path,unknown_inside=False):
    im=Image.new('L',(100,100),0)
    if unknown_inside:im.putpixel((50,50),255)
    im.save(tmp_path/'raw.pgm')
    p=tmp_path/'raw.yaml'
    p.write_text(yaml.safe_dump(dict(mode='raw',negate=0,resolution=.05,origin=[0,0,0],image='raw.pgm')))
    return p


def test_approved_domain_closes_only_excluded_exterior(tmp_path):
    p=raw_map(tmp_path);d=contract(tmp_path)
    old=p.read_bytes();image=(tmp_path/'raw.pgm').read_bytes()
    assert not verify_saved_map(p)['verified']
    result=derive_candidate(p,d)
    assert result['validation']['verified']
    assert not result['navigation_map_authorized']
    assert p.read_bytes()==old and (tmp_path/'raw.pgm').read_bytes()==image


def test_interior_unknown_is_not_excused_by_door_approval(tmp_path):
    result=derive_candidate(raw_map(tmp_path,True),contract(tmp_path))
    assert not result['validation']['verified']
    assert result['validation']['reason']=='FREE_SPACE_TOUCHES_UNKNOWN'


def test_unapproved_or_invalid_domain_rejected(tmp_path):
    p=contract(tmp_path);data=yaml.safe_load(p.read_text());data['operator_approved']=False
    p.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):load_domain(p)
    data['operator_approved']=True;data['interior_polygon']=[[0,0],[1,1],[0,1],[1,0]]
    p.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):load_domain(p)


def test_unknown_inside_and_grid_origin_preserved(tmp_path):
    domain=load_domain(contract(tmp_path));a=[0]*10000;a[5050]=-1
    bounded,_=apply_domain(a,100,100,.05,0,0,0,domain)
    assert bounded[5050]==-1 and bounded[0]==100
    shifted,_=apply_domain(a,100,100,.05,10,0,0,domain)
    assert set(shifted)=={100}


def test_common_start_block_remains_for_every_destination():
    data=[0]*10000;data[50*100+55]=100
    guard=MappingPathRiskGuard();guard.update_grid(data,100,100,.05,0,0)
    for destination in ((1.,1.),(4.,1.),(1.,4.)):
        result=guard.check_path([(2.525,2.525),destination])
        assert not result.allowed and result.samples==1
        assert result.failure_point==(2.525,2.525)


def test_unapproved_north_edge_inside_domain_still_fails(tmp_path):
    p=contract(tmp_path);data=yaml.safe_load(p.read_text())
    data['interior_polygon']=[[.5,.5],[4.5,.5],[4.5,6.],[.5,6.]]
    p.write_text(yaml.safe_dump(data))
    result=derive_candidate(raw_map(tmp_path),p)
    assert not result['validation']['verified']
    assert result['validation']['reason']=='FREE_SPACE_REACHES_MAP_EDGE'
