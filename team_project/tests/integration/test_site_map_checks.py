import math
import cv2,numpy as np,pytest,yaml
from cafe_autonomy.site_map_checks import validate_site_map

def site(tmp_path,mode='trinary',negate=0):
    value=0 if mode=='raw' or negate else 254
    cv2.imwrite(str(tmp_path/'map.pgm'),np.full((40,40),value,dtype=np.uint8))
    (tmp_path/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.1,origin=[10.,-2.,math.pi/2],mode=mode,negate=negate,free_thresh=.25)))
    point={'x':8.,'y':0.}
    (tmp_path/'points.yaml').write_text(yaml.safe_dump({'destinations':{'kitchen':point}}))
    return dict(map_yaml=str(tmp_path/'map.yaml'),waypoints_file=str(tmp_path/'points.yaml'),robots={'robot_01':dict(map_x=8.,map_y=0.)})

@pytest.mark.parametrize('mode,negate',[('trinary',0),('trinary',1),('raw',0),('scale',0)])
def test_rotated_map_and_supported_encodings(tmp_path,mode,negate):
    assert validate_site_map(site(tmp_path,mode,negate))

def test_unknown_cannot_be_used_as_body_clearance(tmp_path):
    c=site(tmp_path,'raw');image=cv2.imread(str(tmp_path/'map.pgm'),0);image[18:22,18:22]=255;cv2.imwrite(str(tmp_path/'map.pgm'),image)
    with pytest.raises(ValueError,match='kitchen'):validate_site_map(c)

def test_alternate_table_approach_is_allowed_but_invalid_start_is_not(tmp_path):
    c=site(tmp_path);good={'x':8.,'y':0.};bad={'x':100.,'y':100.}
    (tmp_path/'points.yaml').write_text(yaml.safe_dump({'destinations':{'table':{'approaches':{'a':bad,'b':good}}}}))
    assert validate_site_map(c)
    c['robots']['robot_01']['map_x']=100.
    with pytest.raises(ValueError,match='initial pose'):validate_site_map(c)

@pytest.mark.parametrize('field,value',[('free_thresh',1.5),('resolution',0.),('negate',2)])
def test_invalid_map_cannot_turn_obstacles_into_free_space(tmp_path,field,value):
    c=site(tmp_path);p=tmp_path/'map.yaml';m=yaml.safe_load(p.read_text());m[field]=value;p.write_text(yaml.safe_dump(m))
    with pytest.raises(ValueError,match='Invalid'):validate_site_map(c)
