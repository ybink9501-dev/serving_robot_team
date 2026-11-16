import hashlib

import pytest
import yaml
from PIL import Image

from serving_robot_mapping.saved_map_verification import verify_saved_map
from serving_robot_mapping.mapping_path_risk import MappingPathRiskGuard


def save_map(tmp_path, edge_free=False):
    image = Image.new('L', (100, 100), 0)
    pixels = image.load()
    for x in range(100):
        pixels[x, 0] = pixels[x, 99] = 100
    for y in range(100):
        pixels[0, y] = pixels[99, y] = 100
    if edge_free:
        pixels[50, 0] = 0
    image.save(tmp_path / 'map.pgm')
    path = tmp_path / 'map.yaml'
    path.write_text(yaml.safe_dump(dict(image='map.pgm', mode='raw', negate=0,
                                        resolution=.05, origin=[0., 0., 0.])))
    return path


def test_saved_file_is_decisive_and_hashed(tmp_path):
    path = save_map(tmp_path)
    result = verify_saved_map(path)
    assert result['verified']
    assert result['image_sha256'] == hashlib.sha256((tmp_path / 'map.pgm').read_bytes()).hexdigest()
    save_map(tmp_path, edge_free=True)
    rejected = verify_saved_map(path)
    assert not rejected['verified']
    assert rejected['reason'] == 'FREE_SPACE_REACHES_MAP_EDGE'
    assert rejected['image_sha256'] != result['image_sha256']


def test_unknown_and_invalid_encoding_fail_closed(tmp_path):
    path = save_map(tmp_path)
    with Image.open(tmp_path / 'map.pgm') as image:
        image.putpixel((50, 50), 255)
        image.save(tmp_path / 'map.pgm')
    assert verify_saved_map(path)['reason'] == 'FREE_SPACE_TOUCHES_UNKNOWN'
    meta = yaml.safe_load(path.read_text())
    meta['mode'] = 'trinary'
    path.write_text(yaml.safe_dump(meta))
    with pytest.raises(ValueError):
        verify_saved_map(path)


def test_diagnostics_do_not_allow_unsafe_start():
    guard = MappingPathRiskGuard()
    data = [0] * 10000
    data[50 * 100 + 56] = 100
    data[50 * 100 + 55] = 100
    assert guard.update_grid(data, 100, 100, .05, 0., 0.)
    result = guard.check_path([(2.525, 2.525), (2., 2.)])
    assert not result.allowed
    assert result.samples == 1
    assert result.failure_point == (2.525, 2.525)
    assert result.minimum_occupied_distance_m == pytest.approx(.25)
