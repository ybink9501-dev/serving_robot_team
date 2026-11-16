"""Every texture used by the bundled person visual must survive a fresh clone."""
from pathlib import Path
import xml.etree.ElementTree as ET


def test_person_visual_has_all_referenced_textures():
    mesh = Path(__file__).resolve().parents[1] / 'serving_robot_person_simulator/assets/standing.dae'
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    images = ET.parse(mesh).findall('.//c:library_images/c:image/c:init_from', ns)
    assert len(images) == 9
    for image in images:
        texture = (mesh.parent / image.text).resolve()
        assert texture.is_relative_to(mesh.parent.resolve())
        assert texture.read_bytes().startswith(bytes.fromhex('89504e470d0a1a0a'))
