"""Read-only identity of an operation profile; shared by web and robot launch."""
import hashlib
import json
from pathlib import Path
import yaml


def load_profile(root, path):
    root = Path(root).resolve()
    path = Path(path)
    data = yaml.safe_load((path if path.is_absolute() else root / path).read_text())
    for key in ('map_yaml', 'waypoints_file', 'zones_file'):
        value = Path(data[key])
        data[key] = str(value if value.is_absolute() else root / value)
    meta = yaml.safe_load(Path(data['map_yaml']).read_text())
    image = Path(meta['image'])
    image = image if image.is_absolute() else Path(data['map_yaml']).parent / image
    digest = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
    identity = {'site_id': data['site_id'], 'frame_id': 'map',
                'robots': data['robots'], 'world_to_map': data['world_to_map'],
                'map': digest(data['map_yaml']), 'image': digest(image),
                'waypoints': digest(data['waypoints_file']), 'zones': digest(data['zones_file'])}
    data['profile_revision'] = hashlib.sha256(json.dumps(identity, sort_keys=True,
                                                        allow_nan=False).encode()).hexdigest()
    data['image_path'] = str(image)
    return data
