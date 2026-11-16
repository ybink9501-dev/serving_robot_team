"""Verify the saved raw map without editing pixels or changing completion rules."""
import hashlib
import io
import math
from pathlib import Path
from PIL import Image
import yaml
from .mapping_completion_verifier import verify_closed_free_region


def verify_saved_map(path):
    path = Path(path)
    yaml_bytes = path.read_bytes()
    try:
        meta = yaml.safe_load(yaml_bytes)
    except yaml.YAMLError as exc:
        raise ValueError('Invalid map YAML') from exc
    if not isinstance(meta, dict):
        raise ValueError('Expected map metadata')
    if meta.get('mode') != 'raw' or meta.get('negate', 0) != 0:
        raise ValueError('Expected the mapping session raw, non-negated map')
    resolution = float(meta['resolution'])
    origin = tuple(map(float, meta['origin']))
    if len(origin) != 3 or not all(math.isfinite(v) for v in origin):
        raise ValueError('Invalid map origin')
    image = path.parent / meta['image']
    image_bytes = image.read_bytes()
    with Image.open(io.BytesIO(image_bytes)) as im:
        if im.mode != 'L':
            raise ValueError('Expected an 8-bit occupancy image')
        width, height = im.size
        pixels = list(im.getdata())
    data = tuple(-1 if pixels[(height-1-y)*width+x] > 100
                 else pixels[(height-1-y)*width+x]
                 for y in range(height) for x in range(width))
    verified, reason = verify_closed_free_region(data, width, height, resolution)
    return dict(verified=verified, reason=reason, width=width, height=height,
                resolution=resolution, origin=origin,
                yaml_sha256=hashlib.sha256(yaml_bytes).hexdigest(),
                image_sha256=hashlib.sha256(image_bytes).hexdigest())
