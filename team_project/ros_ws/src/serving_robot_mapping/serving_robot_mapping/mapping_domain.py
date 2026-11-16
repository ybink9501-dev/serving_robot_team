"""Explicit operator-approved mapping domain; never infers physical walls."""
import hashlib
import math
from pathlib import Path
import yaml
import numpy as np


def load_domain(path):
    raw = Path(path).read_bytes()
    data = yaml.safe_load(raw)
    if (not isinstance(data, dict) or data.get('schema_version') != 1
            or data.get('frame_id') != 'map'
            or data.get('operator_approved') is not True
            or not data.get('approval_basis') or not isinstance(data.get('closed_portals'),list)):
        raise ValueError('Mapping domain requires explicit operator approval')
    from .mapping_polygon import validate_polygon
    data['interior_polygon'] = validate_polygon(data.get('interior_polygon', []))
    data['sha256'] = hashlib.sha256(raw).hexdigest()
    return data


def _convex(points):
    turns=[(b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0])
           for a,b,c in zip(points,points[1:]+points[:1],points[2:]+points[:2])]
    return all(t>0 for t in turns) or all(t<0 for t in turns)


def inside(domain, x, y):
    points = domain['interior_polygon']
    if not _convex(points):
        from .mapping_polygon import contains
        return contains(points,x,y)
    cross = [(b[0]-a[0])*(y-a[1])-(b[1]-a[1])*(x-a[0])
             for a, b in zip(points, points[1:]+points[:1])]
    return all(v >= 0 for v in cross) or all(v <= 0 for v in cross)


def apply_domain(data, width, height, resolution, ox, oy, yaw, domain):
    if (len(data) != width*height or width < 3 or height < 3
            or resolution <= 0 or not all(math.isfinite(v) for v in (resolution, ox, oy, yaw))):
        raise ValueError('Invalid domain grid')
    # Same half-plane predicates in vectorized form; no changed mask semantics.
    c, s = math.cos(yaw), math.sin(yaw)
    px=(np.arange(width,dtype=float)+.5)*resolution
    py=(np.arange(height,dtype=float)+.5)*resolution
    xx=ox+c*px[None,:]-s*py[:,None]
    yy=oy+s*px[None,:]+c*py[:,None]
    positive=np.ones((height,width),dtype=bool)
    negative=positive.copy()
    points=domain['interior_polygon']
    for a,b in zip(points,points[1:]+points[:1]):
        cross=(b[0]-a[0])*(yy-a[1])-(b[1]-a[1])*(xx-a[0])
        positive &= cross>=0
        negative &= cross<=0
    if _convex(points):excluded=~(positive|negative)
    else:
        from .mapping_polygon import polygon_mask
        excluded=~polygon_mask(points,xx,yy)
    result=np.asarray(data,dtype=np.int16).reshape(height,width).copy()
    counts={'excluded_free':int(np.count_nonzero(excluded & (result>=0) & (result<=25))),
            'excluded_unknown':int(np.count_nonzero(excluded & (result==-1))),
            'excluded_cells':int(np.count_nonzero(excluded))}
    result[excluded]=100
    return result.ravel().tolist(),counts



def derive_candidate(raw_yaml, domain_path):
    """Preserve raw input; write a separately labelled, non-published candidate."""
    import io
    import json
    from PIL import Image
    from .saved_map_verification import verify_saved_map
    raw_yaml = Path(raw_yaml)
    raw_bytes = raw_yaml.read_bytes()
    meta = yaml.safe_load(raw_bytes)
    if meta.get('mode') != 'raw' or meta.get('negate', 0) != 0:
        raise ValueError('Expected non-negated raw source map')
    image_path = raw_yaml.parent/meta['image']
    image_bytes = image_path.read_bytes()
    with Image.open(io.BytesIO(image_bytes)) as image:
        if image.mode != 'L':
            raise ValueError('Expected 8-bit raw image')
        w, h = image.size
        pixels = list(image.getdata())
    data = [-1 if pixels[(h-1-y)*w+x] > 100 else pixels[(h-1-y)*w+x]
            for y in range(h) for x in range(w)]
    domain = load_domain(domain_path)
    bounded, counts = apply_domain(data, w, h, float(meta['resolution']),
                                   *map(float, meta['origin']), domain)
    target = raw_yaml.with_name(raw_yaml.stem+'_interior_candidate.yaml')
    target_image = target.with_suffix('.pgm')
    out = bytes(255 if bounded[(h-1-y)*w+x] < 0 else bounded[(h-1-y)*w+x]
                for y in range(h) for x in range(w))
    Image.frombytes('L', (w, h), out).save(target_image)
    meta['image'] = target_image.name
    target.write_text(yaml.safe_dump(meta, sort_keys=False))
    result = dict(source_yaml_sha256=hashlib.sha256(raw_bytes).hexdigest(),
                  source_image_sha256=hashlib.sha256(image_bytes).hexdigest(),
                  domain=domain, exclusions=counts, candidate_yaml=str(target),
                  occupancy_semantics='100 includes operator-approved outside-domain exclusion',
                  navigation_map_authorized=False, validation=verify_saved_map(target))
    target.with_suffix('.provenance.json').write_text(json.dumps(result, indent=2))
    return result
