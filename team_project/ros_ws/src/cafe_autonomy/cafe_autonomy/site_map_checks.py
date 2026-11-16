"""Reject site destinations outside known free map space before launching motion."""
import math
from pathlib import Path
import cv2
import numpy as np
import yaml


def validate_site_map(config):
    path=Path(config['map_yaml']);meta=yaml.safe_load(path.read_text())
    pixels=cv2.imread(str(path.parent/meta['image']),cv2.IMREAD_GRAYSCALE)
    if pixels is None:raise ValueError('Cannot decode map image')
    resolution=float(meta['resolution']);ox,oy,yaw=map(float,meta['origin'])
    if not math.isfinite(resolution) or resolution<=0 or not all(math.isfinite(v) for v in (ox,oy,yaw)):raise ValueError('Invalid map geometry')
    threshold=float(meta.get('free_thresh',.25))
    if not 0.<threshold<1.:raise ValueError('Invalid free map threshold')
    if meta.get('negate',0) not in (0,1):raise ValueError('Invalid map negate')
    mode=meta.get('mode','trinary')
    if mode=='raw':free=pixels<=25
    elif mode in ('trinary','scale'):
        occupancy=pixels.astype(float)/255. if meta.get('negate',0) else 1.-pixels.astype(float)/255.
        free=occupancy<float(meta.get('free_thresh',.25))
    else:raise ValueError('Unsupported map mode: '+mode)
    # Treat unknown and the image boundary as occupied, including cell extent.
    field=cv2.distanceTransform(np.pad(np.flipud(free).astype(np.uint8),1),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)*resolution-resolution/math.sqrt(2.)
    radius=float(config.get('map_body_radius_m',.33))
    if not math.isfinite(radius) or radius<=0:raise ValueError('Invalid robot body radius')
    def safe(pose):
        x=float(pose['x'])-ox;y=float(pose['y'])-oy
        col=math.floor((math.cos(yaw)*x+math.sin(yaw)*y)/resolution)
        row=math.floor((-math.sin(yaw)*x+math.cos(yaw)*y)/resolution)
        return 0<=row<free.shape[0] and 0<=col<free.shape[1] and field[row+1,col+1]>=radius
    points=yaml.safe_load(Path(config['waypoints_file']).read_text())['destinations'];bad=[]
    for name,point in points.items():
        choices=point.get('approaches',{'pose':point}).values()
        if not any(safe(p) for p in choices):bad.append(name)
    for robot,pose in config['robots'].items():
        if not safe({'x':pose['map_x'],'y':pose['map_y']}):bad.append(robot+' initial pose')
    if bad:raise ValueError('Map lacks known body clearance at: '+', '.join(bad))
    return True
