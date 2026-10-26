#!/usr/bin/env python3
"""Create a REVIEW-ONLY occupancy map and waypoint candidates from a Gazebo SDF world.

Only inline primitive collision geometry is supported. Gazebo visuals are not a map.
Does NOT modify a running map_server, Nav2, or destination_server.
"""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

EXCLUDE = ("robot", "person", "pedestrian", "human", "actor", "sun", "ground_plane")
SAMPLE_DIRECTIONS = [(1, 0), (-1, 0), (0, 1), (0, -1)]


def numbers(raw, default=()):
    if raw is None or not raw.strip():
        return tuple(default)
    return tuple(float(t) for t in raw.split())


def planar_pose(element, warnings, context):
    pose_el = element.find('pose')
    if pose_el is not None and pose_el.get('relative_to'):
        warnings.append(f'{context}: relative_to={pose_el.get("relative_to")} unsupported')
    values = numbers(pose_el.text if pose_el is not None else None,
                     (0, 0, 0, 0, 0, 0))
    if len(values) not in (3, 6):
        warnings.append(f'{context}: unsupported pose length {len(values)}')
        return (0, 0, 0, 0)
    if len(values) == 3:
        return (values[0], values[1], 0.0, values[2])
    if abs(values[3]) > 1e-5 or abs(values[4]) > 1e-5:
        warnings.append(f'{context}: nonzero roll/pitch; projection approximate')
    return values[0], values[1], values[2], values[5]


def combine(parent, local):
    px, py, pz, pt = parent
    lx, ly, lz, lt = local
    c, s = math.cos(pt), math.sin(pt)
    return (px + c * lx - s * ly, py + s * lx + c * ly,
            pz + lz, pt + lt)


def rect_contains(x, y, shape, extra=0.0):
    dx, dy = x - shape['x'], y - shape['y']
    c, s = math.cos(shape['yaw']), math.sin(shape['yaw'])
    ux, uy = c * dx + s * dy, -s * dx + c * dy
    return abs(ux) <= shape['sx'] / 2 + extra and abs(uy) <= shape['sy'] / 2 + extra


def hit(x, y, shape, extra=0.0):
    if shape['kind'] == 'box':
        return rect_contains(x, y, shape, extra)
    return math.hypot(x - shape['x'], y - shape['y']) <= shape['radius'] + extra


def shape_bounds(shape):
    if shape['kind'] == 'circle':
        r = shape['radius']
        return (shape['x'] - r, shape['y'] - r, shape['x'] + r, shape['y'] + r)
    c, s = abs(math.cos(shape['yaw'])), abs(math.sin(shape['yaw']))
    rx = (c * shape['sx'] + s * shape['sy']) / 2
    ry = (s * shape['sx'] + c * shape['sy']) / 2
    return shape['x'] - rx, shape['y'] - ry, shape['x'] + rx, shape['y'] + ry


def parse_world(path):
    doc = ET.parse(path)
    world = doc.find('.//world')
    if world is None:
        raise ValueError('SDF file contains no <world> element')
    obstacles, models, warnings, skipped = [], [], [], []

    def visit_model(model, parent=(0.0, 0.0, 0.0, 0.0), prefix=''):
        name = model.get('name', '<unnamed>')
        full = prefix + name
        if any(part in name.lower() for part in EXCLUDE):
            skipped.append(f'{full}: excluded dynamic/world object')
            return
        pose = combine(parent, planar_pose(model, warnings, full))
        local_shapes = []
        for link in model.findall('link'):
            linkname = full + '/' + link.get('name', 'link')
            lp = combine(pose, planar_pose(link, warnings, linkname))
            for collision in link.findall('collision'):
                cname = linkname + '/' + collision.get('name', 'collision')
                cp = combine(lp, planar_pose(collision, warnings, cname))
                geom = collision.find('geometry')
                if geom is None:
                    warnings.append(f'{cname}: no geometry')
                    continue
                box = geom.find('box')
                cylinder = geom.find('cylinder')
                sphere = geom.find('sphere')
                plane = geom.find('plane')
                if plane is not None:
                    skipped.append(f'{cname}: floor plane')
                    continue
                shape = None
                if box is not None:
                    sz = numbers(box.findtext('size'))
                    if len(sz) != 3:
                        warnings.append(f'{cname}: malformed box size')
                        continue
                    sx, sy, height = sz
                    if sx <= 0 or sy <= 0 or height <= 0:
                        continue
                    zlow, zhigh = cp[2] - height / 2, cp[2] + height / 2
                    # Floors are traversable. Elevated fixtures may still be obstacles.
                    if zhigh <= 0.08 and sx > 1.5 and sy > 1.5:
                        skipped.append(f'{cname}: broad floor tile')
                        continue
                    if zlow >= 1.3:
                        skipped.append(f'{cname}: above robot height 1.3m')
                        continue
                    shape = dict(kind='box', x=cp[0], y=cp[1], yaw=cp[3],
                                 sx=sx, sy=sy, source=cname)
                elif cylinder is not None or sphere is not None:
                    obj = cylinder if cylinder is not None else sphere
                    radius = float(obj.findtext('radius', '0'))
                    height = (float(obj.findtext('length', '0')) if cylinder is not None
                              else radius * 2)
                    if radius <= 0:
                        continue
                    if cp[2] - height / 2 >= 1.3:
                        skipped.append(f'{cname}: above robot height 1.3m')
                        continue
                    shape = dict(kind='circle', x=cp[0], y=cp[1], yaw=0,
                                 radius=radius, source=cname)
                else:
                    warnings.append(f'{cname}: unsupported collision geometry; mesh/heightmap/etc')
                if shape:
                    obstacles.append(shape)
                    local_shapes.append(shape)
        models.append(dict(name=full, x=pose[0], y=pose[1], yaw=pose[3],
                           collisions=len(local_shapes)))
        for nested in model.findall('model'):
            visit_model(nested, pose, full + '/')

    for incl in world.findall('include'):
        name = incl.findtext('name') or incl.findtext('uri', '?')
        if any(word in name.lower() for word in EXCLUDE):
            skipped.append(f'include {name}: ignored world/dynamic asset')
        else:
            warnings.append(f'include {name}: external model collision unresolved')
    for model in world.findall('model'):
        visit_model(model)
    for actor in world.findall('actor'):
        skipped.append(f'actor {actor.get("name", "?")}: dynamic, omitted from static map')
    return obstacles, models, warnings, skipped


def make_map(obstacles, models, resolution=0.05, padding=1.2):
    if not obstacles:
        raise ValueError('No primitive collision geometry found. A usable map cannot be made.')
    bounds = [shape_bounds(s) for s in obstacles]
    # Include furniture centers even if their collision geometry was unavailable.
    xs = [b[0] for b in bounds] + [b[2] for b in bounds] + [m['x'] for m in models]
    ys = [b[1] for b in bounds] + [b[3] for b in bounds] + [m['y'] for m in models]
    xmin = math.floor((min(xs) - padding) / resolution) * resolution
    ymin = math.floor((min(ys) - padding) / resolution) * resolution
    xmax = math.ceil((max(xs) + padding) / resolution) * resolution
    ymax = math.ceil((max(ys) + padding) / resolution) * resolution
    width = int(round((xmax-xmin)/resolution))
    height = int(round((ymax-ymin)/resolution))
    if width < 8 or height < 8 or width*height > 12_000_000:
        raise ValueError(f'Unreasonable map size {width}x{height}; inspect SDF collision dimensions')
    img = bytearray([254]) * (width * height)
    margin = 0.5 * resolution * math.sqrt(2)
    for shape in obstacles:
        bx0, by0, bx1, by1 = shape_bounds(shape)
        ix0 = max(0, int(math.floor((bx0-margin-xmin)/resolution)))
        ix1 = min(width-1, int(math.ceil((bx1+margin-xmin)/resolution)))
        iy0 = max(0, int(math.floor((by0-margin-ymin)/resolution)))
        iy1 = min(height-1, int(math.ceil((by1+margin-ymin)/resolution)))
        for iy in range(iy0, iy1+1):
            y = ymin + (iy+0.5)*resolution
            for ix in range(ix0, ix1+1):
                x = xmin + (ix+0.5)*resolution
                if hit(x, y, shape, extra=margin):
                    img[(height-1-iy)*width+ix] = 0
    # Keep robots out of the unmapped area: virtual one-cell map boundary.
    img[:width] = b'\x00' * width
    img[-width:] = b'\x00' * width
    for iy in range(height):
        img[iy * width] = 0
        img[iy * width + width-1] = 0
    return img, width, height, xmin, ymin


def candidate_points(models, obstacles, extent):
    xmin, ymin, xmax, ymax = extent
    result = {}
    for m in models:
        label = m['name'].split('/')[-1]
        if label == 'cafe_counter':
            key = 'kitchen'
        elif label.startswith('table_set_'):
            key = 'table_' + label[len('table_set_'):]
        else:
            continue
        choices = []
        for dx, dy in SAMPLE_DIRECTIONS:
            for distance in (0.75, 0.95, 1.15):
                x, y = m['x']+dx*distance, m['y']+dy*distance
                clearance = 0.35  # tentative robot footprint circumscribed radius
                if not (xmin+clearance < x < xmax-clearance and
                        ymin+clearance < y < ymax-clearance):
                    continue
                if any(hit(x, y, s, extra=clearance) for s in obstacles):
                    continue
                heading = math.atan2(m['y']-y, m['x']-x)
                choices.append({'x': round(x, 3), 'y': round(y, 3),
                                'yaw': round(heading, 5),
                                'yaw_deg': round(math.degrees(heading), 1),
                                'distance_from_center_m': distance,
                                'verified': False})
        result[key] = {'model': m['name'], 'center': [m['x'], m['y']],
                       'candidate_poses': choices,
                       'note': 'Unverified clearance / map alignment / Nav2 path; choose manually'}
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', type=Path, default=Path.home() / 'team_serving_robot/ros_ws/src/serving_robot_gazebo/worlds/cafe.world.sdf')
    parser.add_argument('--output-dir', type=Path, default=Path.home() / 'team_robot_lab/lab_maps/generated')
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--padding', type=float, default=1.2)
    parser.add_argument('--replace', action='store_true', help='replace only generated files, never running map')
    args = parser.parse_args()
    if not args.world.is_file():
        parser.error(f'SDF not found: {args.world}')
    if not (0.02 <= args.resolution <= 0.2):
        parser.error('resolution must be within 0.02-0.2 m')
    obstacles, models, warnings, skipped = parse_world(args.world)
    print(f'Collision objects: {len(obstacles)}, named models: {len(models)}')
    for m in models:
        if 'table' in m['name'] or 'counter' in m['name']:
            print(f'  {m["name"]}: ({m["x"]:.3f}, {m["y"]:.3f}), collisions={m["collisions"]}')
    for w in warnings[:20]:
        print('WARNING:', w)
    img, width, height, xmin, ymin = make_map(obstacles, models, args.resolution, args.padding)
    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)
    mapfile = out/'cafe_full_provisional.pgm'
    yamlfile = out/'cafe_full_provisional.yaml'
    reportfile = out/'cafe_layout_report.json'
    candidatesfile = out/'cafe_waypoint_candidates.json'
    outputs = [mapfile,yamlfile,reportfile,candidatesfile]
    existing = [str(p) for p in outputs if p.exists()]
    if existing and not args.replace:
        raise SystemExit('Output files already exist; stopped without overwrite: ' + ', '.join(existing))
    mapfile.write_bytes(f'P5\n{width} {height}\n255\n'.encode()+img)
    yamlfile.write_text(f'image: {mapfile.name}\nmode: trinary\nresolution: {args.resolution}\norigin: [{xmin:.6f}, {ymin:.6f}, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n', encoding='utf-8')
    extent = (xmin,ymin,xmin+width*args.resolution,ymin+height*args.resolution)
    report = {'status':'PROVISIONAL_NOT_VERIFIED', 'world':str(args.world),
              'resolution':args.resolution,'size_pixels':[width,height],
              'extent':list(extent),'occupied_pixels':img.count(0),
              'collision_count':len(obstacles),'models':models,
              'warnings':warnings,'skipped':skipped,
              'warning':'SDF approximation only; do NOT auto-load into Nav2 without visual/TF/costmap verification'}
    reportfile.write_text(json.dumps(report,ensure_ascii=False,indent=2), encoding='utf-8')
    candidatesfile.write_text(json.dumps(candidate_points(models,obstacles,extent),ensure_ascii=False,indent=2),encoding='utf-8')
    print('\nGENERATED (not loaded in ROS):')
    for p in outputs: print(p)
    print(f'Coverage X {extent[0]:.3f}..{extent[2]:.3f}, Y {extent[1]:.3f}..{extent[3]:.3f}')
    print('WARNING: Candidate waypoints are suggestions only, NOT activated.')
    if warnings:
        print(f'WARNING: {len(warnings)} unsupported/unresolved shapes. Map MAY omit obstacles.')


if __name__ == '__main__':
    main()
