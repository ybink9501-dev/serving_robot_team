"""The active map must allow localization updates at every operational goal."""
from pathlib import Path
import numpy as np
from PIL import Image
import yaml
from nav_msgs.msg import OccupancyGrid
from cafe_autonomy.map_odom_guard import known_fraction


def test_active_map_covers_all_operational_destinations():
    root = Path(__file__).resolve().parents[2]
    system = yaml.safe_load((root/'config/integration/system.yaml').read_text())
    path = root/system['map_yaml']
    config = yaml.safe_load(path.read_text())
    pixels = np.flipud(np.array(Image.open(path.parent/config['image']).convert('L')))
    grid = OccupancyGrid()
    grid.info.width = pixels.shape[1]
    grid.info.height = pixels.shape[0]
    grid.info.resolution = config['resolution']
    grid.info.origin.position.x, grid.info.origin.position.y = config['origin'][:2]
    grid.info.origin.orientation.w = 1.
    values = np.full(pixels.shape,-1,dtype=np.int8)
    values[pixels > 255*(1-config['free_thresh'])] = 0
    values[pixels < 255*(1-config['occupied_thresh'])] = 100
    grid.data = values.ravel().tolist()
    destinations = yaml.safe_load((root/system['waypoints_file']).read_text())['destinations']
    guard = system['localization_guard']
    for name, goal in destinations.items():
        poses = goal.get('approaches',{}).values() if 'approaches' in goal else [goal]
        for pose in poses:
            assert known_fraction(grid,pose['x'],pose['y'],guard['known_radius_m']) >= guard['min_known_fraction'], name
