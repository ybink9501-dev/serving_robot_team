"""Send explicit GO_HOME missions and wait for both robots' verified arrival."""
import argparse
import json
from pathlib import Path
import time
import urllib.request
import uuid

parser = argparse.ArgumentParser()
parser.add_argument('--timeout', type=float, default=240)
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
out = root / '_local_only'
out.mkdir(exist_ok=True)


def api(path, body=None):
    request = urllib.request.Request(
        'http://127.0.0.1:8765/api/' + path,
        data=None if body is None else json.dumps(body).encode(),
        headers={'Content-Type': 'application/json'},
    )
    with urllib.request.urlopen(request, timeout=5) as response:
        reply = json.load(response)
    if body is not None and not reply.get('ok'):
        raise RuntimeError(reply)
    return reply


ids = {r: 'home-' + uuid.uuid4().hex[:12] for r in ('robot_01', 'robot_02')}
log = out / ('home_trial_' + time.strftime('%Y%m%d_%H%M%S') + '.jsonl')
previous = {}
terminal = {}
start = time.monotonic()
with log.open('w') as stream:
    for robot, rid in ids.items():
        print(api('request', {'action': 'SUBMIT', 'robot_id': robot,
                              'request_id': rid, 'mode': 'GO_HOME'}), flush=True)
    while time.monotonic() - start < args.timeout:
        state = api('status')
        stream.write(json.dumps({'elapsed_s': time.monotonic() - start, 'fleet': state}) + '\n')
        stream.flush()
        for robot, mission in state.get('supervisors', {}).items():
            if mission.get('request_id') != ids[robot]:
                continue
            phase = mission.get('state')
            if previous.get(robot) != phase:
                print(robot, phase, mission.get('reason'), flush=True)
                previous[robot] = phase
            if phase in ('COMPLETED', 'ERROR', 'CANCELED'):
                terminal[robot] = mission
        if len(terminal) == 2:
            break
        time.sleep(.5)
success = len(terminal) == 2 and all(
    mission['state'] == 'COMPLETED' and mission.get('confirmation_pose_valid')
    for mission in terminal.values())
report = {'success': success, 'elapsed_s': time.monotonic() - start,
          'request_ids': ids, 'results': terminal, 'log': str(log)}
log.with_suffix('.summary.json').write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2), flush=True)
raise SystemExit(0 if success else 1)
