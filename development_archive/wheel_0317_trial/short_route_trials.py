"""Local-only fresh-spawn repeated table_02 acceptance."""
import csv
import json
import math
import subprocess
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
URL = "http://127.0.0.1:8765/api/"
RESULT = ROOT / "_local_only/wheel_0317_trial/short_route_results.jsonl"
if RESULT.exists():
    raise SystemExit("Refusing to overwrite prior reliability results")
if (ROOT / "_local_only/runtime.json").exists():
    raise SystemExit("A managed simulation is already running")


def get_status():
    with urllib.request.urlopen(URL + "status", timeout=4) as response:
        return json.load(response)


def request(payload):
    req = urllib.request.Request(
        URL + "request",
        json.dumps(payload).encode(),
        {"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(req, timeout=5) as response:
        return json.load(response)


def score(robot, launch_time, table):
    paths = [
        p for p in (ROOT / "_local_only/metrics").glob("measure_*.csv")
        if p.stat().st_mtime >= launch_time
    ]
    if not paths:
        return {"error": "No metrics file"}
    path = max(paths, key=lambda p: p.stat().st_mtime)
    rows = [
        r for r in csv.DictReader(path.open())
        if r["robot"] == robot and r["gt_x"] and r["gt_y"]
    ]
    at_goal = next(
        (r for r in rows if r["mission_state"] == "WAIT_HANDOVER"), None
    )
    if at_goal is None:
        return {"metrics": str(path), "error": "No WAIT_HANDOVER evidence"}
    x, y = float(at_goal["gt_x"]), float(at_goal["gt_y"])
    target = {"table_01": (1.6, 1.95), "table_04": (4.75, -0.15)}[table]
    return {
        "metrics": str(path),
        "actual_error_cm": 100.0 * math.hypot(x - target[0], y - target[1]),
        "tf_gt_cm": (
            100.0 * float(at_goal["xy_error_tf_gt"])
            if at_goal["xy_error_tf_gt"] else None
        ),
        "odom_gt_cm": (
            100.0 * float(at_goal["xy_error_odom_gt"])
            if at_goal["xy_error_odom_gt"] else None
        ),
    }


with RESULT.open("w") as output:
    for index, (robot, table) in enumerate(
        (("robot_01", "table_01"), ("robot_02", "table_04")), 1
    ):
        launch_time = time.time()
        launcher = subprocess.Popen(
            ["bash", "scripts/start_operation.sh", "--headless", "--robot", robot],
            cwd=ROOT,
        )
        trial = {"index": index, "robot": robot, "table": table}
        try:
            ready = False
            for _ in range(120):
                if launcher.poll() is not None:
                    break
                try:
                    status = get_status()
                    if status.get("supervisors", {}).get(robot, {}).get("nav_ready"):
                        ready = True
                        break
                except Exception:
                    pass
                time.sleep(0.5)
            if not ready:
                trial["error"] = "Nav2 did not become ready"
            else:
                rid = f"short08-{robot}-{index}-{int(time.time())}"
                trial["request_id"] = rid
                trial["submitted"] = request({
                    "action": "SUBMIT",
                    "request_id": rid,
                    "robot_id": robot,
                    "table": table,
                    "mode": "DELIVER_DIRECT",
                }).get("ok", False)
                confirmed = set()
                end = time.monotonic() + 180
                while time.monotonic() < end:
                    current = get_status()["supervisors"][robot]
                    state = current.get("state")
                    if current.get("request_id") == rid:
                        if (
                            state in ("WAIT_LOAD", "WAIT_HANDOVER")
                            and state not in confirmed
                        ):
                            action = "LOAD" if state == "WAIT_LOAD" else "COMPLETE"
                            reply = request({
                                "action": action,
                                "request_id": rid,
                                "source": "kitchen" if action == "LOAD" else "table",
                            })
                            if reply.get("ok"):
                                confirmed.add(state)
                        if state in ("COMPLETED", "ERROR", "CANCELED"):
                            trial["final_state"] = state
                            trial["pose_error"] = current.get("pose_error")
                            trial["reason"] = current.get("reason")
                            break
                    time.sleep(0.5)
                trial.update(score(robot, launch_time, table))
                trial["precision_pass"] = (
                    trial.get("final_state") == "COMPLETED"
                    and trial.get("actual_error_cm", float("inf")) < 10.0
                )
        except Exception as exc:
            trial["error"] = repr(exc)
        finally:
            if (ROOT / "_local_only/runtime.json").exists():
                try:
                    subprocess.run(
                        ["bash", "scripts/stop.sh"],
                        cwd=ROOT,
                        timeout=45,
                        check=False,
                    )
                except Exception as exc:
                    trial["shutdown_error"] = repr(exc)
            try:
                launcher.wait(timeout=10)
            except subprocess.TimeoutExpired:
                trial["launcher_lingering"] = True
        output.write(json.dumps(trial) + "\n")
        output.flush()
        print(json.dumps(trial), flush=True)
