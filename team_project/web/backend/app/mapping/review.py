"""Review a one-robot SLAM draft and produce a separate Nav2 map.

Ported from the mapping team's delivery (review/backend/map_review.py). Changes: the draft
folder and the output folder are passed in (no environment lookups, no /mnt guess), and
finalize() also reports where the approved map was written so the backend can adopt it.

The original map and running ROS nodes are never modified.
User approval is distinct from operational Navigation authorization.
"""
import hashlib
import io
import json
import math
import os
import tempfile
from pathlib import Path

import yaml
from PIL import Image


class ReviewError(Exception):
    def __init__(self, message, status=422):
        super().__init__(message)
        self.status = status


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def occupied(value):
    return 65 <= value <= 100


def unknown(value):
    return value > 100 or 25 < value < 65


def points(x1, y1, x2, y2):
    """Bresenham rasterization in top-left image pixel coordinates."""
    dx = abs(x2 - x1)
    dy = -abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    error = dx + dy

    while True:
        yield x1, y1
        if (x1, y1) == (x2, y2):
            break
        twice = 2 * error
        if twice >= dy:
            error += dy
            x1 += sx
        if twice <= dx:
            error += dx
            y1 += sy


def verify_navigation_map(yaml_path):
    """Apply the mapping completion rules to the edited Nav2 artifact.

    Nav2 trinary pixels are decoded to the same occupancy values consumed by
    mapping_completion_verifier.verify_closed_free_region: free=0,
    occupied=100, unknown=-1. Keep the limits and four-neighbour rule aligned.
    """
    path = Path(yaml_path)
    try:
        meta = yaml.safe_load(path.read_text(encoding="utf-8"))
        if not isinstance(meta, dict) or meta.get("mode") != "trinary":
            raise ValueError("Expected a Nav2 trinary map")
        resolution = float(meta["resolution"])
        with Image.open(path.parent / meta["image"]) as image:
            if image.format != "PPM" or image.mode != "L":
                raise ValueError("Expected an 8-bit grayscale PGM")
            width, height = image.size
            pixels = image.tobytes()
        if len(pixels) != width * height:
            raise ValueError("Invalid map image size")
        occupancy = tuple(0 if value == 254 else 100 if value == 0 else -1
                          for value in pixels)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        return {"verified": False, "reason": "SAVED_MAP_INVALID", "error": str(exc)}

    if (width < 3 or height < 3 or not math.isfinite(resolution)
            or resolution <= 0):
        return {"verified": False, "reason": "INVALID_MAP_GEOMETRY"}

    free_count = 0
    for y in range(height):
        for x in range(width):
            index = y * width + x
            if occupancy[index] > 25:
                continue
            free_count += 1
            if x == 0 or y == 0 or x == width - 1 or y == height - 1:
                return {"verified": False, "reason": "FREE_SPACE_REACHES_MAP_EDGE",
                        "width": width, "height": height, "resolution": resolution}
            if any(occupancy[n] == -1 for n in
                   (index - 1, index + 1, index - width, index + width)):
                return {"verified": False, "reason": "FREE_SPACE_TOUCHES_UNKNOWN",
                        "width": width, "height": height, "resolution": resolution}

    if free_count * resolution * resolution < 10.0:
        return {"verified": False, "reason": "INSUFFICIENT_KNOWN_FREE_AREA",
                "width": width, "height": height, "resolution": resolution}
    return {"verified": True, "reason": "CLOSED_FREE_REGION_VERIFIED",
            "width": width, "height": height, "resolution": resolution}


class MapReviewService:
    def __init__(self, draft_dir, output_root, store_id):
        self.store_id = store_id
        self.input = Path(draft_dir)
        self.output_root = Path(output_root)
        self.pgm = self.input / "R16-147_official_raw_map.pgm"
        self.yaml_path = self.input / "R16-147_official_raw_map.yaml"
        self.manifest_path = self.input / "R16-176_map_draft_manifest.json"
        self.session_path = self.input / "R16-147_session_result.json"

        required = (
            self.pgm, self.yaml_path,
            self.manifest_path, self.session_path,
        )
        if not all(p.is_file() for p in required):
            raise ReviewError(
                "자동 매핑 결과가 아직 연결되지 않았습니다.", 503
            )

        self.meta = yaml.safe_load(
            self.yaml_path.read_text(encoding="utf-8")
        )
        if not isinstance(self.meta, dict):
            raise ReviewError("지도 설정을 읽지 못했습니다.")

        if self.meta.get("mode") != "raw":
            raise ReviewError("예상한 자동 매핑 지도 형식이 아닙니다.")

        if self.meta.get("image") != self.pgm.name:
            raise ReviewError("지도 이미지 참조가 일치하지 않습니다.")

        origin = self.meta.get("origin")
        if not isinstance(origin, list) or len(origin) < 3:
            raise ReviewError("지도 원점 정보가 잘못되었습니다.")

        if abs(float(origin[2])) > 1e-9:
            raise ReviewError(
                "현재 경로 검증기는 회전된 지도 원점을 지원하지 않습니다."
            )

        manifest = json.loads(
            self.manifest_path.read_text(encoding="utf-8")
        )
        known = {
            Path(item["path"]).name: item["sha256"]
            for item in manifest["original_map_files"]
        }

        self.source_hash = sha(self.pgm)
        if (
            known.get(self.pgm.name) != self.source_hash
            or known.get(self.yaml_path.name) != sha(self.yaml_path)
        ):
            raise ReviewError("원본 지도의 무결성 검증에 실패했습니다.")

        session = json.loads(
            self.session_path.read_text(encoding="utf-8")
        )
        self.session_status = session
        if session.get("map_draft_ready") is not True:
            raise ReviewError("검토 가능한 지도 초안이 아닙니다.")

        if session.get("navigation_map_authorized") is not False:
            raise ReviewError("예상하지 못한 지도 승인 상태입니다.")

        with Image.open(self.pgm) as image:
            if image.format != "PPM" or image.mode != "L":
                raise ReviewError("지원하지 않는 지도 이미지입니다.")
            self.width, self.height = image.size
            self.raw = image.tobytes()

        if [self.width, self.height] != session["map_dimensions"]:
            raise ReviewError("지도 크기가 매핑 기록과 다릅니다.")

        if len(self.raw) != self.width * self.height:
            raise ReviewError("지도 데이터 크기가 잘못되었습니다.")

        free = sum(v <= 25 for v in self.raw)
        wall = sum(occupied(v) for v in self.raw)
        missing = sum(v > 100 for v in self.raw)

        if (
            free != session["free_cells"]
            or wall != session["occupied_cells"]
            or missing != session["unknown_cells"]
        ):
            raise ReviewError("지도 내용이 매핑 기록과 다릅니다.")

    def at(self, x, y):
        return self.raw[y * self.width + x]

    def suggestions(self):
        """Conservative unknown gaps between measured wall anchors."""
        found = []
        w, h = self.width, self.height

        for gap in range(1, 5):
            for vertical in (False, True):
                for y in range(2, h - 2):
                    for x in range(2, w - 2):
                        dx, dy = (0, 1) if vertical else (1, 0)
                        x2, y2 = x + dx * (gap + 1), y + dy * (gap + 1)

                        if not (0 <= x2 + dx < w and 0 <= y2 + dy < h):
                            continue
                        if not (occupied(self.at(x, y)) and
                                occupied(self.at(x2, y2))):
                            continue
                        if not (
                            occupied(self.at(x - dx, y - dy))
                            and occupied(self.at(x2 + dx, y2 + dy))
                        ):
                            continue
                        if not all(
                            unknown(self.at(x + dx * k, y + dy * k))
                            for k in range(1, gap + 1)
                        ):
                            continue

                        found.append({
                            "id": "wall-gap-%03d" % (len(found) + 1),
                            "x1": x, "y1": y,
                            "x2": x2, "y2": y2,
                            "gap_m": round(
                                gap * float(self.meta["resolution"]), 3
                            ),
                        })
                        if len(found) >= 20:
                            return found
        return found

    def describe(self):
        ox, oy, yaw = self.meta["origin"][:3]
        return {
            "width": self.width,
            "height": self.height,
            "resolution": float(self.meta["resolution"]),
            "origin": {
                "x": float(ox),
                "y": float(oy),
                "yaw": float(yaw),
            },
            "source_hash": self.source_hash,
            "review_available": self.session_status.get("review_available", True) is True,
            "auto_map_verified": self.session_status.get("auto_map_verified") is True,
            "map_complete": self.session_status.get("map_complete") is True,
            "explorer_terminal_status": self.session_status.get("explorer_terminal_status"),
            "source_strict_check": self.session_status.get("source_strict_check"),
            "image_url":
                "/api/stores/%s/map-review.png" % self.store_id,
            "suggestions": self.suggestions(),
        }

    def preview_png(self):
        colors = bytearray()
        for value in self.raw:
            if value <= 25:
                colors.extend((240, 243, 247))
            elif occupied(value):
                colors.extend((29, 39, 53))
            else:
                colors.extend((150, 164, 184))

        image = Image.frombytes(
            "RGB", (self.width, self.height), bytes(colors)
        )
        stream = io.BytesIO()
        image.save(stream, format="PNG")
        return stream.getvalue()

    def finalize(self, body):
        if not isinstance(body, dict):
            raise ReviewError("수정 요청 형식이 잘못되었습니다.")

        if body.get("source_hash") != self.source_hash:
            raise ReviewError(
                "지도가 변경되었습니다. 최신 지도를 다시 확인해 주세요.", 409
            )

        if body.get("confirmed") is not True:
            raise ReviewError("지도 수정 내용의 최종 확인이 필요합니다.")

        strokes = body.get("strokes")
        if not isinstance(strokes, list) or len(strokes) > 100:
            raise ReviewError("한 번에 수정할 수 있는 선의 수를 초과했습니다.")

        changed = bytearray(self.raw)
        resolution = float(self.meta["resolution"])

        total_curve_points = 0
        for stroke in strokes:
            if not isinstance(stroke, dict):
                raise ReviewError("수정 내용 형식이 잘못되었습니다.")

            keys = ("x1", "y1", "x2", "y2", "value")
            try:
                vals = [stroke[k] for k in keys]
            except KeyError:
                raise ReviewError("수정 좌표가 누락되었습니다.")

            if any(type(v) is not int for v in vals):
                raise ReviewError("수정 좌표가 올바르지 않습니다.")

            x1, y1, x2, y2, value = vals

            if value not in (0, 100, 255):
                raise ReviewError("지원하지 않는 수정 도구입니다.")

            if not (
                0 <= x1 < self.width and
                0 <= x2 < self.width and
                0 <= y1 < self.height and
                0 <= y2 < self.height
            ):
                raise ReviewError("지도 밖을 수정할 수 없습니다.")

            polyline = stroke.get("points")

            if polyline is None:
                segments = [((x1, y1), (x2, y2))]
            else:
                if not isinstance(polyline, list) or not (
                    1 <= len(polyline) <= 3000
                ):
                    raise ReviewError("곡선 점 개수가 올바르지 않습니다.")

                total_curve_points += len(polyline)
                if total_curve_points > 10000:
                    raise ReviewError("전체 곡선 수정량이 너무 많습니다.")

                checked = []
                for point in polyline:
                    if not isinstance(point, dict):
                        raise ReviewError("곡선 점 형식이 잘못되었습니다.")
                    px, py = point.get("x"), point.get("y")
                    if type(px) is not int or type(py) is not int:
                        raise ReviewError("곡선 좌표는 정수여야 합니다.")
                    if not (
                        0 <= px < self.width
                        and 0 <= py < self.height
                    ):
                        raise ReviewError("지도 밖으로 곡선을 그릴 수 없습니다.")
                    checked.append((px, py))

                if checked[0] != (x1, y1) or checked[-1] != (x2, y2):
                    raise ReviewError("곡선 시작점과 끝점이 일치하지 않습니다.")

                segments = list(zip(checked, checked[1:]))
                if not segments:
                    segments = [(checked[0], checked[0])]

            for (sx, sy), (ex, ey) in segments:
                distance = math.hypot(ex - sx, ey - sy) * resolution
                if distance > 1.000001:
                    raise ReviewError("곡선의 점 간 거리가 1m를 초과합니다.")
                for x, y in points(sx, sy, ex, ey):
                    changed[y * self.width + x] = value

        # Nav2 trinary encoding: free=254, wall=0, unknown=205.
        # Unknown and ambiguous cells remain nontraversable.
        converted = bytes(
            254 if v <= 25 else 0 if occupied(v) else 205
            for v in changed
        )

        if 254 not in converted:
            raise ReviewError("이동 가능한 공간이 없는 지도입니다.")

        digest = hashlib.sha256(converted).hexdigest()[:16]
        output_root = self.output_root
        output_root.mkdir(parents=True, exist_ok=True)
        target = output_root / ("review_" + digest)

        if target.exists():
            try:
                existing = json.loads((target / "review_record.json").read_text(encoding="utf-8"))
            except (OSError, ValueError):
                existing = {}
            try:
                handoff = json.loads((target / "navigation_handoff.json").read_text(encoding="utf-8"))
            except (OSError, ValueError):
                handoff = None
            return {
                "status": "REVIEW_MAP_SAVED",
                "saved": True,
                "navigation_validated": existing.get("navigation_validated") is True,
                "navigation_validation": existing.get("navigation_validation"),
                "navigation_handoff": handoff,
                "navigation_applied": False,
                "output_dir": str(target),
            }

        pending = Path(
            tempfile.mkdtemp(prefix=".pending_", dir=output_root)
        )
        try:
            pgm_out = pending / "navigation_map.pgm"
            yaml_out = pending / "navigation_map.yaml"

            pgm_out.write_bytes(
                ("P5\n%d %d\n255\n" %
                 (self.width, self.height)).encode("ascii")
                + converted
            )

            nav_meta = {
                "image": "navigation_map.pgm",
                "mode": "trinary",
                "resolution": resolution,
                "origin": self.meta["origin"],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            }
            yaml_out.write_text(
                yaml.safe_dump(nav_meta, sort_keys=False),
                encoding="utf-8"
            )

            with Image.open(pgm_out) as check:
                if check.size != (self.width, self.height):
                    raise ReviewError("생성된 지도 크기가 일치하지 않습니다.")
                if check.tobytes() != converted:
                    raise ReviewError("생성된 지도를 다시 읽지 못했습니다.")

            navigation_validation = verify_navigation_map(yaml_out)
            manual_edits = [
                {
                    "edit_index": index,
                    "source": "operator_manual_edit",
                    "value": stroke.get("value"),
                    "points_top_left_pixels": stroke.get("points") or [
                        {"x": stroke.get("x1"), "y": stroke.get("y1")},
                        {"x": stroke.get("x2"), "y": stroke.get("y2")},
                    ],
                }
                for index, stroke in enumerate(strokes)
            ]
            handoff = {
                "format": "serving_robot_navigation_map_handoff_v1",
                "state": "MAP_CANDIDATE_READY_FOR_AUTONOMY_INTEGRATION",
                "map_frame": "map",
                "map_yaml": "navigation_map.yaml",
                "map_image": "navigation_map.pgm",
                "map_sha256": sha(pgm_out),
                "source_slam_map_sha256": self.source_hash,
                "source_slam_yaml_sha256": sha(self.yaml_path),
                "origin": list(self.meta["origin"][:3]),
                "resolution_m": float(resolution),
                "completion_validation": navigation_validation,
                "source_auto_map_verified": self.session_status.get("auto_map_verified") is True,
                "destination_profile": "PENDING_AUTONOMY_INTEGRATION",
                "nav2_route_validation": "PENDING_AUTONOMY_INTEGRATION",
                "operation_map_authorized": False,
            }
            audit = {
                "source_sha256": self.source_hash,
                "source_yaml_sha256": sha(self.yaml_path),
                "user_final_saved": True,
                "source_auto_map_verified": self.session_status.get("auto_map_verified") is True,
                "source_map_complete": self.session_status.get("map_complete") is True,
                "source_explorer_terminal_status": self.session_status.get("explorer_terminal_status"),
                "navigation_validated": navigation_validation["verified"],
                "navigation_validation": navigation_validation,
                "navigation_map_authorized": False,
                "edit_provenance": "operator_strokes_over_sensor_map",
                "provenance": {
                    "base_map": {
                        "source": "slam_saved_map_from_lidar_pipeline",
                        "source_pgm_sha256": self.source_hash,
                        "source_yaml_sha256": sha(self.yaml_path),
                    },
                    "manual_edits": manual_edits,
                },
                "unknown_is_nontraversable": True,
                "strokes": strokes,
                "map_sha256": sha(pgm_out),
            }
            (pending / "review_record.json").write_text(
                json.dumps(audit, ensure_ascii=False, indent=2),
                encoding="utf-8"
            )
            (pending / "navigation_handoff.json").write_text(
                json.dumps(handoff, ensure_ascii=False, indent=2),
                encoding="utf-8"
            )
            os.rename(pending, target)
        finally:
            if pending.exists():
                import shutil
                shutil.rmtree(pending)

        return {
            "status": "REVIEW_MAP_SAVED",
            "saved": True,
            "navigation_validated": navigation_validation["verified"],
            "navigation_validation": navigation_validation,
            "navigation_handoff": handoff,
            "navigation_applied": False,
            "output_dir": str(target),
        }
