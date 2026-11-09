#!/usr/bin/env python3
"""Read-only ROS telemetry and a local AMap dashboard. No ROS publishers/services."""

import argparse
import copy
import functools
import json
import math
import os
from pathlib import Path
import re
import signal
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit


DEFAULT_TOPICS = {
    "gnss": "/handsfree/rtk/gnss",
    "gcj02": "/amap_nav/gcj02_fix",
    "raw": "/handsfree/rtk/raw",
    "enu": "/amap_nav/enu_pose",
    "origin": "/amap_nav/enu_origin",
    "geo_status": "/amap_nav/geo_status",
    "path": "/viz_path_topic",
    "odom": "/lio/odom",
    "goal": "/goal_point",
    "reached": "/far_reach_goal_status",
}


def finite(value):
    try:
        number = float(value)
        return number if math.isfinite(number) else None
    except (TypeError, ValueError, OverflowError):
        return None


def parse_gga(raw):
    """Read the last well-formed GGA, validating the checksum when supplied."""
    latest = None
    for sentence in re.findall(r"\$[^\r\n$]+", raw):
        if "*" in sentence:
            body, checksum = sentence[1:].split("*", 1)
            if not re.fullmatch(r"[0-9A-Fa-f]{2}", checksum.strip()):
                continue
            actual = 0
            for char in body:
                actual ^= ord(char)
            if actual != int(checksum.strip(), 16):
                continue
        else:
            body = sentence[1:]
        fields = body.split(",")
        if len(fields) < 10 or not re.fullmatch(r"[A-Z0-9]{2}GGA", fields[0]):
            continue
        try:
            quality = int(fields[6])
            satellites = int(fields[7]) if fields[7] else None
        except ValueError:
            continue
        if not 0 <= quality <= 8 or (satellites is not None and satellites < 0):
            continue
        hdop = finite(fields[8])
        latest = {
            "gga_quality": quality,
            "satellites": satellites,
            "hdop": hdop if hdop is not None and hdop >= 0 else None,
        }
    return latest


def stamp_seconds(message):
    try:
        return finite(message.header.stamp.to_sec())
    except AttributeError:
        return None


def fix_record(message):
    lat, lon = finite(message.latitude), finite(message.longitude)
    if lat is None or lon is None or not -90 <= lat <= 90 or not -180 <= lon <= 180:
        return None
    covariance_type = int(message.position_covariance_type)
    accuracy = None
    if covariance_type > 0:
        east = finite(message.position_covariance[0])
        north = finite(message.position_covariance[4])
        if east is not None and north is not None and min(east, north) >= 0:
            accuracy = math.sqrt(max(east, north))
    return {
        "latitude": lat,
        "longitude": lon,
        "altitude": finite(message.altitude),
        "status": int(message.status.status),
        "covariance_type": covariance_type,
        # This is a driver estimate, not independently measured map accuracy.
        "accuracy_m": accuracy,
        "stamp": stamp_seconds(message),
        "received_at": time.time(),
    }


def point_record(point):
    result = {name: finite(getattr(point, name)) for name in ("x", "y", "z")}
    return result if all(value is not None for value in result.values()) else None


class Telemetry:
    def __init__(self, topics=None, max_track_points=3000, stale_after_sec=3.0):
        self.topics = dict(DEFAULT_TOPICS if topics is None else topics)
        self.stale_after_sec = stale_after_sec
        self.started_at = time.time()
        self.lock = threading.Lock()
        self.records = dict.fromkeys(("fix", "gcj02", "quality", "enu", "origin", "geo_status"))
        self.navigation = dict.fromkeys(("path", "odometry", "goal", "reached"))
        self.track = deque(maxlen=max_track_points)
        self.subscribers = []

    def on_fix(self, message, target="fix"):
        record = fix_record(message)
        with self.lock:
            # Invalid fixes immediately invalidate the current marker/planning origin.
            self.records[target] = record
            if target != "gcj02":
                return
            if record is None or record["status"] < 0:
                self.track.clear()
                return
            now = record["received_at"]
            if self.track and now - self.track[-1]["received_at"] > self.stale_after_sec:
                # Never draw a straight segment across a telemetry outage.
                self.track.clear()
            if not self.track or now - self.track[-1]["received_at"] >= 0.2:
                self.track.append({key: record[key] for key in ("longitude", "latitude", "received_at")})

    def on_raw(self, message):
        quality = parse_gga(message.data)
        if quality is not None:
            quality["received_at"] = time.time()
            with self.lock:
                self.records["quality"] = quality

    def on_enu(self, message):
        point = point_record(message.pose.position)
        record = None if point is None else {
            "east": point["x"], "north": point["y"], "up": point["z"],
            "frame_id": message.header.frame_id,
            "received_at": time.time(),
        }
        with self.lock:
            self.records["enu"] = record

    def on_origin(self, message):
        record = fix_record(message)
        with self.lock:
            self.records["origin"] = record
            # Coordinates recorded before a change of ENU origin are no longer valid.
            self.records["enu"] = None

    def on_geo_status(self, message):
        with self.lock:
            self.records["geo_status"] = {"text": message.data[:2000], "received_at": time.time()}

    def on_path(self, message):
        with self.lock:
            if message.action == 3:  # visualization_msgs/Marker.DELETEALL
                self.navigation["path"] = None
                return
            if message.ns != "global_path":
                return
            if message.action == 2:
                self.navigation["path"] = None
                return
            if message.action != 0 or message.type != 4:  # ADD + LINE_STRIP
                return
            points = [point_record(point) for point in message.points]
            # Do not connect points across an invalid part of the path.
            if any(point is None for point in points):
                self.navigation["path"] = None
                return
            if len(points) > 10000:
                stride = math.ceil(len(points) / 9999)
                points = points[::stride] + [points[-1]]
            self.navigation["path"] = {
                "points": points, "frame_id": message.header.frame_id,
                "received_at": time.time(),
            }

    def on_odometry(self, message):
        self._navigation_point("odometry", message.pose.pose.position, message.header.frame_id)

    def on_goal(self, message):
        with self.lock:
            self.navigation["reached"] = None
        self._navigation_point("goal", message.point, message.header.frame_id)

    def _navigation_point(self, target, point, frame):
        record = point_record(point)
        if record is not None:
            record.update(frame_id=frame, received_at=time.time())
        with self.lock:
            self.navigation[target] = record

    def on_reached(self, message):
        with self.lock:
            self.navigation["reached"] = {"value": bool(message.data), "received_at": time.time()}

    def snapshot(self):
        with self.lock:
            return copy.deepcopy({
                "server_time": time.time(), "started_at": self.started_at,
                "topics": self.topics, "track": list(self.track),
                "navigation": self.navigation, **self.records,
            })

    def subscribe(self, rospy):
        from geometry_msgs.msg import PointStamped, PoseStamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import NavSatFix
        from std_msgs.msg import Bool, String
        from visualization_msgs.msg import Marker

        bindings = [
            ("gnss", NavSatFix, self.on_fix),
            ("gcj02", NavSatFix, functools.partial(self.on_fix, target="gcj02")),
            ("raw", String, self.on_raw), ("enu", PoseStamped, self.on_enu),
            ("origin", NavSatFix, self.on_origin), ("geo_status", String, self.on_geo_status),
            ("path", Marker, self.on_path), ("odom", Odometry, self.on_odometry),
            ("goal", PointStamped, self.on_goal), ("reached", Bool, self.on_reached),
        ]
        self.subscribers = [
            rospy.Subscriber(self.topics[key], message_type, callback, queue_size=10)
            for key, message_type, callback in bindings
        ]


class PanelHandler(BaseHTTPRequestHandler):
    server_version = "Go2WMap/1.0"
    sys_version = ""

    def do_GET(self):
        self._serve(False)

    def do_HEAD(self):
        self._serve(True)

    def _serve(self, head):
        route = urlsplit(self.path).path
        if route == "/api/state":
            self._json(self.server.telemetry.snapshot(), head)
        elif route == "/api/config":
            self._json(self.server.panel_config, head)
        else:
            assets = {"/": ("index.html", "text/html"), "/index.html": ("index.html", "text/html"),
                      "/app.js": ("app.js", "text/javascript"), "/style.css": ("style.css", "text/css")}
            if route not in assets:
                self.send_error(404)
                return
            filename, content_type = assets[route]
            try:
                body = (self.server.web_root / filename).read_bytes()
            except OSError:
                self.send_error(404, "Dashboard asset unavailable")
                return
            self._send(body, content_type, head)

    def _json(self, value, head):
        body = json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode("utf-8")
        self._send(body, "application/json", head)

    def _send(self, body, content_type, head):
        self.send_response(200)
        self.send_header("Content-Type", content_type + "; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        if not head:
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

    def log_message(self, format_string, *args):
        # No request URLs/queries (potential credentials) in terminal or ROS logs.
        pass


def web_root():
    source = Path(__file__).resolve().parents[1] / "web"
    if (source / "index.html").is_file():
        return source
    import rospkg
    return Path(rospkg.RosPack().get_path("amap_outdoor_indoor_nav")) / "web"


def load_credentials(path):
    """Load per-machine JS API settings; never include their contents in errors."""
    settings = {}
    source = Path(path).expanduser()
    if source.is_file():
        try:
            settings = json.loads(source.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            raise ValueError("无法读取高德本机配置，请检查 JSON 格式和文件权限") from None
        if not isinstance(settings, dict) or any(
                not isinstance(settings.get(name, ""), str)
                for name in ("amap_key", "amap_security_js_code")):
            raise ValueError("高德本机配置中的 Key 和安全密钥必须是字符串")
    return {
        "amap_key": os.environ.get("AMAP_JS_KEY") or settings.get("amap_key", ""),
        "amap_security_js_code": os.environ.get("AMAP_JS_SECURITY_CODE") or settings.get("amap_security_js_code", ""),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--standalone", action="store_true", help="Serve the UI without ROS (no simulated telemetry)")
    parser.add_argument("--host", default=None)
    parser.add_argument("--port", type=int, default=None)
    parser.add_argument("--credentials-file", default=None, help="Path to per-machine AMap JSON configuration")
    args, remainder = parser.parse_known_args(argv)
    if any(":=" not in value for value in remainder):
        parser.error("unrecognized arguments: " + " ".join(remainder))

    rospy = None
    if not args.standalone:
        try:
            import rospy
            rospy.init_node("amap_panel", disable_signals=True)
        except ImportError:
            parser.exit(1, "请先 source /opt/ros/noetic/setup.bash 和本工作区 devel/setup.bash；离线查看可加 --standalone。\n")

    def param(name, fallback):
        return rospy.get_param("~" + name, fallback) if rospy is not None else fallback

    host = args.host if args.host is not None else param("host", "127.0.0.1")
    port = args.port if args.port is not None else int(param("port", 8088))
    stale = float(param("stale_after_sec", 3.0))
    count = int(param("max_track_points", 3000))
    if not 0 <= port <= 65535 or not math.isfinite(stale) or stale <= 0 or not 1 <= count <= 20000:
        parser.error("invalid port, stale_after_sec or max_track_points")
    topics = {key: param(("gnss" if key == "gnss" else key) + "_topic", value)
              for key, value in DEFAULT_TOPICS.items()}
    telemetry = Telemetry(topics, count, stale)
    assets = web_root()
    if not (assets / "index.html").is_file():
        parser.exit(1, "未找到面板 web/index.html，请检查包安装。\n")
    try:
        server = ThreadingHTTPServer((host, port), PanelHandler)
    except OSError as exc:
        parser.exit(1, "面板监听失败：{}\n".format(exc))
    server.daemon_threads = True
    server.telemetry = telemetry
    server.web_root = assets
    default_credentials = Path(os.environ.get("XDG_CONFIG_HOME", str(Path.home() / ".config"))) / "go2w" / "amap_panel.json"
    credentials_path = args.credentials_file or param("credentials_file", str(default_credentials))
    try:
        server.panel_config = {**load_credentials(credentials_path),
                               "poll_interval_ms": 1000, "stale_after_sec": stale}
    except ValueError as exc:
        server.server_close()
        parser.exit(1, str(exc) + "\n")
    try:
        if rospy is not None:
            telemetry.subscribe(rospy)

        def shutdown(signum, frame):
            raise KeyboardInterrupt

        signal.signal(signal.SIGINT, shutdown)
        signal.signal(signal.SIGTERM, shutdown)
        url_host = "127.0.0.1" if host == "0.0.0.0" else host
        print("高德可视化面板：http://{}:{}（{}）".format(
            url_host, server.server_address[1], "离线界面，无定位数据" if args.standalone else "ROS 只读订阅"), flush=True)
        if host == "0.0.0.0":
            print("局域网访问：http://<机器人IP>:{}".format(server.server_address[1]), flush=True)
        while rospy is None or not rospy.is_shutdown():
            server.timeout = 0.5
            server.handle_request()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        if rospy is not None:
            rospy.signal_shutdown("Dashboard stopped")


if __name__ == "__main__":
    main()
