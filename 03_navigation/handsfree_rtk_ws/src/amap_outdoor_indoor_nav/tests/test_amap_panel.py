"""Offline regression checks for telemetry validity and the read-only HTTP API."""

import importlib.util
import json
import os
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "amap_panel.py"
SPEC = importlib.util.spec_from_file_location("amap_panel", SCRIPT)
panel = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(panel)


def fix(lat=31.2, lon=121.4, status=2, covariance_type=2, covariance=None):
    return NS(latitude=lat, longitude=lon, altitude=float("nan"),
              status=NS(status=status), position_covariance_type=covariance_type,
              position_covariance=covariance or [0.0001, 0, 0, 0, 0.0004, 0, 0, 0, 1],
              header=NS(stamp=NS(to_sec=lambda: 123.0), frame_id="wgs84"))


def sentence(quality=4, talker="GN"):
    body = "{}GGA,071000.00,3112.000,N,12124.000,E,{},18,0.7,10.0,M,0,M,,".format(talker, quality)
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return "${}*{:02X}".format(body, checksum)


class TelemetryTests(unittest.TestCase):
    def test_latest_gga_checks_checksum_and_preserves_quality_zero(self):
        result = panel.parse_gga(sentence(4) + "\r\n" + sentence(0, "GP"))
        self.assertEqual(result["gga_quality"], 0)
        self.assertEqual(result["satellites"], 18)
        self.assertEqual(result["hdop"], 0.7)
        self.assertIsNone(panel.parse_gga(sentence()[:-2] + "00"))
        self.assertIsNone(panel.parse_gga("$GNGGA,,,,,,fixed,,,,"))
        self.assertIsNone(panel.parse_gga("$GNRMC,071000,A,,,,"))

    def test_no_gbas_inference_and_unknown_covariance(self):
        telemetry = panel.Telemetry()
        telemetry.on_fix(fix())
        self.assertIsNone(telemetry.snapshot()["quality"])
        self.assertEqual(telemetry.snapshot()["fix"]["accuracy_m"], 0.02)
        telemetry.on_fix(fix(covariance_type=0))
        self.assertIsNone(telemetry.snapshot()["fix"]["accuracy_m"])
        telemetry.on_fix(fix(covariance=[-1, 0, 0, 0, 1, 0, 0, 0, 1]))
        self.assertIsNone(telemetry.snapshot()["fix"]["accuracy_m"])

    def test_invalid_positions_and_nonfinite_json(self):
        telemetry = panel.Telemetry()
        telemetry.on_fix(fix())
        json.dumps(telemetry.snapshot(), allow_nan=False)
        self.assertIsNone(telemetry.snapshot()["fix"]["altitude"])
        for lat, lon in [(91, 0), (0, 181), (float("nan"), 1), (1, float("inf"))]:
            telemetry.on_fix(fix(lat, lon), "gcj02")
            self.assertIsNone(telemetry.snapshot()["gcj02"])
            self.assertEqual(telemetry.snapshot()["track"], [])

    def test_bounded_track_resets_after_outage_and_no_fix(self):
        telemetry = panel.Telemetry(max_track_points=2, stale_after_sec=3)
        for now in [10.0, 10.5, 11.0]:
            with patch.object(panel.time, "time", return_value=now):
                telemetry.on_fix(fix(), "gcj02")
        self.assertEqual(len(telemetry.snapshot()["track"]), 2)
        with patch.object(panel.time, "time", return_value=15.0):
            telemetry.on_fix(fix(), "gcj02")
        self.assertEqual(len(telemetry.snapshot()["track"]), 1)
        telemetry.on_fix(fix(status=-1), "gcj02")
        self.assertEqual(telemetry.snapshot()["track"], [])

    def test_snapshot_does_not_mutate_shared_state(self):
        telemetry = panel.Telemetry()
        telemetry.on_fix(fix(), "gcj02")
        first = telemetry.snapshot()
        first["track"][0]["latitude"] = -80
        first["gcj02"]["latitude"] = -80
        self.assertEqual(telemetry.snapshot()["track"][0]["latitude"], 31.2)
        self.assertEqual(telemetry.snapshot()["gcj02"]["latitude"], 31.2)

    def test_raw_quality_stops_updating_on_invalid_input(self):
        telemetry = panel.Telemetry()
        with patch.object(panel.time, "time", return_value=1.0):
            telemetry.on_raw(NS(data=sentence(4)))
        with patch.object(panel.time, "time", return_value=10.0):
            telemetry.on_raw(NS(data="$GNGGA,,,,,,oops"))
        self.assertEqual(telemetry.snapshot()["quality"]["received_at"], 1.0)

    def test_far_marker_filter_delete_and_invalid_geometry(self):
        telemetry = panel.Telemetry()
        marker = NS(action=0, type=4, ns="global_path", points=[NS(x=1, y=2, z=0)],
                    header=NS(frame_id="world"))
        telemetry.on_path(marker)
        self.assertEqual(telemetry.snapshot()["navigation"]["path"]["points"][0]["x"], 1)
        marker.ns = "other"
        marker.points = []
        telemetry.on_path(marker)
        self.assertEqual(len(telemetry.snapshot()["navigation"]["path"]["points"]), 1)
        marker.ns, marker.action = "global_path", 2
        telemetry.on_path(marker)
        self.assertIsNone(telemetry.snapshot()["navigation"]["path"])
        marker.action, marker.points = 0, [NS(x=float("nan"), y=0, z=0)]
        telemetry.on_path(marker)
        self.assertIsNone(telemetry.snapshot()["navigation"]["path"])

    def test_new_origin_invalidates_old_enu_and_new_goal_clears_reached(self):
        telemetry = panel.Telemetry()
        telemetry.on_enu(NS(pose=NS(position=NS(x=1, y=2, z=3)), header=NS(frame_id="geo_enu")))
        telemetry.on_origin(fix())
        self.assertIsNone(telemetry.snapshot()["enu"])
        telemetry.on_reached(NS(data=True))
        telemetry.on_goal(NS(point=NS(x=1, y=2, z=0), header=NS(frame_id="world")))
        self.assertIsNone(telemetry.snapshot()["navigation"]["reached"])


class HTTPTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.assets = tempfile.TemporaryDirectory()
        Path(cls.assets.name, "index.html").write_text("<!doctype html><title>面板</title>")
        cls.server = panel.ThreadingHTTPServer(("127.0.0.1", 0), panel.PanelHandler)
        cls.server.telemetry = panel.Telemetry()
        cls.server.panel_config = {"amap_key": "", "amap_security_js_code": ""}
        cls.server.web_root = Path(cls.assets.name)
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.base = "http://127.0.0.1:{}".format(cls.server.server_port)

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join()
        cls.assets.cleanup()

    def test_empty_telemetry_is_explicit_and_api_is_not_cached(self):
        with urlopen(self.base + "/api/state", timeout=2) as response:
            data = json.load(response)
            self.assertEqual(response.headers["Cache-Control"], "no-store")
        self.assertIsNone(data["gcj02"])
        self.assertIsNone(data["quality"])
        self.assertEqual(data["track"], [])

    def test_assets_head_and_read_only_api(self):
        with urlopen(Request(self.base, method="HEAD"), timeout=2) as response:
            self.assertEqual(response.status, 200)
            self.assertEqual(response.read(), b"")
        for path, method, status in [("/../scripts/amap_panel.py", "GET", 404),
                                     ("/api/state", "POST", 501)]:
            with self.assertRaises(HTTPError) as failure:
                urlopen(Request(self.base + path, method=method), timeout=2)
            self.assertEqual(failure.exception.code, status)


class CredentialTests(unittest.TestCase):
    def test_per_machine_credentials_and_environment_precedence(self):
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {}, clear=True):
            path = Path(directory) / "credentials.json"
            self.assertEqual(panel.load_credentials(path)["amap_key"], "")
            path.write_text(json.dumps({"amap_key": "test-key", "amap_security_js_code": "test-code"}))
            self.assertEqual(panel.load_credentials(path)["amap_key"], "test-key")
            with patch.dict(os.environ, {"AMAP_JS_KEY": "env-key"}):
                result = panel.load_credentials(path)
            self.assertEqual(result, {"amap_key": "env-key", "amap_security_js_code": "test-code"})

    def test_invalid_credentials_do_not_echo_file_contents(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "credentials.json"
            path.write_text("sensitive-invalid-json")
            with self.assertRaises(ValueError) as error:
                panel.load_credentials(path)
            self.assertNotIn("sensitive-invalid-json", str(error.exception))


if __name__ == "__main__":
    unittest.main()
