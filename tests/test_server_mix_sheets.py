"""PC-only tests; every server database and upload is isolated in a temp directory.

Run: python -m unittest discover -s tests -v
"""

import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from tools import raw_view


ROOT = Path(__file__).resolve().parents[1]


def read_png(path):
    return cv2.imdecode(np.frombuffer(path.read_bytes(), dtype=np.uint8), cv2.IMREAD_COLOR)


class RawSheetTests(unittest.TestCase):
    def test_vga_100_frames_sampled_without_loading_whole_raw(self):
        with tempfile.TemporaryDirectory() as directory:
            raw = Path(directory) / "影像.raw"
            with raw.open("wb") as output:
                for index in range(100):
                    output.write(bytes([index]) * (640 * 480))
            png = raw.with_suffix(".sheet.png")
            with patch.object(np, "fromfile", side_effect=AssertionError("whole-file read")):
                raw_view.save_contact_sheet(raw, png, expected_frames=100)
            image = read_png(png)
            self.assertEqual(image.shape, (960, 1600, 3))
            for slot, index in enumerate(raw_view.sheet_indices(100, 5, 4)):
                row, column = divmod(slot, 5)
                self.assertEqual(int(image[row * 240 + 120, column * 320 + 160, 0]), index)
            self.assertEqual(raw.stat().st_size, 30_720_000)

    def test_short_clip_leaves_blank_cells(self):
        frames = np.full((1, 24, 32), 77, dtype=np.uint8)
        sheet = raw_view.contact_sheet(frames)
        self.assertEqual(sheet[120, 160, 0], 77)
        self.assertEqual(sheet[120, 480, 0], 0)
        self.assertEqual(sheet[840, 1440, 0], 0)

    def test_invalid_raw_and_metadata_do_not_create_png(self):
        with tempfile.TemporaryDirectory() as directory:
            raw, png = Path(directory) / "input.raw", Path(directory) / "sheet.png"
            for content, expected in ((b"", None), (b"x", None), (b"x" * 12, 2)):
                with self.subTest(size=len(content), expected=expected):
                    raw.write_bytes(content)
                    with self.assertRaises(ValueError):
                        raw_view.save_contact_sheet(raw, png, width=4, height=3, expected_frames=expected)
                    self.assertFalse(png.exists())

    def test_failed_publish_keeps_existing_png_and_cleans_temp(self):
        with tempfile.TemporaryDirectory() as directory:
            raw, png = Path(directory) / "input.raw", Path(directory) / "sheet.png"
            raw.write_bytes(b"x" * 12)
            png.write_bytes(b"existing preview")
            with patch.object(raw_view.os, "replace", side_effect=OSError("publish failure")):
                with self.assertRaises(OSError):
                    raw_view.save_contact_sheet(raw, png, width=4, height=3)
            self.assertEqual(png.read_bytes(), b"existing preview")
            self.assertEqual(list(Path(directory).glob(".sheet-*.tmp")), [])

    def test_cli_sheet_still_works(self):
        with tempfile.TemporaryDirectory() as directory:
            raw, png = Path(directory) / "input.raw", Path(directory) / "sheet.png"
            raw.write_bytes(bytes(range(24)))
            result = subprocess.run(
                [sys.executable, str(ROOT / "tools/raw_view.py"), str(raw),
                 "--width", "4", "--height", "3", "--sheet", str(png)],
                capture_output=True, text=True, check=True,
            )
            self.assertIn("2 frames", result.stdout)
            self.assertEqual(read_png(png).shape, (960, 1600, 3))


class ServerSheetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Importing the legacy server initializes its DB. Redirect that side
        # effect before import so the user's field_mix.db is never touched.
        cls.import_directory = tempfile.TemporaryDirectory()
        with patch.dict(os.environ, {"FIELD_MIX_DATA_ROOT": cls.import_directory.name}):
            spec = importlib.util.spec_from_file_location("server_mix_sheet_tests", ROOT / "server_mix.py")
            cls.server = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(cls.server)

    @classmethod
    def tearDownClass(cls):
        cls.import_directory.cleanup()

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.server.DATA_ROOT = Path(self.directory.name)
        self.server.DATABASE_PATH = self.server.DATA_ROOT / "field_mix.db"
        self.server.initialize_database()
        self.worker = self.server.RawSheetWorker(self.server.database, self.server.now_text, self.server.app.logger)
        self.server.sheet_worker = self.worker
        self.addCleanup(self.worker.stop)
        self.real_start = self.worker.start
        # Most tests run one job synchronously for deterministic assertions;
        # a separate test exercises the real background/ACK interaction.
        self.start_patch = patch.object(self.worker, "start")
        self.start_patch.start()
        self.addCleanup(self.start_patch.stop)
        self.client = self.server.app.test_client()

    def headers(self, body, capture_id="cam-1", frames=100, width=32, height=24):
        return {
            "X-Device-ID": "test-node", "X-Session-ID": "session-1",
            "X-Capture-ID": capture_id, "X-Recorded-At": "1760000000",
            "X-File-Size": str(len(body)), "X-Frame-Count": str(frames),
            "X-Width": str(width), "X-Height": str(height), "X-FPS": "5", "X-Format": "GRAY8",
        }

    def upload_video(self, capture_id="cam-1"):
        body = b"\x55" * (32 * 24 * 100)
        return self.client.post("/upload/video", data=body, headers=self.headers(body, capture_id))

    def job(self):
        with self.server.database() as connection:
            row = connection.execute("SELECT * FROM sheet_jobs").fetchone()
            return dict(row) if row else None

    def wait_ready(self):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            job = self.job()
            if job and job["status"] == "ready":
                return job
            if job and job["status"] == "failed":
                self.fail(job["last_error"])
            time.sleep(0.01)
        self.fail("sheet was not ready in time")

    def test_upload_persists_job_then_generates_preview(self):
        response = self.upload_video()
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json["bytes"], 32 * 24 * 100)
        self.assertEqual(self.job()["status"], "pending")
        self.assertTrue(self.worker.process_next())
        job = self.job()
        self.assertEqual(job["status"], "ready")
        self.assertTrue(Path(job["sheet_path"]).exists())
        upload = self.client.get("/uploads").json[0]
        self.assertEqual(upload["sheet_status"], "ready")
        self.assertEqual(upload["metadata"]["frames"], 100)
        self.assertEqual(Path(upload["file_path"]).stat().st_size, response.json["bytes"])

    def test_background_render_does_not_hold_ack(self):
        entered, release = threading.Event(), threading.Event()
        original = raw_view.save_contact_sheet

        def slow_render(*args, **kwargs):
            entered.set()
            if not release.wait(5):
                raise TimeoutError("test renderer was not released")
            return original(*args, **kwargs)

        with patch.object(raw_view, "save_contact_sheet", side_effect=slow_render):
            self.real_start()
            try:
                started = time.monotonic()
                response = self.upload_video()
                self.assertEqual(response.status_code, 200)
                self.assertLess(time.monotonic() - started, 2)
                self.assertTrue(entered.wait(2))
                self.assertEqual(self.job()["status"], "processing")
                release.set()
                self.wait_ready()
            finally:
                release.set()
                self.worker.stop()

    def test_duplicate_upload_does_not_create_another_job(self):
        self.assertEqual(self.upload_video().status_code, 200)
        self.worker.process_next()
        before = Path(self.job()["sheet_path"]).read_bytes()
        response = self.upload_video()
        self.assertTrue(response.json["duplicate"])
        with self.server.database() as connection:
            self.assertEqual(connection.execute("SELECT COUNT(*) FROM sheet_jobs").fetchone()[0], 1)
            self.assertEqual(connection.execute("SELECT COUNT(*) FROM uploads").fetchone()[0], 1)
        self.assertFalse(self.worker.process_next())
        self.assertEqual(Path(self.job()["sheet_path"]).read_bytes(), before)

    def test_failed_preview_keeps_upload_and_retries_on_restart(self):
        self.assertEqual(self.upload_video().status_code, 200)
        with patch.object(raw_view, "save_contact_sheet", side_effect=ImportError("missing image dependency")):
            with self.assertLogs(self.server.app.logger, level="WARNING"):
                self.worker.process_next()
        self.assertEqual(self.job()["status"], "failed")
        self.assertIn("ImportError", self.job()["last_error"])
        self.assertEqual(self.client.get("/uploads").json[0]["media_type"], "video")
        self.assertTrue(Path(self.client.get("/uploads").json[0]["file_path"]).exists())
        self.real_start()
        self.wait_ready()

    def test_startup_backfills_old_video_uploads(self):
        self.upload_video()
        with self.server.database() as connection:
            connection.execute("DELETE FROM sheet_jobs")
        self.real_start()
        self.wait_ready()

    def test_startup_recovers_interrupted_processing(self):
        self.upload_video()
        with self.server.database() as connection:
            connection.execute("UPDATE sheet_jobs SET status = 'processing'")
        self.real_start()
        self.wait_ready()

    def test_rejected_upload_never_queues_sheet(self):
        body = b"\x55" * 10
        response = self.client.post("/upload/video", data=body, headers=self.headers(body))
        self.assertEqual(response.status_code, 400)
        self.assertIsNone(self.job())
        self.assertEqual(list(self.server.DATA_ROOT.rglob("*.raw")), [])

    def test_audio_and_heartbeat_still_work_without_sheet_jobs(self):
        wav = b"RIFF" + b"\0" * 4 + b"WAVE" + b"\0" * 40
        response = self.client.post("/upload/audio", data=wav, headers=self.headers(wav, "mic-1"))
        self.assertEqual(response.status_code, 200)
        response = self.client.post("/heartbeat", json={"device_id": "test-node", "time": 1760000000})
        self.assertEqual(response.status_code, 200)
        self.assertIsNone(self.job())
        with self.server.database() as connection:
            self.assertEqual(connection.execute("SELECT COUNT(*) FROM heartbeats").fetchone()[0], 1)
        upload = self.client.get("/uploads").json[0]
        self.assertEqual(upload["media_type"], "audio")
        self.assertIsNone(upload["sheet_status"])


if __name__ == "__main__":
    unittest.main()
