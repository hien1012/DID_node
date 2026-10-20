"""Receiving server for the ESP32-S3 camera + microphone node.

Every session produces one GRAY8 RAW video and one WAV recording that share a
session ID; both are stored side by side:

    <DATA_ROOT>/<device_id>/<YYYY-MM-DD>/<YYYY_MM_DD_HHMMSS>.raw
    <DATA_ROOT>/<device_id>/<YYYY-MM-DD>/<YYYY_MM_DD_HHMMSS>.wav

Endpoints
    GET  /time            {"epoch": <unix seconds>} used by the node at boot
    POST /upload/video    raw body + X-* headers, JSON ACK
    POST /upload/audio    raw body + X-* headers, JSON ACK
    POST /heartbeat       JSON status once a minute (docs/heartbeat.md)
    GET  /devices         latest heartbeat per device, with online/offline
    GET  /sessions        recent sessions with their video/audio files
    GET  /uploads         recent uploaded files

Run:  python server_mix.py   (listens on 0.0.0.0:5000)
Data defaults to ./server_mix_data; set FIELD_MIX_DATA_ROOT to override it.
"""

import hashlib
import json
import os
import re
import sqlite3
import tempfile
import threading
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Callable

from flask import Flask, jsonify, request


TAIPEI_TZ = timezone(timedelta(hours=8))
DATA_ROOT = Path(
    os.environ.get(
        "FIELD_MIX_DATA_ROOT",
        str(Path(__file__).resolve().parent / "server_mix_data"),
    )
)
DATABASE_PATH = DATA_ROOT / "field_mix.db"
VALID_ID = re.compile(r"^[A-Za-z0-9_.-]{1,80}$")
# One 60 s VGA GRAY8 recording is about 92 MB.
MAX_UPLOAD_BYTES = 256 * 1024 * 1024
# A device is reported offline after this many seconds without a heartbeat or
# upload. The node pauses heartbeats while recording (68 s) and uploading
# (about 1-2 min per session), so this must cover a whole session.
OFFLINE_AFTER_S = 300
OFFLINE_CHECK_PERIOD_S = 30

app = Flask(__name__)
app.config["MAX_CONTENT_LENGTH"] = MAX_UPLOAD_BYTES


def now_text() -> str:
    return datetime.now(TAIPEI_TZ).isoformat(timespec="seconds")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        while chunk := source.read(64 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def database() -> sqlite3.Connection:
    connection = sqlite3.connect(DATABASE_PATH, timeout=30)
    connection.row_factory = sqlite3.Row
    return connection


def initialize_database() -> None:
    DATA_ROOT.mkdir(parents=True, exist_ok=True)
    with database() as connection:
        connection.executescript(
            """
            CREATE TABLE IF NOT EXISTS uploads (
                capture_id TEXT PRIMARY KEY,
                session_id TEXT NOT NULL,
                device_id TEXT NOT NULL,
                media_type TEXT NOT NULL,
                recorded_at INTEGER NOT NULL,
                received_at TEXT NOT NULL,
                file_path TEXT NOT NULL,
                file_bytes INTEGER NOT NULL,
                sha256 TEXT NOT NULL,
                start_offset_us INTEGER,
                metadata_json TEXT NOT NULL DEFAULT '{}'
            );
            CREATE INDEX IF NOT EXISTS idx_uploads_session
                ON uploads(session_id);
            CREATE INDEX IF NOT EXISTS idx_uploads_device_time
                ON uploads(device_id, recorded_at);
            CREATE TABLE IF NOT EXISTS heartbeats (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                device_id TEXT NOT NULL,
                received_at TEXT NOT NULL,
                received_epoch INTEGER NOT NULL,
                reported_at INTEGER,
                payload_json TEXT NOT NULL
            );
            CREATE INDEX IF NOT EXISTS idx_heartbeats_device
                ON heartbeats(device_id, id);
            """
        )


def error(message: str, status_code: int):
    return jsonify({"status": "error", "message": message}), status_code


def parse_common_headers():
    device_id = request.headers.get("X-Device-ID", "")
    session_id = request.headers.get("X-Session-ID", "")
    capture_id = request.headers.get("X-Capture-ID", "")
    if not all(VALID_ID.fullmatch(value) for value in (device_id, session_id, capture_id)):
        return None, error("invalid device, session or capture id", 400)

    try:
        recorded_at = int(request.headers.get("X-Recorded-At", ""))
        expected_size = int(request.headers.get("X-File-Size", ""))
    except ValueError:
        return None, error("invalid recording time or file size", 400)
    if recorded_at < 1_700_000_000 or expected_size <= 0:
        return None, error("recording time or file size is out of range", 400)
    if expected_size > MAX_UPLOAD_BYTES:
        return None, error("uploaded file is too large", 413)

    offset_text = request.headers.get("X-Start-Offset-US")
    try:
        start_offset_us = int(offset_text) if offset_text is not None else None
    except ValueError:
        return None, error("invalid start offset", 400)

    return {
        "device_id": device_id,
        "session_id": session_id,
        "capture_id": capture_id,
        "recorded_at": recorded_at,
        "expected_size": expected_size,
        "start_offset_us": start_offset_us,
    }, None


def save_upload(
    *,
    media_type: str,
    extension: str,
    common: dict,
    metadata: dict,
    validate_body: Callable[[bytes, int], str | None],
):
    capture_id = common["capture_id"]
    expected_size = common["expected_size"]
    with database() as connection:
        existing = connection.execute(
            "SELECT * FROM uploads WHERE capture_id = ?", (capture_id,)
        ).fetchone()
    if existing is not None:
        # The node retries when an ACK is lost; answer with the stored result.
        if (
            Path(existing["file_path"]).is_file()
            and existing["file_bytes"] == expected_size
            and existing["media_type"] == media_type
        ):
            return jsonify(
                {
                    "status": "ok",
                    "capture_id": capture_id,
                    "bytes": existing["file_bytes"],
                    "sha256": existing["sha256"],
                    "received_at": existing["received_at"],
                    "duplicate": True,
                }
            )
        return error("capture_id exists but stored file is inconsistent", 409)

    recorded_dt = datetime.fromtimestamp(common["recorded_at"], TAIPEI_TZ)
    target_dir = DATA_ROOT / common["device_id"] / recorded_dt.strftime("%Y-%m-%d")
    target_dir.mkdir(parents=True, exist_ok=True)
    target_path = target_dir / f"{recorded_dt.strftime('%Y_%m_%d_%H%M%S')}{extension}"
    target_existed = target_path.exists()

    temp_path = None
    received_size = 0
    digest = hashlib.sha256()
    prefix = bytearray()
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb", dir=target_dir, prefix=".upload-", suffix=".tmp", delete=False
        ) as output:
            temp_path = Path(output.name)
            while True:
                chunk = request.stream.read(256 * 1024)
                if not chunk:
                    break
                received_size += len(chunk)
                if received_size > expected_size:
                    return error("received more bytes than declared", 400)
                if len(prefix) < 12:
                    prefix.extend(chunk[: 12 - len(prefix)])
                digest.update(chunk)
                output.write(chunk)
            output.flush()
            os.fsync(output.fileno())

        if received_size != expected_size:
            return error(
                f"size mismatch: expected {expected_size}, received {received_size}",
                400,
            )
        validation_error = validate_body(bytes(prefix), received_size)
        if validation_error is not None:
            return error(validation_error, 400)

        received_at = now_text()
        duplicate = False
        if target_existed:
            if (
                target_path.stat().st_size != received_size
                or sha256_file(target_path) != digest.hexdigest()
            ):
                return error("another recording already uses this timestamp", 409)
            temp_path.unlink()
            temp_path = None
            duplicate = True
        else:
            os.replace(temp_path, target_path)
            temp_path = None

        with database() as connection:
            connection.execute(
                """
                INSERT INTO uploads
                    (capture_id, session_id, device_id, media_type, recorded_at,
                     received_at, file_path, file_bytes, sha256, start_offset_us,
                     metadata_json)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    capture_id,
                    common["session_id"],
                    common["device_id"],
                    media_type,
                    common["recorded_at"],
                    received_at,
                    str(target_path),
                    received_size,
                    digest.hexdigest(),
                    common["start_offset_us"],
                    json.dumps(metadata, ensure_ascii=False, separators=(",", ":")),
                ),
            )

        offline_watch.note_seen(common["device_id"], int(time.time()))
        print(
            f"[{received_at}] {media_type:5s} saved: device={common['device_id']} "
            f"session={common['session_id']} bytes={received_size} path={target_path}"
        )
        return jsonify(
            {
                "status": "ok",
                "capture_id": capture_id,
                "bytes": received_size,
                "sha256": digest.hexdigest(),
                "received_at": received_at,
                "duplicate": duplicate,
            }
        )
    finally:
        if temp_path is not None:
            temp_path.unlink(missing_ok=True)


@app.get("/time")
def get_time():
    return jsonify({"epoch": int(time.time())})


@app.post("/upload/video")
def upload_video():
    common, response = parse_common_headers()
    if response is not None:
        return response

    try:
        frame_count = int(request.headers.get("X-Frame-Count", ""))
        width = int(request.headers.get("X-Width", ""))
        height = int(request.headers.get("X-Height", ""))
        fps = int(request.headers.get("X-FPS", ""))
    except ValueError:
        return error("invalid video metadata", 400)
    pixel_format = request.headers.get("X-Format", "")
    if min(frame_count, width, height, fps) <= 0 or pixel_format != "GRAY8":
        return error("unsupported video metadata", 400)
    if common["expected_size"] != width * height * frame_count:
        return error("RAW size does not match width, height and frame count", 400)

    metadata = {
        "format": pixel_format,
        "width": width,
        "height": height,
        "fps": fps,
        "frames": frame_count,
    }
    return save_upload(
        media_type="video",
        extension=".raw",
        common=common,
        metadata=metadata,
        validate_body=lambda _prefix, _received_size: None,
    )


@app.post("/upload/audio")
def upload_audio():
    common, response = parse_common_headers()
    if response is not None:
        return response
    if common["expected_size"] <= 44:
        return error("audio file is too small", 400)
    try:
        sample_rate = int(request.headers.get("X-Sample-Rate", "16000"))
    except ValueError:
        return error("invalid sample rate", 400)

    def validate_wav(prefix: bytes, _received_size: int):
        if prefix[:4] != b"RIFF" or prefix[8:12] != b"WAVE":
            return "body is not a WAV file"
        return None

    return save_upload(
        media_type="audio",
        extension=".wav",
        common=common,
        metadata={"format": "WAV", "sample_rate": sample_rate},
        validate_body=validate_wav,
    )


def describe_heartbeat(payload: dict) -> str:
    """One readable console line per heartbeat."""
    wifi = payload.get("wifi") or {}
    storage = payload.get("storage") or {}
    recording = payload.get("last_recording") or {}
    parts = [
        f"state={payload.get('state', '?')}",
        f"rssi={wifi.get('rssi', '?')}",
        f"pending={storage.get('pending_files', '?')}",
        f"broken={storage.get('broken_files', '?')}",
        f"sd_free={storage.get('free_mb', '?')}MB",
    ]
    if recording:
        parts.append(f"last_rec cam={recording.get('camera')} mic={recording.get('mic')}")
    if wifi.get("heartbeat_fail_streak"):
        parts.append(f"hb_fail_streak={wifi['heartbeat_fail_streak']}")
    return " ".join(parts)


@app.post("/heartbeat")
def heartbeat():
    payload = request.get_json(silent=True)
    if not isinstance(payload, dict):
        return error("JSON object required", 400)
    device_id = payload.get("device_id", "")
    if not isinstance(device_id, str) or not VALID_ID.fullmatch(device_id):
        return error("invalid device_id", 400)

    received_at = now_text()
    received_epoch = int(time.time())
    reported_at = payload.get("time")
    with database() as connection:
        connection.execute(
            """
            INSERT INTO heartbeats
                (device_id, received_at, received_epoch, reported_at, payload_json)
            VALUES (?, ?, ?, ?, ?)
            """,
            (
                device_id,
                received_at,
                received_epoch,
                int(reported_at) if isinstance(reported_at, (int, float)) else None,
                json.dumps(payload, ensure_ascii=False, separators=(",", ":")),
            ),
        )
    offline_watch.note_seen(device_id, received_epoch)
    print(f"[{received_at}] heartbeat {device_id}: {describe_heartbeat(payload)}")
    return jsonify({"status": "ok", "device_id": device_id, "received_at": received_at})


@app.get("/devices")
def devices():
    now = int(time.time())
    with database() as connection:
        rows = connection.execute(
            """
            SELECT h.device_id, h.received_at, h.received_epoch, h.payload_json
            FROM heartbeats h
            JOIN (
                SELECT device_id, MAX(id) AS last_id
                FROM heartbeats GROUP BY device_id
            ) latest ON latest.last_id = h.id
            ORDER BY h.device_id
            """
        ).fetchall()
    return jsonify(
        [
            {
                "device_id": row["device_id"],
                "online": now - row["received_epoch"] <= OFFLINE_AFTER_S,
                "last_seen": row["received_at"],
                "seconds_since": now - row["received_epoch"],
                "status": json.loads(row["payload_json"]),
            }
            for row in rows
        ]
    )


@app.get("/sessions")
def sessions():
    try:
        limit = min(max(int(request.args.get("limit", "50")), 1), 500)
    except ValueError:
        return error("limit must be an integer", 400)
    with database() as connection:
        rows = connection.execute(
            """
            SELECT session_id, device_id, MIN(recorded_at) AS recorded_at,
                   MAX(CASE WHEN media_type = 'video' THEN file_path END) AS video,
                   MAX(CASE WHEN media_type = 'audio' THEN file_path END) AS audio,
                   MAX(CASE WHEN media_type = 'video' THEN start_offset_us END)
                       AS video_offset_us,
                   MAX(CASE WHEN media_type = 'audio' THEN start_offset_us END)
                       AS audio_offset_us
            FROM uploads GROUP BY session_id
            ORDER BY recorded_at DESC LIMIT ?
            """,
            (limit,),
        ).fetchall()
    return jsonify([dict(row) for row in rows])


@app.get("/uploads")
def uploads():
    try:
        limit = min(max(int(request.args.get("limit", "100")), 1), 1000)
    except ValueError:
        return error("limit must be an integer", 400)
    with database() as connection:
        rows = connection.execute(
            """
            SELECT capture_id, session_id, device_id, media_type, recorded_at,
                   received_at, file_path, file_bytes, sha256, start_offset_us,
                   metadata_json
            FROM uploads ORDER BY received_at DESC LIMIT ?
            """,
            (limit,),
        ).fetchall()
    return jsonify(
        [
            {
                **{key: row[key] for key in row.keys() if key != "metadata_json"},
                "metadata": json.loads(row["metadata_json"]),
            }
            for row in rows
        ]
    )


class OfflineWatch:
    """Prints once when a device stops sending heartbeats and when it returns."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._last_seen: dict[str, int] = {}
        self._offline: set[str] = set()

    def note_seen(self, device_id: str, epoch: int) -> None:
        with self._lock:
            previous = self._last_seen.get(device_id)
            self._last_seen[device_id] = epoch
            if device_id in self._offline:
                self._offline.discard(device_id)
                gap = epoch - previous if previous else 0
                print(f"[{now_text()}] {device_id} back online after {gap} s")

    def run(self) -> None:
        while True:
            time.sleep(OFFLINE_CHECK_PERIOD_S)
            now = int(time.time())
            with self._lock:
                for device_id, seen in self._last_seen.items():
                    if now - seen > OFFLINE_AFTER_S and device_id not in self._offline:
                        self._offline.add(device_id)
                        print(
                            f"[{now_text()}] WARNING {device_id} offline: "
                            f"no heartbeat for {now - seen} s"
                        )


offline_watch = OfflineWatch()
initialize_database()

if __name__ == "__main__":
    threading.Thread(target=offline_watch.run, daemon=True).start()
    app.run(host="0.0.0.0", port=5000, threaded=True)
