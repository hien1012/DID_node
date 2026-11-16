"""Receiving server for the ESP32-S3 camera + microphone node.

Every session produces one GRAY8 RAW video and one WAV recording that share a
session ID; both are stored side by side:

    <DATA_ROOT>/<device_id>/<YYYY-MM-DD>/<YYYY_MM_DD_HHMMSS>.raw
    <DATA_ROOT>/<device_id>/<YYYY-MM-DD>/<YYYY_MM_DD_HHMMSS>.wav

Endpoints
    GET  /time            {"epoch": <unix seconds>, "epoch_us": <unix microseconds>}
                          used by the node at boot and after every upload pass
    POST /upload/video    raw body + X-* headers, JSON ACK
    POST /upload/audio    raw body + X-* headers, JSON ACK
    POST /heartbeat       JSON status once a minute (docs/heartbeat.md)
    GET  /devices         latest heartbeat per device, with online/offline
    GET  /sessions        recent sessions with their video/audio files
    GET  /uploads         recent uploaded files

Inference firmware (infer_node/, docs/inference_integration_plan.md)
    POST /infer/results   JSON batch of inference results and timing records
    POST /infer/sample    one low-confidence sample: meta JSON + raw data, JSON ACK
    GET  /                dashboard (tools/infer_dashboard.html), also /dashboard?device=<id>
    GET  /nodes           one row per node, S3 and ESP32 alike (tools/infer_nodes.html)
    GET  /api/infer/...   data for the dashboard and the node overview

Samples are stored as
    <DATA_ROOT>/<device_id>/samples/<YYYY-MM-DD>/<audio|video>/<sample_id>.wav|.raw|.jpg + .json
A video sample is the full frame (.raw GRAY8, or .jpg when the node's
INFER_CAM_SAMPLE_JPEG switch is on) followed by the raw 64x64 ROI crops of that frame.

Every received RAW is checked in the background (the ACK is not delayed):
per frame row_jump / col_jump as in tools/raw_view.py --stats. A frame whose
larger jump exceeds the smaller one by FIELD_MIX_JUMP_RATIO (default 1.5) is
suspect: byte-phase slips give col_jump >> row_jump (gray U/V bands), noisy
rows give row_jump >> col_jump (streaks); a clean frame is about 1.1. Results
go to the video_checks table and suspect files print a WARNING line.

Run:  python server_mix.py           (listens on 0.0.0.0:5000)
      python server_mix.py --sheet   also save a contact sheet of every RAW to
                                     <date dir>/sheets/<receive time>.png
Video checks need numpy + opencv-python; without them uploads still work.
"""

import argparse
import hashlib
import importlib.util
import json
import os
import queue
import re
import sqlite3
import struct
import tempfile
import threading
import time
import zlib
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Callable

from flask import Flask, Response, jsonify, request, send_file


TAIPEI_TZ = timezone(timedelta(hours=8))
DATA_ROOT = Path(
    os.environ.get("FIELD_MIX_DATA_ROOT", r"C:\Users\Potato\ESP_project file\server_mix")
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
# Video check (docs/debug_sdcard.md #33-#35): clean frames have col/row jump
# about 1.1; gray-band frames 1.9-13; streak frames row/col 3+.
JUMP_RATIO = float(os.environ.get("FIELD_MIX_JUMP_RATIO", "1.5"))
# Below this both jumps are noise (dark or flat frame): never suspect.
JUMP_MIN = 1.0
RAW_VIEW_PATH = Path(__file__).resolve().parent / "tools" / "raw_view.py"
DASHBOARD_PATH = Path(__file__).resolve().parent / "tools" / "infer_dashboard.html"
NODES_PAGE_PATH = Path(__file__).resolve().parent / "tools" / "infer_nodes.html"
# Node overview: inference counts cover this window; at most this many recent
# records per device and kind are read (a mic sends 1,800 audio records an hour).
NODES_WINDOW_S = 3600
NODES_MAX_RECORDS = 6000
# Inference records keep their JSON as sent; the columns are what the dashboard filters on.
INFER_RECORD_KINDS = ("audio", "roi", "frame", "session", "cnn_ops", "task_usage")
MAX_SAMPLE_META_BYTES = 16 * 1024
# The camera reports every frame at 5 fps (about 430,000 rows a day); only the
# recent timing is useful, inference results are kept.
FRAME_RECORD_KEEP_S = 24 * 3600
PRUNE_PERIOD_S = 3600

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
            CREATE TABLE IF NOT EXISTS video_checks (
                capture_id TEXT PRIMARY KEY,
                device_id TEXT NOT NULL,
                recorded_at INTEGER NOT NULL,
                received_at TEXT NOT NULL,
                checked_at TEXT NOT NULL,
                frames INTEGER NOT NULL,
                suspect_frames INTEGER NOT NULL,
                suspect_list TEXT NOT NULL,
                max_ratio REAL NOT NULL,
                median_row_jump REAL NOT NULL,
                median_col_jump REAL NOT NULL,
                sheet_path TEXT
            );
            CREATE INDEX IF NOT EXISTS idx_video_checks_device_time
                ON video_checks(device_id, recorded_at);
            CREATE TABLE IF NOT EXISTS infer_results (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                device_id TEXT NOT NULL,
                kind TEXT NOT NULL,
                dedupe_key TEXT NOT NULL,
                ts_ms INTEGER NOT NULL,
                received_ms INTEGER NOT NULL,
                session INTEGER,
                label TEXT,
                conf REAL,
                low INTEGER NOT NULL DEFAULT 0,
                payload_json TEXT NOT NULL,
                UNIQUE (device_id, dedupe_key)
            );
            CREATE INDEX IF NOT EXISTS idx_infer_results_device_kind
                ON infer_results(device_id, kind, id);
            CREATE TABLE IF NOT EXISTS infer_samples (
                sample_id TEXT PRIMARY KEY,
                device_id TEXT NOT NULL,
                media TEXT NOT NULL,
                ts_ms INTEGER NOT NULL,
                received_at TEXT NOT NULL,
                received_ms INTEGER NOT NULL,
                data_path TEXT NOT NULL,
                data_bytes INTEGER NOT NULL,
                sha256 TEXT NOT NULL,
                label TEXT,
                conf REAL,
                meta_json TEXT NOT NULL
            );
            CREATE INDEX IF NOT EXISTS idx_infer_samples_device_media
                ON infer_samples(device_id, media, ts_ms);
            CREATE TABLE IF NOT EXISTS infer_devices (
                device_id TEXT PRIMARY KEY,
                updated_ms INTEGER NOT NULL,
                time_ms INTEGER,
                dropped INTEGER NOT NULL DEFAULT 0,
                pending_audio INTEGER NOT NULL DEFAULT 0,
                pending_video INTEGER NOT NULL DEFAULT 0
            );
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
        if media_type == "video" and not duplicate:
            video_checker.submit(
                {
                    "capture_id": capture_id,
                    "device_id": common["device_id"],
                    "recorded_at": common["recorded_at"],
                    "received_at": received_at,
                    "path": target_path,
                    "width": metadata["width"],
                    "height": metadata["height"],
                }
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
    # epoch_us lets the node set sub-second time; epoch stays for older firmware.
    now_us = time.time_ns() // 1000
    return jsonify({"epoch": now_us // 1_000_000, "epoch_us": now_us})


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
    faults = payload.get("faults")
    if isinstance(faults, dict):
        # Camera failures per category since boot (uptime_s decreasing marks a reboot).
        counts = {k: int(v) for k, v in faults.items() if isinstance(v, (int, float))}
        detail = " ".join(f"{k}={v}" for k, v in counts.items() if v)
        parts.append(
            f"cam_broken_since_boot={sum(counts.values())}" + (f" ({detail})" if detail else "")
        )
        last_fault = payload.get("last_fault")
        if last_fault and last_fault != "none":
            parts.append(f"last_fault={last_fault}")
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


def infer_record_columns(record: dict) -> dict | None:
    """Columns for one inference record; None when the record is not usable.

    dedupe_key makes a resent batch harmless: the node resends when a response
    is lost.
    """
    kind = record.get("t")
    if kind not in INFER_RECORD_KINDS:
        return None
    try:
        if kind == "audio":
            ts_ms = int(record["start_ms"])
            key = f"a:{ts_ms}:{int(record['window'])}"
            session = None
        elif kind == "roi":
            ts_ms = int(record["ts_ms"])
            session = int(record["session"])
            key = f"r:{session}:{int(record['frame'])}:{int(record['track'])}"
        elif kind == "frame":
            ts_ms = int(record["ts_ms"])
            session = int(record["session"])
            key = f"f:{session}:{int(record['frame'])}"
        elif kind == "cnn_ops":
            session = int(record["session"])
            ts_ms = session * 1000
            key = f"o:{session}:{int(record['frame'])}"
        elif kind == "task_usage":
            session = None
            ts_ms = int(record["ts_ms"])
            key = f"u:{record['media']}:{ts_ms}"
        else:
            # Cumulative statistics, sent periodically while the camera keeps running.
            session = int(record["session"])
            ts_ms = session * 1000
            key = f"s:{session}:{int(record['frames'])}"
        conf = float(record["conf"]) if "conf" in record else None
    except (KeyError, TypeError, ValueError):
        return None
    label = record.get("label")
    return {
        "kind": kind,
        "key": key,
        "ts_ms": ts_ms,
        "session": session,
        "label": label if isinstance(label, str) else None,
        "conf": conf,
        "low": 1 if record.get("low") else 0,
    }


@app.post("/infer/results")
def infer_results():
    payload = request.get_json(silent=True)
    if not isinstance(payload, dict):
        return error("JSON object required", 400)
    device_id = payload.get("device_id", "")
    if not isinstance(device_id, str) or not VALID_ID.fullmatch(device_id):
        return error("invalid device_id", 400)
    records = payload.get("records")
    if not isinstance(records, list):
        return error("records must be a list", 400)

    received_ms = int(time.time() * 1000)
    rows = []
    skipped = 0
    for record in records:
        columns = infer_record_columns(record) if isinstance(record, dict) else None
        if columns is None:
            skipped += 1
            continue
        rows.append(
            (
                device_id,
                columns["kind"],
                columns["key"],
                columns["ts_ms"],
                received_ms,
                columns["session"],
                columns["label"],
                columns["conf"],
                columns["low"],
                json.dumps(record, ensure_ascii=False, separators=(",", ":")),
            )
        )

    pending = payload.get("pending")
    pending = pending if isinstance(pending, dict) else {}
    with database() as connection:
        connection.executemany(
            """
            INSERT OR IGNORE INTO infer_results
                (device_id, kind, dedupe_key, ts_ms, received_ms, session, label,
                 conf, low, payload_json)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            rows,
        )
        connection.execute(
            """
            INSERT OR REPLACE INTO infer_devices
                (device_id, updated_ms, time_ms, dropped, pending_audio, pending_video)
            VALUES (?, ?, ?, ?, ?, ?)
            """,
            (
                device_id,
                received_ms,
                payload.get("time_ms") if isinstance(payload.get("time_ms"), int) else None,
                payload.get("dropped") if isinstance(payload.get("dropped"), int) else 0,
                pending.get("audio") if isinstance(pending.get("audio"), int) else 0,
                pending.get("video") if isinstance(pending.get("video"), int) else 0,
            ),
        )
    offline_watch.note_seen(device_id, received_ms // 1000)
    return jsonify({"status": "ok", "stored": len(rows), "skipped": skipped})


def sample_image_bytes(meta: dict) -> int | None:
    """Size of the image at the start of a video sample: raw GRAY8 or JPEG."""
    try:
        if meta.get("format", "GRAY8") == "GRAY8":
            return int(meta["width"]) * int(meta["height"])
        if meta.get("format") == "JPEG":
            return int(meta["image_bytes"])
    except (KeyError, TypeError, ValueError):
        pass
    return None


def validate_sample_size(media: str, meta: dict, data_bytes: int) -> str | None:
    if media == "audio":
        return None if data_bytes > 44 else "audio sample is too small"
    try:
        width = int(meta["width"])
        height = int(meta["height"])
        roi_size = int(meta["roi_size"])
        roi_count = int(meta["roi_count"])
    except (KeyError, TypeError, ValueError):
        return "video sample meta needs width, height, roi_size and roi_count"
    if min(width, height, roi_size) <= 0 or roi_count < 0:
        return "video sample meta is out of range"
    image_bytes = sample_image_bytes(meta)
    if image_bytes is None or image_bytes <= 0:
        return "video sample meta needs a valid format and image_bytes"
    if data_bytes != image_bytes + roi_count * roi_size * roi_size:
        return "video sample size does not match its meta"
    return None


def sample_summary(meta: dict, media: str) -> tuple[int, str | None, float | None]:
    """(time, label, confidence) of a sample; a video uses its least confident ROI."""
    if media == "audio":
        conf = meta.get("conf")
        return (
            int(meta.get("start_ms", 0)),
            meta.get("label"),
            float(conf) if isinstance(conf, (int, float)) else None,
        )
    rois = [roi for roi in meta.get("rois", []) if isinstance(roi, dict)]
    worst = min(rois, key=lambda roi: roi.get("conf", 1.0), default={})
    conf = worst.get("conf")
    return (
        int(meta.get("ts_ms", 0)),
        worst.get("label"),
        float(conf) if isinstance(conf, (int, float)) else None,
    )


@app.post("/infer/sample")
def infer_sample():
    """Body = X-Meta-Length bytes of JSON, then X-File-Size bytes of raw data."""
    device_id = request.headers.get("X-Device-ID", "")
    sample_id = request.headers.get("X-Sample-ID", "")
    media = request.headers.get("X-Media", "")
    if not VALID_ID.fullmatch(device_id) or not VALID_ID.fullmatch(sample_id):
        return error("invalid device or sample id", 400)
    if media not in ("audio", "video"):
        return error("X-Media must be audio or video", 400)
    try:
        meta_bytes = int(request.headers.get("X-Meta-Length", ""))
        data_bytes = int(request.headers.get("X-File-Size", ""))
    except ValueError:
        return error("invalid meta length or file size", 400)
    if not 0 < meta_bytes <= MAX_SAMPLE_META_BYTES or data_bytes <= 0:
        return error("meta length or file size is out of range", 400)
    if data_bytes + meta_bytes > MAX_UPLOAD_BYTES:
        return error("uploaded sample is too large", 413)

    meta_raw = bytearray()
    while len(meta_raw) < meta_bytes:
        chunk = request.stream.read(meta_bytes - len(meta_raw))
        if not chunk:
            return error("body ended inside the meta", 400)
        meta_raw.extend(chunk)
    try:
        meta = json.loads(meta_raw.decode("utf-8"))
    except (UnicodeDecodeError, ValueError):
        return error("meta is not valid JSON", 400)
    if not isinstance(meta, dict):
        return error("meta must be a JSON object", 400)
    size_error = validate_sample_size(media, meta, data_bytes)
    if size_error is not None:
        return error(size_error, 400)
    try:
        ts_ms, label, conf = sample_summary(meta, media)
    except (TypeError, ValueError):
        return error("meta has invalid time or confidence", 400)
    if ts_ms < 1_700_000_000_000:
        return error("sample time is out of range", 400)

    with database() as connection:
        existing = connection.execute(
            "SELECT data_path, data_bytes, sha256 FROM infer_samples WHERE sample_id = ?",
            (sample_id,),
        ).fetchone()
    if existing is not None:
        # The node retries when an ACK is lost; answer with the stored result.
        if Path(existing["data_path"]).is_file() and existing["data_bytes"] == data_bytes:
            return jsonify(
                {
                    "status": "ok",
                    "sample_id": sample_id,
                    "bytes": data_bytes,
                    "sha256": existing["sha256"],
                    "duplicate": True,
                }
            )
        return error("sample_id exists but stored file is inconsistent", 409)

    day = datetime.fromtimestamp(ts_ms / 1000, TAIPEI_TZ).strftime("%Y-%m-%d")
    target_dir = DATA_ROOT / device_id / "samples" / day / media
    target_dir.mkdir(parents=True, exist_ok=True)
    is_jpeg = media == "video" and meta.get("format") == "JPEG"
    # A viewer stops at the JPEG end marker, so the ROI bytes after it are harmless.
    extension = ".wav" if media == "audio" else ".jpg" if is_jpeg else ".raw"
    target_path = target_dir / f"{sample_id}{extension}"

    temp_path = None
    received = 0
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
                received += len(chunk)
                if received > data_bytes:
                    return error("received more bytes than declared", 400)
                if len(prefix) < 12:
                    prefix.extend(chunk[: 12 - len(prefix)])
                digest.update(chunk)
                output.write(chunk)
            output.flush()
            os.fsync(output.fileno())
        if received != data_bytes:
            return error(f"size mismatch: expected {data_bytes}, received {received}", 400)
        if media == "audio" and (prefix[:4] != b"RIFF" or prefix[8:12] != b"WAVE"):
            return error("body is not a WAV file", 400)
        if is_jpeg and prefix[:2] != b"\xff\xd8":
            return error("body does not start with a JPEG image", 400)

        os.replace(temp_path, target_path)
        temp_path = None
        target_path.with_suffix(".json").write_text(
            json.dumps(meta, ensure_ascii=False, indent=1), encoding="utf-8"
        )
        received_at = now_text()
        with database() as connection:
            connection.execute(
                """
                INSERT INTO infer_samples
                    (sample_id, device_id, media, ts_ms, received_at, received_ms,
                     data_path, data_bytes, sha256, label, conf, meta_json)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    sample_id,
                    device_id,
                    media,
                    ts_ms,
                    received_at,
                    int(time.time() * 1000),
                    str(target_path),
                    data_bytes,
                    digest.hexdigest(),
                    label if isinstance(label, str) else None,
                    conf,
                    json.dumps(meta, ensure_ascii=False, separators=(",", ":")),
                ),
            )
        offline_watch.note_seen(device_id, int(time.time()))
        return jsonify(
            {
                "status": "ok",
                "sample_id": sample_id,
                "bytes": data_bytes,
                "sha256": digest.hexdigest(),
                "duplicate": False,
            }
        )
    finally:
        if temp_path is not None:
            temp_path.unlink(missing_ok=True)


def gray_png(width: int, height: int, pixels: bytes) -> bytes:
    """Minimal 8-bit grayscale PNG, so viewing a sample needs no numpy or OpenCV."""

    def chunk(tag: bytes, body: bytes) -> bytes:
        return (
            struct.pack(">I", len(body))
            + tag
            + body
            + struct.pack(">I", zlib.crc32(tag + body) & 0xFFFFFFFF)
        )

    rows = b"".join(
        b"\x00" + pixels[row * width : (row + 1) * width] for row in range(height)
    )
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(rows, 6))
        + chunk(b"IEND", b"")
    )


def prune_frame_records() -> None:
    while True:
        cutoff_ms = int((time.time() - FRAME_RECORD_KEEP_S) * 1000)
        try:
            with database() as connection:
                connection.execute(
                    "DELETE FROM infer_results "
                    "WHERE kind IN ('frame', 'cnn_ops', 'task_usage') AND received_ms < ?",
                    (cutoff_ms,),
                )
        except sqlite3.Error as exc:
            print(f"[{now_text()}] WARNING frame record cleanup failed: {exc}")
        time.sleep(PRUNE_PERIOD_S)


def infer_payloads(rows) -> list[dict]:
    return [{**json.loads(row["payload_json"]), "id": row["id"]} for row in rows]


def sample_row_to_json(row) -> dict:
    return {
        "sample_id": row["sample_id"],
        "device_id": row["device_id"],
        "media": row["media"],
        "ts_ms": row["ts_ms"],
        "received_at": row["received_at"],
        "data_bytes": row["data_bytes"],
        "label": row["label"],
        "conf": row["conf"],
        "meta": json.loads(row["meta_json"]),
    }


@app.get("/")
@app.get("/dashboard")
def dashboard():
    if not DASHBOARD_PATH.is_file():
        return error(f"{DASHBOARD_PATH.name} is missing", 404)
    return Response(DASHBOARD_PATH.read_text(encoding="utf-8"), mimetype="text/html")


@app.get("/api/infer/overview")
def infer_overview():
    """Everything the dashboard's live panels show for one device."""
    with database() as connection:
        known = connection.execute(
            """
            SELECT device_id, updated_ms, time_ms, dropped, pending_audio, pending_video
            FROM infer_devices ORDER BY updated_ms DESC
            """
        ).fetchall()
        device_id = request.args.get("device") or (known[0]["device_id"] if known else None)
        response = {
            "now_ms": int(time.time() * 1000),
            "devices": [dict(row) for row in known],
            "device_id": device_id,
        }
        if device_id is None:
            return jsonify(response)

        def latest(kind: str, limit: int) -> list[dict]:
            return infer_payloads(
                connection.execute(
                    """
                    SELECT id, payload_json FROM infer_results
                    WHERE device_id = ? AND kind = ? ORDER BY id DESC LIMIT ?
                    """,
                    (device_id, kind, limit),
                ).fetchall()
            )

        since_ms = response["now_ms"] - 24 * 3600 * 1000
        counts = connection.execute(
            """
            SELECT kind, label, COUNT(*) AS total, SUM(low) AS low
            FROM infer_results
            WHERE device_id = ? AND kind IN ('audio', 'roi') AND ts_ms >= ?
            GROUP BY kind, label
            """,
            (device_id, since_ms),
        ).fetchall()
        sample_counts = connection.execute(
            "SELECT media, COUNT(*) AS total FROM infer_samples WHERE device_id = ? GROUP BY media",
            (device_id,),
        ).fetchall()
        heartbeat_row = connection.execute(
            """
            SELECT received_epoch, payload_json FROM heartbeats
            WHERE device_id = ? ORDER BY id DESC LIMIT 1
            """,
            (device_id,),
        ).fetchone()

        frames = latest("frame", 150)
        if frames:
            frames = [f for f in frames if f.get("session") == frames[0].get("session")]
            # 含候選的幀等 CNN 做完才回報，到達順序不等於幀順序
            frames.sort(key=lambda f: f.get("frame", 0), reverse=True)
        response.update(
            {
                "audio": latest("audio", 60),
                "rois": latest("roi", 40),
                "frames": frames,
                "sessions": latest("session", 8),
                "cnn_ops": latest("cnn_ops", 1),
                "task_usage": latest("task_usage", 1),
                "counts": [dict(row) for row in counts],
                "samples_received": {row["media"]: row["total"] for row in sample_counts},
                "heartbeat": json.loads(heartbeat_row["payload_json"]) if heartbeat_row else None,
                "heartbeat_epoch": heartbeat_row["received_epoch"] if heartbeat_row else None,
            }
        )
    return jsonify(response)


@app.get("/nodes")
def nodes_page():
    if not NODES_PAGE_PATH.is_file():
        return error(f"{NODES_PAGE_PATH.name} is missing", 404)
    return Response(NODES_PAGE_PATH.read_text(encoding="utf-8"), mimetype="text/html")


def node_window_counts(connection, device_id: str, kind: str, since_ms: int) -> tuple[dict, dict | None]:
    """Counts of one result kind since since_ms, plus the newest record.

    Reads the newest NODES_MAX_RECORDS rows backwards on the (device_id, kind, id)
    index instead of filtering ts_ms over the whole (never pruned) history.
    """
    rows = connection.execute(
        """
        SELECT ts_ms, label, low, payload_json FROM infer_results
        WHERE device_id = ? AND kind = ? ORDER BY id DESC LIMIT ?
        """,
        (device_id, kind, NODES_MAX_RECORDS),
    ).fetchall()
    counts = {"total": 0, "low": 0, "labels": {}}
    for row in rows:
        if row["ts_ms"] < since_ms:
            continue
        counts["total"] += 1
        counts["low"] += row["low"]
        label = row["label"] or "?"
        counts["labels"][label] = counts["labels"].get(label, 0) + 1
    newest = None
    if rows:
        payload = json.loads(rows[0]["payload_json"])
        newest = {
            "ts_ms": rows[0]["ts_ms"],
            "label": payload.get("label"),
            "conf": payload.get("conf"),
            "low": bool(payload.get("low")),
        }
    return counts, newest


@app.get("/api/infer/nodes")
def infer_nodes():
    """One summary per inference node for tools/infer_nodes.html."""
    now_ms = int(time.time() * 1000)
    since_ms = now_ms - NODES_WINDOW_S * 1000
    nodes = []
    with database() as connection:
        devices = connection.execute(
            """
            SELECT device_id, updated_ms, dropped, pending_audio, pending_video
            FROM infer_devices ORDER BY device_id
            """
        ).fetchall()
        heartbeats = {
            row["device_id"]: row
            for row in connection.execute(
                """
                SELECT h.device_id, h.received_epoch, h.payload_json
                FROM heartbeats h
                JOIN (SELECT device_id, MAX(id) AS last_id FROM heartbeats GROUP BY device_id) m
                  ON h.id = m.last_id
                """
            ).fetchall()
        }
        samples = {
            row["device_id"]: row
            for row in connection.execute(
                """
                SELECT device_id, COUNT(*) AS total, MAX(received_ms) AS last_ms
                FROM infer_samples GROUP BY device_id
                """
            ).fetchall()
        }
        for device in devices:
            device_id = device["device_id"]
            heartbeat_row = heartbeats.get(device_id)
            heartbeat = json.loads(heartbeat_row["payload_json"]) if heartbeat_row else {}
            inference = heartbeat.get("inference") or {}
            audio_counts, audio_last = node_window_counts(connection, device_id, "audio", since_ms)
            roi_counts, roi_last = node_window_counts(connection, device_id, "roi", since_ms)

            # Role: the heartbeat says which pipeline is enabled; before the first
            # heartbeat fall back to which records the node has sent.
            if (inference.get("video") or {}).get("enabled") or (
                not inference and (roi_last or not audio_last)
            ):
                role = "video"
            else:
                role = "audio"
            counts, last = (roi_counts, roi_last) if role == "video" else (audio_counts, audio_last)

            pipeline = {}
            if role == "video":
                session_row = connection.execute(
                    """
                    SELECT payload_json FROM infer_results
                    WHERE device_id = ? AND kind = 'session' ORDER BY id DESC LIMIT 1
                    """,
                    (device_id,),
                ).fetchone()
                if session_row:
                    session = json.loads(session_row["payload_json"])
                    pipeline = {
                        key: session.get(key)
                        for key in ("frames", "dropped", "gaps", "incomplete", "rois_expired")
                    }
            else:
                audio = inference.get("audio") or {}
                pipeline = {
                    "windows": audio.get("windows"),
                    "dropped": audio.get("dropped"),
                    "overflows": audio.get("overflows"),
                }

            wifi = heartbeat.get("wifi") or {}
            memory = heartbeat.get("memory") or {}
            storage = heartbeat.get("storage") or {}
            # Boot-time counters of the enabled pipeline, as of the last heartbeat.
            counter_keys = (
                ("frames", "dropped", "rois", "low_conf", "saved")
                if role == "video"
                else ("windows", "dropped", "low_conf", "saved", "overflows")
            )
            role_counters = inference.get(role) or {}
            last_upload = heartbeat.get("last_upload")
            sample_row = samples.get(device_id)
            nodes.append(
                {
                    "device_id": device_id,
                    "role": role,
                    "updated_ms": device["updated_ms"],
                    "heartbeat_ms": heartbeat_row["received_epoch"] * 1000 if heartbeat_row else None,
                    "state": heartbeat.get("state"),
                    "uptime_s": heartbeat.get("uptime_s"),
                    "reset_reason": heartbeat.get("reset_reason"),
                    "rssi": wifi.get("rssi"),
                    "disconnects": wifi.get("disconnects"),
                    "wifi_restarts": wifi.get("restarts"),
                    "wifi_last_reason": wifi.get("last_reason"),
                    "heartbeat_fail_streak": wifi.get("heartbeat_fail_streak"),
                    "heap_min_kb": memory.get("heap_min_kb"),
                    "psram_free_kb": memory.get("psram_free_kb"),
                    "sd_free_mb": storage.get("free_mb"),
                    "pending": device["pending_video"] if role == "video" else device["pending_audio"],
                    "upload_after": inference.get("upload_after"),
                    "report_dropped": device["dropped"],
                    "report_sent": inference.get("report_sent"),
                    "counters": {key: role_counters.get(key) for key in counter_keys},
                    "last_upload": last_upload if isinstance(last_upload, dict) else None,
                    "window": counts,
                    "last": last,
                    "pipeline": pipeline,
                    "samples_received": sample_row["total"] if sample_row else 0,
                    "samples_last_ms": sample_row["last_ms"] if sample_row else None,
                }
            )
    return jsonify(
        {
            "now_ms": now_ms,
            "window_s": NODES_WINDOW_S,
            "offline_after_s": OFFLINE_AFTER_S,
            "nodes": nodes,
        }
    )


@app.get("/api/infer/samples")
def infer_samples_list():
    try:
        limit = min(max(int(request.args.get("limit", "24")), 1), 200)
        offset = max(int(request.args.get("offset", "0")), 0)
    except ValueError:
        return error("limit and offset must be integers", 400)
    conditions = []
    values: list = []
    for column, argument in (("device_id", "device"), ("media", "media")):
        value = request.args.get(argument)
        if value:
            conditions.append(f"{column} = ?")
            values.append(value)
    where = f"WHERE {' AND '.join(conditions)}" if conditions else ""
    with database() as connection:
        rows = connection.execute(
            f"SELECT * FROM infer_samples {where} ORDER BY ts_ms DESC LIMIT ? OFFSET ?",
            (*values, limit, offset),
        ).fetchall()
        total = connection.execute(
            f"SELECT COUNT(*) AS total FROM infer_samples {where}", values
        ).fetchone()["total"]
    return jsonify({"total": total, "samples": [sample_row_to_json(row) for row in rows]})


def load_sample(sample_id: str):
    if not VALID_ID.fullmatch(sample_id):
        return None
    with database() as connection:
        row = connection.execute(
            "SELECT * FROM infer_samples WHERE sample_id = ?", (sample_id,)
        ).fetchone()
    if row is None or not Path(row["data_path"]).is_file():
        return None
    return row


@app.get("/api/infer/sample/<sample_id>/frame")
def infer_sample_frame(sample_id: str):
    """The sample's full frame as an image a browser can show: PNG or JPEG."""
    row = load_sample(sample_id)
    if row is None or row["media"] != "video":
        return error("video sample not found", 404)
    meta = json.loads(row["meta_json"])
    with open(row["data_path"], "rb") as source:
        image = source.read(sample_image_bytes(meta))
    if meta.get("format") == "JPEG":
        return Response(image, mimetype="image/jpeg")
    return Response(gray_png(int(meta["width"]), int(meta["height"]), image), mimetype="image/png")


@app.get("/api/infer/sample/<sample_id>/roi/<int:index>.png")
def infer_sample_roi(sample_id: str, index: int):
    row = load_sample(sample_id)
    if row is None or row["media"] != "video":
        return error("video sample not found", 404)
    meta = json.loads(row["meta_json"])
    size = int(meta["roi_size"])
    if not 0 <= index < int(meta["roi_count"]):
        return error("no such ROI", 404)
    with open(row["data_path"], "rb") as source:
        source.seek(sample_image_bytes(meta) + index * size * size)
        pixels = source.read(size * size)
    return Response(gray_png(size, size, pixels), mimetype="image/png")


@app.get("/api/infer/sample/<sample_id>/data")
def infer_sample_data(sample_id: str):
    """The sample's raw file: WAV for audio (playable), frame + ROI bytes for video."""
    row = load_sample(sample_id)
    if row is None:
        return error("sample not found", 404)
    path = Path(row["data_path"])
    mimetype = "audio/wav" if row["media"] == "audio" else "application/octet-stream"
    return send_file(path, mimetype=mimetype, download_name=path.name)


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


def frame_is_suspect(row_jump: float, col_jump: float) -> bool:
    low, high = sorted((row_jump, col_jump))
    return high >= JUMP_MIN and high > low * JUMP_RATIO


class VideoChecker:
    """Checks received RAW files on one background thread, so neither the ACK
    nor the next upload waits for numpy/OpenCV."""

    def __init__(self) -> None:
        self._queue: queue.Queue = queue.Queue()
        self._raw_view = None
        self.save_sheets = False

    def start(self, save_sheets: bool) -> None:
        try:
            spec = importlib.util.spec_from_file_location("raw_view", RAW_VIEW_PATH)
            raw_view = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(raw_view)   # imports numpy and cv2
        except (ImportError, OSError) as exc:
            print(f"[{now_text()}] WARNING video check disabled: {exc} "
                  f"(needs {RAW_VIEW_PATH.name}, numpy and opencv-python)")
            return
        self._raw_view = raw_view
        self.save_sheets = save_sheets
        threading.Thread(target=self._run, daemon=True).start()
        print(f"[{now_text()}] video check on: suspect if jump ratio > {JUMP_RATIO}"
              f"{', contact sheets on' if save_sheets else ''}")

    def submit(self, job: dict) -> None:
        if self._raw_view is not None:
            self._queue.put(job)

    def _run(self) -> None:
        while True:
            job = self._queue.get()
            try:
                self._check(job)
            except Exception as exc:   # one bad file must not stop the checker
                print(f"[{now_text()}] WARNING video check failed for "
                      f"{job['capture_id']}: {exc}")

    def _check(self, job: dict) -> None:
        raw_view = self._raw_view
        np = raw_view.np
        frames = raw_view.load_frames(job["path"], job["width"], job["height"])
        if len(frames) == 0:
            return

        # Same numbers as raw_view.py --stats.
        jumps = []
        for frame in frames:
            f = frame.astype(np.int16)
            jumps.append((float(np.abs(np.diff(f, axis=0)).mean()),
                          float(np.abs(np.diff(f, axis=1)).mean())))
        suspect = [index for index, (row_jump, col_jump) in enumerate(jumps)
                   if frame_is_suspect(row_jump, col_jump)]
        max_ratio = max(max(r, c) / max(min(r, c), 0.01) for r, c in jumps)
        median_row = float(np.median([r for r, _ in jumps]))
        median_col = float(np.median([c for _, c in jumps]))

        sheet_path = None
        if self.save_sheets:
            stamp = datetime.fromisoformat(job["received_at"]).strftime("%Y_%m_%d_%H%M%S")
            sheet_dir = job["path"].parent / "sheets"
            sheet_dir.mkdir(exist_ok=True)
            sheet_path = sheet_dir / f"{stamp}.png"
            counter = 1
            while sheet_path.exists():
                sheet_path = sheet_dir / f"{stamp}_{counter}.png"
                counter += 1
            raw_view.cv2.imwrite(str(sheet_path), raw_view.contact_sheet(frames))

        with database() as connection:
            connection.execute(
                """
                INSERT OR REPLACE INTO video_checks
                    (capture_id, device_id, recorded_at, received_at, checked_at,
                     frames, suspect_frames, suspect_list, max_ratio,
                     median_row_jump, median_col_jump, sheet_path)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    job["capture_id"],
                    job["device_id"],
                    job["recorded_at"],
                    job["received_at"],
                    now_text(),
                    len(frames),
                    len(suspect),
                    json.dumps(suspect),
                    round(max_ratio, 2),
                    round(median_row, 2),
                    round(median_col, 2),
                    str(sheet_path) if sheet_path else None,
                ),
            )

        sheet_note = f" sheet={sheet_path}" if sheet_path else ""
        if suspect:
            shown = ",".join(str(index) for index in suspect[:20])
            more = f",...(+{len(suspect) - 20})" if len(suspect) > 20 else ""
            print(f"[{now_text()}] WARNING video check {job['capture_id']}: "
                  f"{len(suspect)}/{len(frames)} suspect frames [{shown}{more}] "
                  f"max_ratio={max_ratio:.1f} median row/col={median_row:.1f}/{median_col:.1f}"
                  f"{sheet_note}")
        else:
            print(f"[{now_text()}] video check {job['capture_id']}: ok "
                  f"({len(frames)} frames, max_ratio={max_ratio:.1f}){sheet_note}")


offline_watch = OfflineWatch()
video_checker = VideoChecker()
initialize_database()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Receiving server for the DID field node.")
    parser.add_argument("--sheet", action="store_true",
                        help="save a contact sheet PNG of every received RAW, "
                             "named by its receive time, in <date dir>/sheets/")
    args = parser.parse_args()
    threading.Thread(target=offline_watch.run, daemon=True).start()
    threading.Thread(target=prune_frame_records, daemon=True).start()
    video_checker.start(save_sheets=args.sheet)
    print(f"[{now_text()}] dashboard: http://localhost:5000/")
    app.run(host="0.0.0.0", port=5000, threaded=True)
