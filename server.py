import hashlib
import json
import os
import re
import sqlite3
import tempfile
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path

from flask import Flask, jsonify, request


TAIPEI_TZ = timezone(timedelta(hours=8))
DATA_ROOT = Path(os.environ.get("FIELD_DATA_ROOT", "D:/"))
DATABASE_PATH = DATA_ROOT / "field_data.db"
VALID_ID = re.compile(r"^[A-Za-z0-9_.-]{1,64}$")
MAX_AUDIO_BYTES = 64 * 1024 * 1024

app = Flask(__name__)
app.config["MAX_CONTENT_LENGTH"] = MAX_AUDIO_BYTES


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
                device_id TEXT NOT NULL,
                recorded_at INTEGER NOT NULL,
                received_at TEXT NOT NULL,
                file_path TEXT NOT NULL,
                file_bytes INTEGER NOT NULL,
                sha256 TEXT NOT NULL
            );
            CREATE TABLE IF NOT EXISTS heartbeats (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                device_id TEXT NOT NULL,
                received_at TEXT NOT NULL,
                reported_at INTEGER,
                payload_json TEXT NOT NULL
            );
            CREATE INDEX IF NOT EXISTS idx_heartbeats_device_received
                ON heartbeats(device_id, received_at);
            """
        )


def error(message: str, status_code: int):
    return jsonify({"status": "error", "message": message}), status_code


@app.get("/time")
def get_time():
    return jsonify({"epoch": int(time.time())})


@app.post("/upload/audio")
def upload_audio():
    device_id = request.headers.get("X-Device-ID", "")
    capture_id = request.headers.get("X-Capture-ID", "")
    recorded_text = request.headers.get("X-Recorded-At", "")
    expected_size_text = request.headers.get("X-File-Size", "")

    if not VALID_ID.fullmatch(device_id) or not VALID_ID.fullmatch(capture_id):
        return error("invalid device_id or capture_id", 400)
    try:
        recorded_at = int(recorded_text)
        expected_size = int(expected_size_text)
    except ValueError:
        return error("invalid recording time or file size", 400)
    if recorded_at < 1_700_000_000 or expected_size <= 44:
        return error("recording time or file size is out of range", 400)
    if expected_size > MAX_AUDIO_BYTES:
        return error("audio file is too large", 413)

    with database() as connection:
        existing = connection.execute(
            "SELECT * FROM uploads WHERE capture_id = ?", (capture_id,)
        ).fetchone()
    if existing is not None:
        existing_path = Path(existing["file_path"])
        if existing_path.is_file() and existing["file_bytes"] == expected_size:
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

    recorded_dt = datetime.fromtimestamp(recorded_at, TAIPEI_TZ)
    target_dir = (
        DATA_ROOT
        / recorded_dt.strftime("%Y-%m-%d")
        / "sound"
        / device_id
    )
    target_dir.mkdir(parents=True, exist_ok=True)
    target_path = target_dir / f"{recorded_dt.strftime('%Y%m%d_%H%M%S')}.wav"
    target_existed = target_path.exists()

    temp_path = None
    received_size = 0
    digest = hashlib.sha256()
    wav_prefix = bytearray()
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb", dir=target_dir, prefix=".upload-", suffix=".tmp", delete=False
        ) as output:
            temp_path = Path(output.name)
            while True:
                chunk = request.stream.read(64 * 1024)
                if not chunk:
                    break
                received_size += len(chunk)
                if received_size > expected_size:
                    return error("received more bytes than declared", 400)
                if len(wav_prefix) < 12:
                    wav_prefix.extend(chunk[: 12 - len(wav_prefix)])
                digest.update(chunk)
                output.write(chunk)
            output.flush()
            os.fsync(output.fileno())

        if received_size != expected_size:
            return error(
                f"size mismatch: expected {expected_size}, received {received_size}", 400
            )
        if wav_prefix[:4] != b"RIFF" or wav_prefix[8:12] != b"WAVE":
            return error("body is not a WAV file", 400)

        received_at = now_text()
        duplicate = False
        if target_existed:
            existing_size = target_path.stat().st_size
            existing_digest = sha256_file(target_path)
            if existing_size != received_size or existing_digest != digest.hexdigest():
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
                    (capture_id, device_id, recorded_at, received_at,
                     file_path, file_bytes, sha256)
                VALUES (?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    capture_id,
                    device_id,
                    recorded_at,
                    received_at,
                    str(target_path),
                    received_size,
                    digest.hexdigest(),
                ),
            )
        print(
            f"[{received_at}] audio saved: device={device_id} "
            f"capture={capture_id} path={target_path} bytes={received_size}"
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


@app.post("/heartbeat")
def heartbeat():
    payload = request.get_json(silent=True)
    if not isinstance(payload, dict):
        return error("JSON object required", 400)
    device_id = payload.get("device_id", "")
    if not isinstance(device_id, str) or not VALID_ID.fullmatch(device_id):
        return error("invalid device_id", 400)

    received_at = now_text()
    reported_at = payload.get("reported_at")
    with database() as connection:
        connection.execute(
            """
            INSERT INTO heartbeats
                (device_id, received_at, reported_at, payload_json)
            VALUES (?, ?, ?, ?)
            """,
            (
                device_id,
                received_at,
                int(reported_at) if isinstance(reported_at, (int, float)) else None,
                json.dumps(payload, ensure_ascii=False, separators=(",", ":")),
            ),
        )
    print(f"[{received_at}] heartbeat: device={device_id}")
    return jsonify(
        {"status": "ok", "device_id": device_id, "received_at": received_at}
    )


@app.get("/devices")
def devices():
    with database() as connection:
        rows = connection.execute(
            """
            SELECT h.device_id, h.received_at, h.reported_at, h.payload_json
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
                "received_at": row["received_at"],
                "reported_at": row["reported_at"],
                "status": json.loads(row["payload_json"]),
            }
            for row in rows
        ]
    )


@app.get("/uploads")
def uploads():
    try:
        limit = min(max(int(request.args.get("limit", "100")), 1), 1000)
    except ValueError:
        return error("limit must be an integer", 400)
    with database() as connection:
        rows = connection.execute(
            """
            SELECT capture_id, device_id, recorded_at, received_at,
                   file_path, file_bytes, sha256
            FROM uploads ORDER BY received_at DESC LIMIT ?
            """,
            (limit,),
        ).fetchall()
    return jsonify([dict(row) for row in rows])


initialize_database()

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
