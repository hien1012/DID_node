"""One background sheet renderer for the single-process, threaded Flask server."""

import json
import logging
from pathlib import Path
import threading


class RawSheetWorker:
    def __init__(self, database, timestamp, logger=None):
        self._database = database
        self._timestamp = timestamp
        self._logger = logger or logging.getLogger(__name__)
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._thread = None

    def start(self) -> None:
        """Start once; recover interrupted work and backfill existing video uploads."""
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return
            with self._database() as connection:
                # This server uses one process. Retry failed/interrupted jobs once
                # per startup, rather than continuously retrying a broken input.
                connection.execute(
                    "UPDATE sheet_jobs SET status = 'pending', last_error = NULL, "
                    "updated_at = ? WHERE status IN ('processing', 'failed')",
                    (self._timestamp(),),
                )
                connection.execute(
                    "INSERT OR IGNORE INTO sheet_jobs (capture_id, status, updated_at) "
                    "SELECT capture_id, 'pending', ? FROM uploads WHERE media_type = 'video'",
                    (self._timestamp(),),
                )
            self._stop.clear()
            self._thread = threading.Thread(
                target=self._run, name="raw-sheet-worker", daemon=True
            )
            self._thread.start()

    def notify(self) -> None:
        # SQLite is the durable queue; this event is just a wake-up hint, so no
        # recording data or unbounded list of pending jobs is kept in memory.
        self._wake.set()

    def stop(self, timeout: float = 5.0) -> None:
        """Allow a renderer to finish before releasing its files/database."""
        with self._lock:
            self._stop.set()
            self._wake.set()
            if self._thread is not None:
                self._thread.join(timeout)
                if self._thread.is_alive():
                    raise TimeoutError("sheet worker did not stop in time")
                self._thread = None

    def process_next(self) -> bool:
        """Claim one job briefly in SQLite; image I/O runs outside the transaction."""
        with self._database() as connection:
            connection.execute("BEGIN IMMEDIATE")
            job = connection.execute(
                """
                SELECT j.capture_id, u.file_path, u.file_bytes, u.metadata_json
                FROM sheet_jobs j JOIN uploads u ON u.capture_id = j.capture_id
                WHERE j.status = 'pending' AND u.media_type = 'video'
                ORDER BY u.recorded_at, j.capture_id LIMIT 1
                """
            ).fetchone()
            if job is None:
                return False
            connection.execute(
                "UPDATE sheet_jobs SET status = 'processing', updated_at = ? "
                "WHERE capture_id = ?",
                (self._timestamp(), job["capture_id"]),
            )

        status, sheet_path, last_error = "ready", None, None
        try:
            # Optional image dependencies are loaded in the worker. Missing
            # OpenCV/NumPy must not turn a successfully stored upload into failure.
            from .raw_view import save_contact_sheet

            metadata = json.loads(job["metadata_json"])
            if metadata.get("format") != "GRAY8":
                raise ValueError("sheet generation requires GRAY8")
            raw_path = Path(job["file_path"])
            if raw_path.stat().st_size != job["file_bytes"]:
                raise ValueError("stored RAW size differs from upload record")
            result = save_contact_sheet(
                raw_path,
                raw_path.with_suffix(".sheet.png"),
                width=int(metadata["width"]),
                height=int(metadata["height"]),
                expected_frames=int(metadata["frames"]),
            )
            sheet_path = str(result)
        except Exception as exc:
            status = "failed"
            last_error = f"{type(exc).__name__}: {exc}"[:1000]
            self._logger.warning("Sheet failed: %s: %s", job["capture_id"], last_error)

        with self._database() as connection:
            connection.execute(
                "UPDATE sheet_jobs SET status = ?, sheet_path = ?, last_error = ?, "
                "updated_at = ? WHERE capture_id = ?",
                (status, sheet_path, last_error, self._timestamp(), job["capture_id"]),
            )
        if status == "ready":
            self._logger.info("Sheet ready: %s", sheet_path)
        return True

    def _run(self) -> None:
        while not self._stop.is_set():
            self._wake.clear()
            try:
                if self.process_next():
                    continue
            except Exception:
                # A transient DB/filesystem problem cannot terminate the worker
                # silently. A processing job is recovered on the next startup.
                self._logger.exception("Sheet worker error")
            self._wake.wait(5.0)
