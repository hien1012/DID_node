"""Receive the node's debug camera stream (PC side, main/cam_stream.c).

The node connects to this PC and sends one packet per captured frame,
straight from the camera frame buffer (no SD card, no HTTP):
    24-byte header, little-endian:
        "CAMF" | u32 frame index | u16 width | u16 height | u32 length | i64 timestamp_us
    Magic "CAMS" instead of "CAMF": captured while the node was writing the SD
    card in the background (APP_CAM_STREAM_SD_LOAD A/B test); counted separately.
    then `length` bytes of GRAY8 (Y), row by row.

If these frames are corrupted too, the damage happens at capture time.
If they are clean, the recording path (SD writes) is the suspect.

Usage:
    python tools/cam_stream_view.py                       show + per-frame stats
    python tools/cam_stream_view.py --save stream.raw     also append frames to a .raw
                                                          (open later with raw_view.py)
    python tools/cam_stream_view.py --save-bad bad_frames save suspect frames as PNG
    python tools/cam_stream_view.py --no-window           stats only
    python tools/cam_stream_view.py --latest latest.png   newest frame as PNG (no GUI needed)

Firmware side: APP_CAM_STREAM_ENABLE / _HOST / _PORT in main/app_config.h.
Allow TCP port 5002 through the Windows firewall.
Needs numpy + opencv-python (pip install numpy opencv-python).
"""

from __future__ import annotations

import argparse
import socket
import struct
from pathlib import Path

import cv2
import numpy as np

HEADER = struct.Struct("<4sIHHIq")
MAGIC = b"CAMF"
MAGIC_SD_LOAD = b"CAMS"   # captured while the node was writing the SD card


def recv_exact(conn: socket.socket, size: int) -> bytes | None:
    buffer = bytearray(size)
    view = memoryview(buffer)
    received = 0
    while received < size:
        count = conn.recv_into(view[received:], size - received)
        if count == 0:
            return None
        received += count
    return bytes(buffer)


def frame_stats(frame: np.ndarray) -> tuple[float, float, float]:
    """Same numbers as raw_view.py --stats. Byte misalignment (Y/UV phase
    flips) breaks rows apart, so row_jump >> col_jump on a corrupted frame."""
    f = frame.astype(np.int16)
    row_jump = float(np.abs(np.diff(f, axis=0)).mean())
    col_jump = float(np.abs(np.diff(f, axis=1)).mean())
    return row_jump, col_jump, row_jump / col_jump if col_jump else 0.0


def serve(args: argparse.Namespace) -> None:
    raw_out = open(args.save, "ab") if args.save else None
    if args.save_bad:
        args.save_bad.mkdir(parents=True, exist_ok=True)
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(("0.0.0.0", args.port))
    listener.listen(1)
    print(f"listening on port {args.port}; waiting for the node...")

    # Counts per SD-load state: False = "CAMF" (no SD writes), True = "CAMS".
    total = {False: 0, True: 0}
    bad = {False: 0, True: 0}
    window_ok = not args.no_window
    try:
        while True:
            conn, address = listener.accept()
            print(f"connected: {address[0]}")
            expected_index = None
            previous_ts = None
            with conn:
                while True:
                    raw_header = recv_exact(conn, HEADER.size)
                    if raw_header is None:
                        print("node disconnected")
                        break
                    magic, index, width, height, length, ts_us = HEADER.unpack(raw_header)
                    if magic not in (MAGIC, MAGIC_SD_LOAD) or length != width * height:
                        print(f"bad header {raw_header.hex()}; dropping connection")
                        break
                    data = recv_exact(conn, length)
                    if data is None:
                        print("node disconnected mid-frame")
                        break

                    frame = np.frombuffer(data, dtype=np.uint8).reshape(height, width)
                    row_jump, col_jump, ratio = frame_stats(frame)
                    # High ratio: rows broken apart. Very low ratio: the whole frame
                    # is U/V (byte phase flipped from the start), e.g. 0.36 at mean ~128.
                    suspect = ratio > args.bad_ratio or ratio < args.low_ratio
                    sd_load = magic == MAGIC_SD_LOAD
                    total[sd_load] += 1
                    bad[sd_load] += suspect
                    skipped = "" if expected_index in (None, index) else \
                        f"  (skipped {index - expected_index})"
                    gap = "" if previous_ts is None else f"  dt={(ts_us - previous_ts) / 1000:.0f}ms"
                    print(f"frame {index:5d} {'SD ' if sd_load else '-- '} mean {frame.mean():5.1f}  "
                          f"row_jump {row_jump:5.1f}  col_jump {col_jump:5.1f}  ratio {ratio:4.2f}"
                          f"{'  <-- SUSPECT' if suspect else ''}{gap}{skipped}"
                          f"   [no SD {bad[False]}/{total[False]} | SD {bad[True]}/{total[True]}]")
                    expected_index = index + 1
                    previous_ts = ts_us

                    if raw_out:
                        raw_out.write(data)
                        raw_out.flush()
                    if suspect and args.save_bad:
                        cv2.imwrite(str(args.save_bad / f"frame_{index:05d}.png"), frame)
                    view = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                    cv2.putText(view, f"{index}{'  SD' if sd_load else ''}  ratio {ratio:.2f}", (8, 28),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                (0, 0, 255) if suspect else (0, 255, 0), 2)
                    if args.latest:
                        cv2.imwrite(str(args.latest), view)
                    if window_ok:
                        try:
                            cv2.imshow("cam stream", view)
                            if cv2.waitKey(1) & 0xFF == ord("q"):
                                return
                        except cv2.error:
                            # opencv-python-headless (or a GUI-less build): keep receiving.
                            window_ok = False
                            print("no OpenCV GUI; continuing without a window. For a window: "
                                  "python -m pip uninstall -y opencv-python-headless, then "
                                  "python -m pip install opencv-python. Or use --latest latest.png")
    except KeyboardInterrupt:
        pass
    finally:
        print(f"suspect (ratio > {args.bad_ratio} or < {args.low_ratio}): "
              f"no SD load {bad[False]}/{total[False]}, SD load {bad[True]}/{total[True]}")
        listener.close()
        if raw_out:
            raw_out.close()
        if window_ok:
            cv2.destroyAllWindows()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", type=int, default=5002)
    parser.add_argument("--save", type=Path, help="append received frames to this .raw file")
    parser.add_argument("--save-bad", type=Path, help="directory for PNGs of suspect frames")
    parser.add_argument("--bad-ratio", type=float, default=1.5,
                        help="row_jump / col_jump above this marks a frame as suspect")
    parser.add_argument("--low-ratio", type=float, default=0.6,
                        help="ratio below this also marks a frame as suspect (whole frame U/V)")
    parser.add_argument("--no-window", action="store_true")
    parser.add_argument("--latest", type=Path,
                        help="overwrite this PNG with every frame (view it without an OpenCV window)")
    serve(parser.parse_args())


if __name__ == "__main__":
    main()
