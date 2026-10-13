"""Measure how fast server_mix.py can receive an upload, without the ESP32.

Sends a fake GRAY8 RAW recording to /upload/video exactly like the node does
(same headers, raw body, 16 KB writes) and reports the time spent sending the
body versus waiting for the JSON ACK.

    python tools/bench_upload.py                       # 92 MB to 127.0.0.1:5000
    python tools/bench_upload.py --frames 50 --runs 3  # 15 MB, three times
    python tools/bench_upload.py --url http://10.10.10.7:5000

Interpretation: if a local upload is also only a few hundred kB/s, the Flask
server is the bottleneck; if it reaches tens of MB/s, look at the ESP32 side.

Uploaded test files land in <FIELD_MIX_DATA_ROOT>/bench-pc/ and in the
uploads table of field_mix.db; delete them after testing.
"""

import argparse
import http.client
import json
import os
import sys
import time
from urllib.parse import urlparse

WIDTH = 640
HEIGHT = 480
FRAME_BYTES = WIDTH * HEIGHT
DEVICE_ID = "bench-pc"


def upload_once(url: str, frames: int, chunk_bytes: int, recorded_at: int) -> dict:
    parsed = urlparse(url)
    size = frames * FRAME_BYTES
    session_id = f"{DEVICE_ID}-{recorded_at}"
    headers = {
        "Content-Type": "application/octet-stream",
        "Content-Length": str(size),
        "X-Device-ID": DEVICE_ID,
        "X-Session-ID": session_id,
        "X-Capture-ID": f"{session_id}-video",
        "X-Recorded-At": str(recorded_at),
        "X-File-Size": str(size),
        "X-Frame-Count": str(frames),
        "X-Width": str(WIDTH),
        "X-Height": str(HEIGHT),
        "X-FPS": "5",
        "X-Format": "GRAY8",
    }
    # Random bytes so nothing on the path can shortcut a run of zeros.
    chunk = os.urandom(chunk_bytes)

    connection = http.client.HTTPConnection(parsed.hostname, parsed.port or 80, timeout=120)
    started = time.perf_counter()
    connection.putrequest("POST", "/upload/video", skip_accept_encoding=True)
    for name, value in headers.items():
        connection.putheader(name, value)
    connection.endheaders()

    sent = 0
    max_write_s = 0.0
    while sent < size:
        piece = chunk if size - sent >= chunk_bytes else chunk[: size - sent]
        write_started = time.perf_counter()
        connection.send(piece)
        max_write_s = max(max_write_s, time.perf_counter() - write_started)
        sent += len(piece)
    body_done = time.perf_counter()

    response = connection.getresponse()
    reply = response.read()
    finished = time.perf_counter()
    connection.close()

    try:
        ack = json.loads(reply)
    except ValueError:
        ack = {"raw": reply[:200].decode(errors="replace")}
    return {
        "status": response.status,
        "ack": ack,
        "bytes": size,
        "send_s": body_done - started,
        "ack_s": finished - body_done,
        "total_s": finished - started,
        "max_write_ms": max_write_s * 1000,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--url", default="http://127.0.0.1:5000",
                        help="server_mix base URL (default: %(default)s)")
    parser.add_argument("--frames", type=int, default=300,
                        help="frames per upload; 300 = one 60 s recording, 92 MB")
    parser.add_argument("--chunk-kb", type=int, default=16,
                        help="bytes per socket write, like the node (default: 16)")
    parser.add_argument("--runs", type=int, default=1)
    args = parser.parse_args()

    base_time = int(time.time())
    print(f"Uploading {args.runs} x {args.frames * FRAME_BYTES / 1e6:.1f} MB "
          f"to {args.url} in {args.chunk_kb} KB writes")
    rates = []
    for run in range(args.runs):
        # Distinct timestamps: the server names files by recording time.
        result = upload_once(args.url, args.frames, args.chunk_kb * 1024,
                             base_time + run)
        rate = result["bytes"] / result["total_s"] / 1e6
        rates.append(rate)
        print(f"run {run + 1}: HTTP {result['status']} total={result['total_s']:.2f} s "
              f"(send={result['send_s']:.2f} s, ack_wait={result['ack_s'] * 1000:.0f} ms, "
              f"max_write={result['max_write_ms']:.0f} ms) -> {rate:.2f} MB/s")
        if result["status"] != 200:
            print(f"  server reply: {result['ack']}")
            return 1
    print(f"average: {sum(rates) / len(rates):.2f} MB/s "
          f"(ESP32 node today: ~0.27-0.32 MB/s; iperf: ~7.5 MB/s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
