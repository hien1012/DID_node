"""Pretend to be an infer_node board, to try server_mix.py and its dashboard without hardware.

Sends the same requests as the firmware: results to /infer/results every 2 s
(audio) and frame / ROI records in short camera sessions, plus one batch of
synthetic low-confidence samples to /infer/sample. All values are random.

    python server_mix.py                (in another terminal)
    python tools/infer_fake_node.py     then open http://localhost:5000/

Everything goes into the server's real database under the device ID
node-s3-FAKE00000001, so do not point it at a server holding data you care about
unless FIELD_MIX_DATA_ROOT was set to a scratch folder when that server started.
"""

import argparse
import json
import math
import random
import struct
import time
import urllib.error
import urllib.request

AUDIO_LABELS = ["background", "drone", "helicopter"]
VIDEO_LABELS = ["drone", "bird", "background"]
WIDTH, HEIGHT, ROI = 640, 480, 64


def post(url: str, body: bytes, headers: dict) -> dict:
    request = urllib.request.Request(url, data=body, headers=headers, method="POST")
    with urllib.request.urlopen(request, timeout=10) as response:
        return json.loads(response.read())


def random_probs() -> tuple[list[float], int]:
    raw = [random.random() ** 3 for _ in range(3)]
    total = sum(raw) or 1.0
    probs = [round(v / total, 4) for v in raw]
    return probs, probs.index(max(probs))


def fake_frame(seed: int) -> tuple[bytes, list[int]]:
    """Gradient frame with one bright square that moves with seed."""
    x0 = 80 + (seed * 37) % 420
    y0 = 60 + (seed * 23) % 300
    size = 72
    rows = bytearray()
    for y in range(HEIGHT):
        base = 40 + y // 8
        row = bytearray([base] * WIDTH)
        if y0 <= y < y0 + size:
            row[x0:x0 + size] = bytes([220]) * size
        rows += row
    return bytes(rows), [x0, y0, x0 + size - 1, y0 + size - 1]


def fake_wav(seconds: float = 2.0, rate: int = 16000) -> bytes:
    count = int(seconds * rate)
    frequency = random.choice([180, 420, 900])
    samples = (
        int(9000 * math.sin(2 * math.pi * frequency * i / rate) + random.gauss(0, 1200))
        for i in range(count)
    )
    data = b"".join(struct.pack("<h", max(-32768, min(32767, s))) for s in samples)
    header = (
        b"RIFF" + struct.pack("<I", 36 + len(data)) + b"WAVEfmt "
        + struct.pack("<IHHIIHH", 16, 1, 1, rate, rate * 2, 2, 16)
        + b"data" + struct.pack("<I", len(data))
    )
    return header + data


def send_sample(base: str, device: str, media: str, index: int) -> None:
    now_ms = int(time.time() * 1000) - (40 - index) * 3000
    stem = f"{time.strftime('%Y%m%d_%H%M%S', time.localtime(now_ms / 1000))}_{index:05d}"
    if media == "audio":
        probs, pred = random_probs()
        meta = {
            "media": "audio", "window": index, "start_ms": now_ms, "end_ms": now_ms + 2000,
            "sample_rate": 16000, "samples": 32000, "gain": 30, "threshold": 0.5,
            "queue_us": 900, "feature_us": 68000, "invoke_us": 215000, "write_us": 41000,
            "overflows": 0, "pred": pred, "label": AUDIO_LABELS[pred], "conf": probs[pred],
            "labels": AUDIO_LABELS, "probs": probs,
        }
        data = fake_wav()
    else:
        frame, box = fake_frame(index)
        probs, pred = random_probs()
        meta = {
            "media": "video", "session": now_ms // 1000, "frame": index, "ts_ms": now_ms,
            "width": WIDTH, "height": HEIGHT, "format": "GRAY8", "roi_size": ROI, "roi_count": 1,
            "threshold": 0.5, "queue_us": 300, "downscale_us": 4000, "fd_us": 21000,
            "track_us": 80, "write_us": 96000, "labels": VIDEO_LABELS,
            "rois": [{
                "track": index, "bbox": box, "invoke_us": 98000, "result_us": 310000, "pred": pred,
                "label": VIDEO_LABELS[pred], "conf": probs[pred], "low": True, "probs": probs,
            }],
        }
        data = frame + bytes(random.randrange(256) for _ in range(ROI * ROI))
    meta_bytes = json.dumps(meta).encode()
    ack = post(
        f"{base}/infer/sample", meta_bytes + data,
        {
            "Content-Type": "application/octet-stream",
            "X-Device-ID": device,
            "X-Sample-ID": f"{device}-{media[0]}{stem}",
            "X-Media": media,
            "X-Meta-Length": str(len(meta_bytes)),
            "X-File-Size": str(len(data)),
        },
    )
    assert ack["status"] == "ok" and ack["bytes"] == len(data), ack


def send_records(base: str, device: str, records: list[dict], pending: dict) -> None:
    body = {
        "device_id": device, "time_ms": int(time.time() * 1000), "dropped": 0,
        "pending": pending, "records": records,
    }
    post(f"{base}/infer/results", json.dumps(body).encode(), {"Content-Type": "application/json"})


def camera_session(session: int, frames: int) -> list[dict]:
    records = []
    for frame in range(frames):
        ts_ms = session * 1000 + frame * 200
        has_roi = frame % 11 == 3
        fd_parts = [random.randint(3000, 6000) for _ in range(2)] + [
            random.randint(4000, 12000), random.randint(1000, 8000), random.randint(50, 400)
        ]
        downscale = random.randint(3000, 6000)
        fd = sum(fd_parts) + random.randint(20, 80)
        track = random.randint(40, 200)
        crop = random.randint(800, 1500) if has_roi else 0
        cnn = random.randint(90000, 130000) if has_roi else 0
        roi = crop + cnn + (random.randint(2000, 9000) if has_roi else 0)
        detect = downscale + fd + track + random.randint(50, 300)
        records.append({
            "t": "frame", "session": session, "frame": frame, "ts_ms": ts_ms,
            "capture_us": random.randint(90000, 110000), "queue_us": random.randint(100, 900),
            "downscale_us": downscale, "fd_us": fd,
            "fd_diff_us": fd_parts[0], "fd_morph_us": fd_parts[1], "fd_contour_us": fd_parts[2],
            "fd_refine_us": fd_parts[3], "fd_nms_us": fd_parts[4], "track_us": track,
            "detect_us": detect, "cnn_wait_us": random.randint(100, 900) if has_roi else 0,
            "roi_us": roi, "crop_us": crop, "cnn_us": cnn,
            "total_us": detect + roi,
            "boxes": 1 if has_roi else 0, "candidates": 1 if has_roi else 0,
        })
        if has_roi:
            probs, pred = random_probs()
            records.append({
                "t": "roi", "session": session, "frame": frame, "ts_ms": ts_ms, "track": frame,
                "bbox": [100, 80, 171, 151], "invoke_us": random.randint(85000, 120000),
                "result_us": random.randint(280000, 420000),
                "pred": pred, "label": VIDEO_LABELS[pred], "conf": probs[pred],
                "low": probs[pred] < 0.5, "probs": probs,
            })
    records.append({
        "t": "session", "session": session, "frames": frames, "processed": frames, "dropped": 0,
        "gaps": 0, "incomplete": 0, "rois": frames // 11, "rois_expired": 0, "low_conf": 1, "saved": 1,
        "save_skipped": 0, "first_frame_offset_us": 41000, "interval_min_us": 199900,
        "interval_max_us": 200050, "error": 0, "save_wait_avg_us": 300, "save_wait_max_us": 1200,
        "save_avg_us": 96000, "save_max_us": 180000,
    })
    records.append({
        "t": "cnn_ops", "session": session, "frame": frames, "invokes": frames // 11,
        "ops": [["QUANTIZE", 900], ["DEPTHWISE_CONV_2D", 11000], ["CONV_2D", 16000], ["MAX_POOL_2D", 3000],
                ["FULLY_CONNECTED", 25000], ["SOFTMAX", 200]],
    })
    records.append({
        "t": "task_usage", "media": "video", "ts_ms": session * 1000 + frames, "inferences": frames // 11,
        "avg_wall_us": 110000,
        "tasks": [["cam_infer", 0, 92000], ["IDLE1", 1, 60000], ["cam_detect", 1, 24000], ["IDLE0", 0, 9000], ["cam_task", 1, 20000],
                  ["wifi", 0, 3200], ["result_report", -1, 800], ["tiT", -1, 500], ["camera_capture", 1, 90],
                  ["heartbeat", -1, 0]],
    })
    return records


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--server", default="http://localhost:5000")
    parser.add_argument("--device", default="node-s3-FAKE00000001")
    parser.add_argument("--windows", type=int, default=0, help="stop after N audio windows (0 = forever)")
    parser.add_argument("--no-samples", action="store_true")
    args = parser.parse_args()

    if not args.no_samples:
        for index in range(20):
            send_sample(args.server, args.device, "audio", index)
            send_sample(args.server, args.device, "video", index)
        print("sent 20 audio + 20 video samples")

    window = 0
    pending = {"audio": 0, "video": 0}
    while args.windows == 0 or window < args.windows:
        start_ms = int(time.time() * 1000)
        probs, pred = random_probs()
        low = probs[pred] < 0.5
        pending["audio"] = (pending["audio"] + 1) % 20 if low else pending["audio"]
        records = [{
            "t": "audio", "window": window, "start_ms": start_ms, "end_ms": start_ms + 2000,
            "queue_us": random.randint(200, 2000), "feature_us": random.randint(55000, 90000),
            "invoke_us": random.randint(180000, 260000), "overflows": 0, "dropped": 0,
            "pred": pred, "label": AUDIO_LABELS[pred], "conf": probs[pred], "low": low,
            "probs": probs,
        }]
        if window % 10 == 0:
            records += camera_session(int(time.time()), 100)
            pending["video"] = (pending["video"] + 1) % 20
        send_records(args.server, args.device, records, pending)
        window += 1
        print(f"window {window}: {AUDIO_LABELS[pred]} {probs[pred]:.3f}" + (" LOW" if low else ""))
        if args.windows == 0 or window < args.windows:
            time.sleep(2)


if __name__ == "__main__":
    try:
        main()
    except urllib.error.URLError as exc:
        raise SystemExit(f"cannot reach the server: {exc}")
