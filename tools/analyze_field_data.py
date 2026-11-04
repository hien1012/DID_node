#!/usr/bin/env python3
"""分析 server_mix.py 收到的野外資料（cam 影像、mic 音訊、heartbeat）。

只要給 server 的資料夾（裡面有 field_mix.db 與 <device>/<date>/ 檔案）：
    python tools/analyze_field_data.py D:\\server_mix_data_3
    python tools/analyze_field_data.py D:\\server_mix_data_3 --db D:\\other\\field_mix.db --out D:\\report
    python tools/analyze_field_data.py D:\\server_mix_data_3 --since "2026-11-03 12:00" --until "2026-11-04 00:00"

檢查項目：
1. 漏錄：每台裝置、每種媒體，依週期（預設 300 s）列出應有的時間點，找出沒收到的，
   再用 heartbeat 推原因（錄影失敗類別 / 重開機 / 沒有 heartbeat / 仍在 pending）。
2. frame drop：檔案的幀數是否等於 duration × fps；heartbeat 的 faults 累計（開機重算）；
   影像內容：全白、全黑、整張同一亮度、和前一幀完全相同（凍結）、底部整排 0（半幀）。
   註：節點的嚴格檢核會直接刪掉丟幀的影片，所以 frame drop 多半表現為「漏錄」＋ heartbeat fault。
3. mic：clip（碰到 ±32767/-32768 的樣本數與連續段數）、DC offset、RMS（去 DC）、峰值、
   最長連續 0（斷訊）、最低幾個 bit 從沒用到（有效位元數）。
4. heartbeat：重開機與原因、回報中斷、RSSI、heap、SD 剩餘、pending、節點時鐘與 server 的差。

輸出（預設 <data>/analysis_<時間>/）：summary.txt（也印在螢幕）、*.csv、video_levels.png、audio_levels.png。
影像全部讀一遍要幾分鐘（每檔 30 MB），只看 db 可加 --no-video-scan / --no-audio-scan。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sqlite3
import statistics
import sys
import wave
from collections import Counter, defaultdict
from datetime import datetime
from pathlib import Path, PureWindowsPath

import numpy as np

FULL_SCALE = 32768.0
CLIP_HIGH, CLIP_LOW = 32767, -32768
# 旗標門檻（超過就在 summary 標出來）
DC_FLAG_PCT = 1.0            # |DC| > 1 % FS
QUIET_FLAG_DBFS = -70.0      # 去 DC 的 RMS 低於這個值，懷疑沒收到聲音
ZERO_RUN_FLAG_S = 0.05       # 連續 0 超過 50 ms，懷疑 I2S 斷訊
WHITE_MEAN, WHITE_SAT = 235.0, 0.90   # 幀平均 >= 235 或 >= 90 % 像素 >= 250 → 全白
BLACK_MEAN = 10.0
FLAT_STD = 2.0               # 整張幾乎同一個值（感測器沒輸出 / 全飽和）
HB_GAP_S = 300               # heartbeat 間隔超過這個值 = 中斷（S3 錄製+上傳期間會暫停 heartbeat）

RESET_REASONS = {
    0: "unknown", 1: "power_on", 2: "external", 3: "software", 4: "panic",
    5: "int_wdt", 6: "task_wdt", 7: "wdt", 8: "deepsleep", 9: "brownout", 10: "sdio",
}
FAULT_KEYS = ["frame_drop", "fb_incomplete", "frame_gap", "capture", "storage", "other"]

# dataviz 參考調色盤（與 analyze_outage.py 相同）：類別色固定順序
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
C_TEXT, C_MUTED, C_GRID = "#0b0b0b", "#52514e", "#e1e0d9"


def fmt_ts(epoch: float | None) -> str:
    if epoch is None:
        return "-"
    return datetime.fromtimestamp(epoch).strftime("%Y-%m-%d %H:%M:%S")


def parse_local(text: str) -> int:
    return int(datetime.fromisoformat(text).timestamp())


def parse_iso(text: str) -> int:
    return int(datetime.fromisoformat(text).timestamp())


def runs(mask: np.ndarray) -> np.ndarray:
    """回傳 mask 中每一段連續 True 的長度。"""
    if not mask.any():
        return np.zeros(0, dtype=np.int64)
    edges = np.diff(np.concatenate(([0], mask.astype(np.int8), [0])))
    return np.flatnonzero(edges == -1) - np.flatnonzero(edges == 1)


def compress_ranges(slots: list[int], period: int) -> list[tuple[int, int, int]]:
    """把連續的漏錄時間點合併成 (起, 迄, 個數)。"""
    out: list[tuple[int, int, int]] = []
    for s in slots:
        if out and s - out[-1][1] == period:
            out[-1] = (out[-1][0], s, out[-1][2] + 1)
        else:
            out.append((s, s, 1))
    return out


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8-sig")
        return
    keys: list[str] = []
    for row in rows:
        for k in row:
            if k not in keys:
                keys.append(k)
    with path.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


# ---------------------------------------------------------------------------
# 讀 db
# ---------------------------------------------------------------------------

def find_db(data_dir: Path, db_arg: str | None) -> Path:
    if db_arg:
        return Path(db_arg)
    default = data_dir / "field_mix.db"
    if default.exists():
        return default
    found = sorted(data_dir.glob("*.db"))
    if not found:
        sys.exit(f"找不到 db：{data_dir} 裡沒有 *.db，請用 --db 指定")
    return found[0]


def local_path(data_dir: Path, file_path: str) -> Path:
    """db 存的是 server 當時的絕對路徑；資料夾搬過也能找到（取最後 device/date/file 三層）。"""
    parts = PureWindowsPath(file_path).parts[-3:]
    return data_dir.joinpath(*parts)


def load_uploads(conn: sqlite3.Connection, data_dir: Path) -> list[dict]:
    rows = []
    for r in conn.execute("SELECT * FROM uploads ORDER BY device_id, media_type, recorded_at"):
        row = dict(r)
        row["meta"] = json.loads(row.get("metadata_json") or "{}")
        row["path"] = local_path(data_dir, row["file_path"])
        rows.append(row)
    return rows


def normalize_heartbeat(device: str, received_epoch: int, p: dict) -> dict:
    """legacy（平的欄位）與 S3 v1（巢狀）整理成同一種格式。"""
    wifi = p.get("wifi") or {}
    storage = p.get("storage") or {}
    memory = p.get("memory") or {}
    rec = p.get("last_recording") or {}
    reset = p.get("reset_reason")
    if isinstance(reset, int):
        reset = RESET_REASONS.get(reset, str(reset))
    sd_free_mb = storage.get("free_mb")
    if sd_free_mb is None and isinstance(p.get("sd_free_bytes"), (int, float)):
        sd_free_mb = p["sd_free_bytes"] / (1024 * 1024)
    heap_kb = memory.get("heap_free_kb")
    if heap_kb is None and isinstance(p.get("free_heap_bytes"), (int, float)):
        heap_kb = p["free_heap_bytes"] / 1024
    node_time = p.get("time", p.get("reported_at"))
    faults = p.get("faults") if isinstance(p.get("faults"), dict) else None
    return {
        "device": device,
        "received": received_epoch,
        "node_time": node_time,
        "uptime": p.get("uptime_s", p.get("uptime_seconds")),
        "reset_reason": reset,
        "rssi": wifi.get("rssi", p.get("wifi_rssi")),
        "heap_kb": heap_kb,
        "sd_free_mb": sd_free_mb,
        "pending": storage.get("pending_files", p.get("pending_files")),
        "broken": storage.get("broken_files"),
        "wifi_disconnects": wifi.get("disconnects"),
        "rec_ok": p.get("recordings_ok"),
        "rec_fail": p.get("recordings_failed"),
        "up_ok": p.get("uploads_ok"),
        "up_fail": p.get("uploads_failed"),
        "last_rec_at": rec.get("at", p.get("last_recorded_at")),
        "last_rec_error": p.get("last_record_error"),
        "last_fault": rec.get("camera_fault", p.get("last_fault")),
        "last_frames": rec.get("frames", p.get("last_saved_frames")),
        "last_dropped": rec.get("dropped", p.get("last_dropped_frames")),
        "last_gaps": p.get("last_frame_gaps"),
        "last_cam": rec.get("camera"),
        "last_mic": rec.get("mic"),
        "faults": faults,
    }


def load_heartbeats(conn: sqlite3.Connection) -> dict[str, list[dict]]:
    out: dict[str, list[dict]] = defaultdict(list)
    for r in conn.execute(
            "SELECT device_id, received_epoch, payload_json FROM heartbeats ORDER BY device_id, id"):
        try:
            payload = json.loads(r["payload_json"])
        except json.JSONDecodeError:
            continue
        out[r["device_id"]].append(normalize_heartbeat(r["device_id"], r["received_epoch"], payload))
    return out


# ---------------------------------------------------------------------------
# 檔案內容分析
# ---------------------------------------------------------------------------

def analyze_video(path: Path, width: int, height: int) -> dict:
    frame_bytes = width * height
    size = path.stat().st_size
    n = size // frame_bytes
    out = {"frames_in_file": n, "tail_bytes": size % frame_bytes}
    if n == 0:
        return out
    frames = np.fromfile(path, dtype=np.uint8, count=n * frame_bytes).reshape(n, height, width)
    sub = frames[:, ::2, ::2]  # 統計用 1/4 取樣就夠
    means = sub.mean(axis=(1, 2))
    stds = sub.std(axis=(1, 2))
    sat = (sub >= 250).mean(axis=(1, 2))
    white = (means >= WHITE_MEAN) | (sat >= WHITE_SAT)
    # 全黑幀（夜間整張 0）彼此必然相同，不算凍結
    dup = sum(1 for i in range(1, n)
              if means[i] > BLACK_MEAN and np.array_equal(frames[i], frames[i - 1]))
    zero_tail = sum(1 for i in range(n) if means[i] > 20 and not frames[i, -8:, :].any())
    out.update({
        "mean_brightness": round(float(means.mean()), 1),
        "min_frame_mean": round(float(means.min()), 1),
        "max_frame_mean": round(float(means.max()), 1),
        "saturated_pct": round(float(sat.mean()) * 100, 2),
        "white_frames": int(white.sum()),
        "black_frames": int((means <= BLACK_MEAN).sum()),
        "flat_frames": int((stds < FLAT_STD).sum()),
        "duplicate_frames": dup,
        "zero_tail_frames": zero_tail,
    })
    return out


def analyze_audio(path: Path) -> dict:
    with wave.open(str(path), "rb") as w:
        channels, width, rate, n = w.getnchannels(), w.getsampwidth(), w.getframerate(), w.getnframes()
        data = w.readframes(n)
    out = {"channels": channels, "sample_width": width, "wav_rate": rate, "samples": n}
    if width != 2 or n == 0:
        out["error"] = f"unsupported sample width {width}" if width != 2 else "empty"
        return out
    x = np.frombuffer(data, dtype="<i2")
    if channels > 1:
        x = x.reshape(-1, channels)[:, 0]
    xf = x.astype(np.float64)
    dc = float(xf.mean())
    rms_ac = math.sqrt(float(np.mean((xf - dc) ** 2)))
    peak = max(abs(int(x.min())), int(x.max()))
    clip = (x >= CLIP_HIGH) | (x <= CLIP_LOW)
    clip_runs = runs(clip)
    zero_runs = runs(x == 0)
    used_bits = int(np.bitwise_or.reduce(x.view(np.uint16)))
    unused_lsb = 16 if used_bits == 0 else (used_bits & -used_bits).bit_length() - 1
    out.update({
        "dc_lsb": round(dc, 1),
        "dc_pct_fs": round(dc / FULL_SCALE * 100, 3),
        "rms_dbfs": round(20 * math.log10(max(rms_ac, 1e-3) / FULL_SCALE), 1),
        "peak": peak,
        "peak_dbfs": round(20 * math.log10(max(peak, 1) / FULL_SCALE), 2),
        "clip_samples": int(clip.sum()),
        "clip_runs": int(len(clip_runs)),
        "longest_clip_run": int(clip_runs.max()) if len(clip_runs) else 0,
        "near_clip_samples": int((np.abs(xf) >= 0.99 * FULL_SCALE).sum()),
        "longest_zero_run_ms": round(float(zero_runs.max()) * 1000 / rate, 1) if len(zero_runs) else 0.0,
        "unused_lsb_bits": unused_lsb,
    })
    return out


# ---------------------------------------------------------------------------
# heartbeat 分析
# ---------------------------------------------------------------------------

def counter_increase(hbs: list[dict], key: str) -> int:
    """累計計數器的總增量；uptime 變小（重開機）時計數器歸零重算。"""
    total, prev, prev_up = 0, None, None
    for hb in hbs:
        value, up = hb.get(key), hb.get("uptime")
        if value is None:
            continue
        if prev is None or (up is not None and prev_up is not None and up < prev_up):
            total += value if prev is not None else 0
        elif value >= prev:
            total += value - prev
        prev, prev_up = value, up
    return total


def fault_increases(hbs: list[dict]) -> Counter:
    total: Counter = Counter()
    prev: dict | None = None
    prev_up = None
    for hb in hbs:
        faults, up = hb.get("faults"), hb.get("uptime")
        if faults is None:
            continue
        rebooted = prev is not None and up is not None and prev_up is not None and up < prev_up
        for k, v in faults.items():
            if not isinstance(v, (int, float)):
                continue
            if prev is None:
                continue
            total[k] += int(v) if rebooted else max(0, int(v) - int(prev.get(k, 0)))
        prev, prev_up = faults, up
    return total


def heartbeat_events(device: str, hbs: list[dict]) -> list[dict]:
    events = []
    prev = None
    for hb in hbs:
        if prev is not None:
            gap = hb["received"] - prev["received"]
            if gap > HB_GAP_S:
                events.append({"device": device, "time": fmt_ts(prev["received"]), "event": "heartbeat_gap",
                               "detail": f"{gap} s 沒有 heartbeat（到 {fmt_ts(hb['received'])}）"})
            if hb["uptime"] is not None and prev["uptime"] is not None and hb["uptime"] < prev["uptime"]:
                boot = hb["received"] - hb["uptime"]
                events.append({"device": device, "time": fmt_ts(boot), "event": "reboot",
                               "detail": f"reset_reason={hb['reset_reason']}，之前 uptime={prev['uptime']} s"})
            for key, label in (("rec_fail", "recordings_failed"), ("up_fail", "uploads_failed"),
                               ("wifi_disconnects", "wifi_disconnects")):
                a, b = prev.get(key), hb.get(key)
                if a is not None and b is not None and b > a:
                    events.append({"device": device, "time": fmt_ts(hb["received"]), "event": label,
                                   "detail": f"{a} → {b}"})
            pf, cf = prev.get("faults") or {}, hb.get("faults") or {}
            for k, v in cf.items():
                if isinstance(v, (int, float)) and v > pf.get(k, 0) and k in pf:
                    events.append({"device": device, "time": fmt_ts(hb["received"]), "event": f"fault:{k}",
                                   "detail": f"{pf.get(k, 0)} → {v}，last_fault={hb.get('last_fault')}"})
        prev = hb
    return events


def heartbeat_stats(hbs: list[dict]) -> dict:
    def series(key):
        return [hb[key] for hb in hbs if isinstance(hb.get(key), (int, float))]
    rssi, heap, sd, pending = series("rssi"), series("heap_kb"), series("sd_free_mb"), series("pending")
    offsets = [hb["received"] - hb["node_time"] for hb in hbs if isinstance(hb.get("node_time"), (int, float))]
    reboots = sum(1 for a, b in zip(hbs, hbs[1:])
                  if a["uptime"] is not None and b["uptime"] is not None and b["uptime"] < a["uptime"])
    return {
        "heartbeats": len(hbs),
        "first": fmt_ts(hbs[0]["received"]) if hbs else "-",
        "last": fmt_ts(hbs[-1]["received"]) if hbs else "-",
        "reboots": reboots,
        "reset_reason_first": hbs[0]["reset_reason"] if hbs else None,
        "rssi_min": min(rssi) if rssi else None,
        "rssi_median": statistics.median(rssi) if rssi else None,
        "heap_min_kb": round(min(heap), 1) if heap else None,
        "sd_free_mb_first": round(sd[0]) if sd else None,
        "sd_free_mb_last": round(sd[-1]) if sd else None,
        "pending_max": max(pending) if pending else None,
        "pending_last": pending[-1] if pending else None,
        # server 收到時間 - 節點時間（含網路延遲，秒解析度）
        "clock_offset_median_s": statistics.median(offsets) if offsets else None,
        "clock_offset_max_abs_s": max((abs(o) for o in offsets), default=None),
        "recordings_failed": counter_increase(hbs, "rec_fail"),
        "uploads_failed": counter_increase(hbs, "up_fail"),
    }


def session_status(hbs: list[dict]) -> dict[int, dict]:
    """heartbeat 的 last_recording / last_* 欄位 → 每個錄製時間點的結果。"""
    out: dict[int, dict] = {}
    for hb in hbs:
        at = hb.get("last_rec_at")
        if isinstance(at, (int, float)) and at > 0:
            out[int(at)] = hb
    return out


def missing_reason(slot: int, period: int, media: str, hbs: list[dict], sessions: dict[int, dict]) -> str:
    hb = sessions.get(slot)
    if hb is not None:
        parts = []
        if media == "video" and hb.get("last_cam") not in (None, "ok"):
            parts.append(f"camera={hb['last_cam']}")
        if media == "audio" and hb.get("last_mic") not in (None, "ok"):
            parts.append(f"mic={hb['last_mic']}")
        if media == "video" and hb.get("last_fault") not in (None, "none"):
            parts.append(f"fault={hb['last_fault']}")
        if hb.get("last_rec_error"):
            parts.append(f"err=0x{int(hb['last_rec_error']) & 0xFFFFFFFF:X}")
        if hb.get("last_frames") is not None and media == "video":
            parts.append(f"frames={hb['last_frames']} dropped={hb.get('last_dropped')} gaps={hb.get('last_gaps')}")
        if parts:
            return "錄製失敗：" + " ".join(parts)
        return "有錄（heartbeat 記到這次錄製且成功），但 server 沒收到：可能仍 pending 或上傳被拒"
    around = [h for h in hbs if slot - period <= h["received"] <= slot + period]
    if not around:
        return "前後 5 分鐘內沒有 heartbeat（節點可能沒電、當機或斷網）"
    for a, b in zip(hbs, hbs[1:]):
        if a["received"] <= slot + period and b["received"] >= slot and \
                b["uptime"] is not None and a["uptime"] is not None and b["uptime"] < a["uptime"]:
            return f"期間重開機（reset_reason={b['reset_reason']}）"
    return "heartbeat 沒有這次錄製的紀錄（沒有開始錄、或 legacy mic 不回報細節）"


# ---------------------------------------------------------------------------
# 圖
# ---------------------------------------------------------------------------

def plot_series(path: Path, title: str, panels: list[tuple[str, str]], rows: list[dict]) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.dates as mdates
        import matplotlib.pyplot as plt
    except ImportError:
        print("沒有 matplotlib，略過畫圖")
        return
    devices = sorted({r["device"] for r in rows})
    if not devices:
        return
    fig, axes = plt.subplots(len(panels), 1, figsize=(13, 2.6 * len(panels)), sharex=True, squeeze=False)
    for ax, (key, label) in zip(axes[:, 0], panels):
        for i, dev in enumerate(devices):
            pts = [(datetime.fromtimestamp(r["recorded_at"]), r[key]) for r in rows
                   if r["device"] == dev and isinstance(r.get(key), (int, float))]
            if pts:
                xs, ys = zip(*pts)
                ax.plot(xs, ys, color=SERIES[i % len(SERIES)], linewidth=1.2, marker="o",
                        markersize=2.5, label=dev)
        ax.set_ylabel(label, color=C_TEXT, fontsize=9)
        ax.grid(True, color=C_GRID, linewidth=0.6)
        for spine in ("top", "right"):
            ax.spines[spine].set_visible(False)
        ax.tick_params(colors=C_MUTED, labelsize=8)
    axes[0, 0].set_title(title, color=C_TEXT, fontsize=11, loc="left")
    axes[0, 0].legend(fontsize=8, ncol=min(4, len(devices)), frameon=False, loc="upper left",
                      bbox_to_anchor=(0, 1.35 if len(panels) > 1 else 1.25))
    axes[-1, 0].xaxis.set_major_formatter(mdates.DateFormatter("%m-%d %H:%M"))
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("data_dir", help="server 資料夾（含 <device>/<date>/ 檔案與 field_mix.db）")
    ap.add_argument("--db", help="db 路徑（預設 <data_dir>/field_mix.db）")
    ap.add_argument("--out", help="輸出資料夾（預設 <data_dir>/analysis_<時間>）")
    ap.add_argument("--period", type=int, default=300, help="錄製週期秒數（預設 300）")
    ap.add_argument("--duration", type=int, default=20, help="每次錄製秒數（預設 20）")
    ap.add_argument("--since", help='只看這之後的錄製，本地時間 "YYYY-MM-DD HH:MM"')
    ap.add_argument("--until", help='只看這之前的錄製，本地時間 "YYYY-MM-DD HH:MM"')
    ap.add_argument("--no-video-scan", action="store_true", help="不讀影像內容（快）")
    ap.add_argument("--no-audio-scan", action="store_true", help="不讀音訊內容")
    ap.add_argument("--no-plots", action="store_true", help="不畫圖")
    args = ap.parse_args()
    # Windows 主控台多半是 cp950，印不出的字元換成 ? 而不是整個中斷（summary.txt 仍是 UTF-8）
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")

    data_dir = Path(args.data_dir)
    db_path = find_db(data_dir, args.db)
    out_dir = Path(args.out) if args.out else data_dir / f"analysis_{datetime.now():%Y%m%d_%H%M%S}"
    out_dir.mkdir(parents=True, exist_ok=True)
    since = parse_local(args.since) if args.since else None
    until = parse_local(args.until) if args.until else None
    period = args.period

    conn = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    conn.row_factory = sqlite3.Row
    uploads = [u for u in load_uploads(conn, data_dir)
               if (since is None or u["recorded_at"] >= since) and (until is None or u["recorded_at"] <= until)]
    heartbeats = load_heartbeats(conn)
    conn.close()
    if not uploads:
        sys.exit("指定範圍內沒有任何上傳紀錄")

    lines: list[str] = []

    def say(text: str = "") -> None:
        lines.append(text)
        print(text)

    first_slot = min(u["recorded_at"] for u in uploads) // period * period
    last_slot = max(u["recorded_at"] for u in uploads) // period * period
    if since is not None:
        first_slot = -(-since // period) * period
    if until is not None:
        last_slot = until // period * period
    expected_slots = list(range(first_slot, last_slot + 1, period))

    say("=" * 78)
    say(f"資料：{data_dir}   db：{db_path}")
    say(f"範圍：{fmt_ts(first_slot)} ~ {fmt_ts(last_slot)}，週期 {period} s，"
        f"每台每種媒體應有 {len(expected_slots)} 筆")
    say("=" * 78)

    # ---------------- 檔案逐一分析 ----------------
    video_rows, audio_rows = [], []
    total = len(uploads)
    for idx, u in enumerate(uploads, 1):
        row = {
            "device": u["device_id"], "media": u["media_type"], "recorded_at": u["recorded_at"],
            "recorded": fmt_ts(u["recorded_at"]), "received": u["received_at"],
            "upload_delay_s": parse_iso(u["received_at"]) - u["recorded_at"],
            "aligned": u["recorded_at"] % period == 0,
            "start_offset_us": u.get("start_offset_us"),
            "file": str(u["path"]), "db_bytes": u["file_bytes"],
        }
        exists = u["path"].exists()
        row["exists"] = exists
        row["size_ok"] = exists and u["path"].stat().st_size == u["file_bytes"]
        meta = u["meta"]
        try:
            if u["media_type"] == "video":
                fps = int(meta.get("fps", 5))
                row["frames"] = meta.get("frames")
                row["expected_frames"] = args.duration * fps
                if exists and not args.no_video_scan:
                    row.update(analyze_video(u["path"], int(meta.get("width", 640)), int(meta.get("height", 480))))
                video_rows.append(row)
            else:
                rate = int(meta.get("sample_rate", 16000))
                row["expected_samples"] = args.duration * rate
                if exists and not args.no_audio_scan:
                    row.update(analyze_audio(u["path"]))
                audio_rows.append(row)
        except (OSError, ValueError, wave.Error) as exc:
            row["error"] = str(exc)
            (video_rows if u["media_type"] == "video" else audio_rows).append(row)
        if idx % 100 == 0 or idx == total:
            print(f"  已分析 {idx}/{total} 個檔案", file=sys.stderr)

    # ---------------- 漏錄 ----------------
    missing_rows = []
    received: dict[tuple[str, str], set[int]] = defaultdict(set)
    for u in uploads:
        received[(u["device_id"], u["media_type"])].add(u["recorded_at"])
    say("\n【1. 漏錄檢查】")
    for (device, media), got in sorted(received.items()):
        hbs = heartbeats.get(device, [])
        sessions = session_status(hbs)
        miss = [s for s in expected_slots if s not in got]
        extra = sorted(s for s in got if s % period != 0)
        say(f"  {device:24s} {media:5s} 收到 {len(got & set(expected_slots)):4d}/{len(expected_slots)}"
            f"  漏 {len(miss)}" + (f"  未對齊週期 {len(extra)} 筆" if extra else ""))
        for start, end, count in compress_ranges(miss, period):
            reason = missing_reason(start, period, media, hbs, sessions)
            say(f"      漏 {fmt_ts(start)}" + (f" ~ {fmt_ts(end)}（{count} 筆）" if count > 1 else "") + f"：{reason}")
        for s in miss:
            missing_rows.append({"device": device, "media": media, "slot": fmt_ts(s), "slot_epoch": s,
                                 "reason": missing_reason(s, period, media, hbs, sessions)})

    # ---------------- 檔案完整性 ----------------
    say("\n【2. 檔案完整性】")
    file_issues = [r for r in video_rows + audio_rows if not r["exists"] or not r["size_ok"] or r.get("error")]
    say(f"  檔案不存在 / 大小與 db 不符 / 讀取錯誤：{len(file_issues)} 筆")
    for r in file_issues[:20]:
        say(f"      {r['device']} {r['recorded']} exists={r['exists']} size_ok={r['size_ok']} {r.get('error', '')}")
    delays = [r["upload_delay_s"] for r in video_rows + audio_rows]
    say(f"  錄製到 server 收到的延遲：中位數 {statistics.median(delays):.0f} s，最大 {max(delays)} s")

    # ---------------- 影像 ----------------
    say("\n【3. 影像（frame drop / 內容）】")
    for device in sorted({r["device"] for r in video_rows}):
        rows = [r for r in video_rows if r["device"] == device]
        hbs = heartbeats.get(device, [])
        frames = Counter(r.get("frames") for r in rows)
        bad_frames = [r for r in rows if r.get("frames") != r["expected_frames"]]
        say(f"  {device}：{len(rows)} 檔，幀數分布 {dict(sorted(frames.items(), key=lambda kv: str(kv[0])))}"
            + (f"，≠ {rows[0]['expected_frames']} 幀的 {len(bad_frames)} 檔" if bad_frames else ""))
        faults = fault_increases(hbs)
        if any(hb.get("faults") is not None for hb in hbs):
            detail = " ".join(f"{k}={faults.get(k, 0)}" for k in FAULT_KEYS)
            say(f"      heartbeat 錄影失敗累計（這段期間新增）：{sum(faults.values())}（{detail}）")
        if not args.no_video_scan and rows and "mean_brightness" in rows[0]:
            def total(key):
                return sum(r.get(key, 0) or 0 for r in rows)

            def files(key):
                return sum(1 for r in rows if (r.get(key) or 0) > 0)
            say(f"      全白幀 {total('white_frames')}（{files('white_frames')} 檔）  "
                f"全黑幀 {total('black_frames')}（{files('black_frames')} 檔）  "
                f"單一亮度幀 {total('flat_frames')}（{files('flat_frames')} 檔）")
            say(f"      與前一幀完全相同 {total('duplicate_frames')}（{files('duplicate_frames')} 檔）  "
                f"底部 8 行全 0（疑似半幀）{total('zero_tail_frames')}（{files('zero_tail_frames')} 檔）")
            br = [r["mean_brightness"] for r in rows if "mean_brightness" in r]
            say(f"      平均亮度：最低 {min(br):.0f}、中位數 {statistics.median(br):.0f}、最高 {max(br):.0f}")
            worst = sorted((r for r in rows if r.get("white_frames")), key=lambda r: -r["white_frames"])[:5]
            for r in worst:
                say(f"        全白最多：{r['recorded']} {r['white_frames']}/{r['frames_in_file']} 幀，"
                    f"平均亮度 {r['mean_brightness']}")

    # ---------------- 音訊 ----------------
    say("\n【4. 麥克風】")
    if args.no_audio_scan:
        say("  （--no-audio-scan，略過）")
    for device in sorted({r["device"] for r in audio_rows}):
        rows = [r for r in audio_rows if r["device"] == device and "rms_dbfs" in r]
        if not rows:
            continue
        samples = Counter(r["samples"] for r in rows)
        clip_files = [r for r in rows if r["clip_samples"] > 0]
        dc = [r["dc_pct_fs"] for r in rows]
        rms = [r["rms_dbfs"] for r in rows]
        peak = [r["peak_dbfs"] for r in rows]
        say(f"  {device}：{len(rows)} 檔，樣本數分布 {dict(samples)}，"
            f"格式 {rows[0]['wav_rate']} Hz / {rows[0]['sample_width'] * 8} bit / {rows[0]['channels']} ch")
        say(f"      clip：{len(clip_files)} 檔有 clip，總 {sum(r['clip_samples'] for r in rows)} 樣本、"
            f"{sum(r['clip_runs'] for r in rows)} 段，最長連續 {max(r['longest_clip_run'] for r in rows)} 樣本；"
            f">= 99 % FS 的樣本 {sum(r['near_clip_samples'] for r in rows)}")
        say(f"      DC offset（% FS）：中位數 {statistics.median(dc):+.3f}、範圍 {min(dc):+.3f} ~ {max(dc):+.3f}"
            + (f"  !! {sum(1 for d in dc if abs(d) > DC_FLAG_PCT)} 檔 |DC| > {DC_FLAG_PCT} %" if any(
                abs(d) > DC_FLAG_PCT for d in dc) else ""))
        say(f"      RMS（去 DC，dBFS）：中位數 {statistics.median(rms):.1f}、範圍 {min(rms):.1f} ~ {max(rms):.1f}"
            + (f"  !! {sum(1 for v in rms if v < QUIET_FLAG_DBFS)} 檔低於 {QUIET_FLAG_DBFS:.0f}（疑似無訊號）"
               if any(v < QUIET_FLAG_DBFS for v in rms) else ""))
        say(f"      峰值（dBFS）：中位數 {statistics.median(peak):.1f}、最高 {max(peak):.2f}")
        zero = [r for r in rows if r["longest_zero_run_ms"] > ZERO_RUN_FLAG_S * 1000]
        say(f"      連續 0 超過 {ZERO_RUN_FLAG_S * 1000:.0f} ms（疑似斷訊）：{len(zero)} 檔"
            + (f"，最長 {max(r['longest_zero_run_ms'] for r in rows):.0f} ms" if zero else ""))
        lsb = Counter(r["unused_lsb_bits"] for r in rows)
        say(f"      最低位元從未使用的 bit 數分布：{dict(sorted(lsb.items()))}"
            "（0 = 16 bit 都有用到；8 = 實際只有 8 bit 解析度）")
        for r in sorted(clip_files, key=lambda r: -r["clip_samples"])[:5]:
            say(f"        clip 最多：{r['recorded']} {r['clip_samples']} 樣本 / {r['clip_runs']} 段，"
                f"RMS {r['rms_dbfs']} dBFS")

    # ---------------- heartbeat ----------------
    say("\n【5. 節點狀態（heartbeat）】")
    hb_rows, event_rows = [], []
    for device in sorted(heartbeats):
        hbs = [h for h in heartbeats[device]
               if (since is None or h["received"] >= since) and (until is None or h["received"] <= until + period)]
        if not hbs:
            continue
        st = heartbeat_stats(hbs)
        hb_rows.append({"device": device, **st})
        events = heartbeat_events(device, hbs)
        event_rows += events
        say(f"  {device}：{st['heartbeats']} 筆（{st['first']} ~ {st['last']}），重開機 {st['reboots']} 次，"
            f"開頭 reset_reason={st['reset_reason_first']}")
        say(f"      RSSI 最低 {st['rssi_min']}、中位數 {st['rssi_median']}；heap 最低 {st['heap_min_kb']} KB；"
            f"SD 剩 {st['sd_free_mb_first']} → {st['sd_free_mb_last']} MB；pending 最多 {st['pending_max']}、"
            f"最後 {st['pending_last']}")
        say(f"      recordings_failed +{st['recordings_failed']}、uploads_failed +{st['uploads_failed']}；"
            f"server - 節點時鐘：中位數 {st['clock_offset_median_s']} s、最大 |差| {st['clock_offset_max_abs_s']} s")
        kinds = Counter(e["event"] for e in events)
        if kinds:
            say(f"      事件：{dict(kinds)}（細節見 heartbeat_events.csv）")

    # ---------------- 輸出 ----------------
    write_csv(out_dir / "video_files.csv", video_rows)
    write_csv(out_dir / "audio_files.csv", audio_rows)
    write_csv(out_dir / "missing_slots.csv", missing_rows)
    write_csv(out_dir / "heartbeat_summary.csv", hb_rows)
    write_csv(out_dir / "heartbeat_events.csv", event_rows)
    if not args.no_plots:
        if video_rows and "mean_brightness" in video_rows[0]:
            plot_series(out_dir / "video_levels.png", "Video per recording",
                        [("mean_brightness", "mean brightness"), ("white_frames", "white frames"),
                         ("duplicate_frames", "duplicate frames")], video_rows)
        if audio_rows and "rms_dbfs" in audio_rows[0]:
            plot_series(out_dir / "audio_levels.png", "Audio per recording",
                        [("rms_dbfs", "RMS dBFS (DC removed)"), ("dc_pct_fs", "DC offset % FS"),
                         ("peak_dbfs", "peak dBFS"), ("clip_samples", "clipped samples")], audio_rows)
    say(f"\n輸出：{out_dir}")
    (out_dir / "summary.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
