#!/usr/bin/env python3
"""分析 server 斷線期間節點是否持續錄製、之後是否補傳，並檢查每個節點的缺檔。

合併多個 field_mix.db（舊的滿碟那顆 + 擴充後新的那顆 + ...），做四件事：
1. 自動找出「server 收不到上傳」的區間（也可手動指定 --start/--end）。
2. 每個節點：區間內該錄幾筆、實際錄到幾筆（recorded_at = 節點自己的時間）、
   多少是恢復後才補傳（received_at - recorded_at）。
3. 缺檔檢查：每個節點、每種媒體（audio / video 分開）把收到的檔案依 recorded_at 排好，
   相鄰兩筆間隔 > 1.5 個週期就是缺口，再用 heartbeat 判斷缺口原因：
   沒有 heartbeat（節點沒在跑）不算缺檔；有 heartbeat 才算，並區分重開機 / 錄製失敗 / 單純缺檔。
4. 圖：files_timeline.png（每個檔案一條刻度，缺口標紅）、node_status.png（heartbeat 狀態隨時間）。

用法：
    python tools/analyze_outage.py --out analysis_out
    python tools/analyze_outage.py --db A.db --db B.db --db C.db --out analysis_out
    python tools/analyze_outage.py --start "2026-10-22 06:00" --end "2026-10-26 14:00"
"""
from __future__ import annotations

import argparse
import csv
import json
import sqlite3
import statistics
from collections import defaultdict
from datetime import datetime, timedelta, timezone
from pathlib import Path

TZ = timezone(timedelta(hours=8))
DEFAULT_DBS = [
    r"D:\server_data\field_mix.db",
    r"D:\server_mix_data\field_mix.db",
    r"C:\Users\hien\ESP_project\esp32_node\server_mix_data\field_mix.db",
]
# 不納入分析的節點（node-7C0C5F59BA50 只在 10-21 測試約 4 小時）
EXCLUDE_DEVICES = ["node-7C0C5F59BA50"]
MIB = 1024 * 1024
# heartbeat 間隔超過這個值就視為「節點沒在回報」。
# S3 節點錄製 + 上傳期間暫停 heartbeat，最長約 9 分鐘，所以抓 15 分鐘。
HB_GAP_S = 15 * 60
# 狀態圖：server 滿碟期間 heartbeat 只零星存進 DB（約每 47 分鐘），中斷超過 6 小時才斷線
STATUS_GAP_S = 6 * 3600
LATE_UPLOAD_S = 30 * 60  # 錄完超過 30 分鐘才收到 = 補傳

CAUSE_LABEL = {
    "missing": "缺檔（節點在線）",
    "record_fail": "錄製失敗（recordings_failed 上升或 last_recording broken）",
    "reboot": "期間重開機",
    "off": "節點未開機（之後的 heartbeat 顯示開機時間在缺口之後），不計入缺檔",
}

# dataviz 參考調色盤：類別色固定順序；缺檔用 status critical
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
C_ON_TIME, C_LATE, C_MISSING = "#2a78d6", "#eda100", "#d03b3b"
C_NO_HB, C_TEXT, C_MUTED, C_GRID = "#c3c2b7", "#0b0b0b", "#52514e", "#e1e0d9"


def parse_local(text: str) -> float:
    return datetime.fromisoformat(text).replace(tzinfo=TZ).timestamp()


def fmt(ts: float) -> str:
    return datetime.fromtimestamp(ts, TZ).strftime("%m-%d %H:%M:%S")


def fmt_dur(sec: float) -> str:
    sec = int(sec)
    d, r = divmod(sec, 86400)
    h, r = divmod(r, 3600)
    m, _ = divmod(r, 60)
    return (f"{d}d" if d else "") + f"{h}h{m:02d}m"


def load(db_paths: list[str]):
    """回傳 (uploads, heartbeats)；uploads 以 capture_id 去重（保留最早 received），heartbeat 以 (device, 時間) 去重。"""
    uploads: dict[str, dict] = {}
    heartbeats: dict[tuple, dict] = {}
    for path in db_paths:
        if not Path(path).exists():
            print(f"[warn] 找不到 {path}，略過")
            continue
        con = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
        con.row_factory = sqlite3.Row
        for r in con.execute(
            "SELECT capture_id, session_id, device_id, media_type, recorded_at, "
            "received_at, file_bytes FROM uploads"
        ):
            row = dict(r)
            row["received"] = datetime.fromisoformat(row["received_at"]).timestamp()
            old = uploads.get(row["capture_id"])
            if old is None or row["received"] < old["received"]:
                uploads[row["capture_id"]] = row
        for r in con.execute("SELECT device_id, received_epoch, payload_json FROM heartbeats"):
            try:
                p = json.loads(r["payload_json"])
            except json.JSONDecodeError:
                p = {}
            heartbeats[(r["device_id"], r["received_epoch"])] = {
                "device_id": r["device_id"], "t": r["received_epoch"], "p": p}
        con.close()
        print(f"[load] {path}")
    return list(uploads.values()), list(heartbeats.values())


def find_silence(uploads):
    """用全艦隊每小時「收到的上傳數」找 server 無法收檔的區間。

    斷線期間 heartbeat 可能照常收得到（只是存檔失敗），所以不能用 heartbeat 判斷。
    基準 = 有流量小時的中位數；低於基準 30% 的連續小時 = 異常，取最長一段。
    """
    bins = defaultdict(int)
    for u in uploads:
        bins[int(u["received"] // 3600)] += 1
    if not bins:
        return None
    lo, hi = min(bins), max(bins)
    counts = [bins.get(h, 0) for h in range(lo, hi + 1)]
    base = statistics.median([c for c in counts if c > 0])
    low = [c < 0.3 * base for c in counts]
    best, i = (0, None, None), 0
    while i < len(low):
        if low[i]:
            j = i
            while j < len(low) and low[j]:
                j += 1
            if j - i > best[0]:
                best = (j - i, i, j)
            i = j
        else:
            i += 1
    if best[0] < 3:
        return None
    return (lo + best[1]) * 3600, (lo + best[2]) * 3600


def hb_get(p, flat, nested=None):
    """同時支援 legacy 扁平欄位與新版（v1）巢狀 heartbeat。"""
    if flat in p:
        return p[flat]
    if nested:
        cur = p
        for k in nested:
            if not isinstance(cur, dict) or k not in cur:
                return None
            cur = cur[k]
        return cur
    return None


def hb_row(h):
    """把兩種 heartbeat 格式轉成同一組欄位。"""
    if not h:
        return None
    p = h["p"]
    wifi = p.get("wifi") if isinstance(p.get("wifi"), dict) else {}
    if "wifi_rssi" in p:
        rssi = p["wifi_rssi"] if p.get("wifi_connected", True) else None
    else:
        rssi = wifi.get("rssi")
    if rssi is not None and rssi <= -127:  # -127 = 未連線
        rssi = None
    sd = p.get("sd_free_bytes")
    heap = p.get("free_heap_bytes")
    last_rec = p.get("last_recording") if isinstance(p.get("last_recording"), dict) else {}
    return {
        # v1 格式沒有 recordings_failed，改看最近一次錄製各媒體是否 broken
        "broken": {"video": last_rec.get("camera") == "broken", "audio": last_rec.get("mic") == "broken"},
        "t": h["t"],
        "uptime": p.get("uptime_seconds", p.get("uptime_s")),
        "pending": hb_get(p, "pending_files", ("storage", "pending_files")),
        "rec_ok": p.get("recordings_ok"),
        "rec_fail": p.get("recordings_failed"),
        "up_ok": p.get("uploads_ok"),
        "up_fail": p.get("uploads_failed"),
        "sd_mb": sd // MIB if sd is not None else hb_get(p, "free_mb", ("storage", "free_mb")),
        "heap_kb": heap / 1024 if heap is not None else hb_get(p, "heap_free_kb", ("memory", "heap_free_kb")),
        "rssi": rssi,
        "reset": p.get("reset_reason"),
    }


def hb_snapshot(rows, t):
    """t 之前最後一筆 heartbeat。"""
    c = [r for r in rows if r["t"] <= t]
    return c[-1] if c else None


def hb_intervals(rows):
    """heartbeat 連續（間隔 <= HB_GAP_S）的區段 = 節點有在運作。"""
    out = []
    for r in rows:
        if out and r["t"] - out[-1][1] <= HB_GAP_S:
            out[-1][1] = r["t"]
        else:
            out.append([r["t"], r["t"]])
    return out


def covered(intervals, a, b):
    return sum(max(0, min(b, y) - max(a, x)) for x, y in intervals)


def reboots(rows):
    """uptime 變小的時間點。"""
    return [y["t"] for x, y in zip(rows, rows[1:])
            if x["uptime"] is not None and y["uptime"] is not None and y["uptime"] < x["uptime"]]


def infer_period(times):
    diffs = [b - a for a, b in zip(times, times[1:]) if 0 < b - a < 3600]
    return statistics.median(diffs) if diffs else 300


def find_gaps(times, period, rows, intervals, media):
    """相鄰兩筆 recorded_at 間隔 > 1.5 週期 = 缺口，並用 heartbeat 判斷原因。

    server 滿碟期間 heartbeat 也大多沒存進 DB，不能用「有沒有 heartbeat」判斷節點在不在跑；
    改看缺口之後第一筆 heartbeat 的 uptime：開機時間早於缺口起點 = 節點整段都開著。
    """
    gaps = []
    for a, b in zip(times, times[1:]):
        n = round((b - a) / period) - 1
        if b - a <= 1.5 * period or n <= 0:
            continue
        after = next((r for r in rows if r["t"] >= b and r["uptime"] is not None), None)
        if after is not None:
            boot = after["t"] - after["uptime"]
            # 缺口前 1 小時內有 heartbeat = 缺口開始時節點在跑
            running_at_a = any(a - 3600 <= r["t"] <= a for r in rows)
            if boot > a + period and not running_at_a:
                cause = "off"
            elif boot > a + period or reboots([r for r in rows if a <= r["t"] <= b]):
                cause = "reboot"
            else:
                span = [r for r in rows if a - HB_GAP_S <= r["t"] <= after["t"]]
                fails = [r["rec_fail"] for r in span if r["rec_fail"] is not None]
                failed = (len(fails) >= 2 and fails[-1] > fails[0]) or any(r["broken"][media] for r in span)
                cause = "record_fail" if failed else "missing"
        else:
            # 沒有之後的 heartbeat 可查 uptime，退回用 heartbeat 覆蓋率判斷
            cause = "off" if covered(intervals, a, b) < 0.5 * (b - a) else "missing"
        gaps.append({"a": a, "b": b, "n": n, "cause": cause})
    return gaps


def build_series(dev, ups, rows, slot):
    """每種媒體一條序列：收到的檔案時間 + 缺口。"""
    intervals = hb_intervals(rows)
    by_media = defaultdict(dict)
    for u in ups:
        by_media[u["media_type"]][u["recorded_at"]] = u["received"]
    out = []
    for media in sorted(by_media):
        files = sorted(by_media[media].items())
        times = [t for t, _ in files]
        period = slot or infer_period(times)
        out.append({
            "device": dev, "media": media, "period": period,
            "files": files, "gaps": find_gaps(times, period, rows, intervals, media),
        })
    return out


def analyze_window(dev, ups, rows, w0, w1, slot):
    """server 收不到上傳的區間內，這個節點的錄製與補傳狀況（以 session 為單位）。"""
    sessions = {}
    for u in ups:
        s = sessions.setdefault(u["session_id"], {"rec": u["recorded_at"], "rcv": u["received"]})
        s["rec"] = min(s["rec"], u["recorded_at"])
        s["rcv"] = max(s["rcv"], u["received"])
    slots = sorted(sessions.values(), key=lambda s: s["rec"])
    period = slot or infer_period([s["rec"] for s in slots])
    in_win = [s for s in slots if w0 <= s["rec"] <= w1]
    lat = [s["rcv"] - s["rec"] for s in in_win]
    in_hb = [r for r in rows if w0 <= r["t"] <= w1]

    # 第一筆 recordings_failed 開始上升的 heartbeat = 節點開始錄製失敗的時間點
    stop_at = None
    for a, b in zip(in_hb, in_hb[1:]):
        if a["rec_fail"] is not None and b["rec_fail"] is not None and b["rec_fail"] > a["rec_fail"]:
            stop_at = (b["t"], b["sd_mb"])
            break

    expected = int((w1 - w0) // period)
    return {
        "device": dev, "period": period, "expected": expected, "recorded": len(in_win),
        "coverage": len(in_win) / expected if expected else 0.0,
        "backfilled": sum(1 for s in in_win if s["rcv"] > w1),
        "lat_med": statistics.median(lat) if lat else 0,
        "lat_max": max(lat) if lat else 0,
        "hb_in_window": len(in_hb),
        "rebooted": bool(reboots(in_hb)),
        "wifi_off_hb": sum(1 for r in in_hb if r["rssi"] is None),
        "stop_at": stop_at,
        "snaps": {
            "start": hb_snapshot(rows, w0),
            "mid": hb_snapshot(rows, (w0 + w1) / 2),
            "end": hb_snapshot(rows, w1),
            "after": hb_snapshot(rows, w1 + 3 * 3600),
        },
    }


def print_window(results):
    for r in results:
        print(f"== {r['device']}")
        print(f"  錄製週期 {int(r['period'])}s；區間內應錄 {r['expected']} 次，實際錄到 {r['recorded']} 次（{r['coverage']:.0%}）")
        print(f"  其中恢復後才補傳 {r['backfilled']} 次；延遲 中位數 {fmt_dur(r['lat_med'])}，最大 {fmt_dur(r['lat_max'])}")
        print(f"  區間內 heartbeat {r['hb_in_window']} 筆；Wi-Fi 未連線 {r['wifi_off_hb']} 筆；"
              f"{'期間有重開機' if r['rebooted'] else '期間未重開機'}")
        if r["stop_at"]:
            print(f"  !! {fmt(r['stop_at'][0])} 起開始錄製失敗（recordings_failed 上升，SD 剩 {r['stop_at'][1]} MB）")
        for k, label in (("start", "區間開始"), ("mid", "區間中間"), ("end", "區間結束"), ("after", "恢復後3h")):
            x = r["snaps"][k]
            if x:
                print(f"  [{label}] {fmt(x['t'])} uptime={fmt_dur(x['uptime']) if x['uptime'] is not None else '?'} "
                      f"pending={x['pending']} rec_ok={x['rec_ok']} rec_fail={x['rec_fail']} "
                      f"up_ok={x['up_ok']} up_fail={x['up_fail']} sd_free={x['sd_mb']}MB")


def write_window_csv(out: Path, results):
    with (out / "outage_summary.csv").open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["device", "period_s", "expected", "recorded", "coverage", "backfilled",
                    "latency_median_s", "latency_max_s", "hb_in_window", "rebooted",
                    "record_fail_from", "wifi_off_heartbeats", "pending_at_end", "pending_after_3h"])
        for r in results:
            sn = r["snaps"]
            w.writerow([r["device"], int(r["period"]), r["expected"], r["recorded"], f"{r['coverage']:.3f}",
                        r["backfilled"], int(r["lat_med"]), int(r["lat_max"]), r["hb_in_window"], r["rebooted"],
                        fmt(r["stop_at"][0]) if r["stop_at"] else "", r["wifi_off_hb"],
                        sn["end"]["pending"] if sn["end"] else "",
                        sn["after"]["pending"] if sn["after"] else ""])


def report_missing(series, hb_rows, out: Path):
    print("\n===== 缺檔檢查（audio / video 分開；節點未開機的空白不計入）=====")
    for s in series:
        times = [t for t, _ in s["files"]]
        by_cause = defaultdict(int)
        for g in s["gaps"]:
            by_cause[g["cause"]] += g["n"]
        missing = sum(n for c, n in by_cause.items() if c != "off")
        expected = len(times) + missing
        detail = "、".join(f"{CAUSE_LABEL[c].split('（')[0]} {n}" for c, n in by_cause.items() if c != "off")
        print(f"{s['device']:<20} {s['media']:<5} 週期 {int(s['period'])}s  {fmt(times[0])} → {fmt(times[-1])}  "
              f"收到 {len(times)} / 應有 {expected}，缺 {missing}（{missing / expected:.1%}）"
              + (f"：{detail}" if detail else ""))
        last = hb_rows.get(s["device"], [])[-1:] or [None]
        if last[0] and last[0]["pending"]:
            print(f"    最後 heartbeat {fmt(last[0]['t'])} 節點仍有 {last[0]['pending']} 個檔案待上傳，缺口可能之後補上")
        for g in s["gaps"]:
            if g["cause"] == "off":
                print(f"    （不計）{fmt(g['a'])} → {fmt(g['b'])} 節點未開機")

    with (out / "missing_slots.csv").open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["device", "media", "last_before_gap", "first_after_gap", "gap_s", "missing_count", "cause"])
        for s in series:
            for g in s["gaps"]:
                w.writerow([s["device"], s["media"], fmt(g["a"]), fmt(g["b"]), int(g["b"] - g["a"]),
                            g["n"], CAUSE_LABEL[g["cause"]]])

    per_day = defaultdict(lambda: defaultdict(int))
    for s in series:
        for t, _ in s["files"]:
            per_day[(s["device"], s["media"])][datetime.fromtimestamp(t, TZ).strftime("%m-%d")] += 1
    days = sorted({d for v in per_day.values() for d in v})
    with (out / "per_day_counts.csv").open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["device", "media"] + days)
        for (dev, media), v in sorted(per_day.items()):
            w.writerow([dev, media] + [v.get(d, 0) for d in days])
    print(f"[csv] {out / 'missing_slots.csv'}, {out / 'per_day_counts.csv'}（每 300s 一筆時整天 = 288）")


def _plt():
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.dates as mdates
        import matplotlib.pyplot as plt
        return plt, mdates
    except ImportError:
        print("[warn] 沒有 matplotlib，略過圖表（python -m pip install matplotlib）")
        return None, None


def _style_time_axis(ax, mdates, x0, x1):
    ax.set_xlim(x0, x1)
    ax.xaxis_date(tz=TZ)
    ax.xaxis.set_major_locator(mdates.DayLocator(tz=TZ))
    ax.xaxis.set_major_formatter(mdates.DateFormatter("%m-%d", tz=TZ))
    ax.xaxis.set_minor_locator(mdates.HourLocator(byhour=[6, 12, 18], tz=TZ))
    ax.grid(axis="x", which="major", color=C_GRID, lw=.8)
    ax.grid(axis="x", which="minor", color=C_GRID, lw=.4, ls=":")
    ax.tick_params(colors=C_MUTED, labelsize=8)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(C_NO_HB)


def plot_files(series, hb_rows, window, x_range, out: Path):
    """每個檔案依 recorded_at 畫一條刻度；紅 = 缺口（節點在線），灰斜線 = 無 heartbeat。"""
    plt, mdates = _plt()
    if not plt:
        return
    from matplotlib.patches import Patch
    from matplotlib.lines import Line2D

    def d(ts):
        return mdates.date2num(datetime.fromtimestamp(ts, TZ))

    n = len(series)
    fig, ax = plt.subplots(figsize=(16, 0.55 * n + 2.2))
    if window:
        ax.axvspan(d(window[0]), d(window[1]), color="#f0efec", zorder=0)
        ax.text(d(window[0]), -0.9, " server not saving uploads", fontsize=8, color=C_MUTED, va="center")
    for i, s in enumerate(series):
        on = [d(t) for t, rcv in s["files"] if rcv - t <= LATE_UPLOAD_S]
        late = [d(t) for t, rcv in s["files"] if rcv - t > LATE_UPLOAD_S]
        ax.eventplot([on], lineoffsets=i, linelengths=.6, linewidths=.5, colors=C_ON_TIME)
        ax.eventplot([late], lineoffsets=i, linelengths=.6, linewidths=.5, colors=C_LATE)
        last_label = None
        for g in s["gaps"]:
            x, w = d(g["a"]), d(g["b"]) - d(g["a"])
            if g["cause"] == "off":
                ax.broken_barh([(x, w)], (i - .3, .6), facecolor="none", edgecolor=C_NO_HB, hatch="////", lw=0)
            else:
                ax.broken_barh([(x, w)], (i - .3, .6), color=C_MISSING)
                if g["n"] >= 12 and (last_label is None or x + w / 2 - last_label > .25):  # 相鄰標籤至少隔 6 h
                    last_label = x + w / 2
                    ax.text(x + w / 2, i - .38, f"-{g['n']}", ha="center", va="bottom", fontsize=7, color=C_TEXT)
        for t in reboots(hb_rows.get(s["device"], [])):
            ax.plot(d(t), i + .42, marker="v", ms=5, color=C_TEXT, ls="none")
    ax.set_yticks(range(n))
    ax.set_yticklabels([f"{s['device']}  {s['media']}" for s in series], fontsize=8, color=C_TEXT)
    ax.set_ylim(n - .4, -1.2)
    _style_time_axis(ax, mdates, d(x_range[0]), d(x_range[1]))
    ax.legend(handles=[
        Line2D([], [], color=C_ON_TIME, lw=2, label="file received (on time)"),
        Line2D([], [], color=C_LATE, lw=2, label=f"file received later (> {LATE_UPLOAD_S // 60} min, backfill)"),
        Patch(color=C_MISSING, label="missing while node online (-N = count)"),
        Patch(facecolor="none", edgecolor=C_NO_HB, hatch="////", label="node powered off / not started, not counted"),
        Line2D([], [], marker="v", color=C_TEXT, ls="none", label="reboot (uptime reset)"),
    ], loc="upper center", bbox_to_anchor=(.5, -.06), ncol=5, fontsize=8, frameon=False)
    ax.set_title("Received files by recorded time (one tick per file)", loc="left", fontsize=11, color=C_TEXT)
    fig.tight_layout()
    fig.savefig(out / "files_timeline.png", dpi=130)
    plt.close(fig)
    print(f"[png] {out / 'files_timeline.png'}")


def plot_status(hb_rows, window, x_range, out: Path):
    """heartbeat 狀態隨時間；heartbeat 中斷 > STATUS_GAP_S 時線斷開。"""
    plt, mdates = _plt()
    if not plt:
        return

    def d(ts):
        return mdates.date2num(datetime.fromtimestamp(ts, TZ))

    metrics = [
        ("pending", "pending files", 1),
        ("sd_mb", "SD free (GB)", 1 / 1024),
        ("rssi", "Wi-Fi RSSI (dBm)", 1),
        ("uptime", "uptime (h)", 1 / 3600),
        ("heap_kb", "free heap (KB, log)", 1),
    ]
    devs = sorted(hb_rows)
    fig, axes = plt.subplots(len(metrics), 1, figsize=(16, 2.1 * len(metrics) + 1.2), sharex=True)
    for ax, (key, label, scale) in zip(axes, metrics):
        if window:
            ax.axvspan(d(window[0]), d(window[1]), color="#f0efec", zorder=0)
        for k, dev in enumerate(devs):
            xs, ys, prev = [], [], None
            for r in hb_rows[dev]:
                if prev is not None and r["t"] - prev > STATUS_GAP_S:
                    xs.append(float("nan"))
                    ys.append(float("nan"))
                v = r[key]
                xs.append(d(r["t"]))
                ys.append(v * scale if v is not None else float("nan"))
                prev = r["t"]
            ax.plot(xs, ys, lw=1.2, color=SERIES[k % len(SERIES)], label=dev)
        if key == "heap_kb":
            ax.set_yscale("log")  # legacy cam 的 free heap 含 PSRAM，和其他節點差一個數量級
        ax.set_ylabel(label, fontsize=8, color=C_MUTED)
        ax.grid(axis="y", color=C_GRID, lw=.6)
        _style_time_axis(ax, mdates, d(x_range[0]), d(x_range[1]))
    axes[0].legend(loc="lower left", bbox_to_anchor=(0, 1.02), ncol=min(len(devs), 8), fontsize=8, frameon=False)
    axes[0].set_title("Node status from heartbeats (line breaks = no heartbeat > 6 h; shaded = server not saving uploads)",
                      loc="left", fontsize=11, color=C_TEXT, pad=26)
    fig.tight_layout()
    fig.savefig(out / "node_status.png", dpi=130)
    plt.close(fig)
    print(f"[png] {out / 'node_status.png'}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--db", action="append", help="可重複；未指定時用 DEFAULT_DBS")
    ap.add_argument("--start", help="手動指定 server 斷線起點（台北時間，如 '2026-10-22 06:00'）")
    ap.add_argument("--end", help="手動指定恢復時間")
    ap.add_argument("--slot", type=int, default=0, help="錄製週期秒數；預設由各節點資料自動推算")
    ap.add_argument("--exclude", action="append", help="可重複；排除的節點，未指定時用 EXCLUDE_DEVICES")
    ap.add_argument("--out", default="analysis_out", help="輸出資料夾")
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    uploads, heartbeats = load(args.db or DEFAULT_DBS)
    excluded = set(args.exclude or EXCLUDE_DEVICES)
    uploads = [u for u in uploads if u["device_id"] not in excluded]
    heartbeats = [h for h in heartbeats if h["device_id"] not in excluded]
    if excluded:
        print(f"[exclude] {', '.join(sorted(excluded))}")
    if not uploads:
        raise SystemExit("沒有任何 uploads")
    print(f"uploads={len(uploads)}  heartbeats={len(heartbeats)}")

    by_up, hb_rows = defaultdict(list), defaultdict(list)
    for u in uploads:
        by_up[u["device_id"]].append(u)
    for h in sorted(heartbeats, key=lambda h: h["t"]):
        hb_rows[h["device_id"]].append(hb_row(h))

    if args.start and args.end:
        window = (parse_local(args.start), parse_local(args.end))
    else:
        window = find_silence(uploads)
    if window:
        print(f"\nserver 收不到上傳的區間：{fmt(window[0])} → {fmt(window[1])}（{fmt_dur(window[1] - window[0])}）\n")
        results = [analyze_window(dev, by_up[dev], hb_rows.get(dev, []), *window, args.slot) for dev in sorted(by_up)]
        print_window(results)
        write_window_csv(out, results)
    else:
        print("\n找不到 server 收不到上傳的區間（可用 --start/--end 指定），略過區間分析")

    series = [s for dev in sorted(by_up) for s in build_series(dev, by_up[dev], hb_rows.get(dev, []), args.slot)]
    report_missing(series, hb_rows, out)

    all_t = [u["recorded_at"] for u in uploads] + [h["t"] for h in heartbeats]
    x_range = (min(all_t), max(all_t))
    plot_files(series, hb_rows, window, x_range, out)
    plot_status(hb_rows, window, x_range, out)


if __name__ == "__main__":
    main()
