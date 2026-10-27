"""Analyze a Saleae Logic 2 capture of the LA probe pins (main/la_task_trace.h).

Export in Logic 2: File > Export Raw Data > Digital, format CSV, all channels,
"Use ISO8601 timestamps" off. The file has one row per transition:
    Time [s],<D0 name>,<D1 name>,...     (values 0 / 1)

Default channel order (= board_pins.h LA_PIN_* wired to D0..D6):
    D0 task0  41   cam_task     / wifi
    D1 task1  42   cam_writer   / main (HTTP)
    D2 task2  21   mic_capture  / upload_reader
    D3 task3  48   mic_writer   / tiT
    D4 idle0   3   CPU0 in IDLE
    D5 idle1  46   CPU1 in IDLE
    D6 vsync   6   camera VSYNC (active low on this sensor setup)
Task pins mean "this task is on the CPU", idle pins "this core runs IDLE"; both
go low while their core is in a level 1-3 ISR (main/la_isr_trace.S), so ISR time
counts as busy but not as any task, and a task's run is split into several
pulses by the ISRs inside it (pulse count / width stats are per piece).
Which task a pin shows depends on the phase: recording while VSYNC runs, upload
afterwards.

Reports:
  * phases, found automatically: task pins are only bound while recording or
    uploading, so each span of task-pin activity is one phase (VSYNC inside =
    recording, otherwise upload). Per phase: idle % of each core, busy % and
    pulse count / width stats of every task pin.
  * per camera frame of the recording phases (VSYNC to VSYNC): idle % of each
    core, busy ms of each task pin, and the cam_writer burst (300 KB frame write)
  * any extra --window, clamped to the data range.

Usage:
    python tools/la_analyze.py capture.csv
    python tools/la_analyze.py capture.csv --window upload:31.2:58.0
    python tools/la_analyze.py capture.csv --frames-csv frames.csv
    python tools/la_analyze.py capture.csv --channels a,b,c,d,idle0,idle1,vsync
Standard library only.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import statistics
import sys
from dataclasses import dataclass, field
from pathlib import Path

DEFAULT_CHANNELS = ["task0", "task1", "task2", "task3", "idle0", "idle1", "vsync"]
RECORD_LABELS = {"task0": "cam_task", "task1": "cam_writer",
                 "task2": "mic_capture", "task3": "mic_writer"}
UPLOAD_LABELS = {"task0": "wifi", "task1": "main(HTTP)",
                 "task2": "upload_reader", "task3": "tiT"}
TASK_CHANNELS = ["task0", "task1", "task2", "task3"]


@dataclass
class Channel:
    name: str
    highs: list[tuple[float, float]] = field(default_factory=list)  # (rise, fall)
    edges: list[tuple[float, int]] = field(default_factory=list)    # (time, new level)

    def __post_init__(self) -> None:
        self._starts: list[float] | None = None

    def starts(self) -> list[float]:
        if self._starts is None:
            self._starts = [s for s, _ in self.highs]
        return self._starts

    def high_time(self, t0: float, t1: float) -> float:
        """Seconds this channel is high inside [t0, t1)."""
        starts = self.starts()
        i = max(bisect.bisect_left(starts, t0) - 1, 0)
        total = 0.0
        while i < len(self.highs) and self.highs[i][0] < t1:
            s, e = self.highs[i]
            total += max(0.0, min(e, t1) - max(s, t0))
            i += 1
        return total

    def pulses_in(self, t0: float, t1: float) -> list[tuple[float, float]]:
        """High intervals that start inside [t0, t1)."""
        starts = self.starts()
        return self.highs[bisect.bisect_left(starts, t0):bisect.bisect_left(starts, t1)]


def load(path: Path, names: list[str]) -> tuple[dict[str, Channel], float, float]:
    with path.open(newline="", encoding="utf-8-sig") as f:
        reader = csv.reader(f)
        header = next(reader)
        ncols = len(header) - 1
        if ncols < len(names):
            sys.exit(f"{path}: {ncols} channels in the file, {len(names)} names given "
                     f"(header: {header})")
        chans = [Channel(n) for n in names]
        level: list[int | None] = [None] * len(names)
        rise: list[float | None] = [None] * len(names)
        t_first = t_last = None
        for row in reader:
            if not row:
                continue
            t = float(row[0])
            if t_first is None:
                t_first = t
            t_last = t
            for i, ch in enumerate(chans):
                v = 1 if row[i + 1].strip() not in ("0", "") else 0
                if level[i] is None:
                    if v:
                        rise[i] = t
                elif v != level[i]:
                    ch.edges.append((t, v))
                    if v:
                        rise[i] = t
                    else:
                        if rise[i] is not None:
                            ch.highs.append((rise[i], t))
                        rise[i] = None
                level[i] = v
    if t_first is None:
        sys.exit(f"{path}: no data rows")
    # The export ends at the last transition; close highs still open there.
    for i, ch in enumerate(chans):
        if rise[i] is not None and rise[i] < t_last:
            ch.highs.append((rise[i], t_last))
    return {c.name: c for c in chans}, t_first, t_last


def bursts(pulses: list[tuple[float, float]], max_gap: float) -> list[tuple[float, float, float]]:
    """Group pulses whose gaps are < max_gap. Returns (start, end, busy seconds)."""
    out: list[tuple[float, float, float]] = []
    for s, e in pulses:
        if out and s - out[-1][1] < max_gap:
            bs, _, busy = out[-1]
            out[-1] = (bs, e, busy + (e - s))
        else:
            out.append((s, e, e - s))
    return out


def frame_starts(vsync: Channel, active_low: bool) -> list[float]:
    """Frame boundaries = VSYNC leading edges."""
    lead = 0 if active_low else 1
    return [t for t, v in vsync.edges if v == lead]


def phase_segments(chs: dict[str, Channel], gap: float) -> list[tuple[float, float]]:
    """Spans where any task pin is active. Task pins are only bound during the
    recording / upload phases (la_probe_set_phase), so these are the phases."""
    pulses = sorted(p for n in TASK_CHANNELS if n in chs for p in chs[n].highs)
    segs: list[tuple[float, float]] = []
    for s, e in pulses:
        if segs and s - segs[-1][1] < gap:
            segs[-1] = (segs[-1][0], max(segs[-1][1], e))
        else:
            segs.append((s, e))
    return segs


def pct(x: float, span: float) -> str:
    return f"{100.0 * x / span:5.1f}%" if span > 0 else "  -  "


def fmt_us(x: float) -> str:
    return f"{x * 1e6:9.1f}"


def window_report(chs: dict[str, Channel], label: str, t0: float, t1: float,
                  labels: dict[str, str]) -> None:
    span = t1 - t0
    print(f"\n=== {label}: {t0:.3f} .. {t1:.3f} s ({span:.3f} s) ===")
    for core in ("idle0", "idle1"):
        if core in chs:
            idle = chs[core].high_time(t0, t1)
            print(f"  CPU{core[-1]} idle {pct(idle, span)}   busy {pct(span - idle, span)}")
    print(f"  {'pin':<22}{'busy':>8}{'count':>8}{'per s':>8}"
          f"{'min us':>10}{'mean us':>10}{'p95 us':>10}{'max us':>10}")
    for name in TASK_CHANNELS:
        if name not in chs:
            continue
        ch = chs[name]
        widths = sorted(e - s for s, e in ch.pulses_in(t0, t1))
        busy = ch.high_time(t0, t1)
        tag = f"{name} {labels.get(name, '')}"
        if not widths:
            print(f"  {tag:<22}{pct(busy, span):>8}{0:>8}")
            continue
        p95 = widths[min(len(widths) - 1, int(0.95 * len(widths)))]
        print(f"  {tag:<22}{pct(busy, span):>8}{len(widths):>8}{len(widths) / span:>8.1f}"
              f"{fmt_us(widths[0]):>10}{fmt_us(statistics.fmean(widths)):>10}"
              f"{fmt_us(p95):>10}{fmt_us(widths[-1]):>10}")


def frames_report(chs: dict[str, Channel], starts: list[float], burst_gap: float,
                  frames_csv: Path | None) -> None:
    rows = []
    for i in range(len(starts) - 1):
        f0, f1 = starts[i], starts[i + 1]
        span = f1 - f0
        row = {"frame": i, "start_s": f0, "period_ms": span * 1e3}
        for core in ("idle0", "idle1"):
            if core in chs:
                row[f"{core}_pct"] = 100.0 * chs[core].high_time(f0, f1) / span
        for name in TASK_CHANNELS:
            if name in chs:
                row[f"{RECORD_LABELS[name]}_busy_ms"] = chs[name].high_time(f0, f1) * 1e3
        if "task1" in chs:
            # cam_writer: the 300 KB write is one dense burst; its idle
            # 50 ms queue-timeout wakeups are single pulses far apart.
            bl = bursts(chs["task1"].pulses_in(f0, f1), burst_gap)
            longest = max(bl, key=lambda b: b[1] - b[0]) if bl else None
            row["cam_writer_burst_ms"] = (longest[1] - longest[0]) * 1e3 if longest else 0.0
            row["cam_writer_burst_start_ms"] = (longest[0] - f0) * 1e3 if longest else 0.0
        rows.append(row)
    if not rows:
        print("\n(no complete VSYNC-to-VSYNC frame in the capture)")
        return

    print(f"\n=== per frame ({len(rows)} frames) ===")
    keys = [k for k in rows[0] if k not in ("frame", "start_s")]
    print(f"  {'':<26}{'min':>9}{'mean':>9}{'max':>9}")
    for k in keys:
        vals = [r[k] for r in rows]
        print(f"  {k:<26}{min(vals):>9.2f}{statistics.fmean(vals):>9.2f}{max(vals):>9.2f}")
    worst = max(rows, key=lambda r: r.get("cam_writer_burst_ms", 0.0))
    if "cam_writer_burst_ms" in worst:
        print(f"  slowest frame write: frame {worst['frame']} at {worst['start_s']:.3f} s, "
              f"{worst['cam_writer_burst_ms']:.1f} ms "
              f"(starts +{worst['cam_writer_burst_start_ms']:.1f} ms after VSYNC)")

    if frames_csv:
        with frames_csv.open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
        print(f"  per-frame rows written to {frames_csv}")


def parse_window(text: str) -> tuple[str, float, float]:
    try:
        label, a, b = text.split(":")
        return label, float(a), float(b)
    except ValueError:
        raise argparse.ArgumentTypeError("expected label:start:end (seconds)")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", type=Path, help="Logic 2 raw digital export (CSV)")
    ap.add_argument("--channels", default=",".join(DEFAULT_CHANNELS),
                    help="names for the CSV columns in order (default: %(default)s)")
    ap.add_argument("--vsync-active", choices=("low", "high"), default="low")
    ap.add_argument("--burst-gap-ms", type=float, default=20.0,
                    help="cam_writer pulses closer than this belong to one frame write "
                         "(above the ~9 ms SD wait per 32 KB chunk, below the 50 ms "
                         "queue-timeout wakeups; default %(default)s)")
    ap.add_argument("--split-gap-ms", type=float, default=50.0,
                    help="task-pin silence longer than this splits phases (default %(default)s)")
    ap.add_argument("--phase-gap-s", type=float, default=1.0,
                    help="same-kind pieces closer than this are merged (default %(default)s)")
    ap.add_argument("--min-phase-s", type=float, default=0.5,
                    help="ignore phases shorter than this (default %(default)s)")
    ap.add_argument("--window", type=parse_window, action="append", default=[],
                    metavar="LABEL:START:END",
                    help="extra window to report, seconds in capture time (repeatable); "
                         "label starting with 'up' uses the upload pin names")
    ap.add_argument("--frames-csv", type=Path, help="write the per-frame table here")
    args = ap.parse_args()

    names = [n.strip() for n in args.channels.split(",") if n.strip()]
    chs, t_first, t_last = load(args.csv, names)
    print(f"{args.csv}: {t_first:.6f} .. {t_last:.6f} s, channels {', '.join(names)}")
    print("note: the export ends at the last transition, so the capture tail after it "
          "is not included")

    window_report(chs, "whole capture", t_first, t_last, {})

    # Phases: spans of task-pin activity. VSYNC inside = recording, else upload.
    # (VSYNC alone is not enough: the camera also runs during boot warm-up /
    # FPS test and the prepare lead time, when the task pins are not bound.)
    starts = frame_starts(chs["vsync"], args.vsync_active == "low") if "vsync" in chs else []
    # Between phases the task pins are unbound (la_probe_set_phase(NONE)), so
    # even a short silence marks a phase change: split at --split-gap-ms
    # (recording never pauses that long: cam_task runs every ~2.5 ms), label
    # each piece, then merge neighbours with the same label (upload files are
    # bound one by one, with short gaps in between).
    labelled: list[list] = []
    for s, e in phase_segments(chs, args.split_gap_ms / 1e3):
        n_vsync = bisect.bisect_left(starts, e) - bisect.bisect_left(starts, s)
        kind = "recording" if n_vsync >= 2 else "upload"
        if labelled and labelled[-1][0] == kind and s - labelled[-1][2] < args.phase_gap_s:
            labelled[-1][2] = e
        else:
            labelled.append([kind, s, e])

    rec_frames: list[float] = []
    for kind, s, e in labelled:
        if e - s < args.min_phase_s:
            continue
        if kind == "recording":
            window_report(chs, "recording phase", s, e, RECORD_LABELS)
            seg_starts = [t for t in starts if s <= t <= e]
            # frames whose whole VSYNC-to-VSYNC period lies inside the phase
            rec_frames += seg_starts
            rec_frames.append(float("nan"))     # break between sessions
        else:
            window_report(chs, "upload phase", s, e, UPLOAD_LABELS)
            if abs(e - t_last) < 1e-6:
                print("  (still active at the end of the capture: the upload is cut off)")

    # per-frame table over the recording phases only
    if rec_frames:
        groups, cur = [], []
        for t in rec_frames:
            if t != t:          # nan
                if len(cur) >= 2:
                    groups.append(cur)
                cur = []
            else:
                cur.append(t)
        for g in groups:
            frames_report(chs, g, args.burst_gap_ms / 1e3, args.frames_csv)
    else:
        print("\n(no recording phase found: no per-frame table)")

    for label, a, b in args.window:
        a2, b2 = max(a, t_first), min(b, t_last)
        if (a2, b2) != (a, b):
            print(f"\nnote: window {label} {a}..{b} s clamped to the data range "
                  f"{a2:.3f}..{b2:.3f} s")
        if b2 <= a2:
            continue
        labels = UPLOAD_LABELS if label.lower().startswith("up") else RECORD_LABELS
        window_report(chs, label, a2, b2, labels)


if __name__ == "__main__":
    main()
