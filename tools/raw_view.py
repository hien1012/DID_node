"""Decode the node's GRAY8 RAW video (PC side).

File format (written by main/cam_record.c, uploaded as-is by server_mix.py):
    no header; frames back to back, each WIDTH*HEIGHT bytes (640*480 = 307200),
    one byte per pixel (Y / luminance, 0 = black, 255 = white), row by row
    from the top-left. Frame count = file size / 307200. 5 fps.

Usage:
    python tools/raw_view.py FILE.raw --stats            per-frame diagnostics
    python tools/raw_view.py FILE.raw --png 0 150 299    export frames as PNG
    python tools/raw_view.py FILE.raw --mp4 out.mp4      whole file as video
    python tools/raw_view.py FILE.raw --sheet sheet.png  contact sheet (every Nth frame)

Needs numpy + opencv-python (pip install numpy opencv-python).
"""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np


def load_frames(path: Path, width: int, height: int) -> np.ndarray:
    frame_bytes = width * height
    data = np.fromfile(path, dtype=np.uint8)
    count = data.size // frame_bytes
    if data.size % frame_bytes:
        print(f"warning: {data.size % frame_bytes} trailing bytes ignored", file=sys.stderr)
    return data[: count * frame_bytes].reshape(count, height, width)


def print_stats(frames: np.ndarray) -> None:
    """Numbers that separate 'wrong decoding' from 'bad capture'."""
    print("frame  mean   std  zero%  sat%  row_jump  col_jump  diff_prev")
    previous = None
    for index, frame in enumerate(frames):
        f = frame.astype(np.int16)
        # Average brightness change between neighbouring rows / columns. A real
        # image is smooth (small); noise or byte misalignment makes it large.
        row_jump = np.abs(np.diff(f, axis=0)).mean()
        col_jump = np.abs(np.diff(f, axis=1)).mean()
        diff_prev = np.abs(f - previous).mean() if previous is not None else 0.0
        print(f"{index:5d} {f.mean():6.1f} {f.std():5.1f} "
              f"{(frame == 0).mean() * 100:5.1f} {(frame == 255).mean() * 100:5.1f} "
              f"{row_jump:9.1f} {col_jump:9.1f} {diff_prev:10.1f}")
        previous = f


def contact_sheet(frames: np.ndarray, columns: int = 5, rows: int = 4) -> np.ndarray:
    picks = np.linspace(0, len(frames) - 1, columns * rows).astype(int)
    tiles = []
    for index in picks:
        tile = cv2.resize(frames[index], (320, 240), interpolation=cv2.INTER_AREA)
        tile = cv2.cvtColor(tile, cv2.COLOR_GRAY2BGR)
        cv2.putText(tile, str(index), (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        tiles.append(tile)
    lines = [np.hstack(tiles[r * columns:(r + 1) * columns]) for r in range(rows)]
    return np.vstack(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("raw", type=Path)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=5.0)
    parser.add_argument("--stats", action="store_true")
    parser.add_argument("--png", type=int, nargs="*", metavar="INDEX")
    parser.add_argument("--mp4", type=Path)
    parser.add_argument("--sheet", type=Path)
    args = parser.parse_args()

    frames = load_frames(args.raw, args.width, args.height)
    print(f"{args.raw.name}: {len(frames)} frames {args.width}x{args.height} "
          f"({len(frames) / args.fps:.1f} s at {args.fps:g} fps)")

    if args.stats:
        print_stats(frames)
    if args.png is not None:
        for index in args.png or [0]:
            out = args.raw.with_name(f"{args.raw.stem}_f{index:03d}.png")
            cv2.imwrite(str(out), frames[index])
            print(f"wrote {out}")
    if args.sheet:
        cv2.imwrite(str(args.sheet), contact_sheet(frames))
        print(f"wrote {args.sheet}")
    if args.mp4:
        writer = cv2.VideoWriter(str(args.mp4), cv2.VideoWriter_fourcc(*"mp4v"),
                                 args.fps, (args.width, args.height), isColor=False)
        for frame in frames:
            writer.write(frame)
        writer.release()
        print(f"wrote {args.mp4}")


if __name__ == "__main__":
    main()
