"""Decode the node's GRAY8 RAW video (PC side).

File format (written by main/cam_record.c, uploaded as-is by server_mix.py):
    no header; frames back to back, each WIDTH*HEIGHT bytes (640*480 = 307200),
    one byte per pixel (Y / luminance, 0 = black, 255 = white), row by row
    from the top-left. Frame count = file size / 307200. 5 fps.

Usage:
    python tools/raw_view.py FILE.raw --stats            per-frame diagnostics
    python tools/raw_view.py FILE.raw --png 0 150 299    export frames as PNG
    python tools/raw_view.py FILE.raw --mp4 out.mp4      whole file as video
    python tools/raw_view.py FILE.raw --sheet sheet.png  contact sheet (up to 20 evenly spaced frames)

Needs numpy + opencv-python (pip install numpy opencv-python).
"""

import argparse
import os
import sys
import tempfile
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


def sheet_indices(frame_count: int, columns: int, rows: int) -> np.ndarray:
    """Include the first/last frame; short clips leave empty cells, without repeats."""
    if min(frame_count, columns, rows) <= 0:
        raise ValueError("frame count, columns and rows must be positive")
    return np.linspace(0, frame_count - 1, min(frame_count, columns * rows)).astype(int)


def _draw_sheet(indexed_frames, columns: int, rows: int) -> np.ndarray:
    # Allocate one canvas rather than retaining all tiles and concatenation copies.
    sheet = np.zeros((rows * 240, columns * 320, 3), dtype=np.uint8)
    for slot, (index, frame) in enumerate(indexed_frames):
        tile = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_AREA)
        tile = cv2.cvtColor(tile, cv2.COLOR_GRAY2BGR)
        cv2.putText(tile, str(index), (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        row, column = divmod(slot, columns)
        sheet[row * 240:(row + 1) * 240, column * 320:(column + 1) * 320] = tile
    return sheet


def contact_sheet(frames: np.ndarray, columns: int = 5, rows: int = 4) -> np.ndarray:
    picks = sheet_indices(len(frames), columns, rows)
    return _draw_sheet(((int(index), frames[index]) for index in picks), columns, rows)


def save_contact_sheet(
    raw_path: Path,
    output_path: Path,
    *,
    width: int = 640,
    height: int = 480,
    expected_frames: int | None = None,
    columns: int = 5,
    rows: int = 4,
) -> Path:
    """Seek/read only selected GRAY8 frames and atomically publish a PNG.

    Shared by the CLI and server worker. Pixel operations stay in NumPy/OpenCV;
    a sheet-only invocation does not load the complete recording into RAM.
    """
    if min(width, height) <= 0:
        raise ValueError("width and height must be positive")
    raw_path, output_path = Path(raw_path), Path(output_path)
    if raw_path.resolve() == output_path.resolve():
        raise ValueError("sheet output must differ from the RAW input")
    frame_bytes = width * height
    with raw_path.open("rb") as source:
        file_bytes = os.fstat(source.fileno()).st_size
        frame_count, remainder = divmod(file_bytes, frame_bytes)
        if remainder:
            raise ValueError("RAW contains an incomplete frame")
        if expected_frames is not None and frame_count != expected_frames:
            raise ValueError("RAW frame count differs from upload metadata")
        picks = sheet_indices(frame_count, columns, rows)

        def selected_frames():
            for index in picks:
                source.seek(int(index) * frame_bytes)
                data = source.read(frame_bytes)
                if len(data) != frame_bytes:
                    raise ValueError("RAW was truncated while generating the sheet")
                yield int(index), np.frombuffer(data, dtype=np.uint8).reshape(height, width)

        sheet = _draw_sheet(selected_frames(), columns, rows)

    # imencode + Python file I/O also supports Unicode Windows paths. Publish
    # only a complete image; a failed retry leaves any existing PNG intact.
    encoded_ok, encoded = cv2.imencode(".png", sheet)
    if not encoded_ok:
        raise OSError("PNG encoding failed")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb", dir=output_path.parent, prefix=".sheet-", suffix=".tmp", delete=False
        ) as output:
            temp_path = Path(output.name)
            output.write(encoded.tobytes())
            output.flush()
            os.fsync(output.fileno())
        os.replace(temp_path, output_path)
        temp_path = None
    finally:
        if temp_path is not None:
            temp_path.unlink(missing_ok=True)
    return output_path


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

    if min(args.width, args.height, args.fps) <= 0:
        parser.error("width, height and fps must be positive")
    # Stats/frame export/video need every frame; a sheet needs at most 20.
    need_all_frames = args.stats or args.png is not None or args.mp4 is not None
    frames = load_frames(args.raw, args.width, args.height) if need_all_frames else None
    count = args.raw.stat().st_size // (args.width * args.height)
    print(f"{args.raw.name}: {count} frames {args.width}x{args.height} "
          f"({count / args.fps:.1f} s at {args.fps:g} fps)")

    if args.stats:
        print_stats(frames)
    if args.png is not None:
        for index in args.png or [0]:
            out = args.raw.with_name(f"{args.raw.stem}_f{index:03d}.png")
            cv2.imwrite(str(out), frames[index])
            print(f"wrote {out}")
    if args.sheet:
        save_contact_sheet(args.raw, args.sheet, width=args.width, height=args.height)
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
