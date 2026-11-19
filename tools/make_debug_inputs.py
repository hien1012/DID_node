#!/usr/bin/env python3
"""Make SD replay inputs for infer_node (INFER_DEBUG_REPLAY=1).

The node reads them from the SD card folder debug/in/ (copy the output folder's
files there):

  cam_<name>.gray   64x64 uint8 grayscale, row-major, exactly 4096 bytes
  mic_<name>.wav    16 kHz mono PCM16, exactly 2 s (32000 samples)

Sources:
  --image   any picture PIL can open (converted to grayscale, resized to 64x64)
  --wav     any WAV (mono-mixed, resampled to 16 kHz, cut at --offset-sec for 2 s)
  --from-dump  folders written by receive_infer_node_dump.py: reuse roi.gray /
               input.wav of earlier live dumps, so a live case can be replayed
               with both kernel sets

  python tools\\make_debug_inputs.py --image drone.png bird.jpg --wav heli.wav -o debug_in
  python tools\\make_debug_inputs.py --from-dump cam_dump -o debug_in
"""

from __future__ import annotations

import argparse
import math
import re
import shutil
from pathlib import Path

import numpy as np

ROI_SIZE = 64
SAMPLE_RATE = 16000
WINDOW_SAMPLES = 2 * SAMPLE_RATE
NAME_CHARS = 40          # cam_ / mic_ + name + extension stays well below the node's 64-char LFN limit


def clean_name(text: str) -> str:
    """Node dump names are split on spaces by the receiver: keep [A-Za-z0-9_-] only."""
    name = re.sub(r"[^A-Za-z0-9_-]+", "_", text).strip("_")
    return (name or "input")[:NAME_CHARS]


def unique_path(out_dir: Path, prefix: str, name: str, ext: str) -> Path:
    if name.lower().startswith(prefix) and len(name) > len(prefix):
        name = name[len(prefix):]                        # mic_x.wav as input -> mic_x.wav, not mic_mic_x.wav
    path = out_dir / f"{prefix}{name}{ext}"
    index = 2
    while path.exists():
        path = out_dir / f"{prefix}{name}_{index}{ext}"
        index += 1
    return path


def image_to_gray(path: Path, out_dir: Path) -> Path:
    from PIL import Image

    image = Image.open(path).convert("L")
    if image.size != (ROI_SIZE, ROI_SIZE):
        image = image.resize((ROI_SIZE, ROI_SIZE), Image.BILINEAR)
    data = np.asarray(image, dtype=np.uint8)
    out = unique_path(out_dir, "cam_", clean_name(path.stem), ".gray")
    data.tofile(out)
    return out


def write_wav_pcm16(path: Path, pcm: np.ndarray) -> None:
    from scipy.io import wavfile

    wavfile.write(path, SAMPLE_RATE, pcm.astype("<i2"))


def wav_to_window(path: Path, out_dir: Path, offset_sec: float) -> Path:
    from scipy.io import wavfile
    from scipy.signal import resample_poly

    rate, data = wavfile.read(path)
    if data.ndim > 1:
        data = data.mean(axis=1)
    if data.dtype == np.int16 and rate == SAMPLE_RATE:
        pcm = data.astype(np.int16)                      # bit-exact copy, no requantization
    else:
        if np.issubdtype(data.dtype, np.integer):
            x = data.astype(np.float64) / float(np.iinfo(data.dtype).max)
        else:
            x = data.astype(np.float64)
        if rate != SAMPLE_RATE:
            g = math.gcd(rate, SAMPLE_RATE)
            x = resample_poly(x, SAMPLE_RATE // g, rate // g)
        pcm = np.clip(np.rint(x * 32767.0), -32768, 32767).astype(np.int16)
    start = int(round(offset_sec * SAMPLE_RATE))
    window = pcm[start:start + WINDOW_SAMPLES]
    if window.size < WINDOW_SAMPLES:
        raise SystemExit(f"{path}: only {window.size} samples after --offset-sec {offset_sec}, need {WINDOW_SAMPLES}")
    suffix = f"_{offset_sec:g}s".replace(".", "p") if offset_sec else ""
    out = unique_path(out_dir, "mic_", clean_name(path.stem + suffix), ".wav")
    write_wav_pcm16(out, window)
    return out


def from_dumps(root: Path, out_dir: Path) -> list[Path]:
    stems = [root] if (root / "meta.json").is_file() else sorted(p for p in root.iterdir() if (p / "meta.json").is_file())
    written = []
    for stem_dir in stems:
        if (stem_dir / "roi.gray").is_file():
            out = unique_path(out_dir, "cam_", clean_name(stem_dir.name), ".gray")
            shutil.copyfile(stem_dir / "roi.gray", out)
            written.append(out)
        if (stem_dir / "input.wav").is_file():
            out = unique_path(out_dir, "mic_", clean_name(stem_dir.name), ".wav")
            shutil.copyfile(stem_dir / "input.wav", out)
            written.append(out)
    return written


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--image", type=Path, nargs="*", default=[], help="Pictures for the camera node.")
    parser.add_argument("--wav", type=Path, nargs="*", default=[], help="WAV files for the microphone node.")
    parser.add_argument("--offset-sec", type=float, default=0.0, help="Start of the 2 s window in each --wav.")
    parser.add_argument("--from-dump", type=Path, nargs="*", default=[],
                        help="Dump folders (or one <stem>/) whose roi.gray / input.wav are reused.")
    parser.add_argument("-o", "--output-dir", type=Path, default=Path("debug_in"))
    args = parser.parse_args()
    if not (args.image or args.wav or args.from_dump):
        parser.error("give at least one of --image, --wav, --from-dump")

    out_dir = args.output_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    written = [image_to_gray(p, out_dir) for p in args.image]
    written += [wav_to_window(p, out_dir, args.offset_sec) for p in args.wav]
    for root in args.from_dump:
        written += from_dumps(root.resolve(), out_dir)
    for path in written:
        print(f"  {path.name}  ({path.stat().st_size} bytes)")
    print(f"\n{len(written)} file(s) in {out_dir}\nCopy them to the SD card folder debug/in/ "
          f"(cam_*.gray for the camera node, mic_*.wav for the microphone node).")


if __name__ == "__main__":
    main()
