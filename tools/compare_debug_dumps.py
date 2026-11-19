#!/usr/bin/env python3
"""Four-way comparison of infer_node debug dumps (camera and microphone).

For each input (an SD replay file, or one live dump) the report lines up:

  node ESP-NN          dump of a build with ESP_NN_KERNEL_SET=all (per-layer .bin)
  node TFLM reference  dump of a build with ESP_NN_KERNEL_SET=none (per-layer .bin)
  PC C++ reference     TFLite BUILTIN_REF + preserve_all_tensors on the node's own CNN input
  PC Python int8       TFLite default (optimized) kernels, i.e. what a Python evaluation gets

Microphone dumps additionally compare the node's logmel-CMVN stages with the Python
training frontend (TTGO_I2S_Record_Test/log_mel_features.py + logmel_cnn.py, settings
from preprocess_config.json), and run PC Python end-to-end: input.wav -> Python
frontend -> int8.tflite.

Replay dumps of the same input file are paired through meta.json's input_file, so put
the dumps of both builds in one folder:

  python ..\\TTGO_I2S_Record_Test\\tools\\receive_infer_node_dump.py --port COM14 --reset --count N -o replay_dump
  (flash the other kernel set, receive again into the same replay_dump)
  python tools\\compare_debug_dumps.py --dump-dir replay_dump

Live dumps (INFER_CAM_DEBUG_DUMP / INFER_AUDIO_DEBUG_DUMP) work too; each is its own group.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
PROJECTS = REPO.parent
DEFAULT_TTGO_DIR = PROJECTS / "TTGO_I2S_Record_Test"
DEFAULT_CAM_MODEL = PROJECTS / "DID_CAM_SEAN" / "CNN" / "quantize_out" / "model_int8.tflite"
DEFAULT_MIC_MODEL = (DEFAULT_TTGO_DIR / "cnn_output" / "logmel_cmvn" / "0909_outdoor" / "inmp"
                     / "logmel_cnn_classifier_int8.tflite")
FIRMWARE_CAM_MODEL = REPO / "infer_node" / "main" / "vision" / "model.cc"
FIRMWARE_MIC_MODEL = REPO / "infer_node" / "main" / "audio" / "model_data_int8.cc"

ROI_SIZE = 64
DRONE_PROB_GATE = 0.30          # vision/cam_model.cc: drone wins above this
PROB_TOLERANCE = 0.005          # > 1 output LSB (1/256) = a different result
TYPE_DTYPES = {"INT8": np.int8, "UINT8": np.uint8, "INT16": np.int16, "INT32": np.int32,
               "INT64": np.int64, "FLOAT32": np.float32, "BOOL": np.bool_}

# node frontend file, Python stage key, dtype, tolerance kind, tolerance (Python float64 FFT vs C float32)
AUDIO_STAGES = [
    ("fe_mel_power.f32", "mel_power", np.float32, "rel", 1e-4),
    ("fe_log_mel.f32", "log_mel", np.float32, "abs", 1e-2),
    ("fe_delta.f32", "delta", np.float32, "abs", 2e-2),
    ("fe_cmvn.f32", "cmvn", np.float32, "abs", 1e-2),
    ("fe_normalized.f32", "normalized", np.float32, "abs", 1e-2),
    ("fe_quantized.i8", "quantized", np.int8, "lsb", 1),
    ("fe_quantized_prod.i8", "quantized", np.int8, "lsb", 1),
]


# --------------------------------------------------------------------------- helpers

def load_model_bytes(path: Path) -> bytes:
    """A .tflite file, or the xxd-style array of a firmware model .cc."""
    if path.suffix.lower() != ".cc":
        return path.read_bytes()
    text = path.read_text(encoding="ascii")
    body = re.search(r"\[\]\s*(?:DATA_ALIGN_ATTRIBUTE\s*)?(?:alignas\(\d+\)\s*)?=\s*\{(.*?)\};", text, re.S)
    if body is None:
        raise ValueError(f"no embedded model array in {path}")
    return bytes(int(v, 16) for v in re.findall(r"0x([0-9a-fA-F]{2})", body.group(1)))


def compare_arrays(device: np.ndarray, ref: np.ndarray, kind: str, tolerance: float) -> dict:
    if device.shape != ref.shape:
        return {"ok": False, "exact": False, "error": f"shape node={device.shape} pc={ref.shape}"}
    diff = np.abs(device.astype(np.float64) - ref.astype(np.float64))
    result = {"max_abs": float(diff.max()) if diff.size else 0.0,
              "mismatch_count": int(np.count_nonzero(diff)), "values": int(diff.size)}
    if kind == "rel":
        peak = float(np.max(np.abs(ref))) or 1.0
        result["max_rel_to_peak"] = result["max_abs"] / peak
        result["ok"] = result["max_rel_to_peak"] <= tolerance
    elif kind == "lsb":     # integer tensors: at most 1 LSB on at most 1% of the values
        result["ok"] = result["max_abs"] <= tolerance and result["mismatch_count"] <= 0.01 * diff.size
    else:
        result["ok"] = result["max_abs"] <= tolerance
    result["exact"] = result["mismatch_count"] == 0
    return result


def fmt_cmp(c: dict | None) -> str:
    if c is None:
        return "-"
    if "error" in c:
        return "BAD " + c["error"]
    tag = "OK " if c["ok"] else "BAD"
    if c.get("max_rel_to_peak") is not None:
        return f"{tag} rel={c['max_rel_to_peak']:.1e}"
    if c["exact"]:
        return f"{tag} exact"
    return f"{tag} max={c['max_abs']:.3g} n={c['mismatch_count']}/{c['values']}"


def read_index(path: Path) -> dict[int, dict]:
    with path.open(encoding="utf-8") as handle:
        return {int(row["index"]): row for row in csv.DictReader(handle)}


def make_interpreter(model: bytes, reference: bool):
    import tensorflow as tf

    kwargs = {"model_content": model}
    if reference:
        kwargs["experimental_op_resolver_type"] = tf.lite.experimental.OpResolverType.BUILTIN_REF
        kwargs["experimental_preserve_all_tensors"] = True
    interpreter = tf.lite.Interpreter(**kwargs)
    interpreter.allocate_tensors()
    return interpreter


def invoke(interpreter, quantized_input: np.ndarray) -> np.ndarray:
    """Feed an already quantized input; return dequantized probabilities."""
    inp = interpreter.get_input_details()[0]
    out = interpreter.get_output_details()[0]
    interpreter.set_tensor(inp["index"], quantized_input.reshape(inp["shape"]).astype(inp["dtype"]))
    interpreter.invoke()
    scale, zero = out["quantization"]
    raw = interpreter.get_tensor(out["index"])[0]
    return (raw.astype(np.int32) - zero).astype(np.float32) * np.float32(scale)


def rounded(probs) -> list[float] | None:
    return None if probs is None else [round(float(v), 6) for v in probs]


def kernel_label(kernel: str) -> str:
    return "node TFLM reference" if kernel == "none" else f"node ESP-NN ({kernel})"


def check_model(model: bytes, firmware_cc: Path, metas: list[dict], warnings: list[str]) -> None:
    for meta in metas:
        if meta.get("model_bytes") not in (None, len(model)):
            warnings.append(f"node model is {meta['model_bytes']} bytes, PC model is {len(model)} bytes")
            break
    if firmware_cc.is_file() and load_model_bytes(firmware_cc) != model:
        warnings.append(f"PC model is not byte-identical to {firmware_cc.name} in the firmware")


# --------------------------------------------------------------------------- dumps and groups

def dump_kind(meta: dict) -> str | None:
    if meta.get("kind"):
        return meta["kind"]
    return "audio_window" if "window" in meta else None      # audio dumps made before "kind" existed


def collect_groups(dump_dir: Path) -> dict[tuple[str, str], list[dict]]:
    dirs = [dump_dir] if (dump_dir / "meta.json").is_file() else sorted(
        p for p in dump_dir.iterdir() if (p / "meta.json").is_file())
    groups: dict[tuple[str, str], list[dict]] = {}
    for path in dirs:
        meta = json.loads((path / "meta.json").read_text(encoding="utf-8"))
        kind = dump_kind(meta)
        if kind not in ("camera_roi", "audio_window"):
            print(f"skip {path.name}: unknown dump kind {kind!r}")
            continue
        key = (kind, meta.get("input_file") or path.name)
        groups.setdefault(key, []).append(
            {"dir": path, "meta": meta, "kernel": meta.get("esp_nn_kernel_set") or "unknown"})
    for dumps in groups.values():
        # ESP-NN first, reference second, so the columns read the same in every group
        dumps.sort(key=lambda d: (d["kernel"] == "none", d["kernel"], d["dir"].name))
    return groups


def compare_layers(dumps: list[dict], reference) -> tuple[list[dict], list[str]]:
    """Every op output of the PC reference run against each node dump (and node vs node)."""
    findings = []
    indexes = [read_index(d["dir"] / "nn_index.csv") for d in dumps]
    input_index = int(reference.get_input_details()[0]["index"])
    checked = [(-1, "INPUT", input_index)] + [
        (int(op["index"]), op["op_name"], int(t)) for op in reference._get_ops_details() for t in op["outputs"]]
    layers = []
    first_bad: dict[str, str] = {}
    for op_index, op_name, tensor in checked:
        entry = {"op_index": op_index, "op": op_name, "tensor": tensor, "vs_pc": {}, "shape": ""}
        arrays = []
        pc = reference.get_tensor(tensor).reshape(-1)
        for dump, index in zip(dumps, indexes):
            row = index.get(tensor)
            if row is None or not row["file"] or row["file"] == "const":
                entry["vs_pc"][dump["kernel"]] = {"ok": False, "exact": False, "error": "not dumped"}
                arrays.append(None)
            else:
                dtype = TYPE_DTYPES[row["type"]]
                data = np.fromfile(dump["dir"] / row["file"], dtype=dtype)
                integer = np.issubdtype(dtype, np.integer)
                entry["vs_pc"][dump["kernel"]] = compare_arrays(data, pc, "lsb" if integer else "abs",
                                                                1 if integer else 1e-4)
                entry["shape"] = row["shape"]
                arrays.append(data)
            if not entry["vs_pc"][dump["kernel"]]["ok"] and dump["kernel"] not in first_bad:
                first_bad[dump["kernel"]] = f"op{op_index:02d} {op_name} (t{tensor:03d})"
        if len(arrays) == 2 and arrays[0] is not None and arrays[1] is not None:
            entry["node_vs_node"] = compare_arrays(arrays[0], arrays[1], "lsb", 1)
        layers.append(entry)
    for kernel, where in first_bad.items():
        findings.append(f"{kernel_label(kernel)} differs from PC reference first at {where}")
    return layers, findings


def node_result_findings(dumps: list[dict], pc_ref: np.ndarray) -> list[str]:
    findings = []
    for dump in dumps:
        meta, label = dump["meta"], kernel_label(dump["kernel"])
        prod = np.asarray(meta.get("prod_probs") or [np.nan] * len(pc_ref), dtype=np.float64)
        for key, what in (("prod_repeat_probs", "a second run of the production interpreter"),
                          ("dump_probs", "the preserve_all dump interpreter")):
            other = meta.get(key)
            if other is not None and np.nanmax(np.abs(prod - np.asarray(other))) > PROB_TOLERANCE:
                findings.append(f"{label}: production result differs from {what} (nondeterminism / memory planning)")
        if np.nanmax(np.abs(prod - pc_ref)) > PROB_TOLERANCE:
            findings.append(f"{label}: production result differs from PC reference")
    return findings


def print_layers(layers: list[dict], dumps: list[dict]) -> None:
    kernels = [d["kernel"] for d in dumps]
    header = "".join(f"{kernel_label(k) + ' vs PC ref':34s}" for k in kernels)
    if len(kernels) == 2:
        header += f"{kernels[0]} vs {kernels[1]}"
    print(f"  {'layer':30s}{'shape':>13s}  {header}")
    for layer in layers:
        tag = f"op{layer['op_index']:02d}" if layer["op_index"] >= 0 else "in  "
        cols = "".join(f"{fmt_cmp(layer['vs_pc'].get(k)):34s}" for k in kernels)
        if "node_vs_node" in layer:
            cols += fmt_cmp(layer["node_vs_node"])
        print(f"  {tag} {layer['op'][:20]:20s} t{layer['tensor']:03d}{layer['shape']:>13s}  {cols}")


def print_probs(probs: dict[str, list[float] | None], decide) -> None:
    for label, p in probs.items():
        if p is not None:
            print(f"  {label:44s} {p}  -> {decide(p)}")


# --------------------------------------------------------------------------- camera

def compare_camera(name: str, dumps: list[dict], model: bytes, save_png: bool) -> dict:
    report: dict = {"input": name, "kind": "camera_roi", "dumps": [d["dir"].name for d in dumps],
                    "kernels": [d["kernel"] for d in dumps], "warnings": [], "findings": []}
    labels = dumps[0]["meta"].get("labels") or ["drone", "bird", "background"]

    def decide(p):
        p = np.asarray(p)
        return labels[0] if p[0] > DRONE_PROB_GATE else labels[int(np.argmax(p))]

    check_model(model, FIRMWARE_CAM_MODEL, [d["meta"] for d in dumps], report["warnings"])
    rois = [np.fromfile(d["dir"] / "roi.gray", dtype=np.uint8) for d in dumps]
    if any(r.size != ROI_SIZE * ROI_SIZE for r in rois):
        report["error"] = "roi.gray is not 64x64"
        return report
    if any(not np.array_equal(rois[0], r) for r in rois[1:]):
        report["findings"].append("the dumps of this input have different roi.gray (not the same input)")
    roi = rois[0]
    if save_png:
        from PIL import Image
        Image.fromarray(roi.reshape(ROI_SIZE, ROI_SIZE)).resize(
            (ROI_SIZE * 4, ROI_SIZE * 4), Image.NEAREST).save(dumps[0]["dir"] / "roi.png")

    reference = make_interpreter(model, reference=True)
    pc_ref = invoke(reference, roi)                 # every tensor of this run stays readable
    pc_python = invoke(make_interpreter(model, reference=False), roi)

    report["layers"], layer_findings = compare_layers(dumps, reference)
    report["findings"] += layer_findings + node_result_findings(dumps, pc_ref)
    report["probabilities"] = {kernel_label(d["kernel"]): rounded(d["meta"].get("prod_probs")) for d in dumps}
    report["probabilities"]["PC C++ reference (BUILTIN_REF)"] = rounded(pc_ref)
    report["probabilities"]["PC Python int8.tflite"] = rounded(pc_python)
    report["predictions"] = {k: decide(v) for k, v in report["probabilities"].items() if v is not None}
    report["_decide"] = decide
    return report


# --------------------------------------------------------------------------- microphone

class PythonFrontend:
    """The training frontend, imported from TTGO_I2S_Record_Test (not re-implemented)."""

    def __init__(self, ttgo_dir: Path, config_path: Path):
        sys.path.insert(0, str(ttgo_dir))
        import audio_dataset
        import log_mel_features
        import logmel_cnn

        self.audio_dataset, self.lmf, self.cnn = audio_dataset, log_mel_features, logmel_cnn
        self.config = json.loads(config_path.read_text(encoding="utf-8"))
        self.config_path = config_path

    def run(self, wav: Path, input_scale: float, input_zero: int) -> dict[str, np.ndarray]:
        c = self.config
        sr, audio = self.audio_dataset.read_wav_mono(wav, c["target_sr"])     # int16 / 32767, as in training
        audio = self.lmf.fix_audio_length(audio, c["segment_samples"])
        lc = c["log_compression"]
        common = dict(sr=sr, n_fft=c["n_fft"], hop_length=c["hop_length"], n_mels=c["n_mels"],
                      f_min=c["f_min_hz"], f_max=c["f_max_hz"])
        mel_power = self.lmf.compute_mel_power_spectrogram(audio, **common)
        log_mel = self.lmf.extract_log_mel_spectrogram(audio, **common, log_floor=lc["log_floor"],
                                                       top_db=lc["top_db"], reference=lc["reference"])
        mode = c["input_channels"]
        channels = self.cnn.build_input_channels(log_mel[None], mode)[0]           # (mel, frame, C)
        mean = np.asarray(c["normalization_mean"], dtype=np.float32)
        std = np.asarray(c["normalization_std"], dtype=np.float32)
        normalized = ((channels - mean) / (std + np.float32(1e-6))).astype(np.float32)
        mask = c.get("mel_band_mask") or {}
        if mask.get("enabled") and mask.get("indices"):
            normalized[mask["indices"], :, :] = np.float32(mask.get("fill_value_after_normalization", 0.0))
        quantized = np.clip(np.rint(normalized / input_scale + input_zero), -128, 127).astype(np.int8)
        stages = {"mel_power": mel_power, "log_mel": log_mel, "normalized": normalized, "quantized": quantized}
        if mode == "cmvn":
            stages["cmvn"] = channels[:, :, 0]
        if "delta" in mode:
            delta = np.zeros_like(log_mel)
            delta[:, 1:] = log_mel[:, 1:] - log_mel[:, :-1]
            stages["delta"] = delta
        return {k: np.asarray(v).reshape(-1) for k, v in stages.items()}


def compare_audio(name: str, dumps: list[dict], model: bytes, frontend: PythonFrontend) -> dict:
    report: dict = {"input": name, "kind": "audio_window", "dumps": [d["dir"].name for d in dumps],
                    "kernels": [d["kernel"] for d in dumps], "warnings": [], "findings": []}
    labels = dumps[0]["meta"].get("labels") or frontend.config.get("class_names")

    def decide(p):
        return labels[int(np.argmax(p))]

    check_model(model, FIRMWARE_MIC_MODEL, [d["meta"] for d in dumps], report["warnings"])
    wavs = [(d["dir"] / "input.wav").read_bytes() for d in dumps]
    feats = [np.fromfile(d["dir"] / "fe_quantized_prod.i8", dtype=np.int8) for d in dumps]
    if any(w != wavs[0] for w in wavs[1:]):
        report["findings"].append("the dumps of this input have different input.wav (not the same input)")
    elif any(not np.array_equal(feats[0], f) for f in feats[1:]):
        report["findings"].append("same input.wav but different node features between builds "
                                  "(the frontend does not depend on ESP-NN: memory corruption?)")

    reference = make_interpreter(model, reference=True)
    inp = reference.get_input_details()[0]
    in_scale, in_zero = inp["quantization"]
    py = frontend.run(dumps[0]["dir"] / "input.wav", in_scale, in_zero)

    # frontend: node C (first dump; the others must match it bit for bit) vs Python training frontend
    stages = []
    for file_name, key, dtype, kind, tol in AUDIO_STAGES:
        path = dumps[0]["dir"] / file_name
        if key not in py or not path.is_file():
            continue
        result = compare_arrays(np.fromfile(path, dtype=dtype), py[key], kind, tol)
        result["stage"] = file_name
        stages.append(result)
    report["frontend_vs_python"] = stages
    bad = [s["stage"] for s in stages if not s["ok"]]
    if bad:
        report["findings"].append(f"node frontend differs from the Python training frontend first at {bad[0]}")

    pc_python_e2e = invoke(make_interpreter(model, reference=False), py["quantized"])
    pc_python = invoke(make_interpreter(model, reference=False), feats[0])
    pc_ref = invoke(reference, feats[0])            # on the node's features: isolates the CNN

    report["layers"], layer_findings = compare_layers(dumps, reference)
    report["findings"] += layer_findings + node_result_findings(dumps, pc_ref)
    report["probabilities"] = {kernel_label(d["kernel"]): rounded(d["meta"].get("prod_probs")) for d in dumps}
    report["probabilities"]["PC C++ reference (BUILTIN_REF, node features)"] = rounded(pc_ref)
    report["probabilities"]["PC Python int8.tflite (node features)"] = rounded(pc_python)
    report["probabilities"]["PC Python end-to-end (Python logmel-CMVN)"] = rounded(pc_python_e2e)
    report["predictions"] = {k: decide(v) for k, v in report["probabilities"].items() if v is not None}
    report["_decide"] = decide
    return report


def print_frontend(stages: list[dict]) -> None:
    print("  frontend: node C vs Python training frontend")
    for s in stages:
        print(f"    {s['stage']:22s} {fmt_cmp(s)}")


# --------------------------------------------------------------------------- main

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dump-dir", type=Path, required=True,
                        help="Folder of <stem>/ dumps from receive_infer_node_dump.py (both builds), or one <stem>/.")
    parser.add_argument("--cam-model", type=Path, default=DEFAULT_CAM_MODEL, help="Camera int8 .tflite (or model.cc).")
    parser.add_argument("--mic-model", type=Path, default=DEFAULT_MIC_MODEL, help="Audio int8 .tflite (or .cc).")
    parser.add_argument("--mic-config", type=Path, default=None,
                        help="preprocess_config.json of the audio model (default: next to --mic-model).")
    parser.add_argument("--ttgo-dir", type=Path, default=DEFAULT_TTGO_DIR,
                        help="TTGO_I2S_Record_Test (Python training frontend).")
    parser.add_argument("--png", action="store_true", help="Write roi.png (4x) into camera dump folders.")
    parser.add_argument("-o", "--output", type=Path, default=None,
                        help="JSON report (default: <dump-dir>/debug_comparison.json).")
    args = parser.parse_args()

    dump_dir = args.dump_dir.resolve()
    groups = collect_groups(dump_dir)
    if not groups:
        raise SystemExit(f"no dumps with meta.json under {dump_dir}")
    cam_model = mic_model = frontend = None
    reports = []
    for (kind, name), dumps in sorted(groups.items()):
        if kind == "camera_roi":
            cam_model = cam_model or load_model_bytes(args.cam_model.resolve())
            report = compare_camera(name, dumps, cam_model, args.png)
        else:
            mic_model = mic_model or load_model_bytes(args.mic_model.resolve())
            frontend = frontend or PythonFrontend(
                args.ttgo_dir.resolve(), (args.mic_config or args.mic_model.parent / "preprocess_config.json").resolve())
            report = compare_audio(name, dumps, mic_model, frontend)
        reports.append(report)

        print(f"\n=== {name}  ({kind}; dumps: {', '.join(report['dumps'])})")
        for warning in report["warnings"]:
            print("  WARNING:", warning)
        if "error" in report:
            print("  ERROR:", report["error"])
            continue
        if len({d["kernel"] for d in dumps}) < len(dumps):
            print("  WARNING: several dumps with the same kernel set; only the last is shown per column")
        if "frontend_vs_python" in report:
            print_frontend(report["frontend_vs_python"])
        print_layers(report["layers"], dumps)
        print_probs(report["probabilities"], report.pop("_decide"))
        verdict = "; ".join(report["findings"]) or "all node builds match the PC reference at every layer"
        p = [v for v in report["probabilities"].values() if v is not None]
        if not report["findings"] and any(np.max(np.abs(np.asarray(v) - np.asarray(p[0]))) > PROB_TOLERANCE for v in p):
            verdict += " (PC Python differs: optimized-kernel rounding / Python frontend, see above)"
        report["verdict"] = verdict
        print("  VERDICT:", verdict)

    output = (args.output or dump_dir / "debug_comparison.json").resolve()
    for report in reports:
        report.pop("_decide", None)
    output.write_text(json.dumps(reports, indent=2, default=str), encoding="utf-8")
    print(f"\nReport: {output}")


if __name__ == "__main__":
    main()
