#!/usr/bin/env python3
"""
Analyze IFC engine characterization logs and build a feed-forward model.

Input:
  - CSV produced by Integrated Flight Computer/tests/engine_test.ks

Output:
  - *_summary.txt human-readable report
  - *_model.json machine-readable model + step metrics
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from statistics import mean, median
from typing import Dict, List, Optional, Sequence, Tuple


EPS = 1e-9
STEP_EPS = 1e-4


@dataclass
class Segment:
    start_idx: int
    end_idx: int
    cmd: float


@dataclass
class StepMetric:
    step_index: int
    direction: str
    cmd_from: float
    cmd_to: float
    t_start_s: float
    y0_kn: float
    yss_kn: float
    delta_kn: float
    t63_s: Optional[float]
    t90_s: Optional[float]
    t95_s: Optional[float]


def _to_float(value: str, default: float = float("nan")) -> float:
    try:
        return float(value)
    except Exception:
        return default


def _tail_window(values: Sequence[float], frac: float = 0.30, minimum: int = 5) -> Sequence[float]:
    if not values:
        return values
    n = len(values)
    k = max(minimum, int(round(n * frac)))
    if k >= n:
        return values
    return values[n - k :]


def _solve_linear_system(a: List[List[float]], b: List[float]) -> List[float]:
    """Gaussian elimination with partial pivoting."""
    n = len(a)
    mat = [row[:] for row in a]
    vec = b[:]

    for col in range(n):
        pivot = col
        for r in range(col + 1, n):
            if abs(mat[r][col]) > abs(mat[pivot][col]):
                pivot = r
        if abs(mat[pivot][col]) < EPS:
            raise ValueError("Singular matrix")
        if pivot != col:
            mat[col], mat[pivot] = mat[pivot], mat[col]
            vec[col], vec[pivot] = vec[pivot], vec[col]

        diag = mat[col][col]
        for c in range(col, n):
            mat[col][c] /= diag
        vec[col] /= diag

        for r in range(n):
            if r == col:
                continue
            factor = mat[r][col]
            if abs(factor) < EPS:
                continue
            for c in range(col, n):
                mat[r][c] -= factor * mat[col][c]
            vec[r] -= factor * vec[col]

    return vec


def _fit_linear(xs: Sequence[float], ys: Sequence[float]) -> Tuple[float, float]:
    n = float(len(xs))
    sx = sum(xs)
    sy = sum(ys)
    sxx = sum(x * x for x in xs)
    sxy = sum(x * y for x, y in zip(xs, ys))

    coef = _solve_linear_system([[n, sx], [sx, sxx]], [sy, sxy])
    return coef[0], coef[1]  # a0, a1


def _fit_quadratic(xs: Sequence[float], ys: Sequence[float]) -> Tuple[float, float, float]:
    n = float(len(xs))
    sx = sum(xs)
    sxx = sum(x * x for x in xs)
    sxxx = sum(x * x * x for x in xs)
    sxxxx = sum(x * x * x * x for x in xs)
    sy = sum(ys)
    sxy = sum(x * y for x, y in zip(xs, ys))
    sxxy = sum((x * x) * y for x, y in zip(xs, ys))

    coef = _solve_linear_system(
        [
            [n, sx, sxx],
            [sx, sxx, sxxx],
            [sxx, sxxx, sxxxx],
        ],
        [sy, sxy, sxxy],
    )
    return coef[0], coef[1], coef[2]  # a0, a1, a2


def _rmse(y_true: Sequence[float], y_pred: Sequence[float]) -> float:
    if not y_true:
        return float("nan")
    mse = sum((a - b) ** 2 for a, b in zip(y_true, y_pred)) / len(y_true)
    return math.sqrt(mse)


def _parse_metadata_line(line: str) -> Optional[Tuple[str, str]]:
    text = line.strip()
    if not text.startswith("#"):
        return None
    payload = text[1:].strip()
    if "=" not in payload:
        return None
    key, value = payload.split("=", 1)
    return key.strip(), value.strip()


def load_engine_log(path: Path) -> Tuple[Dict[str, str], List[Dict[str, float]]]:
    metadata: Dict[str, str] = {}
    data_lines: List[str] = []

    with path.open("r", newline="", encoding="utf-8") as f:
        for line in f:
            parsed = _parse_metadata_line(line)
            if parsed is not None:
                k, v = parsed
                metadata[k] = v
                continue
            if line.lstrip().startswith("#"):
                continue
            if line.strip() == "":
                continue
            data_lines.append(line)

    if not data_lines:
        raise ValueError(f"No CSV data rows found in {path}")

    rows: List[Dict[str, float]] = []
    reader = csv.DictReader(data_lines)
    for raw in reader:
        row: Dict[str, float] = {}
        for k, v in raw.items():
            if k is None:
                continue
            value = "" if v is None else v.strip()
            row[k] = _to_float(value)
        rows.append(row)

    required = {"t_s", "cmd_throttle", "engine_thrust_kn"}
    if not rows or not required.issubset(rows[0].keys()):
        raise ValueError(f"Missing required columns {sorted(required)} in {path}")

    rows = [r for r in rows if not math.isnan(r["t_s"]) and not math.isnan(r["cmd_throttle"]) and not math.isnan(r["engine_thrust_kn"])]
    if len(rows) < 10:
        raise ValueError(f"Too few valid samples ({len(rows)}) in {path}")

    return metadata, rows


def split_rows_by_channel(rows: Sequence[Dict[str, float]]) -> Dict[int, List[Dict[str, float]]]:
    grouped: Dict[int, List[Dict[str, float]]] = {}
    for r in rows:
        raw_idx = r.get("channel_idx", 1.0)
        if math.isnan(raw_idx):
            ch = 1
        else:
            ch = int(round(raw_idx))
            if ch <= 0:
                ch = 1
        grouped.setdefault(ch, []).append(r)

    for ch, ch_rows in grouped.items():
        ch_rows.sort(key=lambda x: x.get("t_s", float("nan")))
        grouped[ch] = [r for r in ch_rows if not math.isnan(r.get("t_s", float("nan")))]
    return grouped


def channel_metadata(metadata: Dict[str, str], channel_idx: int) -> Dict[str, str]:
    out = dict(metadata)
    prefix = f"channel_{channel_idx}_"
    for k, v in metadata.items():
        if k.startswith(prefix):
            out[k[len(prefix) :]] = v
    out["channel_idx"] = str(channel_idx)
    return out


def build_segments(rows: Sequence[Dict[str, float]]) -> List[Segment]:
    segments: List[Segment] = []
    start = 0
    cmd = rows[0]["cmd_throttle"]

    for i in range(1, len(rows)):
        next_cmd = rows[i]["cmd_throttle"]
        if abs(next_cmd - cmd) > STEP_EPS:
            segments.append(Segment(start, i - 1, cmd))
            start = i
            cmd = next_cmd

    segments.append(Segment(start, len(rows) - 1, cmd))
    return segments


def _find_threshold_time(
    rows: Sequence[Dict[str, float]],
    start_idx: int,
    end_idx: int,
    y0: float,
    yss: float,
    frac: float,
) -> Optional[float]:
    delta = yss - y0
    if abs(delta) < EPS:
        return None

    target = y0 + frac * delta
    sign = 1.0 if delta > 0 else -1.0
    t0 = rows[start_idx]["t_s"]

    for i in range(start_idx, end_idx + 1):
        y = rows[i]["engine_thrust_kn"]
        if (y - target) * sign >= 0:
            return max(0.0, rows[i]["t_s"] - t0)
    return None


def compute_step_metrics(rows: Sequence[Dict[str, float]], segments: Sequence[Segment]) -> List[StepMetric]:
    metrics: List[StepMetric] = []
    if len(segments) < 2:
        return metrics

    for idx in range(1, len(segments)):
        prev_seg = segments[idx - 1]
        cur_seg = segments[idx]

        prev_thrust = [rows[i]["engine_thrust_kn"] for i in range(prev_seg.start_idx, prev_seg.end_idx + 1)]
        cur_thrust = [rows[i]["engine_thrust_kn"] for i in range(cur_seg.start_idx, cur_seg.end_idx + 1)]
        if not prev_thrust or not cur_thrust:
            continue

        y0 = mean(_tail_window(prev_thrust))
        yss = mean(_tail_window(cur_thrust))
        delta = yss - y0
        if abs(delta) < 1e-3:
            continue

        direction = "up" if delta > 0 else "down"
        t63 = _find_threshold_time(rows, cur_seg.start_idx, cur_seg.end_idx, y0, yss, 0.632)
        t90 = _find_threshold_time(rows, cur_seg.start_idx, cur_seg.end_idx, y0, yss, 0.900)
        t95 = _find_threshold_time(rows, cur_seg.start_idx, cur_seg.end_idx, y0, yss, 0.950)

        metrics.append(
            StepMetric(
                step_index=idx,
                direction=direction,
                cmd_from=prev_seg.cmd,
                cmd_to=cur_seg.cmd,
                t_start_s=rows[cur_seg.start_idx]["t_s"],
                y0_kn=y0,
                yss_kn=yss,
                delta_kn=delta,
                t63_s=t63,
                t90_s=t90,
                t95_s=t95,
            )
        )

    return metrics


def collect_steady_points(rows: Sequence[Dict[str, float]], segments: Sequence[Segment]) -> List[Dict[str, float]]:
    points: List[Dict[str, float]] = []
    for seg in segments:
        thrust_samples = [rows[i]["engine_thrust_kn"] for i in range(seg.start_idx, seg.end_idx + 1)]
        mdot_samples = [rows[i].get("engine_massflow_kgps", float("nan")) for i in range(seg.start_idx, seg.end_idx + 1)]

        thrust_tail = _tail_window(thrust_samples)
        point: Dict[str, float] = {
            "cmd": seg.cmd,
            "thrust_kn": mean(thrust_tail),
        }

        valid_mdot = [x for x in _tail_window(mdot_samples) if not math.isnan(x) and x >= 0]
        point["mdot_kgps"] = mean(valid_mdot) if valid_mdot else float("nan")
        points.append(point)

    return points


def fit_static_map(points: Sequence[Dict[str, float]], key: str) -> Dict[str, object]:
    filtered = [p for p in points if not math.isnan(p[key])]
    if len(filtered) < 2:
        return {"ok": False, "reason": f"not enough points for {key}"}

    xs = [p["cmd"] for p in filtered]
    ys = [p[key] for p in filtered]

    unique_cmds = sorted({round(x, 6) for x in xs})
    if len(unique_cmds) >= 3:
        try:
            a0, a1, a2 = _fit_quadratic(xs, ys)
            yhat = [a0 + a1 * x + a2 * x * x for x in xs]
            return {
                "ok": True,
                "form": "a0 + a1*u + a2*u^2",
                "coefficients": [a0, a1, a2],
                "rmse": _rmse(ys, yhat),
                "n_points": len(filtered),
            }
        except ValueError:
            pass

    a0, a1 = _fit_linear(xs, ys)
    yhat = [a0 + a1 * x for x in xs]
    return {
        "ok": True,
        "form": "a0 + a1*u",
        "coefficients": [a0, a1],
        "rmse": _rmse(ys, yhat),
        "n_points": len(filtered),
    }


def summarize_tau(metrics: Sequence[StepMetric], direction: str) -> Dict[str, float]:
    vals_63 = [m.t63_s for m in metrics if m.direction == direction and m.t63_s is not None]
    vals_90 = [m.t90_s for m in metrics if m.direction == direction and m.t90_s is not None]
    vals_95 = [m.t95_s for m in metrics if m.direction == direction and m.t95_s is not None]

    return {
        "count": float(len(vals_63)),
        "t63_median_s": median(vals_63) if vals_63 else float("nan"),
        "t90_median_s": median(vals_90) if vals_90 else float("nan"),
        "t95_median_s": median(vals_95) if vals_95 else float("nan"),
    }


def write_summary(
    summary_path: Path,
    source_log: Path,
    metadata: Dict[str, str],
    thrust_map: Dict[str, object],
    mdot_map: Dict[str, object],
    tau_up: Dict[str, float],
    tau_down: Dict[str, float],
    steps: Sequence[StepMetric],
) -> None:
    lines: List[str] = []
    lines.append("IFC Engine Test Analysis")
    lines.append("========================")
    lines.append(f"Source log: {source_log}")
    if "channel_idx" in metadata:
        lines.append(f"Channel: {metadata.get('channel_idx', '1')}")
    lines.append(f"Craft: {metadata.get('craft_name', 'UNKNOWN')}")
    lines.append(f"Engine part: {metadata.get('engine_part_name', 'UNKNOWN')} ({metadata.get('engine_part_title', 'UNKNOWN')})")
    lines.append(f"Intake part: {metadata.get('intake_part_name', 'UNKNOWN')} ({metadata.get('intake_part_title', 'UNKNOWN')})")
    lines.append("")
    lines.append("Spool summary (median step response times)")
    lines.append(f"  Up   t63={tau_up['t63_median_s']:.3f}s  t90={tau_up['t90_median_s']:.3f}s  t95={tau_up['t95_median_s']:.3f}s  n={int(tau_up['count'])}")
    lines.append(f"  Down t63={tau_down['t63_median_s']:.3f}s  t90={tau_down['t90_median_s']:.3f}s  t95={tau_down['t95_median_s']:.3f}s  n={int(tau_down['count'])}")
    lines.append("")

    if thrust_map.get("ok"):
        coeff = thrust_map["coefficients"]
        lines.append("Steady thrust map")
        lines.append(f"  form: {thrust_map['form']}")
        lines.append(f"  coeff: {coeff}")
        lines.append(f"  rmse: {thrust_map['rmse']:.4f} kN")
    else:
        lines.append(f"Steady thrust map: unavailable ({thrust_map.get('reason', 'unknown')})")

    lines.append("")

    if mdot_map.get("ok"):
        coeff = mdot_map["coefficients"]
        lines.append("Steady massflow map")
        lines.append(f"  form: {mdot_map['form']}")
        lines.append(f"  coeff: {coeff}")
        lines.append(f"  rmse: {mdot_map['rmse']:.4f} kg/s")
    else:
        lines.append(f"Steady massflow map: unavailable ({mdot_map.get('reason', 'unknown')})")

    lines.append("")
    lines.append("Recommended first-order model")
    lines.append("  thrust_ss(u): use the steady thrust map above.")
    lines.append("  if thrust_ss(u) > thrust_now: tau = tau_up_t63.")
    lines.append("  else: tau = tau_down_t63.")
    lines.append("  thrust_dot = (thrust_ss(u) - thrust_now) / tau.")
    lines.append("")
    lines.append("Per-step metrics")
    lines.append("idx,dir,cmd_from,cmd_to,y0_kn,yss_kn,delta_kn,t63_s,t90_s,t95_s")
    for m in steps:
        lines.append(
            f"{m.step_index},{m.direction},{m.cmd_from:.3f},{m.cmd_to:.3f},"
            f"{m.y0_kn:.3f},{m.yss_kn:.3f},{m.delta_kn:.3f},"
            f"{m.t63_s if m.t63_s is not None else 'NA'},"
            f"{m.t90_s if m.t90_s is not None else 'NA'},"
            f"{m.t95_s if m.t95_s is not None else 'NA'}"
        )

    summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_model_dict(
    source_log: Path,
    channel_idx: int,
    metadata: Dict[str, str],
    thrust_map: Dict[str, object],
    mdot_map: Dict[str, object],
    tau_up: Dict[str, float],
    tau_down: Dict[str, float],
    steps: Sequence[StepMetric],
) -> Dict[str, object]:
    return {
        "source_log": str(source_log),
        "channel_idx": channel_idx,
        "metadata": metadata,
        "thrust_steady_state_map": thrust_map,
        "massflow_steady_state_map": mdot_map,
        "tau_up": tau_up,
        "tau_down": tau_down,
        "recommended_model": {
            "type": "first_order_with_directional_tau",
            "equation": "thrust_dot = (thrust_ss(u) - thrust_now) / tau_dir",
            "tau_dir_rule": "tau_up.t63_median_s if thrust_ss(u) > thrust_now else tau_down.t63_median_s",
            "steady_map_ref": "thrust_steady_state_map",
        },
        "step_metrics": [
            {
                "step_index": m.step_index,
                "direction": m.direction,
                "cmd_from": m.cmd_from,
                "cmd_to": m.cmd_to,
                "t_start_s": m.t_start_s,
                "y0_kn": m.y0_kn,
                "yss_kn": m.yss_kn,
                "delta_kn": m.delta_kn,
                "t63_s": m.t63_s,
                "t90_s": m.t90_s,
                "t95_s": m.t95_s,
            }
            for m in steps
        ],
    }


def find_latest_log(log_dir: Path) -> Path:
    candidates = sorted(log_dir.glob("engine_test_log_*.csv"))
    if not candidates:
        raise FileNotFoundError(f"No engine_test_log_*.csv found in {log_dir}")
    return candidates[-1]


def parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    ifc_root = script_dir.parent.parent
    default_log_dir = ifc_root / "engine test logs"

    parser = argparse.ArgumentParser(description="Analyze IFC engine test CSV logs")
    parser.add_argument("--log", type=Path, default=None, help="Path to engine_test_log_XXXXXXXX.csv")
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=default_log_dir,
        help="Directory containing engine test logs (used by --latest/default)",
    )
    parser.add_argument("--latest", action="store_true", help="Use latest log in --log-dir")
    parser.add_argument("--print-json", action="store_true", help="Also print resulting model JSON to stdout")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.log is not None:
        source_log = args.log
    elif args.latest:
        source_log = find_latest_log(args.log_dir)
    else:
        source_log = find_latest_log(args.log_dir)

    source_log = source_log.resolve()
    metadata, rows = load_engine_log(source_log)
    rows_by_channel = split_rows_by_channel(rows)
    channel_ids = sorted(rows_by_channel.keys())
    multi_channel = len(channel_ids) > 1

    models_written: List[Tuple[int, Path, Path, Dict[str, object]]] = []
    for ch in channel_ids:
        ch_rows = rows_by_channel[ch]
        if len(ch_rows) < 10:
            continue

        ch_md = channel_metadata(metadata, ch)
        segments = build_segments(ch_rows)
        step_metrics = compute_step_metrics(ch_rows, segments)
        steady_points = collect_steady_points(ch_rows, segments)

        thrust_map = fit_static_map(steady_points, "thrust_kn")
        mdot_map = fit_static_map(steady_points, "mdot_kgps")
        tau_up = summarize_tau(step_metrics, "up")
        tau_down = summarize_tau(step_metrics, "down")

        model = build_model_dict(source_log, ch, ch_md, thrust_map, mdot_map, tau_up, tau_down, step_metrics)

        suffix = "" if not multi_channel else f"_ch{ch:02d}"
        model_path = source_log.with_name(source_log.stem + suffix + "_model.json")
        summary_path = source_log.with_name(source_log.stem + suffix + "_summary.txt")
        model_path.write_text(json.dumps(model, indent=2), encoding="utf-8")
        write_summary(summary_path, source_log, ch_md, thrust_map, mdot_map, tau_up, tau_down, step_metrics)
        models_written.append((ch, model_path, summary_path, model))

    if not models_written:
        raise ValueError("No channel had enough valid samples to analyze.")

    if multi_channel:
        index = {
            "source_log": str(source_log),
            "channel_count": len(models_written),
            "channels": [
                {
                    "channel_idx": ch,
                    "model_json": str(model_path),
                    "summary_txt": str(summary_path),
                }
                for ch, model_path, summary_path, _ in models_written
            ],
        }
        index_path = source_log.with_name(source_log.stem + "_multi_model_index.json")
        index_path.write_text(json.dumps(index, indent=2), encoding="utf-8")
        print(f"Multi-channel index: {index_path}")

    print(f"Source log: {source_log}")
    for ch, model_path, summary_path, model in models_written:
        print(f"Channel {ch}:")
        print(f"  Model JSON:   {model_path}")
        print(f"  Summary text: {summary_path}")
        if args.print_json:
            print(json.dumps(model, indent=2))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
