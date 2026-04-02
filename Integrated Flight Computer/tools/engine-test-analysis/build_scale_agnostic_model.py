#!/usr/bin/env python3
"""
Build a tweakscale-agnostic engine model from multiple engine-test logs.

Each run must provide:
  - log CSV path
  - engine diameter (m)
  - intake diameter (m)

Example:
  python build_scale_agnostic_model.py \
    --run "../../engine test logs/engine_test_log_00000001.csv" 1.25 1.25 \
    --run "../../engine test logs/engine_test_log_00000002.csv" 0.75 0.75
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from statistics import median
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import pandas as pd


EPS = 1e-9
STEP_EPS = 1e-4


@dataclass
class RunData:
    path: Path
    engine_diameter_m: float
    intake_diameter_m: float
    metadata: Dict[str, str]
    df: pd.DataFrame
    steady_df: pd.DataFrame
    tau_up_t63_s: float
    tau_down_t63_s: float


def parse_engine_log(path: Path, channel_idx: int = 1) -> Tuple[Dict[str, str], pd.DataFrame]:
    metadata: Dict[str, str] = {}
    data_lines: List[str] = []

    with path.open("r", encoding="utf-8", newline="") as f:
        for line in f:
            s = line.strip()
            if not s:
                continue
            if s.startswith("#"):
                payload = s[1:].strip()
                if "=" in payload:
                    k, v = payload.split("=", 1)
                    metadata[k.strip()] = v.strip()
                continue
            data_lines.append(line)

    if not data_lines:
        raise ValueError(f"No CSV content in {path}")

    rows = list(csv.DictReader(data_lines))
    if not rows:
        raise ValueError(f"No data rows parsed from {path}")

    df = pd.DataFrame(rows)
    for c in df.columns:
        numeric = pd.to_numeric(df[c], errors="coerce")
        if numeric.notna().any():
            df[c] = numeric

    if "channel_idx" in df.columns:
        df["channel_idx"] = pd.to_numeric(df["channel_idx"], errors="coerce")
        df = df[df["channel_idx"].round().astype("Int64") == int(channel_idx)].copy()
    elif channel_idx != 1:
        raise ValueError(f"Requested channel {channel_idx} but log has no channel_idx column: {path}")

    required = ["t_s", "cmd_throttle", "engine_thrust_kn"]
    for c in required:
        if c not in df.columns:
            raise ValueError(f"Missing required column '{c}' in {path}")
        df[c] = pd.to_numeric(df[c], errors="coerce")

    df = df.dropna(subset=required).copy()
    df = df.sort_values("t_s").reset_index(drop=True)
    if len(df) < 10:
        raise ValueError(f"Too few usable rows in {path}: {len(df)}")
    return metadata, df


def tail_window(values: np.ndarray, frac: float = 0.30, minimum: int = 5) -> np.ndarray:
    n = len(values)
    if n <= 0:
        return values
    k = max(minimum, int(round(n * frac)))
    if k >= n:
        return values
    return values[-k:]


def build_segments(df: pd.DataFrame) -> List[Tuple[int, int, float]]:
    segs: List[Tuple[int, int, float]] = []
    start = 0
    cmd = float(df.loc[0, "cmd_throttle"])
    for i in range(1, len(df)):
        nxt = float(df.loc[i, "cmd_throttle"])
        if abs(nxt - cmd) > STEP_EPS:
            segs.append((start, i - 1, cmd))
            start = i
            cmd = nxt
    segs.append((start, len(df) - 1, cmd))
    return segs


def _segment_tail_mean(df: pd.DataFrame, a: int, b: int, col: str) -> float:
    if col not in df.columns:
        return float("nan")
    arr = pd.to_numeric(df.loc[a:b, col], errors="coerce").values.astype(float)
    arr = arr[np.isfinite(arr)]
    if arr.size == 0:
        return float("nan")
    return float(np.mean(tail_window(arr)))


def segment_steady_points(df: pd.DataFrame, segs: Sequence[Tuple[int, int, float]]) -> pd.DataFrame:
    pts: List[Dict[str, float]] = []
    for a, b, cmd in segs:
        pts.append(
            {
                "cmd": float(cmd),
                "thrust_ss_kn": _segment_tail_mean(df, a, b, "engine_thrust_kn"),
                "massflow_ss_kgps": _segment_tail_mean(df, a, b, "engine_massflow_kgps"),
                "duration_s": float(df.loc[b, "t_s"] - df.loc[a, "t_s"]),
            }
        )
    out = pd.DataFrame(pts)
    out = out.groupby("cmd", as_index=False).agg(
        thrust_ss_kn=("thrust_ss_kn", "median"),
        massflow_ss_kgps=("massflow_ss_kgps", "median"),
        duration_s=("duration_s", "median"),
    )
    return out.sort_values("cmd").reset_index(drop=True)


def _find_tfrac(df: pd.DataFrame, a: int, b: int, y0: float, yss: float, frac: float) -> Optional[float]:
    delta = yss - y0
    if abs(delta) < EPS:
        return None
    target = y0 + frac * delta
    sign = 1.0 if delta > 0 else -1.0
    t0 = float(df.loc[a, "t_s"])
    ys = pd.to_numeric(df.loc[a:b, "engine_thrust_kn"], errors="coerce").values.astype(float)
    ts = pd.to_numeric(df.loc[a:b, "t_s"], errors="coerce").values.astype(float)
    for y, t in zip(ys, ts):
        if np.isfinite(y) and (y - target) * sign >= 0:
            return max(0.0, float(t - t0))
    return None


def median_taus(df: pd.DataFrame, segs: Sequence[Tuple[int, int, float]]) -> Tuple[float, float]:
    up: List[float] = []
    dn: List[float] = []
    for i in range(1, len(segs)):
        pa, pb, _ = segs[i - 1]
        ca, cb, _ = segs[i]
        y0 = _segment_tail_mean(df, pa, pb, "engine_thrust_kn")
        y1 = _segment_tail_mean(df, ca, cb, "engine_thrust_kn")
        if not np.isfinite(y0) or not np.isfinite(y1):
            continue
        if abs(y1 - y0) < 1e-3:
            continue
        t63 = _find_tfrac(df, ca, cb, y0, y1, 0.632)
        if t63 is None:
            continue
        if y1 > y0:
            up.append(float(t63))
        else:
            dn.append(float(t63))
    return (
        float(median(up)) if up else float("nan"),
        float(median(dn)) if dn else float("nan"),
    )


def _polyfit_safe(x: np.ndarray, y: np.ndarray) -> Tuple[int, np.ndarray]:
    mask = np.isfinite(x) & np.isfinite(y)
    x = x[mask]
    y = y[mask]
    if len(x) < 2:
        raise ValueError("Need at least 2 points for steady thrust fit")
    deg = 2 if len(np.unique(np.round(x, 6))) >= 3 else 1
    coefs = np.polyfit(x, y, deg=deg)
    return deg, coefs


def _fit_log_slope(xs: Sequence[float], ys: Sequence[float]) -> float:
    x = np.asarray(xs, dtype=float)
    y = np.asarray(ys, dtype=float)
    mask = np.isfinite(x) & np.isfinite(y) & (np.abs(x) > EPS)
    x = x[mask]
    y = y[mask]
    if len(x) == 0:
        return float("nan")
    # Through-origin fit in log-space (reference run anchored at ratio 1).
    return float(np.dot(x, y) / np.dot(x, x))


def _resolve_output_path(args_output: Optional[str], runs: Sequence[RunData]) -> Path:
    if args_output:
        return Path(args_output).resolve()
    # Default: next to first run.
    return runs[0].path.with_name("engine_scale_agnostic_model.json")


def _sanitize_json(obj):
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {k: _sanitize_json(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_sanitize_json(v) for v in obj]
    return obj


def _warn_saturation(steady_df: pd.DataFrame) -> Optional[str]:
    if "cmd" not in steady_df or "thrust_ss_kn" not in steady_df:
        return None
    cmd05 = steady_df.loc[np.isclose(steady_df["cmd"], 0.5), "thrust_ss_kn"]
    cmd10 = steady_df.loc[np.isclose(steady_df["cmd"], 1.0), "thrust_ss_kn"]
    if len(cmd05) == 0 or len(cmd10) == 0:
        return None
    a = float(cmd05.iloc[0])
    b = float(cmd10.iloc[0])
    if not (np.isfinite(a) and np.isfinite(b)):
        return None
    if max(abs(a), abs(b), 1e-6) > 0 and abs(a - b) / max(abs(a), abs(b), 1e-6) < 0.05:
        return "0.5 and 1.0 throttle steady thrust are nearly identical; likely airflow/prop-limit saturation."
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description="Build tweakscale-agnostic engine model from multiple logs.")
    parser.add_argument(
        "--run",
        action="append",
        nargs=3,
        metavar=("CSV_PATH", "ENGINE_DIAM_M", "INTAKE_DIAM_M"),
        required=True,
        help="Add one run with explicit engine/intake diameters in meters.",
    )
    parser.add_argument(
        "--channel",
        type=int,
        default=1,
        help="Channel index to analyze for multi-engine logs (default: 1).",
    )
    parser.add_argument("--output", default="", help="Output JSON path.")
    args = parser.parse_args()

    runs: List[RunData] = []
    for csv_path, eng_d, int_d in args.run:
        p = Path(csv_path).expanduser().resolve()
        md, df = parse_engine_log(p, channel_idx=args.channel)
        segs = build_segments(df)
        steady_df = segment_steady_points(df, segs)
        tau_up, tau_down = median_taus(df, segs)
        runs.append(
            RunData(
                path=p,
                engine_diameter_m=float(eng_d),
                intake_diameter_m=float(int_d),
                metadata=md,
                df=df,
                steady_df=steady_df,
                tau_up_t63_s=tau_up,
                tau_down_t63_s=tau_down,
            )
        )

    if len(runs) < 2:
        raise ValueError("Need at least 2 runs to estimate scaling exponents.")

    # Reference = largest engine diameter.
    ref = max(runs, key=lambda r: r.engine_diameter_m)
    deg, thrust_coefs = _polyfit_safe(ref.steady_df["cmd"].values.astype(float), ref.steady_df["thrust_ss_kn"].values.astype(float))
    _, mdot_coefs = _polyfit_safe(ref.steady_df["cmd"].values.astype(float), ref.steady_df["massflow_ss_kgps"].values.astype(float))

    def thrust_ref(u: float) -> float:
        return float(np.polyval(thrust_coefs, u))

    def mdot_ref(u: float) -> float:
        return float(np.polyval(mdot_coefs, u))

    run_diags: List[Dict[str, object]] = []
    x_diam: List[float] = []
    y_gain_thrust: List[float] = []
    y_gain_mdot: List[float] = []
    y_tau_up: List[float] = []
    y_tau_down: List[float] = []

    for r in runs:
        merged = pd.merge(
            r.steady_df[["cmd", "thrust_ss_kn", "massflow_ss_kgps"]],
            ref.steady_df[["cmd"]],
            on="cmd",
            how="inner",
        ).sort_values("cmd")

        gains_t: List[float] = []
        gains_m: List[float] = []
        per_cmd: List[Dict[str, float]] = []
        for _, row in merged.iterrows():
            cmd = float(row["cmd"])
            t_obs = float(row["thrust_ss_kn"])
            m_obs = float(row["massflow_ss_kgps"])
            t_ref = thrust_ref(cmd)
            m_ref = mdot_ref(cmd)
            g_t = t_obs / t_ref if np.isfinite(t_obs) and np.isfinite(t_ref) and abs(t_ref) > EPS else float("nan")
            g_m = m_obs / m_ref if np.isfinite(m_obs) and np.isfinite(m_ref) and abs(m_ref) > EPS else float("nan")
            if np.isfinite(g_t):
                gains_t.append(g_t)
            if np.isfinite(g_m):
                gains_m.append(g_m)
            per_cmd.append(
                {
                    "cmd": cmd,
                    "thrust_obs_kn": t_obs,
                    "thrust_ref_kn": t_ref,
                    "thrust_gain": g_t,
                    "massflow_obs_kgps": m_obs,
                    "massflow_ref_kgps": m_ref,
                    "massflow_gain": g_m,
                }
            )

        gain_t = float(median(gains_t)) if gains_t else float("nan")
        gain_m = float(median(gains_m)) if gains_m else float("nan")

        diam_ratio = r.engine_diameter_m / ref.engine_diameter_m
        x = math.log(diam_ratio) if diam_ratio > 0 else float("nan")
        x_diam.append(x)
        y_gain_thrust.append(math.log(gain_t) if gain_t > 0 else float("nan"))
        y_gain_mdot.append(math.log(gain_m) if gain_m > 0 else float("nan"))

        tau_up_ratio = r.tau_up_t63_s / ref.tau_up_t63_s if (r.tau_up_t63_s > 0 and ref.tau_up_t63_s > 0) else float("nan")
        tau_dn_ratio = r.tau_down_t63_s / ref.tau_down_t63_s if (r.tau_down_t63_s > 0 and ref.tau_down_t63_s > 0) else float("nan")
        y_tau_up.append(math.log(tau_up_ratio) if tau_up_ratio > 0 else float("nan"))
        y_tau_down.append(math.log(tau_dn_ratio) if tau_dn_ratio > 0 else float("nan"))

        warnings: List[str] = []
        sat = _warn_saturation(r.steady_df)
        if sat:
            warnings.append(sat)

        run_diags.append(
            {
                "log_path": str(r.path),
                "engine_diameter_m": r.engine_diameter_m,
                "intake_diameter_m": r.intake_diameter_m,
                "tau_up_t63_s": r.tau_up_t63_s,
                "tau_down_t63_s": r.tau_down_t63_s,
                "thrust_gain_vs_ref": gain_t,
                "massflow_gain_vs_ref": gain_m,
                "per_cmd": per_cmd,
                "warnings": warnings,
            }
        )

    alpha_thrust = _fit_log_slope(x_diam, y_gain_thrust)
    alpha_mdot = _fit_log_slope(x_diam, y_gain_mdot)
    beta_tau_up = _fit_log_slope(x_diam, y_tau_up)
    beta_tau_down = _fit_log_slope(x_diam, y_tau_down)

    model = {
        "model_kind": "engine_tweakscale_agnostic_v1",
        "reference_run": {
            "log_path": str(ref.path),
            "engine_diameter_m": ref.engine_diameter_m,
            "intake_diameter_m": ref.intake_diameter_m,
        },
        "steady_map_reference": {
            "thrust_vs_cmd": {
                "poly_degree": int(deg),
                "poly_coefficients": [float(x) for x in thrust_coefs],
            },
            "massflow_vs_cmd": {
                "poly_degree": int(deg),
                "poly_coefficients": [float(x) for x in mdot_coefs],
            },
        },
        "scale_laws": {
            "thrust_gain_vs_engine_diameter": {
                "form": "(D_engine / D_ref) ^ alpha_thrust",
                "alpha_thrust": float(alpha_thrust),
            },
            "massflow_gain_vs_engine_diameter": {
                "form": "(D_engine / D_ref) ^ alpha_massflow",
                "alpha_massflow": float(alpha_mdot),
            },
            "tau_up_vs_engine_diameter": {
                "form": "tau_up_ref * (D_engine / D_ref) ^ beta_tau_up",
                "tau_up_ref_t63_s": float(ref.tau_up_t63_s),
                "beta_tau_up": float(beta_tau_up),
            },
            "tau_down_vs_engine_diameter": {
                "form": "tau_down_ref * (D_engine / D_ref) ^ beta_tau_down",
                "tau_down_ref_t63_s": float(ref.tau_down_t63_s),
                "beta_tau_down": float(beta_tau_down),
            },
        },
        "diagnostics": {
            "run_count": len(runs),
            "runs": run_diags,
            "notes": [
                "This model assumes throttle-shape invariance across scales and captures scale effects as gain/tau exponents.",
                "If warnings mention saturation, collect cleaner runs (adequate intake airflow, clear separation between 0.5 and 1.0 steady thrust).",
            ],
        },
    }

    out_path = _resolve_output_path(args.output, runs)
    out_path.write_text(json.dumps(_sanitize_json(model), indent=2), encoding="utf-8")
    print(f"Wrote {out_path}")
    print(f"Reference run: {ref.path.name} @ D={ref.engine_diameter_m:.3f} m")
    print(f"alpha_thrust={alpha_thrust:.4f}  alpha_massflow={alpha_mdot:.4f}")
    print(f"beta_tau_up={beta_tau_up:.4f}  beta_tau_down={beta_tau_down:.4f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
