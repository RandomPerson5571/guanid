#!/usr/bin/env python3
"""Offline guanid CSV analysis. No robot connection and no WPILog dependency."""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

TESTS = {(test, direction) for test in ("quasistatic", "dynamic")
         for direction in ("forward", "reverse")}
NUMERIC = ["elapsed_s", "commanded_volts"] + [
    f"{side}_{field}" for side in ("left", "right")
    for field in ("applied_volts", "position_m", "velocity_mps")]


def load_runs(paths):
    """Preserve file/run boundaries: never differentiate across separate tests."""
    runs = []
    for path in paths:
        with Path(path).open(newline="", encoding="utf-8-sig") as stream:
            reader = csv.DictReader(stream)
            required = {"test", "direction", *NUMERIC}
            if not required.issubset(reader.fieldnames or []):
                raise ValueError(f"{path}: missing CSV columns: {sorted(required - set(reader.fieldnames or []))}")
            current = None
            seen = set()
            for line, row in enumerate(reader, 2):
                try:
                    label = row["test"], row["direction"]
                    if label not in TESTS or None in row:
                        raise ValueError("invalid test/direction or extra columns")
                    values = {name: float(row[name]) for name in NUMERIC}
                    if not all(math.isfinite(value) for value in values.values()):
                        raise ValueError("nonfinite measurement")
                    if values["elapsed_s"] < 0:
                        raise ValueError("negative time")
                    for name in ("commanded_volts", "left_applied_volts", "right_applied_volts"):
                        if abs(values[name]) > 12:
                            raise ValueError("voltage outside V5 range")
                    key = (*label, row.get("run_id", "0"))
                    if current is None or current["key"] != key:
                        if key in seen:
                            raise ValueError("noncontiguous run; assign a unique run_id")
                        current = {"key": key, "source": str(path), "rows": []}
                        runs.append(current)
                        seen.add(key)
                    if current["rows"] and values["elapsed_s"] <= current["rows"][-1]["elapsed_s"]:
                        raise ValueError("timestamps must strictly increase; use a new run_id for repeated tests")
                    current["rows"].append(values)
                except (ValueError, TypeError, KeyError) as error:
                    raise ValueError(f"{path}:{line}: {error}") from error
    if not runs:
        raise ValueError("no samples")
    return runs


def prepare(run, side, *, median_window=5, derivative_window=5,
            min_velocity=0.02, trim_start=0.05, max_gap=0.1):
    rows = run["rows"]
    t = np.array([row["elapsed_s"] for row in rows])
    raw = np.array([row[f"{side}_velocity_mps"] for row in rows])
    volts = np.array([row[f"{side}_applied_volts"] for row in rows])
    velocity = raw.copy()
    radius = median_window // 2
    for i in range(radius, len(raw) - radius):
        velocity[i] = np.median(raw[i - radius:i + radius + 1])
    half = derivative_window // 2
    acceleration = np.full(len(raw), np.nan)
    for i in range(half + radius, len(raw) - half - radius):
        window = slice(i - half, i + half + 1)
        support = slice(i - half - radius, i + half + radius + 1)
        # Exclude windows straddling dropouts, rest, or direction changes.
        if (t[i] - t[0] < trim_start or np.max(np.diff(t[support])) > max_gap or
                np.any(np.abs(raw[support]) <= min_velocity) or
                np.any(np.sign(raw[support]) != np.sign(raw[i]))):
            continue
        local_t = t[window] - t[i]
        acceleration[i] = np.polynomial.polynomial.polyfit(local_t, velocity[window], 2)[1]
    keep = np.isfinite(acceleration) & (np.abs(velocity) > min_velocity)
    return {"run": run, "side": side, "time": t, "raw_velocity": raw,
            "velocity": velocity, "volts": volts, "acceleration": acceleration, "keep": keep}


def r_squared(actual, predicted):
    variance = float(np.sum((actual - np.mean(actual)) ** 2))
    return None if variance <= 1e-20 else 1 - float(np.sum((actual - predicted) ** 2)) / variance


def fit(prepared):
    kept = [part for part in prepared if np.any(part["keep"])]
    if not kept:
        raise ValueError("no moving samples after filtering")
    velocity = np.concatenate([part["velocity"][part["keep"]] for part in kept])
    voltage = np.concatenate([part["volts"][part["keep"]] for part in kept])
    acceleration = np.concatenate([part["acceleration"][part["keep"]] for part in kept])
    if len(velocity) < 12:
        raise ValueError("need at least 12 retained samples")
    # WPILib-style acceleration regression: a = bU*U + bV*v + bS*sign(v).
    # Scale columns before SVD least squares to avoid unit-dependent rank tests.
    x = np.column_stack((voltage, velocity, np.sign(velocity)))
    scale = np.linalg.norm(x, axis=0)
    if np.any(scale == 0) or not np.all(np.isfinite(scale)):
        raise ValueError("unexcited model: missing voltage or motion")
    coefficients, _, rank, singular = np.linalg.lstsq(x / scale, acceleration, rcond=None)
    condition = float(singular[0] / singular[-1]) if singular[-1] > 0 else math.inf
    if rank < 3 or condition > 1e6:
        raise ValueError("rank-deficient or ill-conditioned data; capture ramps and acceleration transients")
    b_u, b_v, b_s = coefficients / scale
    if b_u <= 0 or not np.isfinite(b_u):
        raise ValueError("nonpositive voltage-to-acceleration gain; check signs, timing, and excitation")
    k_a, k_v, k_s = 1 / b_u, -b_v / b_u, -b_s / b_u
    predicted_voltage = k_s * np.sign(velocity) + k_v * velocity + k_a * acceleration
    metrics = {"acceleration_r_squared": r_squared(acceleration, x @ (coefficients / scale)),
               "voltage_rmse_v": float(np.sqrt(np.mean((voltage - predicted_voltage) ** 2))),
               "scaled_condition_number": condition}
    observations = {"max_abs_velocity_mps": float(np.max(np.abs(velocity))),
                    "max_abs_filtered_acceleration_mps2": float(np.max(np.abs(acceleration)))}
    warnings = []
    if k_s < 0 or k_v <= 0:
        warnings.append("Nonphysical kS/kV; verify signs and measurements before using these gains.")
    if metrics["acceleration_r_squared"] is None or metrics["acceleration_r_squared"] < 0.2:
        warnings.append("Weak acceleration fit; kA may be unreliable.")
    diagnostics = []
    for part in prepared:
        keep = part["keep"]
        v, a, u = part["velocity"][keep], part["acceleration"][keep], part["volts"][keep]
        expected_sign = 1 if part["run"]["key"][1] == "forward" else -1
        if len(v) and np.mean(np.sign(v) != expected_sign) > 0.1:
            warnings.append(f"{part['run']['key']}: measured motion disagrees with test direction.")
        diagnostics.append({"source": part["run"]["source"], "test": part["run"]["key"][0],
            "direction": part["run"]["key"][1], "run_id": part["run"]["key"][2],
            "samples_total": len(keep), "samples_used": int(np.sum(keep)),
            "voltage_rmse_v": float(np.sqrt(np.mean((u - (k_s * np.sign(v) + k_v * v + k_a * a)) ** 2))) if len(v) else None})
    return {"kS": float(k_s), "kV": float(k_v), "kA": float(k_a), "metrics": metrics,
            "observations": observations,
            "samples_used": len(velocity), "warnings": warnings, "runs": diagnostics}


def analyze(runs, *, allow_incomplete=False, **settings):
    sides, all_prepared = {}, {}
    for side in ("left", "right"):
        prepared = [prepare(run, side, **settings) for run in runs]
        available = {part["run"]["key"][:2] for part in prepared if np.any(part["keep"])}
        missing = sorted(TESTS - available)
        if missing and not allow_incomplete:
            raise ValueError(f"{side}: missing usable tests {missing}; collect all four or use --allow-incomplete")
        result = fit(prepared)
        if missing:
            result["warnings"].append(f"Incomplete test coverage: {missing}")
        sides[side], all_prepared[side] = result, prepared
    return {"schema_version": 1, "model": "V = kS*sign(v) + kV*v + kA*a",
            "units": {"kS": "V", "kV": "V/(m/s)", "kA": "V/(m/s^2)"},
            "settings": settings, "sides": sides}, all_prepared


def plot_diagnostics(report, prepared, directory):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    for side, parts in prepared.items():
        gains = report["sides"][side]
        for index, part in enumerate(parts):
            keep, t = part["keep"], part["time"]
            v, a = part["velocity"], part["acceleration"]
            predicted = gains["kS"] * np.sign(v) + gains["kV"] * v + gains["kA"] * a
            fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
            axes[0].plot(t, part["raw_velocity"], label="Raw")
            axes[0].plot(t, v, label="Median filtered")
            axes[0].set_ylabel("Velocity (m/s)")
            axes[1].plot(t, part["volts"], label="Delivered")
            axes[1].plot(t[keep], predicted[keep], ".", label="Model")
            axes[1].set_ylabel("Voltage (V)")
            axes[2].plot(t[keep], (part["volts"] - predicted)[keep], label="Voltage residual")
            axes[2].axhline(0, color="gray", linewidth=0.7)
            axes[2].set_ylabel("Residual (V)")
            axes[2].set_xlabel("Elapsed time (s)")
            for axis in axes:
                axis.legend()
                axis.grid(alpha=0.2)
            fig.suptitle(f"{side}: {' / '.join(part['run']['key'])}")
            fig.tight_layout()
            fig.savefig(directory / f"{side}-{index:03d}.png", dpi=150)
            plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="+", type=Path)
    parser.add_argument("--out", type=Path, required=True, help="JSON gains and diagnostics")
    parser.add_argument("--plot-dir", type=Path, help="Optional diagnostic PNGs (requires matplotlib)")
    parser.add_argument("--median-window", type=int, default=5)
    parser.add_argument("--derivative-window", type=int, default=5)
    parser.add_argument("--min-velocity", type=float, default=0.02, help="m/s motion threshold")
    parser.add_argument("--trim-start", type=float, default=0.05, help="Seconds to trim from each run")
    parser.add_argument("--max-gap", type=float, default=0.1, help="Reject derivative windows with larger gaps (s)")
    parser.add_argument("--allow-incomplete", action="store_true")
    args = parser.parse_args()
    if args.median_window < 1 or args.median_window % 2 != 1:
        parser.error("--median-window must be a positive odd integer")
    if args.derivative_window < 3 or args.derivative_window % 2 != 1:
        parser.error("--derivative-window must be odd and at least 3")
    for name in ("min_velocity", "trim_start", "max_gap"):
        value = getattr(args, name)
        if not math.isfinite(value) or value < 0 or (name == "max_gap" and value == 0):
            parser.error(f"invalid --{name.replace('_', '-')}")
    try:
        report, prepared = analyze(load_runs(args.csv), allow_incomplete=args.allow_incomplete,
            median_window=args.median_window, derivative_window=args.derivative_window,
            min_velocity=args.min_velocity, trim_start=args.trim_start, max_gap=args.max_gap)
        if args.plot_dir:
            plot_diagnostics(report, prepared, args.plot_dir)
        args.out.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    except (ValueError, OSError, ImportError) as error:
        parser.exit(2, f"guanid: {error}\n")
    for side, result in report["sides"].items():
        print(f"{side}: kS={result['kS']:.6g} V, kV={result['kV']:.6g} V/(m/s), kA={result['kA']:.6g} V/(m/s^2)")
        for warning in result["warnings"]:
            print(f"  warning: {warning}")


if __name__ == "__main__":
    main()
