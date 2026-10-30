import csv
import importlib.util
import tempfile
import unittest
from pathlib import Path

import numpy as np

SPEC = importlib.util.spec_from_file_location("analyze", Path(__file__).parents[1] / "tools/analyze.py")
analyze = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(analyze)


def synthetic_runs():
    """Analytic solutions with jittered timestamps and independent side gains."""
    rng = np.random.default_rng(4)
    runs = []
    for test, direction in sorted(analyze.TESTS):
        sign = 1 if direction == "forward" else -1
        times = np.concatenate(([0.0], np.cumsum(rng.uniform(0.017, 0.023, 200))))
        rows = []
        for t in times:
            row = {"elapsed_s": t, "commanded_volts": sign * (3 if test == "dynamic" else 1.5 * t)}
            for side, (ks, kv, ka) in {"left": (0.25, 2.1, 0.45), "right": (0.3, 2.4, 0.6)}.items():
                tau = ka / kv
                if test == "dynamic":
                    voltage = 3
                    velocity = (voltage - ks) / kv * (1 - np.exp(-t / tau))
                    position = (voltage - ks) / kv * (t - tau * (1 - np.exp(-t / tau)))
                else:
                    voltage = 1.5 * t
                    moving_t = max(0, t - ks / 1.5)
                    velocity = 1.5 / kv * (moving_t - tau * (1 - np.exp(-moving_t / tau)))
                    position = 1.5 / kv * (moving_t ** 2 / 2 - tau * moving_t + tau ** 2 * (1 - np.exp(-moving_t / tau)))
                row.update({f"{side}_applied_volts": sign * voltage,
                            f"{side}_velocity_mps": sign * velocity, f"{side}_position_m": sign * position})
            rows.append(row)
        runs.append({"key": (test, direction, "1"), "source": "synthetic", "rows": rows})
    return runs


class AnalysisTests(unittest.TestCase):
    def test_recovers_known_gains(self):
        report, _ = analyze.analyze(synthetic_runs(), median_window=1)
        for side, expected in {"left": (0.25, 2.1, 0.45), "right": (0.3, 2.4, 0.6)}.items():
            fit = report["sides"][side]
            for name, value in zip(("kS", "kV", "kA"), expected):
                self.assertAlmostEqual(fit[name], value, delta=value * 0.025)
            self.assertGreater(fit["metrics"]["acceleration_r_squared"], 0.99)

    def test_noise_and_filtering(self):
        runs = synthetic_runs()
        rng = np.random.default_rng(7)
        for run in runs:
            for row in run["rows"]:
                for side in ("left", "right"):
                    row[f"{side}_velocity_mps"] += rng.normal(0, 0.003)
        report, _ = analyze.analyze(runs)
        self.assertAlmostEqual(report["sides"]["left"]["kA"], 0.45, delta=0.045)

    def test_requires_four_usable_tests(self):
        with self.assertRaisesRegex(ValueError, "missing usable tests"):
            analyze.analyze(synthetic_runs()[:2])

    def test_rejects_unexcited_data(self):
        runs = synthetic_runs()
        for run in runs:
            for row in run["rows"]:
                for side in ("left", "right"):
                    row[f"{side}_velocity_mps"] = 1
                    row[f"{side}_applied_volts"] = 3
        with self.assertRaisesRegex(ValueError, "rank-deficient"):
            analyze.analyze(runs)

    def test_gap_and_reversal_windows_are_excluded(self):
        run = synthetic_runs()[0]
        run["rows"][40]["left_velocity_mps"] *= -1
        for row in run["rows"][100:]:
            row["elapsed_s"] += 0.5
        part = analyze.prepare(run, "left", median_window=1)
        self.assertFalse(any(part["keep"][38:43]))
        self.assertFalse(any(part["keep"][98:102]))

    def test_csv_validation_and_boundaries(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "data.csv"
            runs = synthetic_runs()
            with path.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=["test", "direction", "run_id", *analyze.NUMERIC])
                writer.writeheader()
                for run in runs:
                    for row in run["rows"]:
                        writer.writerow(dict(test=run["key"][0], direction=run["key"][1], run_id=1, **row))
            self.assertEqual(len(analyze.load_runs([path])), 4)
            self.assertEqual(len(analyze.load_runs([path, path])), 8)
            contents = path.read_text()
            # Original guanid schema has no run_id column.
            legacy = list(csv.reader(contents.splitlines()))
            run_id_column = legacy[0].index("run_id")
            for row in legacy:
                del row[run_id_column]
            with path.open("w", newline="") as stream:
                csv.writer(stream).writerows(legacy)
            self.assertEqual(len(analyze.load_runs([path])), 4)
            malformed = list(csv.reader(contents.splitlines()))
            malformed[1][malformed[0].index("elapsed_s")] = "nan"
            with path.open("w", newline="") as stream:
                csv.writer(stream).writerows(malformed)
            with self.assertRaisesRegex(ValueError, "nonfinite"):
                analyze.load_runs([path])
            # Duplicate timestamps within a run are never silently concatenated.
            lines = contents.splitlines()
            path.write_text("\n".join([lines[0], lines[1], lines[1]]) + "\n")
            with self.assertRaisesRegex(ValueError, "strictly increase"):
                analyze.load_runs([path])


if __name__ == "__main__":
    unittest.main()
