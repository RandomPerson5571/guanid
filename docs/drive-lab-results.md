# Reported Drive Lab physical characterization

These are real-robot characterization results reported by the maintainer on
November 6, 2026, transcribed from VexRobotics
`firmware/drive-lab/SYSID_LOG.md`. They are physical measurements, rather than
simulation output. Drive Lab uses its vendored guanid capture/analysis path;
these sessions do not establish that the new 0.2.0 controller interface itself
was run on hardware.

The record contains fitted gains, voltage RMSEs and operator observations.
Raw CSVs, exact firmware revisions, battery/load/surface conditions, independent
prediction checks and measured stop latencies were not included in the supplied
log. Values labeled as feedforward estimates are model predictions, rather than
measured motion limits. Do not copy these robot-specific gains into another robot.

## 2026-11-06

Wheel-space SI fit results:

- **Left:** RMSE = 0.1925 V; kS = 0.35532 V;
  kV = 7.49289 V s/m; kA = 1.221174 V s²/m.
- **Right:** RMSE = 0.2367 V; kS = 0.37410 V;
  kV = 7.32462 V s/m; kA = 1.21216 V s²/m.

Observation: The drivetrain is not driving completely straight, which may
explain the discrepancies between the left and right results.

## 2026-11-06 - second capture

Wheel-space inch-based fit results, as displayed on the Brain:

- **Left:** RMSE = 0.1866 V; kS = 0.30722 V;
  kV = 0.19053 V s/in; kA = 0.03261 V s²/in.
- **Right:** RMSE = 0.1863 V; kS = 0.35632 V;
  kV = 0.18578 V s/in; kA = 0.02849 V s²/in.

These gains use inches. The [unit guide](units.md) gives the conversions needed
to compare them with the SI fit from the first capture.

## 2026-11-06 - third capture

Wheel-space inch-based fit results, as displayed on the Brain:

- **Left:** RMSE = 0.1627 V; kS = 0.46073 V;
  kV = 0.18816 V s/in; kA = 0.02617 V s²/in.
- **Right:** RMSE = 0.1569 V; kS = 0.45342 V;
  kV = 0.18892 V s/in; kA = 0.02523 V s²/in.

The log records no observed motion peaks or fit R² for this capture.

Reported 12 V feedforward estimates:

- **Maximum velocity:** 61.12 in/s.
- **Linear acceleration:** 440.87 in/s².
- **Braking acceleration:** 915.44 in/s².
- **Angular velocity:** 519.68 deg/s.

Braking acceleration is the estimated initial deceleration under full reverse
voltage at maximum straight-line speed. These are model estimates, separate
from the observed motion peaks.
