# Changelog

Versions follow Semantic Versioning. Until 1.0, minor releases may change APIs;
patch releases preserve existing public interfaces unless fixing incorrect behavior.

## 0.1.0 — initial release candidate

- Named PROS Motor/MotorGroup channels with internal voltage commands and encoder
  telemetry; blocking and periodic characterization without subsystems or LemLib.
- Quasistatic and dynamic forward/reverse tests, voltage limits, cancellation,
  state observers, and stop-on-exit handling.
- Named linear/angular telemetry and CSV capture, with legacy drivetrain APIs.
- Pure C++ simple/elevator/arm regression, gravity gains, fit diagnostics,
  per-direction comparisons, feedforward, and optional feedback suggestions.
- Standalone C++ analyzer with JSON, SVG, diagnostic CSV, and drivetrain transforms.
- Effective-track-width calibration, repeated-trial statistics, and snapshot helpers.
- Header-only PROS Conductor template, reproducible release archives/checksums,
  package ownership checks, consumer build CI, and installation/release guides.

Hardware validation is still required; see docs/hardware-validation.md.
