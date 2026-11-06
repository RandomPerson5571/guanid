# Changelog

## 0.2.0 — 2026-11-06

- Port DriveLab's command-voltage observer to `ProsSysIdRoutine`.
- Add configurable four-profile drivetrain sessions, retry/rollback, in-memory
  fitting, drive-limit estimates and a PROS controller mode with repositioning.
- Expose individual tests for custom controls and optional telemetry/CSV sinks.
- Support independent meter/inch and radian/degree result units, with automatic
  ideal lateral-to-angular feedforward conversion from supplied track width.
- Keep track-width calibration unchanged.
- Limit manual repositioning to 3 V by default and honor lower capture voltage caps;
  allow explicit reposition limits and disable manual motion with zero volts.
- Re-arm after a single sampled L1 release, including cancellation/retry.
- Gate published releases on GCC/Clang sanitizer tests, Windows MSVC tests, and
  fresh PROS 4.2.1/4.2.2 installation/link checks. Maintain a cumulative stable depot.
- Include the three reported Drive Lab physical characterization results and their
  evidence limits in the distributed documentation.
- Add inch/degree capture factories, CSV export, analysis gains and track-width adapter.
- Analyzer reads tagged SI/public CSV and defaults to schema-3 inch/degree gains,
  diagnostics and feedback. `--units si` retains SI output; `--angular-inches`
  takes effective width in inches. Legacy `--angular` still takes metres.

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

See docs/drive-lab-results.md for reported physical characterization and
docs/hardware-validation.md for additional robot-specific validation.
