# GuanID C++ API reference

This reference covers the public C++ interface in GuanID 0.2.0. Symbols live
in namespace `guanid` and headers are included as `<guanid/header.hpp>`.
`guanid::detail` contains implementation helpers. Portable headers require
C++17. The two PROS headers also require the robot project's PROS toolchain.

## C++ reference by topic

- [Shared types and legacy capture](api/core.md) covers `guanid.hpp` and `routine.hpp`.
- [Named capture and feedforward](api/capture.md) covers `sysid.hpp`.
- [Analysis and output units](api/analysis.md) covers `analysis.hpp` and `public_units.hpp`.
- [Track-width calibration](api/track-width.md) covers `track_width.hpp` and its inch adapter in `public_units.hpp`.
- [PROS motor channels and routines](api/pros.md) covers `pros.hpp`.
- [Drivetrain sessions and limits](api/drivetrain.md) covers `drivetrain.hpp` and `pros_drivetrain.hpp`.

## Units and runtime conventions

Capture records, fit settings, and geometry use metres or radians, with time in
seconds unless a field is marked `_ms`.

PROS motor output takes millivolts. The
named capture engine requests volts and the PROS adapters perform the conversion.
The public-unit adapters expose inch/degree output by default. See the
[unit guide](units.md) for examples.

Capture callbacks run synchronously on the calling task. Keep them fast, avoid
reentering the routine, and ensure stop and observer callbacks do not throw.
Keep borrowed motors and callback dependencies alive for the routine's lifetime.
Give the calling task exclusive control of the motors during capture. Use the
[hardware validation protocol](hardware-validation.md) before powered testing.

## Desktop analyzer output

The standalone `guanid_analyze` executable reads the named CSV schema and the
legacy left/right SI schema. It normalizes input to SI, then converts the output.
`--units in-deg` is the default and writes JSON schema version 3. `--units si`
writes schema version 2. Use `--help` for the argument list and see the
[unit guide](units.md) for width and velocity threshold arguments.

JSON contains `schema_version`, `model`, `settings`, and a `channels` object
keyed by channel name. Each channel identifies its position, velocity,
acceleration, and gain units, reports its effective `min_velocity`, and
and includes the joint `fit`, `forward` and `reverse` fits, and `warnings`.

Requested feedback estimates appear in `velocity_feedback` and
`position_feedback`. Undefined fit metrics appear as JSON `null`.

With `--plot-dir`, the analyzer writes an SVG and processed diagnostic CSV per
channel using retained samples. Keep the raw CSV to inspect discarded samples.

Exit status is 0 on success or `--help`. Invalid input, a failed joint fit for
a selected channel, or an output error returns 2 with a message on stderr.
