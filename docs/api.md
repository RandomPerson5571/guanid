# API and output reference

Preferred robot-facing APIs and output contracts are documented in [units.md](units.md).
`public_units.hpp` adds `analyzePublic`, `writePublicRecord`, reciprocal-gain
conversion and `TrackWidthCalibrationInches`; `pros.hpp` adds `linearInches`
and `angularDegrees`. Existing SI APIs retain their semantics.

Public headers are under `include/guanid/`. All APIs are in namespace `guanid`.

- `pros.hpp`: `ProsSysIdRoutine`, `ProsMotorChannel`, `prosRuntime`, `Geometry`, and the legacy two-sided `run` adapter.
- `sysid.hpp`: portable `SysIdRoutine`, `SysIdLog`, `MotorSample`, `Record`,
  named CSV writers, simple/elevator/arm feedforward and track-width helpers.
- `analysis.hpp`: `analyze`, `AnalysisConfig`, `Gains`, `Fit`, `Analysis`,
  drivetrain channel derivation and optional feedback suggestions.
- `track_width.hpp`: `TrackWidthTrial`, `TrackWidthCalibration`, `TrackWidthResult`.
- `routine.hpp`: legacy portable `runRoutine` mechanism contract.
- `guanid.hpp`: legacy configuration, state, samples, CSV and feedforward.

The main README contains complete usage, units and model equations. Constructors
copy configuration, channels and callbacks; referenced motors/captures must outlive
the routine. A routine is noncopyable and should only be called by its owning
task. Do not concurrently start/update/cancel it from another task; signal the
owner's permit predicate instead. There is no internal synchronization.

`ProsSysIdRoutine(config, channels, sink, observer = {})` accepts a vector or
brace-enclosed list of `ProsMotorChannel` values. It does not inherit from the
portable callback engine and exposes no drive/log callback constructor.

Create channels with `ProsMotorChannel::linear(name, motorOrGroup, geometry,
drive_direction = 1)` or `ProsMotorChannel::angular(name, motorOrGroup,
output_rev_per_motor_rev = 1, horizontal_offset_rad = 0, drive_direction = 1)`.
Only `pros::Motor&` and `pros::MotorGroup&` are accepted. Both factories read degree
encoders and measured voltage internally; group telemetry is averaged. A direction
of `-1` reverses only commanded voltage, preserving measured telemetry signs.
Channel names must be unique and nonempty. Invalid channel settings and repeated
motor objects invalidate the configuration; no nonzero command is sent. All
channels receive a stop attempt even if one fails. Do not use overlapping groups.

`startQuasistatic`/`startDynamic` return bool. `update`, blocking `quasistatic`/
`dynamic`, and `cancel` return Result. Results are `running`, `completed`,
`cancelled`, `invalid_config`, `hardware_error`, or `log_error`. A failed cleanup
stop reports hardware_error even if another failure triggered cleanup. Observers
receive the active test state and idle on finish. Callbacks must be fast and
observer callbacks must not throw.

## Captures and analysis

`writeRecordHeader(FILE*)`/`writeRecord(FILE*,Record)` return bool. Check buffered
flush/close too. Named CSV is one channel per row:

`run_id,test,direction,time_s,motor,units,voltage,position,velocity`

Fields are unquoted. Channel names allow letters, digits, hyphen, underscore and
period. Test is quasistatic/dynamic, direction forward/reverse, position units
m/rad, velocity units m/s or rad/s. Arm angles use radians from horizontal. Each
run must have unique identity and increasing time per channel. Clock zero is the
start of that test. No CSV or console output is automatic unless your sink writes
it. The example uses `/usd/guanid.csv`; in-memory sinks need no file.

`analyze(records,name,settings)` returns an Analysis with `ok`/`error`. On success,
read `gains`, `samples`, `voltage_rmse`, `acceleration_r_squared`,
`voltage_r_squared`, `excitation_ratio`, `forward`, `reverse`, `warnings`, and
`diagnostics`. Inspect ok before using gains. Undefined R² is NaN. Diagnostic
samples contain the original Record, filtered velocity, estimated acceleration,
predicted voltage and residual. Filter/trim configuration uses SI units/seconds.
Gravity/friction cannot be separated in a one-direction elevator fit; the joint
fit needs both directions.

The desktop analyzer writes JSON schema version 2, with model, settings, and a
channels object keyed by name. Each channel has position_unit, joint fit,
forward/reverse fits, warnings, and optional velocity_feedback/position_feedback.
Undefined JSON metrics are null. With `--plot-dir`, each channel also gets an SVG
and processed diagnostic CSV. Exit status is 0 on success, 2 on invalid input or
an output failure. See `guanid_analyze --help` for command options. Preserve raw
captures: processed plots intentionally contain only retained samples.

TrackWidthCalibration returns an explicit result instead of printing. valid means
at least one trial was accepted; track_width_m is the arithmetic mean and
sample_stddev_m reports trial spread. This is a calibration measurement, not a
proof that width is constant across surfaces or loads.
