# Robot-facing output units

Use `ProsMotorChannel::linearInches(name, motors, wheelDiameterIn,
wheelTurnsPerMotorTurn, driveDirection)` or
`angularDegrees(name, motors, outputTurnsPerMotorTurn, horizontalOffsetDeg,
driveDirection)`. The optional direction defaults to +1. Motor encoder units
must still be degrees. The existing factories and `Geometry` remain SI APIs.

Use `writeRecordHeader(file)` once and `writePublicRecord(file, row)` for
inch/degree CSV by default. Pass an `OutputUnits` third argument to select
meters/inches and radians/degrees independently. Both CSV writers receive SI
`Record` objects. Use `writeRecord` for SI output. For example, this exports
wheel data in inches and angle data in radians:

```cpp
const guanid::OutputUnits units{guanid::LinearUnit::inches, guanid::AngularUnit::radians};
const bool written = guanid::writePublicRecord(file, row, units);
```

For in-memory results include `guanid/public_units.hpp` and call
`analyzePublic(records, name, configSI, outputUnits)`. The optional `OutputUnits`
argument defaults to `{LinearUnit::inches, AngularUnit::degrees}`. Use
`LinearUnit::meters` and/or `AngularUnit::radians` for SI output on either axis. Check `result.si.ok` before using
`result.gains()` (selected units), `result.positionUnit()`, or
`result.calculate(position, velocity, acceleration)` (volts). Its `.si` member
explicitly exposes the SI fit, warnings and diagnostic records. Configuration
thresholds for this adapter retain SI units, as the `configSI` argument states.
Arm positions for `calculate` use the selected angular unit from horizontal; gravity is converted
to radians internally. Do not call the SI `Gains::calculate` arm overload with
degree positions. `kS` and `kG` stay in volts. Converting SI `kV` and `kA` to
inch gains multiplies them by 0.0254. Converting to degree gains multiplies them
by π/180. The standalone CLI offers the two presets `in-deg` and `si`.

The CLI accepts named CSV unit tokens `m`, `rad`, `in`, and `deg`; every record
is normalized before derivatives, pairing and fitting. Incompatible dimensions
in a channel are rejected. Legacy left/right `_m`/`_mps` CSV remains readable.

```sh
guanid_analyze straight.csv --combine drive-left drive-right drive \
  --motor drive --out linear-gains.json
guanid_analyze turns.csv --angular-inches drive-left drive-right turn 13.5 \
  --motor turn --out angular-gains.json
```

Default `--units in-deg` emits JSON `schema_version: 3`; channels explicitly name
position, velocity, acceleration and gain units. Gains, optional voltage-output
feedback gains, diagnostic CSV values and SVG velocity axes use those units.
Channel `min_velocity` records its effective threshold. With no explicit
threshold, the original physical default is preserved separately for each axis;
the top-level setting is null and `min_velocity_explicit` is false. When supplied,
`--min-velocity` uses output-axis units/s. Timestamps, voltage/RMSE, R², run IDs
and filtering windows retain their meaning. `--position-omega` is a design
frequency in rad/s, not a robot heading rate, and retains that convention.

Use `--units si` for schema-2 SI models and SI thresholds. Existing `--angular`
continues to accept width in metres; `--angular-inches` is the preferred public
width option. The optional Python legacy analyzer remains SI-only.

`TrackWidthCalibrationInches::addSpin` takes signed wheel snapshots in inches
and independently measured continuous headings in degrees. `result()` returns
`trackWidthIn`, `sampleStddevIn`, `minWidthIn`, `maxWidthIn`, validity and trial
count. Do not infer that rotation from wheel encoders and an assumed width.

The unit-contract changes ship in 0.2.0. Published versions are immutable;
`--units si` remains available when consuming the older SI output contract.

For automatic lateral-to-angular kS/kV/kA conversion and configurable driver
tests, see [drivetrain tuning](drivetrain.md).
