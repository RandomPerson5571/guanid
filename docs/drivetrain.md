# Drivetrain tuning

Include `guanid/pros_drivetrain.hpp` for a ready-to-use driver test mode:

```cpp
guanid::DrivetrainConfig config;
config.quasistatic.run_id = pros::millis(); // Session ID; increments on every attempt.
config.quasistatic.duration_ms = 3000;
config.quasistatic.interval_ms = 20;
config.quasistatic.ramp_volts_per_second = 2;
config.quasistatic.max_volts = 6;
config.dynamic.duration_ms = 2000;
config.dynamic.interval_ms = 20;
config.dynamic.step_volts = 7;
config.dynamic.max_volts = 12;
config.reposition_max_volts = 3; // Zero disables X + sticks manual movement.
config.output = {guanid::LinearUnit::inches, guanid::AngularUnit::degrees};
config.track_width_m = 13.5 * guanid::metersPerInch;

const auto result = guanid::runDrivetrainTuning(
    leftMotors, rightMotors, controller,
    {3.25 * guanid::metersPerInch, 36.0 / 48.0}, config);
if (result.capture == guanid::Result::completed && result.analysis.ok) {
    const auto left = result.analysis.left.gains();
    const auto right = result.analysis.right.gains();
    const auto lateral = result.analysis.lateral;
    if (result.analysis.angular_valid) {
        const auto angular = result.analysis.angular;
        // Use angular.kS, kV, kA with degrees/s and degrees/s^2 here.
    }
}
```

Use this inside a dedicated opcontrol mode. Both groups must drive forward with
positive voltage, with consistent gearing within a group. Other motor-writing
tasks must yield ownership until it returns. The mode changes every motor's
encoder units to degrees and restores the previous individual units on exit.
It creates no background tasks and requires no LemLib or SD card.

The controller displays the next profile. Release L1 before the first test, then
hold L1 for each profile: ramp forward, ramp reverse, step forward, step reverse.
Release between profiles and reposition as needed. Releasing during a profile
stops it and discards that attempt from the in-memory fit; hold again to retry.
Hold X with L1 released to drive using left-Y/right-X arcade for repositioning.
Manual voltage defaults to 3 V, bounded by both capture profiles' `max_volts`.
Set `reposition_max_volts` in [0,12] to choose a lower limit, or zero to disable it.
B, disconnect, disable, autonomous, or a false optional `permit` callback stops
outputs and exits. All four completed profiles trigger the in-memory analysis.
Capture completion and fit validity are separate: inspect `analysis.ok` and
`analysis.left.si.error` / `analysis.right.si.error` if fitting fails.

Each profile has its own `SysIdRoutine::Config`: duration, sampling interval,
voltage ceiling, ramp slope and step voltage are configurable. Commands clamp
to that profile's `max_volts`. `quasistatic.run_id` seeds the session's attempt
IDs; `dynamic.run_id` is unused by the session. `analysis` controls SI filtering
and fit thresholds (simple model only). `max_records` bounds the record buffer,
defaulting to 4000; increase it for longer captures or faster sampling. Exhausting
it stops with `log_error`, rather than fitting truncated data. `limit_volts`
selects the voltage used for drive-limit estimates, defaulting to 12 V.

The optional sink receives SI `Record`s for CSV/telemetry. Return false to abort
capture, or handle an optional logging failure yourself and return true to keep
the in-memory fit. The sink sees cancelled attempts as they happen; the session
removes them only from its own record buffer. The example
[`drivetrain_sysid.cpp`](../examples/drivetrain_sysid.cpp) shows optional SD
logging that continues without a card. `runDrivetrainTuning` also accepts a
command-voltage observer after the permit callback. It observes capture voltage
requests, including zero, but manual arcade commands are not SysId commands.
A request is observed even if a motor rejects it; measured applied voltage in
each record remains the source for fitting.

## Custom controls

Include `guanid/drivetrain.hpp` for the portable `DrivetrainTuner`. Supply a
`SysIdRoutine::Mechanism` with exactly the SI linear channels `drive-left` and
`drive-right`, plus a `SysIdRoutine::Runtime`. On PROS, the helpers
`prosDrivetrainMechanism(left, right, geometry, optionalCommandObserver)` and
`prosRuntime()` supply these callbacks; configure degree encoders yourself.

```cpp
guanid::DrivetrainTuner tuner(config,
    guanid::prosDrivetrainMechanism(leftMotors, rightMotors, geometry),
    guanid::prosRuntime(), optionalSink);
// Bind your own buttons or UI to any of these:
tuner.startNext();
// Or startQuasistatic(Direction::forward/reverse), startDynamic(...).
// Each combination can complete once; cancelled attempts can be retried.
tuner.update(operatorHoldingEnable && travelWithinLimits);
// Call update at least as frequently as the configured sample interval.
tuner.cancel();
// When tuner.complete(): tuner.analyzeResults().
```

`running()`, `state()`, `result()`, `completedProfiles()`, `complete()` and
`records()` expose state for displays, logging and custom control logic. Starting
while running cancels the current test and returns false. Records remain SI;
while running they include the active attempt, and when idle only completed
attempts remain. Sinks/observers must be fast, must not reenter the tuner, and
cleanup callbacks must not throw. Motor groups and callback dependencies must
outlive the tuner. Lower-level `SysIdRoutine`, `ProsSysIdRoutine`, and their
blocking/nonblocking single-test functions remain available for other mechanisms.
`ProsSysIdRoutine` accepts an optional fifth `CommandObserver(double volts)`
argument, matching DriveLab's command telemetry hook.

## Results and angular conversion

`output.linear` selects `meters` or `inches`; `output.angular` selects `radians`
or `degrees`. These apply independently to result gains, drive limits and
`PublicAnalysis::calculate`. Raw records, analysis thresholds, geometry and
`.si` diagnostics remain SI. `left` and `right` retain fit quality/warnings and
direction-specific SI fits. `lateral` is the arithmetic mean of their gains.
`limits` uses the slower side for straight velocity and acceleration. Braking
is the estimated initial deceleration under full reverse voltage at that common
maximum speed, not a constant stopping cap. Angular speed assumes an in-place
spin and the supplied width. Check `limits.valid` and `limits.angular_valid`.

When `track_width_m > 0`, angular gains are estimated automatically:

```
kS_angular = kS_lateral
kV_angular = kV_lateral * track_width / 2
kA_angular = kA_lateral * track_width / 2
```

Here width uses the lateral distance unit and angular gains initially use
radians. For degree gains, multiply kV/kA by pi/180. The same conversion is
available separately as `lateralToAngular(gains, width, outputUnits)`; width is
in the selected linear unit and gains must already use that unit. Invalid width
or nonfinite gains return NaNs. `gainsInUnits(siGains, dimension, outputUnits)`
converts SI gains without changing their mechanism dimension.

This is an ideal differential-drive estimate at wheel-equivalent voltage.
Turning scrub and unequal directional friction can require a separately
measured angular fit using paired spin records. No track-width calibration is
performed or changed. Leaving width at zero disables angular estimates while
preserving linear results.
