# Drivetrain sessions and limits

[API index](../api.md) · [Controller walkthrough](../drivetrain.md)

## `drivetrain.hpp`

### `DrivetrainConfig`

The default constructor starts from two
[`SysIdRoutine::Config`](capture.md#nested-types-and-callbacks) values.
Public fields:

- `SysIdRoutine::Config quasistatic` controls both ramps. The constructor
  sets its duration to 3000 ms and slope to 2 V/s. Its `run_id` seeds attempt
  IDs for the whole session.
- `SysIdRoutine::Config dynamic` controls both steps. Its duration is set to
  2000 ms. The named routine defaults supply the 7 V step. The session uses
  the quasistatic seed for both profile types.
- `AnalysisConfig analysis` controls SI filtering and fit thresholds.
  The drivetrain session requires `Model::simple`.
- `OutputUnits output` selects linear and angular result units
  independently, defaulting to inches and degrees.
- `double track_width_m = 0` is a supplied effective width in metres.
  Positive width enables ideal angular estimates. Zero skips them.
- `double limit_volts = 12` is the assumed voltage for model-based limit
  estimates. Valid range is (0, 12] V.
- `double reposition_max_volts = 3` is the PROS manual repositioning ceiling.
  Valid range is [0, 12] V. The driver mode also applies both capture caps.
  Zero disables manual motion.
- `std::size_t max_records = 4000` caps the number of stored channel rows
  across the session. Each normal sample adds two rows. Use a positive count.

`bool valid() const` checks both profile configurations, the analysis
settings and model, the output selectors, the finite voltage and width
ranges, and the record cap.

### `DriveLimits` and `estimateDriveLimits`

`DriveLimits` has the following public fields:

- `bool valid = false` indicates that the linear estimates were computed.
- `double velocity = 0` is estimated maximum straight speed in the selected
  linear unit per second.
- `double acceleration = 0` is estimated near-rest straight acceleration in
  the selected linear unit per second squared.
- `double braking_acceleration = 0` is the initial deceleration magnitude
  under full reverse voltage at the estimated maximum straight speed, in
  the same acceleration units.
- `bool angular_valid = false` indicates that the turn speed estimate is available.
- `double angular_velocity = 0` is estimated in-place turn speed in the selected
  angular unit per second.

```cpp
DriveLimits estimateDriveLimits(const Fit& left, const Fit& right,
                               double trackWidthM = 0, OutputUnits output = {},
                               double volts = 12);
```

`left` and `right` are successful SI simple-model fits of each side. Each
must have finite `kS` in [0, volts) and finite positive `kV` and `kA`.
`trackWidthM` is the nonnegative effective width in metres. Zero skips the
angular estimate. `output` selects the returned units, and `volts` is the
finite voltage assumed available in (0, 12] V.

The function returns the lower side estimate for straight speed, acceleration,
and braking separately. Turn speed uses the sum of the side speed estimates
divided by the width. Invalid input or nonfinite results return the default
invalid `DriveLimits`.

### `DrivetrainAnalysis`

Public fields returned by `analyzeResults()`:

- `bool ok = false` indicates both side fits succeeded.
- `PublicAnalysis left` and `PublicAnalysis right` are each side's fit,
  with SI diagnostics and gains available in the selected output units.
- `Gains lateral` contains the arithmetic mean of the converted side gains.
- `Gains angular` contains the ideal turn gains in the selected angular
  units. Check `bool angular_valid = false`, which indicates that the
  session had successful fits and a positive supplied width.
- `DriveLimits limits` holds the estimated speed, acceleration, and braking
  results. Check its validity fields separately from `ok`.

### `DrivetrainTuner`

Portable, noncopyable four-test session. The destructor attempts to cancel an
active test. The class has no public data members.

```cpp
DrivetrainTuner(DrivetrainConfig config, SysIdRoutine::Mechanism mechanism,
                SysIdRoutine::Runtime runtime, Sink sink = {},
                SysIdRoutine::Observer observer = {});
```

- `config` sets profile, buffer, output, and fit options.
- `mechanism` applies signed volts and logs two stable SI linear channels
  named `drive-left` and `drive-right`. Its callbacks follow the
  [portable runner contract](capture.md#nested-types-and-callbacks).
- `runtime` supplies the clock and delay functions required by the engine.
- `sink` is an optional `DrivetrainTuner::Sink`, an alias of
  `SysIdRoutine::Sink`. It receives each SI record with the session's
  attempt ID. Returning false aborts the attempt with `log_error` if the
  stop succeeds. The sink may have already saved rows from failed attempts.
- `observer` is an optional active/idle state callback for the engine.

Public methods:

- `bool startNext()` attempts the first unfinished profile in this order:
  ramp forward, ramp reverse, step forward, step reverse. Returns false when
  none remain.
- `bool startQuasistatic(Direction d)` and `bool startDynamic(Direction d)`
  attempt the ramp or step in direction `d`. A completed combination cannot
  be repeated. A cancelled combination may be retried. Start methods return
  true only if a profile starts. If a profile is already running, a start
  request cancels it and returns false.
- `Result update(bool permit = true)` advances an active profile and returns
  its status. False cancels it. Supply the current operator and mechanism
  permission and call at the sample interval or faster.
- `Result cancel()` stops an active profile and returns its status. Both
  `update()` and `cancel()` return the stored status when idle.
- `bool running() const` reports whether an attempt is active.
- `Result result() const` returns the latest status. The initial value is
  `completed`. Use `complete()` to check the full session.
- `State state() const` returns the active state or idle.
- `unsigned completedProfiles() const` counts completed combinations (0–4).
- `bool complete() const` reports completion of all four combinations.
- `const std::vector<Record>& records() const` returns the tuner-owned SI
  buffer. While running it includes rows from the active attempt. Failed or
  cancelled attempts are removed when they finish. Copy rows to retain them
  across buffer changes or after destroying the tuner.
- `DrivetrainAnalysis analyzeResults() const` fits both sides and calculates
  optional estimates. It returns the default invalid result unless the
  session is complete, idle, valid, and has final status `completed`.

An exhausted record cap stops the attempt with `log_error` if cleanup succeeds.

## `pros_drivetrain.hpp`

### `prosDrivetrainMechanism`

```cpp
SysIdRoutine::Mechanism prosDrivetrainMechanism(
    pros::MotorGroup& left, pros::MotorGroup& right, Geometry geometry,
    ProsSysIdRoutine::CommandObserver command = {});
```

`left` and `right` are borrowed groups. Set degree encoders and signs so
positive voltage drives forward on both sides. `geometry` supplies the
shared SI diameter and wheel turn ratio. `command` optionally receives the
signed voltage request after both group writes are attempted, including zero.
The returned mechanism commands both groups and logs SI `drive-left` and
`drive-right` channels. Its callbacks report success as true and should be
used with [`prosRuntime()`](pros.md#prosruntime) or equivalent timing callbacks.

### `ProsTuningResult` and `runDrivetrainTuning`

`ProsTuningResult` exposes:

- `Result capture = Result::cancelled` for the final capture or cleanup status.
- `unsigned completed_profiles = 0` for the number of completed combinations.
- `DrivetrainAnalysis analysis` for the fit and estimated limits, populated
  when capture and cleanup succeed. Check `analysis.ok` for fit success.

```cpp
ProsTuningResult runDrivetrainTuning(
    pros::MotorGroup& left, pros::MotorGroup& right, pros::Controller& controller,
    Geometry geometry, DrivetrainConfig config = {}, DrivetrainTuner::Sink sink = {},
    std::function<bool()> permit = [] { return true; },
    ProsSysIdRoutine::CommandObserver command = {});
```

- `left` and `right` are separate borrowed groups, both positive-forward.
- `controller` is the borrowed PROS controller used for input and prompts.
- `geometry` supplies the shared SI diameter and wheel turn ratio.
- `config` sets capture, analysis, and repositioning options.
- `sink` optionally receives SI records as the capture progresses. False
  aborts the attempt. For best-effort logging, handle your write failure
  and return true to continue fitting in memory.
- `permit` is an optional additional interlock called without arguments.
  Its default returns true. Return false to end the session.
- `command` optionally observes voltage requests through the capture
  adapter, including its zero requests. Manual arcade drive voltages use
  direct motor writes.

The call blocks in opcontrol. It temporarily sets degree encoders, checks
controller connection and competition state, and attempts to stop and restore
previous encoder units on exit. L1 starts or holds each profile. Release L1
to cancel an active attempt or re-arm for the next profile. X plus the sticks
repositions between attempts. B exits. The returned `capture` can report a
hardware failure during setup or cleanup. A completed capture can still yield
an unsuccessful fit.
