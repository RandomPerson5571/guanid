# guanid

System identification for VEX V5 / PROS, in pure C++. Configure a routine with
named PROS motor channels, run four operator-controlled tests, then fit
feedforward gains in your C++ code or with the bundled C++ analyzer. No subsystem,
command scheduler, WPILib, Python, or background task is required.

Models:

- Simple motor, drivetrain, flywheel: `V = kS*sign(v) + kV*v + kA*a`
- Elevator: `V = kS*sign(v) + kV*v + kA*a + kG`
- Arm: `V = kS*sign(v) + kV*v + kA*a + kG*cos(angle)`

The portable library is header-only C++17. The PROS adapter uses your installed
kernel's C++ standard (the repository's PROS headers compile with C++23). The
standalone analyzer is C++17 and uses only the standard library. guanid CSV is
**not WPILog**; use guanid's analyzer rather than importing it into WPILib SysId.

## PROS motor routine

Install `guanid@0.1.0.zip` from a release (or build it locally with `make template`):

```sh
# Run inside your existing V5 PROS 4.2.1+ project:
pros conduct fetch /absolute/path/to/guanid@0.1.0.zip
pros conduct apply guanid@0.1.0
pros make
```

See [installation and upgrades](docs/installation.md),
[compatibility](docs/compatibility.md), and [release packaging](docs/releasing.md).
The template is header-only and installs only `include/guanid/` and
`share/guanid/`; it adds no application entry points or dependency archives.
The primary C++ runtime/analyzer has no Python dependency. Python is used only
for maintainer packaging/tests and the optional legacy analyzer.

Alternatively copy `include/guanid/` into your project's `include/`. This example uses your
existing `pros::MotorGroup left`, `right`, and `pros::Controller controller`:

```cpp
#include "guanid/pros.hpp"
#include "pros/misc.hpp"

// Open once per capture session; check failures and close after stopping motors.
std::FILE* csv = std::fopen("/usd/guanid.csv", "w");
if (!csv) return;
if (!guanid::writeRecordHeader(csv)) { std::fclose(csv); return; }

guanid::ProsSysIdRoutine::Config config; // 1 V/s ramp, 7 V step, 10 s timeout
config.run_id = pros::millis();
guanid::Geometry geometry{0.08255, 1.0}; // wheel diameter m, wheel rev / motor rev

guanid::ProsSysIdRoutine sysId(config, {
    guanid::ProsMotorChannel::linear("drive-left", left, geometry),
    guanid::ProsMotorChannel::linear("drive-right", right, geometry)
}, [csv](const guanid::Record& row) { return guanid::writeRecord(csv, row); });

// Invoke ONE test when its operator button is pressed. This blocks until the
// timeout, button release, disable, or a callback failure. Repeat separately
// for each of the four test/direction combinations, repositioning between runs.
const auto result = sysId.quasistatic(guanid::Direction::forward, [&] {
    return controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) &&
           !pros::competition::is_disabled();
});
// Other calls: quasistatic(Direction::reverse, permit),
// dynamic(Direction::forward, permit), dynamic(Direction::reverse, permit).

sysId.cancel();
const bool flushed = std::fflush(csv) == 0;
const bool closed = std::fclose(csv) == 0;
// Check result, flushed, and closed before accepting the capture.
```

For a drivetrain-only example using existing PROS motor groups, see
[`examples/drivetrain_sysid.cpp`](examples/drivetrain_sysid.cpp). It binds all four
tests to controller buttons and writes left/right telemetry to an SD-card CSV.
It requires no LemLib objects or dependencies.

`examples/sysid_pros_example.cpp` contains a complete periodic button-binding
example: hold L1, press Up/Down for quasistatic forward/reverse, Right/Left for
dynamic forward/reverse, B to exit. It accepts existing motors and controller;
it introduces no robot subsystem or driver configuration.

For a periodic loop use `startQuasistatic(direction)` or
`startDynamic(direction)`, followed by `update(permit)` every `interval_ms` or
faster. `running()`, `state()`, and `result()` expose lifecycle state. During a
capture the result is `Result::running`. `cancel()` stops immediately. Starting
a second test while one runs cancels the first and returns false; start the next
explicitly. Configuration is copied at construction. `run_id` increments per test.
Use a unique initial ID or a separate file for a new routine/session.

Both APIs stop with zero volts on completion, cancellation, invalid configuration,
telemetry failure, and sink failure. A failed stop reports `hardware_error`.
The destructor stops an active routine; blocking calls and periodic `update()`
also stop during exception unwinding when exceptions are enabled. Observer
callbacks must not throw. All callbacks run synchronously on your
calling task. Neither timeout nor operator cancellation can interrupt a blocked
callback. Prevent other tasks from writing to these motors. Deleting a PROS task
without stack unwinding cannot trigger cleanup.

Defaults apply to `ProsSysIdRoutine::Config`: `duration_ms=10000`, `interval_ms=20`,
`ramp_volts_per_second=1`, `step_volts=7`, `max_volts=12`. Adjust these to your
mechanism before testing. Voltages are clamped by `max_volts`, which must be at
most 12. A quasistatic test ramps in the selected direction; dynamic applies a
step. The first sample records zero voltage before applying the test command.
Telemetry is read before the next command; timestamps are actual elapsed time.
Missed deadlines are skipped, and unsigned elapsed time handles clock rollover.
An optional final constructor argument receives the test state and `State::idle`.

## Other mechanisms and units

`ProsMotorChannel` accepts only an existing `pros::Motor` or `pros::MotorGroup`.
The runner commands voltage and reads motor encoders internally; there are no
custom drive, telemetry, or sensor callbacks in the PROS constructor. No LemLib,
chassis, subsystem, or scheduler objects are needed. Sink, state observer, and
operator permit callbacks remain available.

Use `ProsMotorChannel::linear(name, motors, Geometry{diameter_m, output_rev_per_motor_rev})`
for wheels or elevator spools. Use
`ProsMotorChannel::angular(name, motors, output_rev_per_motor_rev, horizontal_offset_rad)`
for flywheels and arms. Both factories accept a single motor or a motor group.
The runner requires degree encoder units, converts RPM, and averages measured
voltage, position, and velocity across every motor in a group. Configure cartridges,
reversal, encoder units, and gearing before testing. Every motor in a group must
use matching output gearing. Group averages can hide disagreement.

An arm's zero angle must be horizontal; home its motor encoder before testing and
set the horizontal offset in radians. Position is absolute; routines never reset
encoders. PROS telemetry reads are sequential and may have sensor latency.

Names must be unique and contain only letters, digits, `-`, `_`, or `.`. An empty
channel list, invalid geometry/offset, duplicate names, or a repeated motor object
is rejected at test start with `invalid_config`, after attempting zero voltage on
all channels. Referenced motor objects must outlive the routine. Do not register
overlapping groups or register a motor separately from a group containing it.

Both factories accept an optional final `drive_direction` of `1` (default) or `-1`.
For a turn-in-place test, use `linear("drive-left", left, geometry, -1)` and leave
the right channel at `1`. This changes only the voltage command; logged encoder
and voltage signs remain in each motor's configured coordinate system.

The portable `SysIdRoutine` engine remains available for host tests and simulation.
Robot applications should use `ProsSysIdRoutine`; desktop analysis requires only
`analysis.hpp` and has no PROS dependency.

## Analyze entirely in code

Replace the CSV sink with a bounded in-memory capture (keep it alive across all
four tests):

```cpp
#include "guanid/analysis.hpp"
std::vector<guanid::Record> capture;
auto sink = [&](const guanid::Record& row) {
    if (capture.size() >= 5000) return false; // stops with log_error
    capture.push_back(row);
    return true;
};
// Pass sink to your routine constructor. Analyze after the motors stop.
guanid::AnalysisConfig settings;
settings.model = guanid::Model::arm; // simple or elevator also supported
const auto fit = guanid::analyze(capture, "arm", settings);
if (fit.ok) {
    const auto& g = fit.gains;
    guanid::ArmFeedforward ff(g.kS, g.kV, g.kA, g.kG);
    const double volts = ff.calculate(targetAngleRad, targetRadps, targetRadps2);
    armMotors.move_voltage(std::lround(std::clamp(volts, -12.0, 12.0) * 1000));
}
```

`SimpleMotorFeedforward{kS,kV,kA}.calculate(v,a)` and
`ElevatorFeedforward(kS,kV,kA,kG).calculate(v,a)` provide the other models.
All outputs are volts. At zero velocity the static-friction term is zero;
use a nonzero velocity setpoint when commanding breakaway. Feedforward does not
clamp automatically. Gains use V, V/(m/s), V/(m/s²), or the equivalent radian
units; kG is volts.

An `Analysis` includes `ok`, `error`, gains, retained sample count, voltage RMSE,
voltage and acceleration R², independent forward/reverse fits, warnings, and
per-sample diagnostics. Undefined R² is NaN in C++, `null` in JSON. Independent
elevator direction fits are rank deficient because gravity and static friction
are indistinguishable in only one direction; the combined fit identifies both.

Processing is isolated by channel, run ID, test, and direction. A centred median
filter precedes a local quadratic derivative fit on actual timestamps. Windows
crossing rest, direction changes, or excessive timestamp gaps are excluded. The
regression fits acceleration against voltage, velocity, sign, and the selected
gravity term using scaled, reorthogonalized QR, then recovers kS/kV/kA/kG. The
reported excitation ratio is the smallest normalized QR diagonal, **not** a
condition number. Rank deficiency, invalid data/units, nonpositive voltage gain,
and missing tests fail analysis. Low R², nonphysical gains, direction mismatches,
and forward/reverse differences over 25% produce warnings.

Set `median_window` (default 5), `derivative_window` (5), `min_velocity` (0.02),
`trim_start_s` (0.05), `trim_end_s` (0), and `max_gap_s` (0.1) to your sampling and
mechanism. Velocity thresholds use the channel's m/s or rad/s units. Both windows
must be positive odd numbers; derivative windows need at least 3 samples. Use a
median window of 1 to disable filtering. `require_four_tests=false` permits
explicitly incomplete datasets, with a warning. Trim hard-stop impacts or noisy
regions before analysis; this library cannot detect every collision or bad sensor.

Feedback suggestions are optional continuous-time starting points, with output
in volts and kI=0. `suggestVelocityFeedback(gains,tau_s)` chooses P for a target
velocity time constant: `kP=kA/tau-kV`.
`suggestPositionFeedback(gains,omega_radps,damping=1)` chooses position PD:
`kP=kA*omega²`, `kD=2*damping*omega*kA-kV`. Invalid inputs or negative resulting
gains return `valid=false`. These assume a friction/gravity-compensated mechanism,
no sensor delay or saturation, and consistent SI units. Validate discretization,
noise, voltage limits, and stability on the actual robot; they are not automatic
hardware tuning or WPILib's LQR gain calculator.

## Standalone C++ analyzer and plots

```sh
cmake -S . -B /tmp/guanid-build -DGUANID_BUILD_EXAMPLES=ON
cmake --build /tmp/guanid-build
ctest --test-dir /tmp/guanid-build --output-on-failure
/tmp/guanid-build/guanid_callback_capture > /tmp/guanid-demo.csv
/tmp/guanid-build/guanid_analyze /tmp/guanid-demo.csv \
  --out /tmp/gains.json --plot-dir /tmp/guanid-plots
```

Without CMake:

```sh
c++ -std=c++17 -I include examples/callback_capture.cpp -o /tmp/guanid-demo
c++ -std=c++17 -I include tools/analyze.cpp -o /tmp/guanid-analyze
/tmp/guanid-demo > /tmp/guanid-demo.csv
/tmp/guanid-analyze /tmp/guanid-demo.csv --out /tmp/gains.json
```

The simulation runs the actual callbacks, all four routines, and in-memory
analysis. Its expected gains are approximately kS=0.25, kV=2.1, kA=0.45 in radian
units. These are simulation values, not gains to copy onto a robot.

Copy `/usd/guanid.csv` from the SD card and pass one or several captures to the
analyzer. Use `--model simple|elevator|arm`, `--motor NAME`, `--min-velocity`,
`--trim-start`, `--trim-end`, `--median-window`, `--derivative-window`, `--max-gap`,
and `--allow-incomplete` to select analysis settings. `--velocity-tau SECONDS` and
`--position-omega RAD_PER_SEC` add feedback suggestions to JSON. Each input file
receives a separate run namespace, preventing differentiation across files.

The analyzer writes JSON gains/metrics/warnings and optional self-contained SVG
plots and diagnostic CSV per channel. Plots show retained raw/filtered velocity,
measured/model voltage, and residual versus time for each run. These are in-sample
fit diagnostics, not independent simulations. Inspect the original capture too:
trimmed samples are not plotted. No plotting libraries or network are used.

The named CSV schema is:
`run_id,test,direction,time_s,motor,units,voltage,position,velocity`.
Units are `m` or `rad`; derivatives use seconds. Write one header per file. The
C++ reader also accepts the earlier left/right drivetrain CSV schema, with or
without run_id. Repeated tests in a file must have distinct run IDs and strictly
increasing timestamps per channel/run. Do not mix unrelated console output into
CSV; this format uses simple unquoted fields and restricted motor names.

## Drivetrain sides, combined and angular

Analyze `drive-left` and `drive-right` independently by default. To add a combined
linear channel:

```sh
guanid_analyze capture.csv --combine drive-left drive-right drive \
  --motor drive --out gains.json
```

For angular identification, run a **separate set of four turn-in-place tests**:
positive command applies negative left voltage and positive right voltage. Keep
logging both wheel channels in metres. Supply the calibrated effective track width:

```sh
guanid_analyze turns.csv --angular drive-left drive-right turn 0.30 \
  --motor turn --out angular-gains.json
```

The in-code equivalent is `deriveDrivetrain(records,left,right,name,output,track)`.
Omit track (or use zero) for combined linear data. Paired samples must share exact
run/test/direction/timestamps and linear units. Linear mode averages both sides;
angular mode computes position/velocity `(right-left)/track` and differential
voltage `(right-left)/2`. Thus angular gains output differential volts per rad/s;
apply `left=-V`, `right=V` for a pure turn. This wheel-derived angular model assumes
an appropriate effective track width and does not replace an IMU under wheel slip.
Do not mix straight and turn captures into the same named fit.

## Calibrate effective track width

Tank-style drivetrains scrub sideways during turns. Their effective track width
can differ from the physical wheel spacing and depends on the wheels, surface,
and load. Calibrate on your normal driving surface, then use the result for
turning kinematics, encoder odometry, and angular SysId channel derivation.

1. Mark the starting heading or record a gyro's continuous rotation reading.
2. Spin in place in one direction for several complete turns (for example 10).
   You can use encoder travel to command the spin, but count the **actual** turns
   against the floor mark or measure them with a gyro.
3. Measure each side's total encoder travel magnitude in metres. Compute
   `effective_width_m = (abs(left_distance_m) + abs(right_distance_m)) / abs(rotation_rad)`.
   Ten actual turns are `10 * 2 * pi` radians. Do not divide wheel distances by
   the rotation predicted from those same encoders and your old track width:
   that would just reproduce the old value.
4. Repeat, including trials in both turn directions, and average the widths.
   Compare their spread; large variation calls for checking slip, wheel-distance
   scaling, heading measurements, and whether the surface/load changed.

The allocation-free C++ helper is available from `guanid/track_width.hpp`,
`guanid/sysid.hpp`, or `guanid/pros.hpp`:

```cpp
guanid::TrackWidthCalibration calibration; // keep across repeated trials
constexpr double pi = 3.14159265358979323846;

// Positive travel magnitudes and independently counted actual rotation.
calibration.add({10.0530964915, 10.0530964915, 10 * 2 * pi});
// Add more trials separately; each trial contributes equal weight.
const auto result = calibration.result();
if (result.valid) {
    const double effectiveTrackWidthM = result.track_width_m; // approximately 0.32
    // Use in your drivetrain kinematics or pass to deriveDrivetrain(...).
}
```

For PROS encoder/gyro snapshots, use signed wheel positions converted to metres
(for example from a linear channel capture) and an **unwrapped** gyro rotation converted to
radians. With a `pros::Imu`, use `get_rotation()` (degrees), rather than wrapped
`get_heading()`. Record start values before your explicitly controlled spin and
end values after stopping, including any coast-down travel:

```cpp
// Both wheel encoders report positive travel when driving forward.
const bool accepted = calibration.addSpin(
    leftStartM, leftEndM, rightStartM, rightEndM,
    gyroStartDegrees * pi / 180.0, gyroEndDegrees * pi / 180.0);
```

`addSpin` checks that the sides moved in opposite directions; do not reset sensors
or reverse the spin within a trial. Release the operator enable to stop using the
existing routine's permit callback. The helper itself commands no motors.
`add`/`addSpin` return false on invalid/nonfinite/zero measurements and leave the
previous result unchanged. `result()` reports the arithmetic mean in metres,
trial count, minimum, maximum, and sample standard deviation (zero for one trial).
`clear()` starts a new calibration. Clockwise and counterclockwise angles are both
accepted. A single `TrackWidthTrial::effectiveTrackWidthM()` returns NaN for an
invalid trial. The helper cannot verify that a supplied angle is independently
measured or that wheel slip remained consistent.

## Existing APIs and checks

The original `run()`, `runRoutine()`, `Sample`, `SideSample`, `writeCsvHeader()`,
`writeCsvSample()`, and `Feedforward` remain available. Their legacy `Config`
defaults remain 3 seconds, 0.5 V/s, 3 V step, 6 V cap. See
`examples/pros_example.cpp` and `examples/simulated_capture.cpp`. The latter
simulates left/right gains `(0.25,2.1,0.45)` and `(0.30,2.4,0.60)`. The earlier
`tools/analyze.py` remains an optional legacy drivetrain analyzer; it is not used
by the C++ workflow.

Portable tests cover profiles, CSV, feedforward, cancellation/error cleanup,
rollover, scheduling, callback captures, gravity gain recovery, rank deficiency,
unit errors, hard-stop exclusion, drivetrain transforms, feedback formulas, and
effective-track-width recovery/averaging with both turn directions.
The CMake CLI check also exercises capture, JSON, SVG, and malformed CSV rejection.
Build PROS projects with `pros make`; host tests and header syntax checks do not
establish hardware behavior. Start with low voltages, a charged battery, clear
travel, and an independent disable. Include travel/temperature limits in the
permit predicate for your mechanism and inspect repeated captures before applying
gains. Timeout/zero-voltage cleanup does not guarantee a gravity-loaded mechanism
will hold position; provide appropriate mechanical support and braking.

References: [WPILib routines](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/creating-routine.html),
[model and workflow](https://docs.wpilib.org/en/latest/docs/software/advanced-controls/system-identification/introduction.html),
[PROS motor-group telemetry](https://pros.cs.purdue.edu/v5/pros-4/group__cpp-motor-group.html).

## Project documents and license

- [API and output reference](docs/api.md)
- [Installation and upgrades](docs/installation.md)
- [Compatibility and validation limits](docs/compatibility.md)
- [Release candidate validation record](docs/validation.md)
- [Hardware validation protocol](docs/hardware-validation.md)
- [Troubleshooting](docs/troubleshooting.md)
- [Changelog](CHANGELOG.md), [contributing](CONTRIBUTING.md), and [releasing](docs/releasing.md)
- [Security reports](SECURITY.md)

MIT licensed; see [LICENSE](LICENSE) and [NOTICE](NOTICE). This is an independent
implementation inspired by WPILib SysId. Upstream dependencies retain their own
licenses and are not bundled into the PROS template.
