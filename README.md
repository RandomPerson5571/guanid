# guanid

System identification for VEX V5 / PROS 4, using WPILib SysId's routine and
analysis workflow as a reference. A C++17 header-only runner collects four tests;
an offline Python tool fits independent left/right drivetrain feedforward gains:

`V = kS * sign(v) + kV * v + kA * a`

The portable core has no PROS dependency. The PROS adapter requires the language
standard supported by your kernel (the local headers need C++20 or newer).
No robot firmware changes, desktop GUI, or PROS template installation are included.
guanid CSV is **not WPILog** and cannot be imported directly into WPILib SysId.

## Use with PROS

Copy `include/guanid/` into your project's `include/` directory. Include
`guanid/pros.hpp`; see `examples/pros_example.cpp` for an explicitly invoked,
hold-to-run test. Select `Test::quasistatic` or `Test::dynamic` and
`Direction::forward` or `Direction::reverse`. Run each of the four combinations
separately, with room to travel and time to stop between tests.

Set wheel diameter in metres and gearing as **wheel revolutions per motor
revolution**. Configure every motor's cartridge, reversal, degrees encoder units,
and coast brake mode yourself. Forward voltage and reported travel must share the
same sign on both sides. The adapter checks encoder units and telemetry for
**every motor**, then averages delivered volts, position, and velocity separately
for each side. Each group must contain motors with matching output gearing and
cartridges. Motor RPM is converted to wheel m/s. Positions are absolute encoder
travel in metres; encoders are not reset. Telemetry calls are sequential, not a
simultaneous hardware snapshot; internal motor telemetry delays still apply.

`run()` blocks, calls your `keep_running()` predicate every iteration, and attempts
zero voltage on both groups on every return path. A failed final stop returns
`hardware_error`. It returns `completed`,
`cancelled`, `invalid_config`, `hardware_error`, or `log_error`. Commands are
limited to `max_volts` (default 6 V, never above 12 V); default tests last 3 seconds
with a 20 ms interval, a 0.5 V/s ramp, or a 3 V step. Unsigned elapsed timestamps
handle a single millisecond clock rollover. Samples follow elapsed-time deadlines;
missed deadlines are skipped without catch-up bursts. Logging and scheduling can
lengthen the interval, so timestamps use the actual clock.

An optional final callback receives `State::quasistatic_forward`,
`State::quasistatic_reverse`, `State::dynamic_forward`, or `State::dynamic_reverse`
when starting, and `State::idle` on exit. `stateName()` returns readable labels.
Send these state messages to a separate log, not the CSV stream. Stop-on-exit also
covers C++ stack unwinding; cleanup callbacks must not throw.

For other mechanisms or external encoders, include `guanid/routine.hpp` and call
`runRoutine()` with a mechanism providing `nowMs()`, `sleepMs(ms)`,
`setVoltage(millivolts)`, and `read(left, right)`. See the header for the contract
and `tests/routine.cpp` for a fake implementation. Custom mechanisms can enforce
travel limits in `keep_running()` and supply converted sensor values themselves.
No scheduler or task ownership is imposed by the library.

CSV records test, direction, elapsed seconds, active commanded voltage, then
separate left/right delivered volts, position in metres, velocity in m/s, and a
trailing `run_id`. Give repeated tests unique `Config::run_id` values in one file.
The first row captures zero commanded voltage before the test starts. Each row
describes telemetry before the next command is issued. Write a header once per
CSV file and append rows across tests; keep unrelated serial output out of that
file. The example prints a header per invocation for separate captures. The
analyzer also reads the original CSV schema without `run_id`; keep repeated
captures with that schema in separate files.

## Analyze the four captures

From this directory:

```sh
python3 -m venv /tmp/guanid-analysis
/tmp/guanid-analysis/bin/python -m pip install -r tools/requirements.txt
/tmp/guanid-analysis/bin/python tools/analyze.py \
  quasistatic-forward.csv quasistatic-reverse.csv \
  dynamic-forward.csv dynamic-reverse.csv \
  --out /tmp/gains.json --plot-dir /tmp/guanid-plots
```

One file containing all four runs also works. NumPy is required; matplotlib is
only needed for `--plot-dir`. JSON records each side's `kS` in V, `kV` in V/(m/s),
`kA` in V/(m/s²), settings, sample counts, per-run voltage residual RMSE,
acceleration R², and a condition number for the scaled regression matrix. Plots
show raw/filtered velocity, measured/model voltage, and voltage residuals per run
and side. These plots use measured acceleration; they are not independent
forward simulations of velocity. Their residuals measure in-sample fit quality.

Processing stays within each file/test/direction/run ID. Velocity uses a centred
median filter; a local quadratic fit on actual timestamps estimates acceleration.
Windows spanning zero speed, sign changes, or timestamp gaps are excluded.
Tune `--median-window` (default 5), `--derivative-window` (5),
`--min-velocity` (0.02 m/s), `--trim-start` (0.05 s), and `--max-gap` (0.1 s).
Use `--median-window 1` to disable median filtering. Large windows can erase fast
V5 acceleration transients, while short windows can amplify sensor noise.

The fit uses acceleration as the dependent variable:
`a = bU*V + bV*v + bS*sign(v)`, then derives
`kA = 1/bU`, `kV = -bV/bU`, and `kS = -bS/bU`.
Columns are scaled and solved with SVD least squares. There is no bias term,
gravity compensation, automatic sensor-delay correction, or feedback/PID tuning.
This follows the model and acceleration-regression approach described by WPILib;
it does not reproduce the desktop tool's complete preprocessing or diagnostics.

Malformed data, nonincreasing timestamps, rank deficiency, and nonpositive
voltage-to-acceleration gain fail analysis. All four tests must retain usable
samples unless `--allow-incomplete` is explicitly set. Negative friction or
velocity gains, low acceleration R², and direction mismatches produce warnings.
Review warnings and plots before transferring gains. A good fit alone does not
establish that measurements, gearing, or units were correct.

Apply a side's fitted gains with `guanid::Feedforward{kS, kV, kA}.calculate(v, a)`.
The result is volts, with zero static term at zero velocity; the helper does not
clamp output. Multiply by 1000 and clamp to the motor command range yourself.
Observed maximum speed and filtered acceleration are included for context, not
as certified trajectory limits. These tests do not establish safe cornering,
traction, braking distance, or jerk limits.

Use fast, nonblocking callbacks: a blocked sink or predicate prevents the loop
from checking its timeout or stop input. Ensure no other task writes to the
drivetrain. Use a clear test area, validate sensor signs at low voltage, and keep
an independent means to disable the robot. Task deletion, power loss, and failed
motor communication cannot be handled by a C++ scope guard. This base runner does
not monitor obstacles, temperature, or travel limits. It leaves the chosen brake
mode in place and does not collect a post-test coast-down trace. Group averages
can hide individual motor disagreement; verify configuration before testing.

## Host checks

Only `guanid/guanid.hpp` is needed for the portable core:

```sh
cmake -S . -B /tmp/guanid-build
cmake --build /tmp/guanid-build
ctest --test-dir /tmp/guanid-build --output-on-failure
```

Or compile `tests/core.cpp` directly with a C++17 compiler and `-I include`.
Compile and run `tests/routine.cpp` the same way to check the portable runner, or
use CTest for both executables. Analysis tests require NumPy:

```sh
/tmp/guanid-analysis/bin/python -m unittest discover -s tests -p 'test_*.py' -v
```

Tests cover profiles, units in CSV, feedforward signs, cancellation/error cleanup,
state callbacks, clock rollover, missed deadlines, known-gain recovery on jittered
and noisy synthetic data, filtering boundaries, and invalid datasets. They do not
establish hardware behavior or validate gains on a real robot.

References: [WPILib routines](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/creating-routine.html),
[analysis](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/analyzing-gains.html),
and [diagnostics](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/viewing-diagnostics.html).
The adapter uses [PROS 4 motor-group telemetry](https://pros.cs.purdue.edu/v5/pros-4/group__cpp-motor-group.html).
