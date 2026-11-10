# Named capture and feedforward

[API index](../api.md) · Header `<guanid/sysid.hpp>`

- [Samples and logging](#motorsample) and [CSV writers](#csv-writers)
- [Capture callbacks](#nested-types-and-callbacks) and [routine lifecycle](#constructor-and-lifecycle)
- [Feedforward types](#feedforward-types)

## `Units` and `Model`

`Units::linear` identifies metre-based `Record` motion. `Units::angular`
identifies radian-based motion. These values describe dimension even when
output is converted to inches or degrees.

`Model::simple` fits static friction, velocity, and acceleration.
`Model::elevator` adds constant gravity voltage. `Model::arm` adds a gravity
voltage proportional to the cosine of the angle from horizontal.

## `MotorSample`

Public fields:

- `std::string name` is the channel identifier. Use a nonempty string of
  ASCII letters, digits, hyphens, underscores, or periods.

- `Units units = Units::linear` identifies the motion dimension.

- `double volts = 0` is the measured voltage.

- `double position = 0` is signed position in metres or radians.

- `double velocity = 0` is signed velocity in m/s or rad/s.

- `unsigned fields = 0` tracks supplied values. Its bits are 1 for voltage,
  2 for linear position, 4 for linear velocity, 8 for angular position, and 16
  for angular velocity. Populate it through the setters.

The following methods return `MotorSample&` to allow chaining. Motion setters
also set `units` to their dimension and mark their bit in `fields`.

- `voltage(double v)` records measured volts.

- `linearPosition(double m)` records position in metres.

- `linearVelocity(double mps)` records velocity in m/s.

- `angularPosition(double radians)` records position in radians.

- `angularVelocity(double radps)` records velocity in rad/s.

`bool valid() const` checks the name, finite measurements, and voltage
magnitude at most 12 V. It requires exactly voltage plus a position and velocity
of one dimension. Mixing linear and angular setters on the same sample fails
validation.

## `SysIdLog`

- `std::vector<MotorSample> motors` contains the current telemetry snapshot
  in channel order. Keep the channel names unique and the order and units
  consistent across snapshots within a test.

- `MotorSample& motor(const std::string& name)` looks up that channel or
  appends a new sample with that name. Finish filling the sample before adding
  another channel, which may invalidate the returned reference.

- `bool valid() const` checks that the vector is nonempty and every sample
  passes `MotorSample::valid()`.

## `Record` and named CSV

A record holds one channel at one instant. Public fields:

- `std::uint32_t run_id = 0` identifies the attempt.

- `Test test = Test::quasistatic` and `Direction direction = Direction::forward`
  identify the selected profile and command direction.

- `double time_s = 0` is elapsed seconds from the test start.

- `MotorSample motor` holds the channel name and SI measurements.

### CSV writers

```cpp
bool writeRecordHeader(std::FILE* file);
bool writeRecord(std::FILE* file, const Record& row);
```

`writeRecordHeader` writes the following header to a
caller-owned writable stream. It also serves as the header for
[`writePublicRecord`](analysis.md#writepublicrecord).

```text
run_id,test,direction,time_s,motor,units,voltage,position,velocity
```

`writeRecord` writes a single SI
record. `file` is the writable stream and `row` is the input record.
The `units` column is `m` or `rad` and velocity uses the same unit per second.
Fields are unquoted.

**Returns**

Both writers return false for a null stream or a reported
write failure. The row writer also rejects invalid samples, invalid test or
direction values, and negative or nonfinite timestamps. Check flush and close
on the caller-owned file.

## `SysIdRoutine`

Portable single-test runner. The object is noncopyable. Its destructor attempts
to stop an active test. It exposes the types and methods below, with no public
instance fields.

### Nested types and callbacks

- `Direction` aliases `guanid::Direction`.

- `Config` inherits [`guanid::Config`](core.md#config) and its fields and
  `valid()`. Its default constructor changes `duration_ms` to 10000,
  `ramp_volts_per_second` to 1, `step_volts` to 7, and `max_volts` to 12.
  The 20 ms interval and run ID of zero are inherited unchanged.

- `Mechanism::drive` is `std::function<bool(double)>`. Its argument is
  signed volts, including zero for startup and stop. Return true when the
  write succeeds. Attempt every output even when one fails.

- `Mechanism::log` is `std::function<bool(SysIdLog&)>`. Fill the supplied
  snapshot with SI measurements and return true on success.

- `Runtime::nowMs` is `std::function<std::uint32_t()>`, a millisecond
  clock that may wrap. `Runtime::sleepMs` is
  `std::function<void(std::uint32_t)>`, a delay in milliseconds used by the
  blocking methods. Both callbacks are required even for periodic use.

- `Sink` is `std::function<bool(const Record&)>`. It receives one record
  per channel per sample. Return false to stop with `log_error` if cleanup
  succeeds. Copy a record to keep it beyond the callback.

- `Observer` is `std::function<void(State)>`. It receives the active
  state after startup and `State::idle` during cleanup.

### Constructor and lifecycle

#### Constructor

```cpp
SysIdRoutine(Config config, Mechanism mechanism, Runtime runtime, Sink sink,
             Observer observer = {});
```

**Parameters**

- `config` supplies the test settings. Its `run_id` seeds IDs for successful starts.

- `mechanism` supplies the hardware callbacks.

- `runtime` supplies the clock callbacks.

- `sink` is the required record callback.

- `observer` is the optional state callback.

Construction does not command the mechanism.

#### Start, update, and cancel

- `bool startQuasistatic(Direction direction)` starts the signed ramp.
  `bool startDynamic(Direction direction)` starts the signed step. Both
  return true when the test enters the running state. Invalid setup or a failed
  startup zero command returns false and sets `result()`. Calling start during
  a test cancels it and returns false. Call start again to begin the next one.

- `Result update(bool permit = true)` advances a running test. Supply the
  current operator, competition, and mechanism permission as `permit`.
  False cancels. Call at the configured interval or faster. The runner reads
  and logs telemetry before issuing the next voltage and skips missed deadlines.
  It returns the current status, or the stored status when idle.

- `Result quasistatic(Direction direction, Permit permit)` and
  `Result dynamic(Direction direction, Permit permit)` are templates.
  They start the requested signed profile, call the no-argument `permit`
  predicate at each update, and block until the final result. The predicate
  returns whether motion may continue.

- `Result cancel()` attempts to stop a running test and returns the resulting
  status. When idle it returns the stored status.
#### Status accessors

- `bool running() const` reports whether a test is active.

- `State state() const` returns the active state or `State::idle`.

- `Result result() const` returns the current status. Its initial value is
  `completed`, so confirm a start before using it as evidence of a finished test.

#### Failure and cleanup

A failed drive or invalid snapshot reports `hardware_error`. Changing the
number, order, names, or dimensions of logged channels during a test reports
`log_error`. Cleanup attempts zero volts and a failed stop takes precedence
over the original result.

## Feedforward types

### `SimpleMotorFeedforward`

`SimpleMotorFeedforward` has public `double kS = 0`, `kV = 0`, and
`kA = 0`. `kS` is volts. `kV` is volts per unit of velocity and `kA` is
volts per unit of acceleration. Use the units of the fitted gains consistently.

`double SimpleMotorFeedforward::calculate(double velocity,
double acceleration = 0) const` takes the desired signed velocity and
acceleration and returns `kS * sign(velocity) + kV * velocity + kA * acceleration`
in volts. Static friction contributes zero at zero velocity.

### `ElevatorFeedforward` and `ArmFeedforward`

`ElevatorFeedforward` and `ArmFeedforward` each inherit the three public
gains and add a public `double kG` in volts. Both constructors take
`(double s, double v, double a, double g)` and assign those arguments to
`kS`, `kV`, `kA`, and `kG`, respectively.

- `double ElevatorFeedforward::calculate(double velocity, double acceleration = 0) const`
  uses the desired signed velocity and acceleration and adds a constant `kG`.

- `double ArmFeedforward::calculate(double angle_radians, double velocity,
  double acceleration = 0) const` uses angle from horizontal in radians and
  desired angular velocity and acceleration in the units of the gains.
  It adds `kG * cos(angle_radians)` to the simple motor terms.

All `calculate` methods return unclamped volts.
