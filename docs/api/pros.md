# PROS motor channels and routines

[API index](../api.md) · Header `<guanid/pros.hpp>`

- [Geometry](#geometry) and [motor channels](#prosmotorchannel)
- [Named runner](#prossysidroutine) and [legacy runner](#legacy-run)

## `Geometry`

Public fields:

- `double wheel_diameter_m` is the effective wheel or spool diameter in
  metres. Supply it explicitly. There is no member initializer.

- `double wheel_revolutions_per_motor_revolution = 1` is the output-to-motor
  turn ratio, such as `36.0 / 48.0` when a 36-tooth driver turns a 48-tooth
  output gear.

`bool valid() const` returns true when both values are finite and positive.

## `prosRuntime`

```cpp
SysIdRoutine::Runtime prosRuntime();
```

Returns the
clock and sleep callbacks backed by `pros::millis()` and `pros::delay()`.

## `ProsMotorChannel`

A descriptor for an existing `pros::Motor` or `pros::MotorGroup`. Create
it with a static factory. The descriptor borrows the motor or group and has
no public data members. Set motor signs and degree encoder units before
capturing. A group averages measured voltage, position, and velocity across
its motors, which should have matching output gearing.

### Linear factories

```cpp
static ProsMotorChannel linear(std::string name, pros::Motor& motor,
                               Geometry geometry, int drive_direction = 1);
static ProsMotorChannel linear(std::string name, pros::MotorGroup& motors,
                               Geometry geometry, int drive_direction = 1);

template <typename Motor>
static ProsMotorChannel linearInches(std::string name, Motor& motors,
                                     double wheelDiameterIn,
                                     double wheelTurnsPerMotorTurn = 1,
                                     int driveDirection = 1);
```

**Parameters**

- `name` is the channel identifier. Use the
  [named sample rules](capture.md#motorsample) and a unique name in the routine.

- `motor` or `motors` is the borrowed single motor or group. The template
  accepts the same two hardware types as the overloads.

- `geometry` supplies the diameter in metres and output turn ratio.
  `wheelDiameterIn` supplies the same diameter in inches, and
  `wheelTurnsPerMotorTurn` supplies the ratio directly.

- `drive_direction` or `driveDirection` is a command multiplier of +1 or −1.
  The motor's configured telemetry signs are preserved.

**Returns**

A linear descriptor. Capture records contain metres
and m/s regardless of the factory used.

### Angular factories

```cpp
static ProsMotorChannel angular(std::string name, pros::Motor& motor,
                                double output_revolutions_per_motor_revolution = 1,
                                double horizontal_offset_radians = 0,
                                int drive_direction = 1);
static ProsMotorChannel angular(std::string name, pros::MotorGroup& motors,
                                double output_revolutions_per_motor_revolution = 1,
                                double horizontal_offset_radians = 0,
                                int drive_direction = 1);

template <typename Motor>
static ProsMotorChannel angularDegrees(std::string name, Motor& motors,
                                       double outputTurnsPerMotorTurn = 1,
                                       double horizontalOffsetDeg = 0,
                                       int driveDirection = 1);
```

**Parameters**

- `name`, `motor` or `motors`, and `drive_direction` or `driveDirection`
  follow the linear factories' rules.

- `output_revolutions_per_motor_revolution` or `outputTurnsPerMotorTurn`
  is the finite positive number of output turns per motor turn.

- `horizontal_offset_radians` or `horizontalOffsetDeg` is the finite angle
  added to the scaled motor position. For an arm, choose it so the result is
  zero at horizontal. It leaves velocity and the motor encoder unchanged.

**Returns**

An angular descriptor. Capture records contain radians
and rad/s regardless of the factory used.

## `ProsSysIdRoutine`

A noncopyable wrapper around [`SysIdRoutine`](capture.md#sysidroutine).
It uses existing PROS motors and the PROS clock, and its destructor attempts
to stop an active test. It has no public data members.

### Public aliases

- `Config = SysIdRoutine::Config` supplies the named test defaults.

- `Direction = guanid::Direction` selects the command sign.

- `Sink = SysIdRoutine::Sink` receives each measured `Record` and reports
  acceptance.

- `Observer = SysIdRoutine::Observer` receives active and idle states.

- `CommandObserver = std::function<void(double)>` receives the signed
  routine-level voltage request in volts after motor writes are attempted.
  It receives requests even if a write fails, including startup and cleanup
  zeroes. Per-channel command reversal does not change this observer's argument.
  Use the record's measured voltage for regression.

### Constructor

```cpp
ProsSysIdRoutine(Config config, std::vector<ProsMotorChannel> channels, Sink sink,
                 Observer observer = {}, CommandObserver commandObserver = {});
```

**Parameters**

- `config` controls profile timing, voltage, and the run ID seed.

- `channels` is the nonempty list of borrowed motor descriptors, copied into
  the routine. Each must have valid geometry or output gearing, a valid offset
  and multiplier, a nonempty motor or group, and a unique name. Repeated
  references to the same motor object are rejected. Avoid overlapping physical
  ports across distinct objects.

- `sink` is the required record callback.

- `observer` and `commandObserver` are optional state and voltage callbacks.

Invalid setup is reported when a test is started, with a stop attempt on the
supplied channels.

### Start, update, and cancel

These methods follow the
[portable runner contract](capture.md#constructor-and-lifecycle):

- `bool startQuasistatic(Direction direction)` or
  `bool startDynamic(Direction direction)` starts the signed ramp or step.
  True means active. A second start cancels the active test and returns false.

- `Result update(bool permit = true)` advances the test or cancels it when
  `permit` is false. Call it at the sample interval or faster. Pass the
  current operator enable and competition state.

- `Result quasistatic(Direction direction, Permit permit)` and
  `Result dynamic(Direction direction, Permit permit)` are blocking templates.
  They take the signed direction and a no-argument boolean permit callback,
  and return the final result.

- `Result cancel()` attempts zero voltage when active and returns the
  resulting status. When idle it returns the stored status.
### Status accessors

- `bool running() const` reports an active test.

- `State state() const` returns the active state or idle.

- `Result result() const` reports the current outcome. Its initial value is `completed`.

## Legacy `run`

```cpp
template <typename KeepRunning, typename Sink, typename Observer = IgnoreState>
Result run(pros::MotorGroup& left, pros::MotorGroup& right,
           const Geometry& geometry, Test test, Direction direction,
           const Config& config, KeepRunning keep_running, Sink sink,
           Observer observer = {});
```

**Parameters**

- `left` and `right` are existing motor groups. Set both groups to degree
  encoders with positive voltage driving forward and consistent gearing.

- `geometry` supplies the SI diameter and output turn ratio for both sides.

- `test` and `direction` select the ramp or step and command sign.

- `config` is the [legacy `guanid::Config`](core.md#config).

- `keep_running` is the no-argument boolean permit callback.

- `sink` accepts a `const Sample&` containing both sides and returns whether
  it was accepted.

- `observer` receives active and idle states, with a no-op default.

**Returns**

The call blocks for one legacy test and returns the same results as
[`runRoutine`](core.md#runroutine). Invalid geometry triggers a zero-voltage
attempt and an idle notification. A failed stop reports `hardware_error`.
