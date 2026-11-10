# Shared types and legacy capture

[API index](../api.md)

- [Test and result types](#test-direction-state-and-result) and [configuration](#config)
- [Samples](#sidesample), [CSV](#legacy-csv-functions), and [feedforward](#feedforward)
- [Legacy runner](#runroutine)

## `guanid.hpp`

### `Test`, `Direction`, `State`, and `Result`

- `Test::quasistatic` applies a voltage ramp. `Test::dynamic` applies a voltage step.

- `Direction::forward` requests positive voltage. `Direction::reverse` requests
  negative voltage. Physical motion follows the configured motor signs.

- `State::idle` means no test is active. The active states are
  `quasistatic_forward`, `quasistatic_reverse`, `dynamic_forward`, and `dynamic_reverse`.

- `Result::running` means a test is active. `completed` means it reached its
  duration and the runner's stop succeeded. Fit success is reported separately.

- `Result::cancelled` means an operator or caller cancelled the test.
  `invalid_config` reports rejected setup. `hardware_error` reports a failed
  hardware operation, invalid telemetry, or failed cleanup stop.
  `log_error` reports a rejected sink write or capture data that violates the
  runner's logging contract.

### State helpers

```cpp
State testState(Test test, Direction direction);
const char* stateName(State state);
```

`testState` takes a profile and command direction and returns their active state.
Pass enumerators defined by those types.

`stateName` returns a static string for the supplied
state. Active states use hyphenated labels such as `"dynamic-forward"`.
Idle and unrecognized values return `"none"`.

### `Config`

Public fields and legacy defaults:

- `std::uint32_t duration_ms = 3000` is the test timeout in milliseconds.
  Use a value in [1, 2³¹).

- `std::uint32_t interval_ms = 20` is the sample interval in milliseconds.
  Use a value from 1 through `duration_ms`.

- `double ramp_volts_per_second = 0.5` is the positive quasistatic ramp slope.

- `double step_volts = 3.0` is the positive dynamic voltage magnitude.

- `double max_volts = 6.0` caps either command magnitude. Use a finite value
  greater than zero and at most 12 V. Slope and step must also be finite.

- `std::uint32_t run_id = 0` labels rows from this test. Legacy callers choose
  a fresh ID for each attempt when appending to one file.

`bool valid() const` checks the timing and voltage constraints above. It does
not impose a constraint on the run ID.

### `commandVolts`

```cpp
double commandVolts(const Config& config, Test test, Direction direction,
                    std::uint32_t elapsed_ms);
```

**Parameters**

- `config` is the validated test configuration.

- `test` selects a ramp or step.

- `direction` selects the command sign.

- `elapsed_ms` is the time since test start in milliseconds.

**Returns**

Signed volts, capped at `max_volts` in magnitude. At or beyond
`duration_ms`, the function returns zero.

### `SideSample`

`SideSample` describes one side of a drivetrain. All three public fields are
`double` and default to zero.

- `applied_volts` is measured motor voltage in volts.

- `position_m` is signed wheel travel position in metres.

- `velocity_mps` is signed wheel speed in metres per second.

`SideSample::valid() const` returns true when all fields are finite and
`abs(applied_volts) <= 12`.

### `Sample`

`Sample` describes both sides at one instant.

**Public fields**

- `Test test = Test::quasistatic` and `Direction direction = Direction::forward`
  identify the selected profile and direction.

- `std::uint32_t elapsed_ms = 0` is milliseconds since the test began.

- `double commanded_volts = 0` is the signed command active when telemetry was read.

- `SideSample left` and `SideSample right` hold the two measured sides.

- `std::uint32_t run_id = 0` labels the test attempt.

### Legacy CSV functions

```cpp
bool writeCsvHeader(std::FILE* stream);
bool writeCsvSample(std::FILE* stream, const Sample& sample);
```

`writeCsvHeader` writes the legacy two-sided CSV header
to a caller-owned writable stream. Call it once before the rows.

`writeCsvSample` writes one
two-sided row to that stream. `sample` supplies both measurements and the
active command. The writer converts `elapsed_ms` to seconds. Validate sample
contents before passing them to the writer.

**Returns**

Both return false for a null stream or a reported write failure. The caller
owns the file and checks the final flush and close.

### `Feedforward`

The legacy linear voltage model has public `double` fields `kS` (V), `kV`
(V·s/m), and `kA` (V·s²/m). All default to zero.

#### `calculate`

```cpp
double calculate(double velocity_mps, double acceleration_mps2 = 0) const;
```

Takes the desired linear velocity in m/s and acceleration in m/s². It returns
unclamped volts as `kS * sign(velocity_mps) + kV * velocity_mps + kA *
acceleration_mps2`. The static term is zero at zero velocity.

## `routine.hpp`

### `IgnoreState`

`void operator()(State) const` accepts a state, ignores it, and returns nothing.
It is the default observer for the legacy runners.

### `runRoutine`

```cpp
template <typename Mechanism, typename KeepRunning, typename Sink,
          typename Observer = IgnoreState>
Result runRoutine(Mechanism& mechanism, Test test, Direction direction,
                  const Config& config, KeepRunning keep_running, Sink sink,
                  Observer observer = {});
```

**Parameters**

- `mechanism` is borrowed for the call. It supplies `uint32_t nowMs()` for
  the millisecond clock, `void sleepMs(uint32_t ms)` for scheduling,
  `bool setVoltage(int32_t millivolts)` for both sides, and
  `bool read(SideSample& left, SideSample& right)` to fill a measured snapshot.
  Both hardware methods report success as true. The voltage callback must attempt
  both sides, including for zero.

- `test` selects the ramp or step. `direction` selects its sign.

- `config` sets timeout, sampling, voltage and run ID.

- `keep_running` is a no-argument predicate. Return true while the operator
  and mechanism permit motion. The runner checks it before reading and again
  after sending a sample to the sink.

- `sink` receives a `const Sample&` and returns whether it accepted the row.
  Returning false ends the test with `log_error` if cleanup succeeds.

- `observer` receives the active `State` on start and `State::idle` on finish.
  The default ignores state.

**Returns**

The call blocks for one test and returns its final `Result`. It attempts a
zero-voltage stop on exit. A failed stop returns `hardware_error` even when
another error or cancellation initiated cleanup.
