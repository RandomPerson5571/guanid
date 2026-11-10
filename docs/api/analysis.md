# Analysis and output units

[API index](../api.md)

- [Fit settings](#analysisconfig) and [fit results](#fit)
- [Analysis functions](#analyze) and [feedback suggestions](#feedback-suggestions)
- [Output units](#constants-and-unit-selectors) and [CSV export](#writepublicrecord)

## `analysis.hpp`

### `AnalysisConfig`

All fields are public. `bool valid() const` checks the constraints below.
These settings apply in SI even when output gains use inches or degrees.

- `Model model = Model::simple` selects the [voltage model](capture.md#units-and-model).

- `std::size_t median_window = 5` is the centred velocity median window in
  samples. Use a positive odd count. A value of 1 leaves velocity unfiltered.

- `std::size_t derivative_window = 5` is the local quadratic derivative window
  in samples. Use an odd count of at least 3.

- `double min_velocity = 0.02` is the minimum absolute motion threshold in m/s
  or rad/s. The differentiation window must stay above it. Use a finite,
  nonnegative threshold.

- `double trim_start_s = 0.05` and `double trim_end_s = 0` are seconds to
  exclude from the beginning and end of each run, measured from its first and
  last record. Use finite, nonnegative values.

- `double max_gap_s = 0.1` is the largest allowed interval between adjacent
  timestamps within a filter and derivative window. Use finite positive seconds.

- `bool require_four_tests = true` requires retained samples from both profiles
  in both directions. False allows incomplete coverage and emits a warning.
  The fit still needs enough independent measurements.

### `Gains`

All four public `double` fields default to zero.

- `kS` is the static-friction coefficient in volts, multiplied
  by the sign of the requested velocity in the voltage model.

- `kV` is the velocity coefficient in V·s/m or V·s/rad for an SI fit.

- `kA` is the acceleration coefficient in V·s²/m or V·s²/rad for an SI fit.

- `kG` is the gravity coefficient in volts. Simple fits set it to zero.

#### `calculate`

```cpp
double calculate(Model model, double position, double velocity,
                 double acceleration) const;
```

**Parameters**

- `model` selects the gravity term.

- `position` is angle from horizontal in radians for an arm. Other models ignore it.

- `velocity` and `acceleration` are desired signed setpoints in the units of the gains.

**Returns**

Unclamped volts. Static compensation is zero at zero velocity. The elevator
adds `kG` and the arm adds `kG * cos(position)`.

### `Diagnostic`

Public fields for one retained sample:

- `Record record` is the original SI measurement and attempt metadata.

- `double velocity = 0` is the median-filtered velocity in m/s or rad/s.

- `double acceleration = 0` is the local derivative in m/s² or rad/s².

- `double predicted_voltage = 0` is the joint model's predicted voltage.

- `double residual = 0` is measured minus predicted voltage, in volts.

The prediction and residual are populated after a successful joint fit.

### `Fit`

`Fit` has these public fields:

- `bool ok = false` indicates successful regression. Check it before using
  the coefficients or metrics.

- `std::string error` describes a failed regression or rejected input.

- `Gains gains` contains the fitted voltage model coefficients in SI.

- `std::size_t samples = 0` counts the retained samples supplied to the fit.

- `double acceleration_r_squared` measures the regression's acceleration fit.

- `double voltage_r_squared` measures the rearranged voltage model's fit.
  Both R² fields default to NaN and remain NaN for effectively constant targets.

- `double voltage_rmse = 0` is the voltage residual root mean square in volts.

- `double excitation_ratio = 0` is the smallest diagonal of the QR factor
  after normalizing the regression columns. Smaller values indicate weaker
  excitation. Use it alongside the rank and fit results.

### `Analysis`

`Analysis` inherits the public fields of `Fit` for the joint fit and adds these fields.

- `Units units = Units::linear` is the selected channel's motion dimension.

- `Model model = Model::simple` identifies the requested model.

- `Fit forward` and `Fit reverse` are independent fits split by recorded test
  direction. Check each one's `ok` separately. A one-direction elevator
  capture typically cannot separate static friction from gravity.

- `std::vector<std::string> warnings` contains observations such as incomplete
  coverage, weak acceleration fit, direction mismatches, or asymmetric gains.

- `std::vector<Diagnostic> diagnostics` holds retained samples. A failed fit can
  still return filtered data, so use `ok` to determine whether model predictions
  and residuals are available.

### `analyze`

```cpp
Analysis analyze(const std::vector<Record>& records, const std::string& motor,
                 const AnalysisConfig& config = {});
```

**Parameters**

- `records` supplies SI measurements. For the selected channel, timestamps
  must be finite, nonnegative, and strictly increasing within each
  `(run_id, test, direction)` group. Assign new IDs to retries.

- `motor` is the exact channel name. Other channels are skipped.

- `config` selects model, filtering, trimming, and coverage requirements.
  Arm analysis requires angular records with zero at horizontal.

**Returns**

An `Analysis` by value for the selected channel.

Filtering and differentiation run independently for each group. Windows
crossing rest, a direction change, or an excessive gap are skipped. Each
regression requires at least 12 retained samples, sufficient rank, and a
positive voltage-to-acceleration coefficient. A failed joint fit sets
`ok = false` with an explanation in `error`.

### `deriveDrivetrain`

```cpp
bool deriveDrivetrain(const std::vector<Record>& records, const std::string& left,
                      const std::string& right, const std::string& name,
                      std::vector<Record>& derived, double track_width_m = 0);
```

**Parameters**

- `records` contains the two SI linear channels. Their rows must pair one
  for one by run ID, test, direction, and exact timestamp.

- `left` and `right` are the distinct source channel names.

- `name` names the new channel. Use a valid channel identifier absent from
  the source records.

- `derived` is the destination. On success the function appends the paired
  results, preserving existing rows. On validation failure it returns false
  and leaves this vector unchanged.

- `track_width_m` is the effective width in metres. Zero averages both sides'
  voltage, position, and velocity into a linear channel. Positive width derives
  angular position and velocity as `(right - left) / width` and voltage as
  `(right - left) / 2` from turn-in-place data. Negative or nonfinite width fails.

**Returns**

True when pairing and derivation succeed. Call `analyze` on the new
channel to fit it.

### `FeedbackGains`

`FeedbackGains` has public `bool valid = false` and public `double kP = 0`,
`kI = 0`, and `kD = 0`. Gains produce volts when applied to error, integrated
error, and error rate in the units of the controller. The suggestions set
`kI = 0` and return the default invalid result for unsupported inputs.

### Feedback suggestions

- `FeedbackGains suggestVelocityFeedback(const Gains& gains, double tau_s)`
  takes the mechanism's fitted gains with positive `kA` and `kV`. `tau_s`
  is the desired positive time constant in seconds. It returns velocity P
  with `kP = kA / tau_s - kV` and `kD = 0`.

- `FeedbackGains suggestPositionFeedback(const Gains& gains, double omega_radps,
  double damping = 1)` takes the fitted gains, desired positive natural
  frequency in rad/s, and positive dimensionless damping ratio. It returns
  position PD with `kP = kA * omega_radps²` and
  `kD = 2 * damping * omega_radps * kA - kV`. The frequency is a design
  parameter for linear mechanisms too.

Negative suggested gains, nonfinite computed values, or invalid design inputs
give `valid = false`. These continuous-time formulas assume compensation for
friction and gravity and omit saturation, delay, and sampling effects.

## `public_units.hpp`

### Constants and unit selectors

- `inline constexpr double metersPerInch = 0.0254` converts inches to metres.

- `inline constexpr double radiansPerDegree = π / 180` converts degrees to radians.

- `LinearUnit::meters` and `LinearUnit::inches` select a linear position unit.

- `AngularUnit::radians` and `AngularUnit::degrees` select an angular position unit.

`OutputUnits` exposes `LinearUnit linear = LinearUnit::inches` and
`AngularUnit angular = AngularUnit::degrees`. Derivatives use the selected
position unit per second or per second squared.

- `bool valid() const` checks both fields for supported enumerators.

- `double scale(Units dimension) const` returns SI units per selected unit
  for the requested linear or angular dimension. Multiply by it to get SI
  motion values and divide by it to convert from SI.

- `const char* name(Units dimension) const` returns the corresponding static
  CSV token, `m`, `in`, `rad`, or `deg`. Use valid selectors and dimension
  when calling `scale` and `name`.

- `double siPerPublicUnit(Units dimension)` returns the scale for the default
  inch or degree unit of that dimension.

### Gain conversion

`Gains gainsInUnits(Gains gainsSI, Units dimension, OutputUnits units = {})`
takes SI gains, their linear or angular dimension, and valid output units. It
returns a converted copy, multiplying `kV` and `kA` by the selected scale.
`kS` and `kG` stay in volts. `Gains gainsInPublicUnits(Gains gains,
Units dimension)` takes the same SI input and uses the default inch/degree
units.

#### `lateralToAngular`

```cpp
Gains lateralToAngular(Gains lateral, double trackWidth, OutputUnits units = {});
```

Converts a simple straight-line model to an ideal in-place turning model.

**Parameters**

- `lateral` contains simple-model gains already expressed in `units.linear`.

- `trackWidth` is the finite positive effective width in that same linear unit.

- `units` declares the input linear unit and requested angular output unit.

**Returns**

Angular gains with `kS` unchanged, `kG = 0`, and `kV` and `kA`
multiplied by half the width and the radians-per-output-unit scale. Invalid
output selectors, width, or nonfinite input `kS`, `kV`, or `kA` return four
NaNs. This estimates turn gains from straight motion. A separate spin capture
can account for effects such as scrub.

### `PublicAnalysis`

`PublicAnalysis` exposes `Analysis si` with the original fit and diagnostics
and `OutputUnits output` with the selected gain and setpoint units.

- `const char* positionUnit() const` returns the static unit token for
  `si.units` using `output`.

- `Gains gains() const` returns the joint gains in the selected output units.

- `double calculate(double position, double velocity, double acceleration) const`
  returns unclamped model volts. Pass desired signed setpoints in the selected
  units. `position` is used only for the arm, where it measures angle from
  horizontal and is converted to radians internally.

### `analyzePublic`

```cpp
PublicAnalysis analyzePublic(const std::vector<Record>& records,
                             const std::string& motor,
                             const AnalysisConfig& configSI = {},
                             OutputUnits output = {});
```

**Parameters**

- `records` supplies SI records.

- `motor` is the exact channel name.

- `configSI` supplies the same SI analysis settings as `analyze`.

- `output` selects the desired units. Pass valid selectors.

**Returns**

The SI fit alongside the chosen units. Check `si.ok` before evaluating the result.

### `writePublicRecord`

```cpp
bool writePublicRecord(std::FILE* file, const Record& row, OutputUnits output = {});
```

**Parameters**

- `file` is the caller-owned writable stream.

- `row` contains SI measurements.

- `output` selects the emitted position and velocity units and their token.

Voltage and time are unchanged. Write the header with
[`writeRecordHeader`](capture.md#record-and-named-csv).

**Returns**

False on an invalid output selector, record, null stream, or reported write failure. The caller checks final flush and close.
