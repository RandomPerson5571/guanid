# Track-width calibration

[API index](../api.md)

Calibration estimates effective track width from wheel travel during an
in-place spin and independently measured, continuous rotation. Both
calibrators start empty, accept repeated trials with equal weight, and
report statistics without commanding motors.

## `track_width.hpp`

### `TrackWidthTrial`

Public fields are `double left_distance_m = 0` and
`double right_distance_m = 0` for positive wheel travel magnitudes in
metres, and `double rotation_rad = 0` for independently measured unwrapped
rotation in radians. Either rotation sign is accepted.

`double effectiveTrackWidthM() const` returns
`(left_distance_m + right_distance_m) / abs(rotation_rad)` in metres.
Nonfinite inputs, nonpositive travel, zero rotation, or a nonfinite or
nonpositive width produce NaN.

### `TrackWidthResult`

- `bool valid = false` indicates at least one accepted trial.

- `std::size_t trials = 0` counts accepted trials.

- `double track_width_m = 0` is their arithmetic mean in metres.

- `double sample_stddev_m = 0` is sample standard deviation in metres.
  It is zero with one trial.

- `double min_width_m = 0` and `double max_width_m = 0` are the
  smallest and largest accepted widths in metres.

### `TrackWidthCalibration`

- `bool add(const TrackWidthTrial& trial)` accepts one spin's distance
  magnitudes and independent rotation. True means the statistics were updated.
  Invalid measurements or values that would overflow the running statistics
  return false and leave them unchanged.

- `bool addSpin(double left_start_m, double left_end_m, double right_start_m,
  double right_end_m, double heading_start_rad, double heading_end_rad)` takes
  signed start and end wheel positions in metres for each side, followed by
  independently measured start and end headings in continuous radians. Both
  wheel encoders should report forward travel as positive. The method uses the
  magnitude of each side's displacement and the heading difference. It rejects
  nonfinite values, stationary sides, or sides moving in the same direction,
  and otherwise returns the result of `add`.

- `TrackWidthResult result() const` returns the current summary by value.

- `void clear()` resets the count and statistics for a new set of trials.

## `TrackWidthCalibrationInches`

Header `<guanid/public_units.hpp>`.

### `addSpin`

```cpp
bool addSpin(double leftStartIn, double leftEndIn, double rightStartIn,
             double rightEndIn, double headingStartDeg, double headingEndDeg);
```

**Parameters**

- `leftStartIn` and `leftEndIn` are the signed left wheel snapshots in inches.

- `rightStartIn` and `rightEndIn` are the signed right wheel snapshots in inches.

- `headingStartDeg` and `headingEndDeg` are independent headings in continuous degrees.

**Returns**

True if the converted trial is accepted. A failed trial leaves the result unchanged.

### `result`

`Result result() const` returns a snapshot of the nested
`TrackWidthCalibrationInches::Result`, with these public fields:

- `bool valid` indicates at least one accepted trial.

- `std::size_t trials` counts accepted trials.

- `double trackWidthIn` is the arithmetic mean width in inches.

- `double sampleStddevIn` is the sample standard deviation in inches,
  zero with one trial.

- `double minWidthIn` and `double maxWidthIn` are the smallest and largest
  accepted widths in inches.

The empty result returned by either calibrator has `valid = false` and zero
counts and measurements.

### `clear`

`void clear()` resets the inch calibrator. Both
calibrator classes keep their running state private.
