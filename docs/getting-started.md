# Getting started with guanid

See the [code structure and flow diagrams](code-map.md) for a visual overview.

guanid measures how a motor or mechanism responds to voltage and fits
feedforward gains: `kS` for friction, `kV` for velocity, and `kA` for
acceleration. Arm and elevator models also fit a gravity term, `kG`.
It does not tune a LemLib PID controller.

## Install

Install the versioned ZIP or register the stable Conductor depot using the
[installation guide](installation.md). Alternatively, copy `include/guanid/`
into your PROS project's `include/` directory. No source files or LemLib dependency
are needed.

Build robot firmware with `pros make` inside your robot project.

## Capture a drivetrain

1. Copy [`examples/drivetrain_sysid.cpp`](../examples/drivetrain_sysid.cpp)
   into your robot project's `src/` directory.
2. Use separate left and right motor groups and your existing controller.
   Configure reversal so positive voltage drives both sides forward. The driver
   mode temporarily sets each motor's encoder units to degrees.
3. Declare the example function before calling it from your dedicated test mode:

   ```cpp
   guanid::Result drivetrainSysId(pros::MotorGroup&, pros::MotorGroup&,
                                pros::Controller&, double, double);

   // Inside the test mode, using your existing objects:
   drivetrainSysId(leftMotors, rightMotors, controller, 3.25, 36.0 / 48.0);
   ```

   Include `guanid/pros_drivetrain.hpp` in the calling file. Replace `3.25` with your wheel
   diameter in inches and `36.0 / 48.0` with wheel turns per motor turn. Replace
   the example's 13.5-inch effective track width with your calibrated width, or
   set `config.track_width_m = 0` to skip turn estimates.
4. Insert an SD card if you want a CSV; fitting can also run in memory.
   Review the example's voltage and duration settings for
   your robot, clear the travel path, and suspend other motor commands.
5. Follow the controller prompt and hold **L1** for each profile: ramp forward,
   ramp reverse, step forward, then step reverse. Release L1 between profiles.
   Releasing early cancels the current attempt; hold L1 again to retry it.
   Hold **X** with L1 released and use left-Y/right-X sticks to reposition.
   Repositioning defaults to 3 V, bounded by both capture voltage caps; lower
   `config.reposition_max_volts` further or set it to zero to disable manual motion.
   **B** exits.

On completion, the example prints lateral gains and any available angular and
drive-limit estimates to the console. Its return value reports capture status.
Use `runDrivetrainTuning` directly to inspect fit success and per-side diagnostics.

With a writable SD card, the example exports inch-based records to
`/usd/drivetrain_sysid.csv`. A new session replaces that file. Save the raw CSV
and note the run IDs of any retries or interrupted tests before starting over.
The in-memory fit discards partial attempts. To reproduce it on your computer,
copy the header and rows from only the four completed attempts to a separate CSV.

## Analyze the capture

From the guanid source directory on your computer:

```sh
cmake -S . -B /tmp/guanid-build
cmake --build /tmp/guanid-build
/tmp/guanid-build/guanid_analyze /path/to/drivetrain_sysid.csv \
  --out /tmp/drivetrain-gains.json --plot-dir /tmp/guanid-plots
```

On Windows, follow the [MSVC or CMake commands](installation.md#windows-cmake-build-from-the-source-zip).

The JSON contains separate left/right fits and warnings. Inspect the plots and
warnings before using the gains. Default analyzer output uses inches and degrees;
keep the controller's units consistent with the reported gain units. Feedforward
outputs volts, while PROS `move_voltage()` accepts millivolts.

For other mechanisms, in-code analysis, or combined drivetrain fits, see the
[README](../README.md), [unit guide](units.md), and [API reference](api.md).
For failed captures or fits, see [troubleshooting](troubleshooting.md).
