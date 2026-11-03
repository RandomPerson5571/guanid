# Getting started with guanid

See the [code structure and flow diagrams](code-map.md) for a visual overview.

guanid measures how a motor or mechanism responds to voltage and fits
feedforward gains: `kS` for friction, `kV` for velocity, and `kA` for
acceleration. Arm and elevator models also fit a gravity term, `kG`.
It does not tune a LemLib PID controller.

## Install

Copy `include/guanid/` into your PROS project's `include/` directory.
No source files or LemLib dependency are needed. Alternatively, follow the
[PROS template installation guide](installation.md).

Build robot firmware with `pros make` inside your robot project.

## Capture a drivetrain

1. Copy [`examples/drivetrain_sysid.cpp`](../examples/drivetrain_sysid.cpp)
   into your robot project's `src/` directory.
2. Use your existing left/right motor groups and controller. Set motor encoders
   to degrees and motor reversal so both sides drive forward with positive voltage.
3. Declare the example function before calling it from your dedicated test mode:

   ```cpp
   guanid::Result drivetrainSysId(pros::MotorGroup&, pros::MotorGroup&,
                                pros::Controller&, double, double);

   // Inside the test mode, using your existing objects:
   drivetrainSysId(leftMotors, rightMotors, controller, 3.25, 36.0 / 48.0);
   ```

   Include `guanid/pros.hpp` in the calling file. Replace `3.25` with your wheel
   diameter in inches and `36.0 / 48.0` with wheel turns per motor turn.
4. Insert an SD card. Review the example's voltage and duration settings for
   your robot, clear the travel path, and suspend other motor commands.
5. Hold **L1** and press **Up** for a slow forward ramp, **Down** for a slow
   reverse ramp, **Right** for a forward voltage step, or **Left** for a reverse
   voltage step. Collect all four tests, repositioning between them.
   Release L1 to stop; release it between tests to re-arm. **B** exits.

The example writes `/usd/drivetrain_sysid.csv`. Entering the test mode again
replaces that file, so save each capture before starting another session.

## Analyze the capture

From the guanid source directory on your computer:

```sh
cmake -S . -B /tmp/guanid-build
cmake --build /tmp/guanid-build
/tmp/guanid-build/guanid_analyze /path/to/drivetrain_sysid.csv \
  --out /tmp/drivetrain-gains.json --plot-dir /tmp/guanid-plots
```

The JSON contains separate left/right fits and warnings. Inspect the plots and
warnings before using the gains. Default analyzer output uses inches and degrees;
keep the controller's units consistent with the reported gain units. Feedforward
outputs volts, while PROS `move_voltage()` accepts millivolts.

For other mechanisms, in-code analysis, or combined drivetrain fits, see the
[README](../README.md), [unit guide](units.md), and [API reference](api.md).
For failed captures or fits, see [troubleshooting](troubleshooting.md).
