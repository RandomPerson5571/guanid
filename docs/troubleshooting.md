# Troubleshooting

- **Conductor cannot find guanid:** fetch the actual ZIP first, then apply its
  exact name/version. A local source directory is not a registered remote depot.
- **Wrong kernel:** the template targets V5 PROS 4.2.1+ before 5.0. Inspect
  `pros conduct info-project` and update the kernel/toolchain deliberately.
- **Missing headers:** copy/install the complete `include/guanid/` tree. Use your
  kernel's C++ standard and build robot firmware through PROS.
- **L1 does not start the next profile:** enter the test mode in enabled
  opcontrol with a connected controller. Release L1 and hold it again. For
  manual repositioning, release L1, hold X, and check the configured voltage cap.
- **No motion or immediate error:** check the permit predicate, competition state,
  motor connections, degree encoder units, valid telemetry and sink return value.
  Inspect the routine's result after `running()` becomes false.
- **Empty/incomplete captures:** collect each test/direction, use unique run IDs,
  write one header, and check final flush/close. Avoid console text in CSV.
- **Desktop fit differs from the robot fit:** use only completed attempt IDs
  from the exported CSV, use equivalent analysis settings, and compare gains in
  the same units. The optional SD example continues fitting after a write error,
  so inspect the file for incomplete runs.
- **Capture completed but gains are missing:** inspect `analysis.ok` and
  each side's `.si.error` with `runDrivetrainTuning`. The example function
  returns capture status even when regression fails.
- **Bad gains/rank deficiency:** check units, signs, timestamps and excitation;
  retain dynamic acceleration transients and quasistatic motion. Both directions
  are required to separate elevator gravity from friction.
- **Arm kG is incorrect:** verify horizontal zero, output gearing, and sensor
  delay. `Gains::calculate` takes radians for the arm model.
  `PublicAnalysis::calculate` takes the selected angular unit.
- **Stops late:** callbacks must be nonblocking; they run on the calling task.
  Another task must not command these motors. A task kill is not stack unwinding.
- **Turning/odometry remains inaccurate:** calibrate effective track width using
  independent actual rotation, then check slip, surface, load and wheel diameter.

Include the exact versions/configuration and a minimal sanitized capture when
reporting a problem. Preserve the original logs before trimming data.
