# Troubleshooting

- **Conductor cannot find guanid:** fetch the actual ZIP first, then apply its
  exact name/version. A local source directory is not a registered remote depot.
- **Wrong kernel:** the template targets V5 PROS 4.2.1+ before 5.0. Inspect
  `pros conduct info-project` and update the kernel/toolchain deliberately.
- **Missing headers:** copy/install the complete `include/guanid/` tree. Use your
  kernel's C++ standard and build robot firmware through PROS.
- **No motion or immediate error:** check the permit predicate, competition state,
  motor connections, degree encoder units, valid telemetry and sink return value.
  Inspect the routine's result after `running()` becomes false.
- **Empty/incomplete captures:** collect each test/direction, use unique run IDs,
  write one header, and check final flush/close. Avoid console text in CSV.
- **Bad gains/rank deficiency:** check units, signs, timestamps and excitation;
  retain dynamic acceleration transients and quasistatic motion. Both directions
  are required to separate elevator gravity from friction.
- **Arm kG is incorrect:** angle must be radians from horizontal, not raw motor
  degrees or a zero set at a different pose. Check sensor delay and gearing.
- **Stops late:** callbacks must be nonblocking; they run on the calling task.
  Another task must not command these motors. A task kill is not stack unwinding.
- **Turning/odometry remains inaccurate:** calibrate effective track width using
  independent actual rotation, then check slip, surface, load and wheel diameter.

Include the exact versions/configuration and a minimal sanitized capture when
reporting a problem. Preserve the original logs before trimming data.
