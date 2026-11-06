# Physical validation protocol

Three reported powered Drive Lab characterization sessions are included in
[drive-lab-results.md](drive-lab-results.md), with their provenance and limitations.
They provide fitted-gain evidence, but no measured stop latency or independent
validation of the new controller mode. Do not infer safe robot behavior from
fit metrics, host tests, example gains, or consumer compilation.

1. Support gravity-loaded mechanisms and provide independent disable access.
   Clear the entire motion path, including coast-down, and verify travel limits.
2. Check cartridge configuration, encoder units, wheel/spool diameter, gearing,
   and sensor/voltage signs at low voltage. Check each motor before averaging.
3. Suspend other motor-command tasks. Put hold-to-run, competition disable, travel
   and temperature checks in your permit predicate. Confirm release stops output.
4. Start below the example voltage/time defaults. Collect quasistatic and dynamic
   forward/reverse independently, repositioning and stopping between tests. Use a
   charged battery and repeat under consistent load/surface conditions.
5. Preserve the raw CSV. Record robot revision, kernel/compiler, wheel/gearing,
   units, load, battery conditions, test settings, timestamps, and operator notes.
6. Inspect retained samples, voltage residuals, R², direction comparisons and
   warnings. Trim impacts/dropouts; rerun invalid captures. Verify arm horizontal
   zero and gravity direction before applying kG.
7. Validate feedforward at low speed before feedback. Clamp volts and begin with
   conservative feedback. Suggested gains do not account for saturation or delay.
8. For effective width, independently measure unwrapped rotation; repeat both
   directions on the normal surface/load. Record spread, not only the mean.

Document observed stop latency and communication failures. Zero commanded voltage
may permit a loaded mechanism to fall. Do not bypass an independent disable or
remove mechanical support based solely on successful software checks.
