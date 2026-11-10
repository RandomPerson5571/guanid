# Code structure and flow

These Mermaid diagrams cover the named-channel API used by the current examples.
Arrows in the structure diagram mean “uses.” The library is header-only;
`tools/analyze.cpp` is a separate desktop program.

## Code structure

```mermaid
flowchart TD
    Robot["Robot application / examples"] --> Pros["pros.hpp<br/>ProsSysIdRoutine + ProsMotorChannel"]
    Pros --> Engine["sysid.hpp<br/>SysIdRoutine, Record, CSV, feedforward"]
    Robot --> Mode["pros_drivetrain.hpp<br/>Controller mode + motor groups"]
    Mode --> Pros
    Mode --> Tuner["drivetrain.hpp<br/>DrivetrainTuner"]
    Tuner --> Engine
    Tuner --> Units
    Pros --> Hardware["Existing PROS motors + clock"]
    Engine --> Base["guanid.hpp<br/>Config, test profiles, states, results"]
    App["Desktop or in-code analysis"] --> Analysis["analysis.hpp<br/>Filtering, fitting, diagnostics"]
    CLI["tools/analyze.cpp<br/>CSV input, JSON / SVG output"] --> Analysis
    App --> Units["public_units.hpp<br/>Inch / degree adapters"]
    CLI --> Units
    Units --> Analysis
    Analysis --> Engine
    Engine --> Track["track_width.hpp<br/>Track-width calibration helpers"]
    Pros --> Legacy["routine.hpp<br/>Legacy left / right capture API"]
    Legacy --> Base
```

Public headers live in `include/guanid/`. `examples/` shows robot and simulated
captures; `tests/` and `scripts/` cover host behavior and packaging. The
track-width helpers are also usable on their own and command no motors.

## Capture flow

```mermaid
flowchart TD
    Start["Caller selects test + direction"] --> Validate["Validate config / channels<br/>Command zero volts"]
    Validate --> Ready{"Ready?"}
    Ready -- No --> Stop["finish: attempt zero volts<br/>Set result + idle state"]
    Ready -- Yes --> Init["Assign run ID and start time"]
    Init --> Update["update with operator permit"]
    Update --> Continue{"Permitted and within duration?"}
    Continue -- No --> Stop
    Continue -- Yes --> Due{"Sample due?"}
    Due -- No --> Wait["Wait for next update"]
    Due -- Yes --> Read["Read motor telemetry<br/>Validate channel names / units"]
    Read --> Sink["Send Record per channel<br/>to CSV or in-memory sink"]
    Sink --> Drive["Apply next ramp or step voltage"]
    Drive --> Wait
    Wait --> Update
    Read -. Failure .-> Stop
    Sink -. Failure .-> Stop
    Drive -. Failure .-> Stop
    Stop --> Result["Return completed, cancelled, or error"]
```

Blocking `quasistatic()` / `dynamic()` calls run this update loop internally.
The periodic API leaves update scheduling to the caller. Telemetry is recorded
before the next command; the first sample is taken at zero commanded voltage.
An explicit `cancel()` also enters the stop path. If stopping fails, the result
is `hardware_error`. Run forward and reverse tests for both profiles separately.

## Drivetrain session

The PROS driver mode checks its operator and competition interlocks, temporarily
sets degree encoders, and hands each hold-to-run profile to `DrivetrainTuner`.
A cancelled profile rolls back to its record checkpoint and can be retried.
An external CSV sink has already seen those records. After four completed
profiles, the session fits each side, averages the side gains, and calculates
drive-limit and optional angular estimates. See [drivetrain tuning](drivetrain.md)
for custom controls and result fields.

## Analysis flow

```mermaid
flowchart TD
    CSV["Capture CSV"] --> CLI["Desktop analyzer<br/>Read and normalize units to SI"]
    Records["In-memory Record vector<br/>Already SI"] --> Select
    CLI --> Select["Select channel<br/>Optionally derive combined / turn channel"]
    Select --> Group["Group by run, test, direction<br/>Validate data and required tests"]
    Group --> Filter["Median-filter velocity<br/>Estimate acceleration from timestamps"]
    Filter --> Trim["Exclude rest, gaps, and trimmed windows"]
    Trim --> Fit["Fit simple / elevator / arm model<br/>Recover kS, kV, kA, optional kG"]
    Fit --> Report["Gains, fit metrics, warnings, diagnostics"]
    Report --> Memory["C++ Analysis result<br/>or analyzePublic inch / degree adapter"]
    Report --> Output["CLI: convert output units<br/>Write JSON and optional SVG / diagnostic CSV"]
```

Analysis failures return errors rather than usable gains. Inspect warnings and
fit diagnostics before applying a result. See [getting started](getting-started.md),
[units](units.md), and the [API reference](api.md) for details.
