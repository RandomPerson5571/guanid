# Validation records

## Reproduce the checks

From a guanid source checkout or extracted source ZIP, run `make test` for
the host and packaging suites, `python3 scripts/test_cmake.py` for CTest
and the installed CMake consumer, and `make check-template` for the template
manifest. Follow the [consumer check](installation.md#build-a-template-locally)
to repeat the real PROS compile and link checks. Record hardware behavior
separately using the [physical validation protocol](hardware-validation.md).

## 0.1.0 candidate — November 1, 2026

Historical local validation for the 0.1.0 source (not a claim about 0.2.0).

- AppleClang 21, C++17: all five portable suites passed with AddressSanitizer and
  UndefinedBehaviorSanitizer. The C++ capture/analyzer round trip also passed.
- CMake 4.4.4: all six CTest cases passed, including the capture/JSON/SVG/invalid
  CSV integration check. Installation and a separate `find_package(guanid)`
  consumer linked successfully.
- Packaging tests: deterministic archives, manifest ownership/completeness,
  checksums, source contents, invalid version/file rejection, and parsing with
  the real PROS Conductor loader passed (PROS CLI 3.5.6).
- PROS 4.2.1 and 4.2.2: independently created new V5 projects, fetched/applied the
  guanid ZIP with Conductor, compiled the installed PROS examples plus inert entry
  points, and linked both monolithic and hot/cold firmware. ARM GCC 14.3.1 was used
  from the installed VS Code PROS toolchain. No robot connection/upload occurred.
- The GitHub workflow YAML parsed locally. Remote GitHub jobs and draft release
  creation have not been executed by this local preparation.

The source ZIP is intended to rebuild the same template and tools; it contains
all public headers, build files, package/check scripts, examples and tests. Release
SHA256SUMS identify exact archive bytes. This document records a local candidate
check, not validation of future changes or every supported kernel/toolchain.
CI repeats the PROS checks using its pinned ARM toolchain.

No physical characterization, motor-stop latency measurement, gravity-load test,
or effective-width trial is claimed. Follow hardware-validation.md and record
robot-specific evidence before relying on fitted gains in competition.

## Physical characterization — November 6, 2026

Three real Drive Lab captures are transcribed in
[drive-lab-results.md](drive-lab-results.md). They report left/right fits with
voltage RMSEs between 0.1569 and 0.2367 V. They do not include stop-latency,
gravity-load, or independent prediction tests. The 12 V speed/acceleration limits
in the third entry are estimates from the model, not measured limits.

## 0.2.0 release gates

Tagged releases require the seven host suites (including the drivetrain
controller/cancellation suite), CLI and public-unit integration tests, package
regression tests, Windows MSVC CMake installation/consumer tests, and real
PROS 4.2.1 and 4.2.2 fresh-project monolithic/hot-cold builds in GitHub Actions.
The published checksums refer to the tested release artifacts. Consult the
specific tag's Actions run for the actual tool versions and results.

Local 0.2.0 preparation on November 6, 2026 passed all seven host suites with
AppleClang 21 and ASan/UBSan, all nine CTest cases with CMake 4.4.4, installation
and a separate `find_package` consumer, and eight package/depot regression tests.
The Windows results and release-time PROS builds are recorded by the tagged CI
run; local macOS tests do not stand in for those platform checks.

Fresh local Conductor installs of the packaged 0.2.0 template also compiled all
three installed PROS examples and linked monolithic plus hot/cold firmware with
both PROS 4.2.1 and 4.2.2. The local ARM GCC toolchain was 14.3.1. No firmware was
uploaded. Published artifacts are independently verified by the release workflow.
