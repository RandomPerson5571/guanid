# Release candidate validation record

Local validation performed November 1, 2026, for guanid 0.1.0 source.

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
