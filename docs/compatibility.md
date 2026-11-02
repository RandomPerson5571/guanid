# Compatibility and dependencies

The PROS Conductor template is named `guanid`, targets `v5`, and declares
`^4.2.1` kernel compatibility (4.2.1 through versions before 5.0). The release
consumer workflow checks PROS 4.2.1 and 4.2.2 using the real kernel and ARM tools.
A declaration is not evidence that every future kernel or hardware combination
has been tested. Inspect the specific release's CI evidence before adoption.

The portable headers and desktop analyzer use C++17. The PROS adapter uses the
kernel's language standard; the local 4.2.2 headers have been syntax-checked with
C++23. Do not force an older standard than your installed kernel supports.
There is no LemLib, guantune, WPILib, scheduler, or subsystem dependency. The
robot uses PROS for voltage commands, motor telemetry and time. Only the desktop
analyzer uses `std::filesystem`; it is not included by the robot public headers.

The library is header-only. No ARM archive or startup code is installed. The
consumer compiles the used routines; no hidden task or motor object is created.
PROS motor telemetry is sequential, and measured values may have sensor latency.
Motor channels accept only PROS Motor/MotorGroup objects and require degree
encoder units and matching output gearing. Desktop analysis remains independent
of PROS.

Host tests/simulation verify numerical and lifecycle behavior. A consumer link
check verifies installation, real headers, and linker compatibility, not physical
robot behavior. No real-robot validation session is currently bundled. Read
hardware-validation.md before running a powered test.
