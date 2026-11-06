# Contributing

Work from the guanid repository root, or `libraries/guanid` when using the
VexRobotics submodule checkout. Commit and push library changes in this repository
before updating the monorepo submodule pointer. Use conventional commit messages.
C++ APIs target C++17 without exceptions as a requirement; the PROS adapter must
also compile using the installed kernel's toolchain. Keep new algorithms portable,
keep callbacks synchronous, and preserve stop/error behavior. The public PROS
runner accepts named Motor/MotorGroup channels; keep drive and encoder reads
inside the adapter. Do not add a hidden
task, a scheduler dependency, or automatic motor movement during initialization.

Run from this directory:

```sh
python3 scripts/test_host.py
python3 -m unittest discover -s tests/packaging -v
python3 scripts/package.py --out-dir dist
python3 scripts/check_package.py dist/guanid@0.2.0.zip
```

A C++ compiler and Python 3.9+ are required for these maintainer commands. Python
is build/test tooling only; the primary library and analyzer remain pure C++.
The host script enables ASan/UBSan by default; set `SANITIZE=0` on unsupported
platforms. Set `CXX` and `CXXSTD` to select a compiler/standard. CMake/CTest is
also supported; enable `GUANID_BUILD_EXAMPLES=ON` for the CLI integration check.
The optional legacy Python tests require `tools/requirements.txt`.

With PROS CLI and ARM tools installed, run the disposable consumer check described
in docs/installation.md. Never run tests by uploading automatically to a robot.
Document units, input validation, failure behavior, and the limits of evidence.
New behavior should have focused numerical/lifecycle tests. Include build results
and any actual hardware evidence in a pull request; never label simulation as a
physical validation. See docs/releasing.md for versioning and release packaging.
