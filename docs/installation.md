# Installation and upgrades

Install the official PROS CLI/toolchain and create or open a V5 PROS 4 project.
Download [guanid@0.2.0.zip](https://github.com/RandomPerson5571/guanid/releases/download/guanid-v0.2.0/guanid%400.2.0.zip)
and [SHA256SUMS](https://github.com/RandomPerson5571/guanid/releases/download/guanid-v0.2.0/SHA256SUMS)
from the [0.2.0 release](https://github.com/RandomPerson5571/guanid/releases/tag/guanid-v0.2.0),
or build the template from this source. Compare the ZIP's SHA-256 digest with its
entry in SHA256SUMS before installing. On macOS use `shasum -a 256 guanid@0.2.0.zip`;
on Linux use `sha256sum guanid@0.2.0.zip`. On PowerShell:

```powershell
Get-FileHash .\guanid@0.2.0.zip -Algorithm SHA256
Get-Content .\SHA256SUMS
```

Keep the ZIP intact and run these commands **inside your robot project**:

```sh
pros conduct fetch /absolute/path/to/guanid@0.2.0.zip
pros conduct apply guanid@0.2.0
pros make
```

Include `guanid/pros_drivetrain.hpp` for the drivetrain driver mode, or
`guanid/pros.hpp` for named motor channels and `guanid/analysis.hpp` for in-code
fitting. Your robot code supplies the motors and controller.
Follow [getting started](getting-started.md) for a first drivetrain session.

The template owns only `include/guanid/` and `share/guanid/`. Documentation,
examples, the license and desktop analyzer source live under `share/guanid/`.
It never installs `src/main.cpp`, a Makefile, project configuration, PROS headers,
or a dependency archive. It therefore needs no archive link flags. Treat installed
headers as managed files; keep local changes in your application or contribute
upstream. Conductor upgrades replace the template's system files.

The analyzer can be compiled from the installed source with a desktop compiler,
from your robot project's root (this does not build robot firmware):

```sh
c++ -std=c++17 -I include share/guanid/tools/analyze.cpp -o guanid_analyze
```

On Windows, open **Developer PowerShell for Visual Studio 2022** with the
Desktop development with C++ workload installed:

```powershell
cl /nologo /EHsc /std:c++17 /I include share\guanid\tools\analyze.cpp /Fe:guanid_analyze.exe
.\guanid_analyze.exe C:\captures\drivetrain_sysid.csv --out gains.json --plot-dir plots
```

The source release includes CMake for a complete host build. Robot projects must
be built with `pros make`, not a desktop compiler.

To upgrade, fetch the newer ZIP and apply its exact version. To remove:

```sh
pros conduct uninstall guanid
```

Your captured data and application code remain your responsibility. Review an
upgrade's CHANGELOG first. If the old headers were copied manually, back up any
edits and remove the copied tree before applying the managed template; resolve
Conductor ownership conflicts rather than blindly forcing an overwrite.

## Build a template locally

From the guanid source directory:

```sh
make template
python3 scripts/check_package.py dist/guanid@0.2.0.zip
```

Or run `python3 scripts/package.py --out-dir dist` without Make. Python 3.9+ is
needed only for packaging; no PROS kernel download, ARM compiler, or CMake is
required to create this header-only ZIP. This uses Conductor's template format
rather than `pros make template`, which normally packages a compiled library.
`dist` also receives a complete source archive and SHA256SUMS.

With PROS and ARM tools installed, test installation/linking in a disposable new
project (the destination must not already exist):

```sh
python3 scripts/test_template.py /tmp/guanid-consumer dist/guanid@0.2.0.zip \
  --kernel-version 4.2.2
```

The check compiles inert application entry points plus all three PROS examples, tests
monolithic and hot/cold output, and never uploads firmware. Install/fetch caches
are handled by PROS. Pin the kernel version when recording release evidence.

## Stable Conductor depot and upgrades

Register the maintained depot once, inside your robot project:

```sh
pros conduct add-depot guanid https://raw.githubusercontent.com/RandomPerson5571/guanid/depot/guanid-depot.json
pros conduct apply guanid@0.2.0
pros make
```

The `depot` branch is updated only after a versioned release is published and
retains earlier entries, so older versions remain installable. For reproducible
builds, pin the exact version. To upgrade, review the release notes, then apply
the newer exact version. Conductor replaces files it manages; do not edit the
installed library headers in place. If depot registration or downloading fails,
use the direct ZIP installation above.

Each release also includes its own `guanid-depot.json` snapshot. The stable URL
above accumulates those entries; `releases/latest/download/guanid-depot.json`
would contain only the newest snapshot and is not the stable depot.

## Desktop CMake installation

For a host application, CMake can install the portable headers, analyzer and
package configuration separately from the PROS template. Run these commands
from the source checkout or extracted source ZIP:

```sh
cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/your/prefix
cmake --build build
cmake --install build
```

A consuming CMake project can use `find_package(guanid CONFIG REQUIRED)` and
`target_link_libraries(your_target PRIVATE guanid::guanid)`, with the chosen prefix
on CMAKE_PREFIX_PATH. Including the PROS adapter still requires real PROS headers;
portable analysis/feedforward does not. The installed target requires C++17.

### Windows CMake build from the source ZIP

Download and extract [guanid-0.2.0-source.zip](https://github.com/RandomPerson5571/guanid/releases/download/guanid-v0.2.0/guanid-0.2.0-source.zip).
With CMake and Visual Studio 2022's C++ workload installed, open PowerShell in
its `guanid-0.2.0` directory:

```powershell
cmake -S . -B build -DGUANID_BUILD_EXAMPLES=ON
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
.\build\Release\guanid_analyze.exe C:\captures\drivetrain_sysid.csv --out gains.json --plot-dir plots
```

The `Release` subdirectory applies to the default Visual Studio generator.
The desktop tool needs no PROS installation. Building robot firmware still
requires the official PROS CLI and ARM toolchain.
