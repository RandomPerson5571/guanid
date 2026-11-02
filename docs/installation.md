# Install and remove

Install the official PROS CLI/toolchain and create or open a V5 PROS 4 project.
Obtain `guanid@0.1.0.zip` from a published release or build it from this source.
Keep the ZIP intact and run these commands **inside your robot project**:

```sh
pros conduct fetch /absolute/path/to/guanid@0.1.0.zip
pros conduct apply guanid@0.1.0
pros make
```

Include `guanid/pros.hpp` for the named motor-channel runner and
`guanid/analysis.hpp` for in-code fitting. Your robot code owns all motors and
sensors. See ../README.md and the examples for explicit test invocations.

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

On Windows, use the equivalent compiler/output suffix for your compiler shell.
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
python3 scripts/check_package.py dist/guanid@0.1.0.zip
```

Or run `python3 scripts/package.py --out-dir dist` without Make. Python 3.9+ is
needed only for packaging; no PROS kernel download, ARM compiler, or CMake is
required to create this header-only ZIP. This uses Conductor's template format
rather than `pros make template`, which normally packages a compiled library.
`dist` also receives a complete source archive and SHA256SUMS.

With PROS and ARM tools installed, test installation/linking in a disposable new
project (the destination must not already exist):

```sh
python3 scripts/test_template.py /tmp/guanid-consumer dist/guanid@0.1.0.zip \
  --kernel-version 4.2.2
```

The check compiles inert application entry points plus both PROS examples, tests
monolithic and hot/cold output, and never uploads firmware. Install/fetch caches
are handled by PROS. Pin the kernel version when recording release evidence.

## Optional release-specific depot

Release packaging can generate `guanid-depot.json` with a concrete download URL.
After that release is published, register its hosted JSON URL:

```sh
pros conduct add-depot guanid https://HOST/PATH/guanid-depot.json
pros conduct apply guanid@0.1.0
```

Replace HOST/PATH with the real published asset location. This is a release-specific
snapshot, not a rolling stable depot; register a maintained stable URL separately
if the project later provides one. No nonexistent public depot is assumed.

References: [PROS Conductor guide](https://pros.cs.purdue.edu/v5/cli/conductor.html),
[template loader](https://github.com/purduesigbots/pros-cli/blob/master/pros/conductor/templates/external_template.py).

## Desktop CMake installation

For a host application, CMake can install the portable headers, analyzer and
package configuration separately from the PROS template:

```sh
cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/your/prefix
cmake --build build
cmake --install build
```

A consuming CMake project can use `find_package(guanid CONFIG REQUIRED)` and
`target_link_libraries(your_target PRIVATE guanid::guanid)`, with the chosen prefix
on CMAKE_PREFIX_PATH. Including the PROS adapter still requires real PROS headers;
portable analysis/feedforward does not. The installed target requires C++17.
