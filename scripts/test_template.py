"""Install the release ZIP into a disposable real PROS project and link inert firmware."""
import argparse
from pathlib import Path
import shutil
import subprocess

from check_package import check

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('destination', type=Path)
    parser.add_argument('archive', type=Path)
    parser.add_argument('--kernel-version', default='4.2.2')
    args = parser.parse_args()
    destination = args.destination.expanduser().resolve()
    archive = args.archive.resolve()
    if destination.exists():
        parser.error('Destination must not exist; use a disposable new directory')
    if ROOT == destination or ROOT in destination.parents:
        parser.error('Consumer must be outside the library source directory')
    manifest = check(archive)
    pros = shutil.which('pros')
    if not pros:
        parser.error('Install PROS CLI and ARM toolchain and put pros on PATH')
    subprocess.run([pros,'conduct','new-project',str(destination),'v5',args.kernel_version,'--no-default-libs'],check=True)
    if not (destination/'project.pros').is_file():
        raise SystemExit('PROS did not create the consumer project')

    def run(*args):
        subprocess.run([pros,*args],cwd=destination,check=True)

    run('conduct','fetch',str(archive))
    run('conduct','apply','guanid@'+manifest['version'])
    for name in ('main.c','main.cc','main.cpp'):
        (destination/'src'/name).unlink(missing_ok=True)
    # No motor objects or movement: this program is compile/link evidence only.
    (destination/'src/main.cpp').write_text('''#include "guanid/pros.hpp"
#include "guanid/analysis.hpp"
void initialize() {
    guanid::TrackWidthCalibration calibration;
    calibration.add({1.0, 1.0, 6.283185307179586});
    (void)calibration.result();
    guanid::AnalysisConfig settings;
    settings.require_four_tests = false;
    auto fit = guanid::analyze({}, "unused", settings);
    (void)fit;
    guanid::ProsSysIdRoutine routine({}, {}, [](const guanid::Record&) { return true; });
    routine.cancel();
}
void disabled() {}
void competition_initialize() {}
void autonomous() {}
void opcontrol() {}
''')
    # Compile examples from the INSTALLED archive, never from the checkout.
    for name in ('pros_example.cpp','sysid_pros_example.cpp','drivetrain_sysid.cpp'):
        shutil.copy2(destination/'share/guanid/examples'/name,destination/'src'/('guanid_'+name))
    run('make','all','USE_PACKAGE=0')
    if not (destination/'bin/monolith.bin').is_file():
        raise SystemExit('Missing monolithic consumer firmware')
    run('make','all','USE_PACKAGE=1')
    for name in ('cold.package.bin','hot.package.bin'):
        if not (destination/'bin'/name).is_file():
            raise SystemExit('Missing '+name)
    print('Installed guanid and linked both PROS firmware formats; no upload performed')


if __name__ == '__main__':
    main()
