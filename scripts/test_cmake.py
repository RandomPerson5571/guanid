"""Verify CMake build/tests/install and a separate find_package consumer."""
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    cmake, ctest = shutil.which('cmake'), shutil.which('ctest')
    if not cmake or not ctest:
        raise SystemExit('Install CMake/CTest and put them on PATH')
    with tempfile.TemporaryDirectory(prefix='guanid-cmake-') as tmp:
        work = Path(tmp)
        subprocess.run([cmake,'-S',str(ROOT),'-B',str(work/'build'),'-DGUANID_BUILD_EXAMPLES=ON',
                        '-DCMAKE_INSTALL_PREFIX='+str(work/'install')],check=True)
        subprocess.run([cmake,'--build',str(work/'build'),'--config','Release','--parallel'],check=True)
        subprocess.run([ctest,'--test-dir',str(work/'build'),'-C','Release','--output-on-failure'],check=True)
        subprocess.run([cmake,'--install',str(work/'build'),'--config','Release'],check=True)
        consumer=work/'consumer'; consumer.mkdir()
        (consumer/'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.16)
project(consumer LANGUAGES CXX)
find_package(guanid CONFIG REQUIRED)
add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE guanid::guanid)
''')
        (consumer/'main.cpp').write_text('''#include <guanid/analysis.hpp>
int main() { guanid::TrackWidthCalibration c; return c.add({1,1,6.28}) ? 0 : 1; }
''')
        subprocess.run([cmake,'-S',str(consumer),'-B',str(work/'consumer-build'),
                        '-DCMAKE_PREFIX_PATH='+str(work/'install')],check=True)
        subprocess.run([cmake,'--build',str(work/'consumer-build'),'--config','Release'],check=True)
    print('CMake build, CTest, install and separate consumer passed')


if __name__ == '__main__':
    main()
