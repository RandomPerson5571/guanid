"""Compile and run portable tests without requiring CMake. No robot connection."""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    compiler = shlex.split(os.environ.get('CXX', 'c++'))
    flags = ['-std=' + os.environ.get('CXXSTD', 'c++17'), '-Wall', '-Wextra', '-Wpedantic', '-Werror']
    if os.environ.get('SANITIZE', '1') == '1':
        flags += ['-fsanitize=address,undefined', '-fno-sanitize-recover=all']
    with tempfile.TemporaryDirectory(prefix='guanid-host-') as directory:
        work = Path(directory)
        for name in ('core', 'routine', 'sysid', 'analysis', 'track_width', 'pros'):
            binary = work / name
            includes = ['-I'+str(ROOT/'include')]
            if name == 'pros':
                includes.append('-I'+str(ROOT/'tests/stubs'))
            subprocess.run([*compiler, *flags, *includes, str(ROOT/'tests'/f'{name}.cpp'), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)
        for name, source in [('capture', 'examples/callback_capture.cpp'), ('analyze', 'tools/analyze.cpp')]:
            subprocess.run([*compiler,*flags,'-I'+str(ROOT/'include'),str(ROOT/source),'-o',str(work/name)],check=True)
        with (work/'capture.csv').open('w') as stream:
            subprocess.run([str(work/'capture')],stdout=stream,check=True)
        subprocess.run([str(work/'analyze'),str(work/'capture.csv'),'--out',str(work/'gains.json'),
                        '--plot-dir',str(work/'plots')],check=True)
    print('Six host suites and C++ capture/analysis passed')


if __name__ == '__main__':
    main()
