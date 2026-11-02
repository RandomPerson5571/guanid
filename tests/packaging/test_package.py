import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[2]


def load(name):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


package = load('package')
checker = load('check_package')


class PackagingTests(unittest.TestCase):
    def test_reproducible_complete_and_isolated(self):
        with tempfile.TemporaryDirectory() as tmp:
            a, b = Path(tmp)/'a', Path(tmp)/'b'
            archive, source = package.build(a,'RandomPerson5571/VexRobotics')
            archive2, source2 = package.build(b,'RandomPerson5571/VexRobotics')
            self.assertEqual(archive.read_bytes(),archive2.read_bytes())
            self.assertEqual(source.read_bytes(),source2.read_bytes())
            manifest = checker.check(archive)
            self.assertEqual(manifest['version'],package.version())
            with zipfile.ZipFile(source) as z:
                prefix=f'guanid-{package.version()}/'
                for name in ('LICENSE','Makefile','CMakeLists.txt','cmake/guanidConfig.cmake.in','scripts/package.py','tests/packaging/test_package.py','tools/analyze.cpp'):
                    self.assertIn(prefix+name,z.namelist())
                self.assertFalse(any('__pycache__' in name or '/dist/' in name for name in z.namelist()))
                z.extractall(Path(tmp)/'source')
            # The distributed source must reproduce the installable template,
            # without relying on any omitted checkout files.
            rebuilt, _ = package.build(Path(tmp)/'rebuilt', root=Path(tmp)/'source'/f'guanid-{package.version()}')
            self.assertEqual(archive.read_bytes(),rebuilt.read_bytes())
            for line in (a/'SHA256SUMS').read_text().splitlines():
                digest, name = line.split('  ')
                self.assertEqual(digest,hashlib.sha256((a/name).read_bytes()).hexdigest())
            depot=json.loads((a/'guanid-depot.json').read_text())[0]
            self.assertIn('/guanid-v'+package.version()+'/',depot['metadata']['location'])

    def test_unsafe_or_foreign_files_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            archive,_=package.build(Path(tmp))
            with zipfile.ZipFile(archive,'a') as z:
                z.writestr('src/main.cpp','overwrite')
            with self.assertRaises(ValueError):
                checker.check(archive)

    def test_bad_version_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            (root/'VERSION').write_text('../unsafe')
            with self.assertRaises(ValueError):
                package.version(root)

    def test_manifest_is_understood_by_real_conductor(self):
        try:
            from pros.conductor.templates import ExternalTemplate
        except ImportError:
            self.skipTest('Optional PROS CLI not installed')
        with tempfile.TemporaryDirectory() as tmp:
            archive,_=package.build(Path(tmp))
            template=ExternalTemplate(str(archive),error_on_decode=True)
            self.assertEqual(template.name,'guanid')
            self.assertEqual(template.version,package.version())
            self.assertIn('include/guanid/pros.hpp',template.system_files)
            self.assertEqual(template.user_files,[])


if __name__ == '__main__':
    unittest.main()
