"""Validate template ownership, manifest completeness and required public files."""
import argparse
import json
from pathlib import Path, PurePosixPath
import zipfile


def check(path):
    with zipfile.ZipFile(path) as package:
        names = package.namelist()
        if len(names) != len(set(names)):
            raise ValueError('Duplicate ZIP entries')
        for name in names:
            if PurePosixPath(name).is_absolute() or '..' in PurePosixPath(name).parts or '\\' in name:
                raise ValueError('Unsafe ZIP path: ' + name)
            if name != 'template.pros' and not name.startswith(('include/guanid/', 'share/guanid/')):
                raise ValueError('Template owns a consumer/dependency file: ' + name)
        manifest = json.loads(package.read('template.pros'))
        def require(condition, message):
            if not condition:
                raise ValueError(message)
        require(manifest['name'] == 'guanid' and manifest['target'] == 'v5', 'Wrong template identity/target')
        require(manifest['supported_kernels'] == '^4.2.1', 'Unexpected kernel compatibility')
        require(manifest['user_files'] == [], 'Unexpected user files')
        require(set(manifest['system_files']) == set(names) - {'template.pros'}, 'Manifest/payload mismatch')
        require(len(manifest['system_files']) == len(set(manifest['system_files'])), 'Duplicate manifest files')
        required = {'include/guanid/' + name for name in (
            'guanid.hpp','routine.hpp','sysid.hpp','pros.hpp','analysis.hpp','track_width.hpp')}
        required |= {'share/guanid/' + name for name in ('LICENSE','NOTICE','README.md','VERSION','CHANGELOG.md')}
        require(required.issubset(names), 'Missing public headers or release documents')
        require(package.read('share/guanid/VERSION').decode().strip() == manifest['version'], 'Version mismatch')
        require(all(package.getinfo(name).compress_type == zipfile.ZIP_DEFLATED for name in names), 'Uncompressed entries')
        if package.testzip() is not None:
            raise ValueError('Corrupt ZIP')
        return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    args = parser.parse_args()
    manifest = check(args.archive)
    print(f"Validated {manifest['name']}@{manifest['version']}: {len(manifest['system_files'])} owned files")
