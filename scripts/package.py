"""Create reproducible header-only PROS and complete source release archives."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import zipfile

ROOT = Path(__file__).resolve().parents[1]
KERNELS = '^4.2.1'


def version(root=ROOT):
    value = (root / 'VERSION').read_text().strip()
    if not re.fullmatch(r'\d+\.\d+\.\d+', value):
        raise ValueError('VERSION must contain a stable major.minor.patch version')
    return value


def sources(root=ROOT):
    paths = {}
    for name in ('VERSION', 'README.md', 'LICENSE', 'NOTICE', 'CHANGELOG.md',
                 'CONTRIBUTING.md', 'SECURITY.md', 'CMakeLists.txt', 'Makefile'):
        paths[name] = (root / name).read_bytes()
    for directory, suffixes in (
        ('include/guanid', {'.hpp'}), ('docs', {'.md'}), ('examples', {'.cpp'}),
        ('tools', {'.cpp', '.py', '.txt'}), ('scripts', {'.py'}),
        ('tests', {'.cpp', '.py', '.cmake', '.hpp', '.h'}), ('cmake', {'.in'})):
        for path in sorted((root / directory).rglob('*')):
            if path.is_file() and path.suffix in suffixes and '__pycache__' not in path.parts:
                if path.is_symlink():
                    raise ValueError('Refusing symlink in release: ' + str(path))
                paths[path.relative_to(root).as_posix()] = path.read_bytes()
    return paths


def archive(path, files):
    # ZIP timestamps/permissions/order are fixed so rebuilding yields identical bytes.
    with zipfile.ZipFile(path, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=9) as z:
        for name, data in sorted(files.items()):
            info = zipfile.ZipInfo(name, date_time=(2026, 1, 1, 0, 0, 0))
            info.create_system = 3
            info.external_attr = 0o100644 << 16
            info.compress_type = zipfile.ZIP_DEFLATED
            z.writestr(info, data, compresslevel=9)


def build(out_dir, repository=None, root=ROOT):
    v = version(root)
    files = sources(root)
    payload = {name: data for name, data in files.items() if name.startswith('include/guanid/')}
    for name, data in files.items():
        if name in ('VERSION','README.md','LICENSE','NOTICE','CHANGELOG.md','CONTRIBUTING.md','SECURITY.md') or name.startswith(('docs/','examples/','tools/')):
            payload['share/guanid/' + name] = data
    manifest = {
        'name': 'guanid', 'version': v, 'target': 'v5', 'supported_kernels': KERNELS,
        'system_files': sorted(payload), 'user_files': [],
        'metadata': {'description': 'Motor-channel system identification for PROS V5 (header-only)'}
    }
    payload['template.pros'] = (json.dumps(manifest, indent=2) + '\n').encode()
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    template = out_dir / f'guanid@{v}.zip'
    source = out_dir / f'guanid-{v}-source.zip'
    archive(template, payload)
    archive(source, {f'guanid-{v}/' + name: data for name, data in files.items()})
    artifacts = [template, source]
    if repository:
        if not re.fullmatch(r'[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+', repository):
            raise ValueError('Repository must be OWNER/REPO')
        depot = out_dir / 'guanid-depot.json'
        entry = {key: manifest[key] for key in ('name','version','target','supported_kernels')}
        entry['py/object'] = 'pros.conductor.templates.base_template.BaseTemplate'
        entry['metadata'] = {'location': f'https://github.com/{repository}/releases/download/guanid-v{v}/guanid%40{v}.zip'}
        depot.write_text(json.dumps([entry], indent=2) + '\n')
        artifacts.append(depot)
    (out_dir / 'SHA256SUMS').write_text(''.join(
        hashlib.sha256(path.read_bytes()).hexdigest() + '  ' + path.name + '\n' for path in artifacts))
    return template, source


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out-dir', type=Path, default=ROOT/'dist')
    parser.add_argument('--repository', help='OWNER/REPO for a release-specific Conductor depot')
    args = parser.parse_args()
    for path in build(args.out_dir, args.repository):
        print(path)


if __name__ == '__main__':
    main()
