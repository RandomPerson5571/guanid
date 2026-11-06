"""Merge a published release snapshot into the stable Conductor depot.

Existing entries are retained. Reusing an existing version with different
metadata is rejected: release archives and their locations are immutable.
"""
import argparse
import json
from pathlib import Path
import re


def merge(existing, snapshot):
    entries = {}
    for entry in [*existing, *snapshot]:
        if not isinstance(entry, dict):
            raise ValueError('Depot entries must be objects')
        version = entry.get('version', '')
        if (entry.get('name') != 'guanid' or entry.get('target') != 'v5' or
                not re.fullmatch(r'\d+\.\d+\.\d+', version) or
                entry.get('py/object') != 'pros.conductor.templates.base_template.BaseTemplate'):
            raise ValueError('Invalid guanid depot entry')
        location = entry.get('metadata', {}).get('location', '')
        if not re.fullmatch(r'https://github\.com/[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+/releases/download/'
                            r'guanid-v' + re.escape(version) + r'/guanid%40' + re.escape(version) + r'\.zip', location):
            raise ValueError('Depot must point to a versioned GitHub release archive')
        if version in entries and entries[version] != entry:
            raise ValueError('Refusing to replace existing version ' + version)
        entries[version] = entry
    return [entries[v] for v in sorted(entries, key=lambda v: tuple(map(int, v.split('.'))), reverse=True)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('snapshot', type=Path)
    parser.add_argument('--existing', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    existing = json.loads(args.existing.read_text()) if args.existing else []
    snapshot = json.loads(args.snapshot.read_text())
    if not isinstance(existing, list) or not isinstance(snapshot, list) or not snapshot:
        raise ValueError('Expected depot lists and a nonempty release snapshot')
    args.out.write_bytes((json.dumps(merge(existing, snapshot), indent=2) + '\n').encode())


if __name__ == '__main__':
    main()
