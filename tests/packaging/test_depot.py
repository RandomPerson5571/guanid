import copy
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('update_depot', Path(__file__).resolve().parents[2] / 'scripts/update_depot.py')
depot = importlib.util.module_from_spec(spec)
spec.loader.exec_module(depot)


def entry(version):
    return {'name': 'guanid', 'version': version, 'target': 'v5', 'supported_kernels': '^4.2.1',
            'py/object': 'pros.conductor.templates.base_template.BaseTemplate',
            'metadata': {'location': f'https://github.com/RandomPerson5571/guanid/releases/download/guanid-v{version}/guanid%40{version}.zip'}}


class DepotTests(unittest.TestCase):
    def test_retains_old_versions_and_sorts_semantically(self):
        self.assertEqual(depot.merge([entry('0.2.0'), entry('0.1.0')], [entry('0.10.0')]),
                         [entry('0.10.0'), entry('0.2.0'), entry('0.1.0')])

    def test_identical_retry_is_idempotent(self):
        self.assertEqual(depot.merge([entry('0.2.0')], [entry('0.2.0')]), [entry('0.2.0')])

    def test_rejects_replacing_existing_version(self):
        changed = copy.deepcopy(entry('0.2.0'))
        changed['supported_kernels'] = '^5.0.0'
        with self.assertRaises(ValueError):
            depot.merge([entry('0.2.0')], [changed])

    def test_rejects_unversioned_or_foreign_download(self):
        for location in ('https://example.com/package.zip',
                         'https://github.com/RandomPerson5571/guanid/releases/latest/download/guanid.zip'):
            invalid = entry('0.2.0')
            invalid['metadata']['location'] = location
            with self.assertRaises(ValueError):
                depot.merge([], [invalid])


if __name__ == '__main__':
    unittest.main()
