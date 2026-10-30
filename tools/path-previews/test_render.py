"""Diff and failure regressions; real solver/video smoke checks are also run locally."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import render


class DiffTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.cwd = Path.cwd()
        os.chdir(self.directory.name)
        self.run_git('init')
        self.root = Path(render.ROOT)
        self.root.mkdir(parents=True)
        (self.root / 'project.mayhem').write_text('{}')
        (self.root / 'Old.mtraj').write_text('{"name":"path"}')
        self.commit()
        self.base = self.run_git('rev-parse', 'HEAD').strip()

    def tearDown(self):
        os.chdir(self.cwd)
        self.directory.cleanup()

    def run_git(self, *args):
        return subprocess.check_output(['git', *args], stderr=subprocess.DEVNULL).decode()

    def commit(self):
        self.run_git('add', '.')
        self.run_git('-c', 'user.name=Preview Test', '-c', 'user.email=test@example.invalid',
                     'commit', '-m', 'fixture')

    def preview(self, failure=False):
        with patch.object(render, 'prepare', side_effect=ValueError('unsolvable') if failure else None,
                          return_value={'times': [0, 2]}), patch.object(render, 'render'):
            args = argparse.Namespace(base=self.base, head='HEAD', output='output')
            if failure:
                with self.assertRaises(SystemExit):
                    render.preview(args)
            else:
                render.preview(args)
        return json.loads(Path('output/manifest.json').read_text())['paths']

    def test_rename_reports_deleted_and_added_names(self):
        (self.root / 'Old.mtraj').rename(self.root / 'New.mtraj')
        self.commit()
        paths = self.preview()
        self.assertEqual({p['name']: p['status'] for p in paths},
                         {'Old': 'deleted', 'New': 'added'})

    def test_project_change_includes_unchanged_paths(self):
        (self.root / 'project.mayhem').write_text('{"robot":"changed"}')
        self.commit()
        self.assertEqual(self.preview()[0]['name'], 'Old')

    def test_failed_new_solve_does_not_emit_media(self):
        (self.root / 'Old.mtraj').write_text('{"name":"changed"}')
        self.commit()
        entry = self.preview(failure=True)[0]
        self.assertEqual(entry['status'], 'failed')
        self.assertNotIn('media', entry)

    def test_unchanged_diff_has_no_previews(self):
        self.assertEqual(self.preview(), [])


if __name__ == '__main__':
    unittest.main()
