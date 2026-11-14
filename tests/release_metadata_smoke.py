#!/usr/bin/env python3
"""Release versions stay consistent across platforms, tags and reruns."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from release_metadata import resolve_version


class ReleaseVersionTests(unittest.TestCase):
    cmake = 'project(lidar_studio VERSION 0.2.0 LANGUAGES C CXX)'

    def test_main_has_a_unique_numeric_version_and_reruns_reuse_it(self):
        self.assertEqual(resolve_version(self.cmake, 'refs/heads/main', '53'), '0.2.53')
        self.assertEqual(resolve_version(self.cmake, 'refs/heads/main', '53'), '0.2.53')
        self.assertEqual(resolve_version(self.cmake, 'refs/heads/main', '54'), '0.2.54')

    def test_manual_tag_is_used_exactly(self):
        self.assertEqual(resolve_version(self.cmake, 'refs/tags/v1.4.7', '53'), '1.4.7')

    def test_series_bumps_and_nonzero_base_patch(self):
        self.assertEqual(resolve_version('project(lidar_studio VERSION 1.3.2)', 'refs/heads/main', '53'), '1.3.55')

    def test_pull_request_artifacts_are_versioned_without_using_branch_text(self):
        self.assertEqual(resolve_version(self.cmake, 'refs/pull/12/merge', '53'), '0.2.53')

    def test_invalid_or_unsafe_tags_are_rejected(self):
        for ref in ['refs/tags/v1', 'refs/tags/v1.2.3-rc.1', 'refs/tags/v01.2.3', 'refs/tags/v1.2.3\nother=value']:
            with self.subTest(ref=ref), self.assertRaises(ValueError):
                resolve_version(self.cmake, ref, '53')

    def test_invalid_build_number_and_project_are_rejected(self):
        for number in ['0', '-1', '', '1\ntag=v1']:
            with self.subTest(number=number), self.assertRaises(ValueError):
                resolve_version(self.cmake, 'refs/heads/main', number)
        with self.assertRaises(ValueError):
            resolve_version('project(other VERSION 0.2.0)', 'refs/heads/main', '53')


if __name__ == '__main__':
    unittest.main()
