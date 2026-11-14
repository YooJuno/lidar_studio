#!/usr/bin/env python3
"""Exercise promotion boundaries without changing a GitHub repository."""
import hashlib
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from publish_release import publish

SHA = 'a' * 40


class FakeGitHub:
    def __init__(self, folder, *, tag_sha=None, published=False, missing_upload=False, corrupt_upload=False):
        self.folder, self.tag_sha, self.published = folder, tag_sha, published
        self.missing_upload, self.corrupt_upload = missing_upload, corrupt_upload
        self.calls = []; self.created = False; self.uploaded = False

    def run(self, *args, allow_missing=False):
        self.calls.append(args)
        if args[0] == 'api':
            path = next(a for a in args if a.startswith('repos/'))
            if '/git/ref/tags/' in path:
                return {'object': {'sha': self.tag_sha, 'type': 'commit'}} if self.tag_sha else None
            if path.endswith('/git/refs'):
                self.tag_sha = SHA; return {}
            if '/releases/tags/' in path:
                if not self.created and not self.published: return None
                assets = [{'name': p.name, 'size': p.stat().st_size,
                           'digest': 'sha256:' + hashlib.sha256(p.read_bytes()).hexdigest()}
                          for p in self.folder.iterdir() if p.is_file()] if self.uploaded else []
                if self.missing_upload: assets = assets[:-1]
                if self.corrupt_upload and assets: assets[0]['digest'] = 'sha256:' + '0' * 64
                return {'draft': not self.published, 'html_url': 'https://example.test/release', 'assets': assets}
        if args[:2] == ('release', 'create'): self.created = True
        if args[:2] == ('release', 'upload'): self.uploaded = True
        if args[:2] == ('release', 'edit'): self.published = True
        return ''


class ReleasePublishTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(); self.addCleanup(self.temporary.cleanup)
        self.folder = Path(self.temporary.name)
        for name in ['Linux-x86_64.deb', 'Linux-x86_64.tar.gz', 'Windows-AMD64.zip', 'Darwin-arm64.zip', 'Darwin-arm64.dmg']:
            (self.folder / ('lidar_studio-0.2.53-' + name)).write_bytes(('package ' + name).encode())

    def publish(self, gh):
        return publish(self.folder, 'owner/repo', SHA, '0.2.53', 'v0.2.53', 'https://example.test/run', gh)

    def test_assets_and_checksums_are_verified_before_publishing(self):
        gh = FakeGitHub(self.folder); self.publish(gh)
        create = next(i for i, c in enumerate(gh.calls) if c[:2] == ('release', 'create'))
        upload = next(i for i, c in enumerate(gh.calls) if c[:2] == ('release', 'upload'))
        edit = next(i for i, c in enumerate(gh.calls) if c[:2] == ('release', 'edit'))
        self.assertLess(create, upload); self.assertLess(upload, edit)
        self.assertIn('--draft', gh.calls[create]); self.assertIn('--latest', gh.calls[edit])
        self.assertEqual(len((self.folder / 'SHA256SUMS').read_text().splitlines()), 5)
        self.assertTrue(gh.published)

    def test_missing_platform_never_creates_a_tag_or_release(self):
        next(self.folder.glob('*.dmg')).unlink(); gh = FakeGitHub(self.folder)
        with self.assertRaises(ValueError): self.publish(gh)
        self.assertEqual(gh.calls, [])

    def test_missing_or_corrupt_upload_keeps_the_release_a_draft(self):
        for options in [dict(missing_upload=True), dict(corrupt_upload=True)]:
            with self.subTest(options=options):
                gh = FakeGitHub(self.folder, **options)
                with self.assertRaises(RuntimeError): self.publish(gh)
                self.assertTrue(gh.created); self.assertFalse(gh.published)

    def test_existing_tag_cannot_be_moved(self):
        gh = FakeGitHub(self.folder, tag_sha='b' * 40)
        with self.assertRaises(ValueError): self.publish(gh)
        self.assertFalse(gh.created)

    def test_published_rerun_does_not_replace_assets_or_mark_old_release_latest(self):
        gh = FakeGitHub(self.folder, tag_sha=SHA, published=True); self.publish(gh)
        self.assertFalse(any(c[0] == 'release' for c in gh.calls))

    def test_failed_draft_can_be_resumed(self):
        gh = FakeGitHub(self.folder, tag_sha=SHA); gh.created = True; self.publish(gh)
        self.assertFalse(any(c[:2] == ('release', 'create') for c in gh.calls))
        self.assertTrue(gh.published)


if __name__ == '__main__':
    unittest.main()
