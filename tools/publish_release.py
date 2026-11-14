#!/usr/bin/env python3
"""Publish only complete, verified platform bundles, using a draft until ready."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile


class GitHub:
    def run(self, *args, allow_missing=False):
        result = subprocess.run(['gh', *args], capture_output=True, text=True)
        if result.returncode:
            if allow_missing and 'HTTP 404' in result.stderr:
                return None
            raise RuntimeError(result.stderr.strip() or result.stdout.strip())
        return json.loads(result.stdout) if args[0] == 'api' and result.stdout.strip() else result.stdout.strip()


def digest(path):
    checksum = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            checksum.update(chunk)
    return checksum.hexdigest()


def package_files(directory, version):
    files = sorted(path for path in directory.iterdir() if path.is_file() and path.name != 'SHA256SUMS')
    expected = [('Linux', '.deb'), ('Linux', '.tar.gz'), ('Windows', '.zip'), ('Darwin', '.zip'), ('Darwin', '.dmg')]
    if len(files) != len(expected):
        raise ValueError('Expected exactly five platform packages: Linux DEB/TGZ, Windows ZIP, macOS ZIP/DMG')
    for system, suffix in expected:
        matches = [p for p in files if p.name.startswith(f'lidar_studio-{version}-{system}-') and p.name.endswith(suffix)]
        if len(matches) != 1 or not matches[0].stat().st_size:
            raise ValueError(f'Missing, empty or ambiguous {system} {suffix} package for {version}')
    return files


def publish(directory, repo, sha, version, tag, run_url, gh=None):
    if not re.fullmatch(r'[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+', repo):
        raise ValueError('Invalid repository')
    if not re.fullmatch(r'[0-9a-f]{40}', sha) or not re.fullmatch(r'(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)', version) or tag != 'v' + version:
        raise ValueError('Release source, version and tag must match the CI metadata')
    files = package_files(directory, version)
    checksum_path = directory / 'SHA256SUMS'
    checksum_path.write_text(''.join(f'{digest(path)}  {path.name}\n' for path in files), encoding='ascii')
    files.append(checksum_path)
    gh = gh or GitHub()
    api_root = f'repos/{repo}'
    reference = gh.run('api', f'{api_root}/git/ref/tags/{tag}', allow_missing=True)
    if reference is None:
        gh.run('api', '--method', 'POST', f'{api_root}/git/refs', '-f', f'ref=refs/tags/{tag}', '-f', f'sha={sha}')
    else:
        commit = reference['object']['sha']
        if reference['object']['type'] == 'tag':
            commit = gh.run('api', f'{api_root}/commits/{tag}')['sha']
        if commit != sha:
            raise ValueError('Refusing to move an existing release tag to a different commit')
    release = gh.run('api', f'{api_root}/releases/tags/{tag}', allow_missing=True)
    if release and not release['draft']:
        print(f'{tag} is already published from this commit; published assets remain unchanged.')
        return release['html_url']
    notes = (
        f'Automatically built from [{sha[:7]}](https://github.com/{repo}/commit/{sha}) after all platform checks passed.\n\n'
        f'[Build and validation]({run_url})\n\n'
        '- **Ubuntu 24.04:** ROS 2 Jazzy bag/live LiDAR and IMU support, MCAP recording, ground segmentation and ROI tools.\n'
        '- **Windows / macOS:** standalone CPU point-cloud inspection, ground segmentation, detection adapters and ROI/dataset tools; ROS 2 is not bundled.\n'
        '- Learned detection and training require the optional model/runtime setup described in the included README.\n'
        '- macOS bundles use ad-hoc signing and are not notarized.\n\n'
        'Download the package for your platform and verify it with `SHA256SUMS`.\n'
    )
    if release is None:
        with tempfile.TemporaryDirectory(prefix='lidar-release-') as temporary:
            note_path = Path(temporary) / 'notes.md'
            note_path.write_text(notes, encoding='utf-8')
            gh.run('release', 'create', tag, '--repo', repo, '--verify-tag', '--target', sha,
                   '--draft', '--title', f'LiDAR Studio {tag}', '--notes-file', str(note_path))
    gh.run('release', 'upload', tag, '--repo', repo, *map(str, files), '--clobber')
    uploaded = gh.run('api', f'{api_root}/releases/tags/{tag}')
    assets = {asset['name']: asset for asset in uploaded['assets']}
    for path in files:
        asset = assets.get(path.name)
        if not asset or asset['size'] != path.stat().st_size:
            raise RuntimeError(f'Release remains a draft: incomplete upload for {path.name}')
        if asset.get('digest') and asset['digest'] != 'sha256:' + digest(path):
            raise RuntimeError(f'Release remains a draft: checksum mismatch for {path.name}')
    gh.run('release', 'edit', tag, '--repo', repo, '--draft=false', '--latest')
    return uploaded['html_url']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets', type=Path, required=True)
    args = parser.parse_args()
    repo = os.environ['GITHUB_REPOSITORY']
    url = publish(args.assets.resolve(), repo, os.environ['GITHUB_SHA'], os.environ['RELEASE_VERSION'],
                  os.environ['RELEASE_TAG'], f'https://github.com/{repo}/actions/runs/{os.environ["GITHUB_RUN_ID"]}')
    print('Release:', url)


if __name__ == '__main__':
    main()
