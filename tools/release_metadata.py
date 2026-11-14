#!/usr/bin/env python3
"""Resolve one native-compatible package version for every CI platform."""
import argparse
import os
from pathlib import Path
import re


def resolve_version(cmake_text: str, ref: str, run_number: str) -> str:
    project = re.search(r'\bproject\s*\(\s*lidar_studio\s+VERSION\s+(\d+)\.(\d+)\.(\d+)\b', cmake_text, re.I)
    if not project:
        raise ValueError('CMake project must declare a numeric major.minor.patch version')
    if ref.startswith('refs/tags/'):
        tag = re.fullmatch(r'refs/tags/v(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)', ref)
        if not tag:
            raise ValueError('Release tags must use vMAJOR.MINOR.PATCH')
        return '.'.join(tag.groups())
    if not re.fullmatch(r'[1-9]\d*', run_number):
        raise ValueError('A positive workflow run number is required')
    major, minor, patch = map(int, project.groups())
    # A numeric version works in Debian, Windows and macOS bundle metadata.
    return f'{major}.{minor}.{patch + int(run_number)}'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cmake', type=Path, default=Path('CMakeLists.txt'))
    args = parser.parse_args()
    version = resolve_version(args.cmake.read_text(), os.environ['GITHUB_REF'], os.environ['GITHUB_RUN_NUMBER'])
    print(f'version={version}')
    print(f'tag=v{version}')


if __name__ == '__main__':
    main()
