#!/usr/bin/env python3
"""Sign bundled native runtimes inside-out, then seal and verify the app."""
from pathlib import Path
import subprocess
import sys

MACHO = {bytes.fromhex(value) for value in ['cffaedfe', 'cefaedfe', 'feedfacf', 'feedface', 'cafebabe', 'bebafeca', 'cafebabf', 'bfbafeca']}


def sign(app):
    app = app.resolve()
    runtime = app / 'Contents/Resources/detector/runtime'
    if runtime.is_dir():
        for path in sorted(runtime.rglob('*')):
            if path.is_symlink() or not path.is_file():
                continue
            with path.open('rb') as source:
                native = source.read(4) in MACHO
            if native:
                subprocess.run(['codesign', '--force', '--sign', '-', str(path)], check=True)
    subprocess.run(['codesign', '--force', '--deep', '--sign', '-', str(app)], check=True)
    subprocess.run(['codesign', '--verify', '--deep', '--strict', '--verbose=2', str(app)], check=True)


if __name__ == '__main__':
    sign(Path(sys.argv[1]))
