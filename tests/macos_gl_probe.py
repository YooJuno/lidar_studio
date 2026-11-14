#!/usr/bin/env python3
"""Diagnose native GLFW context creation without depending on the app."""
import argparse
import ctypes
import json
from pathlib import Path
import subprocess
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--packages', type=Path, required=True)
args = parser.parse_args()
with tempfile.TemporaryDirectory(prefix='lidar-gl-probe-') as temporary:
    root = Path(temporary)
    archive = next(args.packages.glob('*.zip'))
    subprocess.run(['ditto', '-x', '-k', str(archive.resolve()), str(root)], check=True)
    library = next(root.rglob('libglfw.3.dylib'))
    glfw = ctypes.CDLL(str(library))
    callback_type = ctypes.CFUNCTYPE(None, ctypes.c_int, ctypes.c_char_p)
    errors = []
    callback = callback_type(lambda code, message: errors.append(dict(code=code, message=message.decode())))
    glfw.glfwSetErrorCallback.argtypes = [callback_type]
    glfw.glfwSetErrorCallback(callback)
    glfw.glfwInit.restype = ctypes.c_int
    glfw.glfwWindowHint.argtypes = [ctypes.c_int, ctypes.c_int]
    glfw.glfwCreateWindow.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_char_p, ctypes.c_void_p, ctypes.c_void_p]
    glfw.glfwCreateWindow.restype = ctypes.c_void_p
    glfw.glfwDestroyWindow.argtypes = [ctypes.c_void_p]
    results = []
    if glfw.glfwInit():
        for major, minor in [(3, 3), (4, 1)]:
            errors.clear()
            glfw.glfwDefaultWindowHints()
            for hint, value in [(0x22002, major), (0x22003, minor), (0x22006, 1), (0x22008, 0x32001), (0x20004, 0)]:
                glfw.glfwWindowHint(hint, value)
            window = glfw.glfwCreateWindow(640, 480, b'LiDAR context probe', None, None)
            results.append(dict(version=f'{major}.{minor}', created=bool(window), errors=list(errors)))
            if window:
                glfw.glfwDestroyWindow(window)
        glfw.glfwTerminate()
    else:
        results.append(dict(initialized=False, errors=list(errors)))
    print(json.dumps(results, indent=2), flush=True)
