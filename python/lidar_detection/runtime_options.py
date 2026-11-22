# SPDX-License-Identifier: Apache-2.0
"""Bound the CPU inference thread pool to leave capacity for the viewer."""

from pathlib import Path
import importlib.util
import os
import platform


def preload_cuda_libraries(ort):
    """Locate optional cuDNN wheels without importing a training framework."""
    if not hasattr(ort, "preload_dlls"):
        return
    try:
        spec = importlib.util.find_spec("nvidia.cudnn")
    except ImportError:
        spec = None
    if spec and spec.submodule_search_locations:
        windows = platform.system() == "Windows"
        library = "cudnn64_9.dll" if windows else "libcudnn.so.9"
        for root in spec.submodule_search_locations:
            directory = Path(root) / ("bin" if windows else "lib")
            if (directory / library).is_file():
                ort.preload_dlls(cuda=False, cudnn=True, directory=str(directory))
                break
    ort.preload_dlls()


def _available_cpus():
    if hasattr(os, "sched_getaffinity"):
        try:
            return len(os.sched_getaffinity(0))
        except OSError:
            pass
    return getattr(os, "process_cpu_count", os.cpu_count)() or 2


def make_session_options(ort, *, cpu_only):
    options = ort.SessionOptions()
    options.inter_op_num_threads = 1
    options.intra_op_num_threads = 2
    if cpu_only:
        available = _available_cpus()
        options.intra_op_num_threads = min(6, available, max(2, available // 2))
        # Sleep between inference jobs instead of holding extra CPU cores.
        options.add_session_config_entry("session.intra_op.allow_spinning", "0")
    return options
