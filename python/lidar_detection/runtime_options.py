# SPDX-License-Identifier: Apache-2.0
"""Bound the CPU inference thread pool to leave capacity for the viewer."""

import os


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
