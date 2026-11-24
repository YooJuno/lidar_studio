# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 TIER IV, Inc. (original preprocessing and decoding conventions)
"""The unchanged Autoware TransFusion v2.1 graph and its LiDAR adapter."""
from pathlib import Path
import time

import numpy as np

from .centerpoint import suppress
from .models import get_model, sha256_file
from .native import backend_name, fill_transfusion, filter_pillar_points, nms_backend_name, stable_order
from .runtime_options import make_session_options, preload_cuda_libraries

# Geometry and decoding follow autoware_lidar_transfusion at 8361f0ef2dd4.
SPEC = get_model('transfusion')
MINIMUM = np.asarray(SPEC.minimum, np.float32)
MAXIMUM = np.asarray(SPEC.maximum, np.float32)
POINTS_PER_PILLAR = 20
PROPOSALS = 500
SCORE_FLOOR = np.float32(.10)
YAW_NORMS = np.array([.3, .3, .3, .3, 0], np.float32)
OUTPUTS = ('cls_score0', 'bbox_pred0', 'dir_cls_pred0')


def pillarize(points):
    """Pack one aligned XYZI sweep into Autoware's zero-padded pillars."""
    if points.ndim != 2 or points.shape[1] != 4:
        raise ValueError('TransFusion requires XYZI points with original reflectivity')
    step = np.float32(SPEC.voxel[0])
    filtered = filter_pillar_points(points, MINIMUM, MAXIMUM, SPEC.grid, step)
    if filtered is None:
        x, y, z, intensity = points.T
        in_range = ((x > MINIMUM[0]) & (x < MAXIMUM[0]) & (y > MINIMUM[1]) &
                    (y < MAXIMUM[1]) & (z > MINIMUM[2]) & (z < MAXIMUM[2]))
        have_intensity = np.isfinite(intensity)
        missing = int(np.count_nonzero(in_range & ~have_intensity))
        points = points[in_range & have_intensity]
        intensity = points[:, 3]
        normalized = (len(intensity) and 0 <= intensity.min() and intensity.max() <= 1
                      and np.any(intensity != np.trunc(intensity)))
        x = np.clip(np.floor((points[:, 0] - MINIMUM[0]) / step).astype(np.int32), 0, SPEC.grid - 1)
        y = np.clip(np.floor((points[:, 1] - MINIMUM[1]) / step).astype(np.int32), 0, SPEC.grid - 1)
        keys = y * SPEC.grid + x
    else:
        points, keys, missing, normalized = filtered
    stats = {
        'pillars': 0,
        'truncated_pillars': 0,
        'preprocess_backend': backend_name(len(points)),
        'missing_reflectivity': missing,
        'clipped_reflectivity': 0,
    }
    if not len(points):
        if stats['missing_reflectivity']:
            raise ValueError('TransFusion needs original uint8 reflectivity; this scan has none in range.')
        return None, stats

    if normalized:
        raise ValueError('TransFusion expects uint8 reflectivity (0–255); this scan only has normalized intensity.')

    # The upstream shuffles points before filling capped pillars. A fixed seed
    # gives uniform sampling with repeatable results on paused or replayed scans.
    rng = np.random.default_rng(0)
    shuffled = rng.permutation(len(keys))
    order = stable_order(keys, shuffled, SPEC.grid)
    ordered_keys = keys[order]
    changes = np.flatnonzero(ordered_keys[1:] != ordered_keys[:-1]) + 1
    edges = np.concatenate(([0], changes, [len(order)]))
    starts = edges[:-1]
    counts = np.diff(edges)
    total_pillars = len(starts)
    if total_pillars > SPEC.max_voxels:
        sample = np.sort(rng.choice(total_pillars, SPEC.max_voxels, replace=False))
        starts, counts = starts[sample], counts[sample]

    counts = np.minimum(counts, POINTS_PER_PILLAR).astype(np.int32)
    voxels = np.zeros((len(starts), POINTS_PER_PILLAR, SPEC.features), np.float32)
    clipped_reflectivity = fill_transfusion(points, order, starts, counts, voxels)
    if clipped_reflectivity is None:
        slots = np.arange(POINTS_PER_PILLAR)
        occupied = slots < counts[:, None]
        selected = points[order[(starts[:, None] + slots)[occupied]]]
        reflectivity = selected[:, 3]
        clipped_reflectivity = int(np.count_nonzero((reflectivity < 0) | (reflectivity > 255)))
        reflectivity[:] = np.clip(reflectivity, 0, 255).astype(np.uint8)
        voxels[occupied, :4] = selected
    # Feature 4 is time lag. Zero is the newest scan and the padding value.
    coords = np.zeros((len(starts), 4), np.int32)
    coords[:, 2] = ordered_keys[starts] // SPEC.grid
    coords[:, 3] = ordered_keys[starts] % SPEC.grid
    stats.update(
        pillars=len(starts), truncated_pillars=total_pillars - len(starts),
        clipped_reflectivity=clipped_reflectivity,
    )
    return {'voxels': voxels, 'num_points': counts, 'coors': coords}, stats


def decode(outputs, score_threshold):
    scores, geometry, direction = outputs
    if (scores.shape, geometry.shape, direction.shape) != (
        (1, len(YAW_NORMS), PROPOSALS), (1, 8, PROPOSALS), (1, 2, PROPOSALS),
    ):
        raise ValueError('TransFusion output schema does not match the pinned model')
    # The ONNX head already applies the classification activation.
    scores = scores[0].T
    labels = np.argmax(scores, axis=1)
    scores = scores[np.arange(PROPOSALS), labels]
    sine, cosine = direction[0]
    keep = (scores > max(SCORE_FLOOR, np.float32(score_threshold))) & (scores <= 1)
    keep &= np.isfinite(sine) & np.isfinite(cosine)
    keep &= np.hypot(sine, cosine) >= YAW_NORMS[labels]
    geometry = geometry[0, :6, keep]
    with np.errstate(over='ignore', invalid='ignore'):
        sizes = np.exp(geometry[:, 3:])
    boxes = np.column_stack((
        geometry[:, :2] * SPEC.downsample * np.float32(SPEC.voxel[0]) + MINIMUM[:2],
        geometry[:, 2], sizes, np.arctan2(sine[keep], cosine[keep]), scores[keep], labels[keep],
    )).astype(np.float32)
    valid = np.isfinite(boxes).all(axis=1) & (boxes[:, 3:6] > 0).all(axis=1)
    valid &= (boxes[:, 3:5] <= 100).all(axis=1) & (boxes[:, 5] <= 30)
    # TransFusion uses the same circle and rotated BEV IoU stages as CenterPoint.
    return suppress(boxes[valid])


class TransFusion:
    def __init__(self, folder, provider='auto'):
        import onnxruntime as ort

        self.spec = SPEC
        name, _, digest = self.spec.files[0]
        path = Path(folder) / name
        if not path.is_file() or sha256_file(path) != digest:
            raise RuntimeError(f'Missing or invalid model file: {name}. Install TransFusion from Objects.')

        providers = ['CPUExecutionProvider']
        if provider != 'cpu' and 'CUDAExecutionProvider' in ort.get_available_providers():
            preload_cuda_libraries(ort)
            providers.insert(0, ('CUDAExecutionProvider', {
                'cudnn_conv_algo_search': 'HEURISTIC',
                'arena_extend_strategy': 'kSameAsRequested',
                'use_tf32': 0,
            }))
        options = make_session_options(ort, cpu_only=providers == ['CPUExecutionProvider'])
        self.session = ort.InferenceSession(str(path), sess_options=options, providers=providers)
        self.provider = self.session.get_providers()[0]
        warm_points = np.zeros((1, 4), np.float32)
        warm_inputs, _ = pillarize(warm_points)
        decode(self.session.run(OUTPUTS, warm_inputs), SCORE_FLOOR)

    def infer(self, points, score_threshold=SPEC.default_score):
        started = time.perf_counter()
        inputs, stats = pillarize(points)
        prepared = time.perf_counter()
        if inputs is None:
            stats.update(preprocess_ms=(prepared - started) * 1000, latency_ms=(prepared - started) * 1000,
                         provider=self.provider, postprocess_backend=nms_backend_name())
            return np.empty((0, 9), np.float32), stats
        outputs = self.session.run(OUTPUTS, inputs)
        inferred = time.perf_counter()
        boxes = decode(outputs, score_threshold)
        finished = time.perf_counter()
        stats.update(preprocess_ms=(prepared - started) * 1000, inference_ms=(inferred - prepared) * 1000,
                     postprocess_ms=(finished - inferred) * 1000, latency_ms=(finished - started) * 1000,
                     provider=self.provider, postprocess_backend=nms_backend_name())
        return boxes, stats
