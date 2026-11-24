"""Check the TransFusion v2.1 input and output contract independently of ONNX."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
from lidar_detection.transfusion import decode, pillarize


class TransFusionTests(unittest.TestCase):
    def test_single_sweep_coordinates_reflectivity_and_zero_padding(self):
        scan = np.array([[0, 0, 0, 0], [.01, .02, .03, 83], [.30, 0, 0, 255],
                         [.01, .02, .03, np.nan], [93, 0, 0, 100]], np.float32)
        packed, stats = pillarize(scan)
        np.testing.assert_array_equal(packed['coors'], [[0, 0, 384, 384], [0, 0, 384, 385]])
        np.testing.assert_array_equal(packed['num_points'], [2, 1])
        np.testing.assert_array_equal(np.sort(packed['voxels'][0, :2, 3]), [0, 83])
        self.assertEqual(packed['voxels'][1, 0, 3], 255)
        self.assertEqual(np.count_nonzero(packed['voxels'][..., 4]), 0)
        self.assertEqual(np.count_nonzero(packed['voxels'][0, 2:]), 0)
        self.assertEqual(stats['missing_reflectivity'], 1)
        repeated, _ = pillarize(scan)
        for key, value in packed.items():
            np.testing.assert_array_equal(repeated[key], value)

    def test_reports_absent_and_normalized_reflectivity(self):
        with self.assertRaisesRegex(ValueError, 'none in range'):
            pillarize(np.array([[0, 0, 0, np.nan]], np.float32))
        with self.assertRaisesRegex(ValueError, 'normalized intensity'):
            pillarize(np.array([[0, 0, 0, .5]], np.float32))
        empty, _ = pillarize(np.empty((0, 4), np.float32))
        self.assertIsNone(empty)

    def test_native_packing_matches_numpy_for_capped_and_invalid_points(self):
        rng = np.random.default_rng(134)
        # Overfill a pillar and exceed the total voxel cap with distinct cells.
        grid_x, grid_y = np.meshgrid(np.arange(304), np.arange(200))
        sparse = np.column_stack(((grid_x.ravel() + .5) * .24 - 92.16,
                                  (grid_y.ravel() + .5) * .24 - 92.16,
                                  np.zeros(grid_x.size), rng.uniform(-2, 259, grid_x.size))).astype(np.float32)
        dense = np.column_stack((rng.uniform(-.01, .01, (500, 3)), rng.uniform(-5, 266, 500))).astype(np.float32)
        edges = np.array([[-92.16, 0, 0, 200], [92.16, 0, 0, 200], [0, 0, -3, 200],
                          [0, 0, 7, 200], [0, np.inf, 0, 200], [0, 0, 0, np.nan],
                          [0, 0, 0, 300], [0, 0, 0, -1]], np.float32)
        for points in (np.concatenate((dense, edges)), np.concatenate((sparse, dense, edges))):
            with patch('lidar_detection.native.library', return_value=None):
                baseline, expected = pillarize(points)
            actual, stats = pillarize(points)
            for key, value in baseline.items():
                np.testing.assert_array_equal(actual[key], value)
            for key, value in expected.items():
                if key != 'preprocess_backend':
                    self.assertEqual(stats[key], value)
            self.assertEqual(stats['missing_reflectivity'], 1)
            self.assertGreater(stats['clipped_reflectivity'], 0)
            self.assertEqual(int(actual['num_points'].max()), 20)
        self.assertGreater(stats['truncated_pillars'], 0)

    def test_decoder_world_coordinates_class_and_yaw(self):
        scores = np.zeros((1, 5, 500), np.float32)
        geom = np.zeros((1, 8, 500), np.float32)
        direction = np.zeros((1, 2, 500), np.float32)
        scores[0, 4, 4] = .87  # Person
        scores[0, 0, 5] = .8   # Car below the car yaw-norm gate
        geom[0, :6, 4] = [100, 90, .8, np.log(1.), np.log(.5), np.log(1.8)]
        direction[0, 0, 4] = 1.
        expected = [[3.84, -5.76, .8, 1, .5, 1.8, np.pi / 2, .87, 4]]
        np.testing.assert_allclose(decode((scores, geom, direction), .25), expected, atol=1e-5)


if __name__ == '__main__':
    unittest.main()
