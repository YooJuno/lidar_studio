"""Check the TransFusion v2.1 input and output contract independently of ONNX."""
from pathlib import Path
import sys
import unittest

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
