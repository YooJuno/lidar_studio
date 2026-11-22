"""Check the TerraSeg transport without requiring model downloads or a GPU."""

from io import BytesIO
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
import terraseg_worker as worker


class FragmentedInput(BytesIO):
    def readinto(self, target):
        return super().readinto(target[:3])


class FragmentedOutput(BytesIO):
    def write(self, data):
        return super().write(data[:5])


def packet(revision, count, tag=b"TGQ1"):
    return worker.REQUEST.pack(tag, revision, count) + bytes(count * worker.POINT_BYTES)


def responses(output):
    output.seek(0)
    while header := output.read(worker.RESPONSE.size):
        kind, revision, count, message_length = worker.RESPONSE.unpack(header)
        message = output.read(message_length).decode("utf-8")
        yield kind, revision, output.read(count), message


class TerraSegWorkerTests(unittest.TestCase):
    def test_index_aligned_mask_then_empty_reply(self):
        source = FragmentedInput(packet(5, 3) + packet(9, 0))
        output = FragmentedOutput()

        def ground_mask(payload, count):
            self.assertEqual(len(payload), 36)
            self.assertEqual(count, 3)
            return bytes([1, 0, 1])

        self.assertEqual(worker.serve(source, output, ground_mask, "CUDA · test"), 0)
        self.assertEqual(list(responses(output)), [
            (b"TGR1", 0, b"", "CUDA · test"),
            (b"TGR1", 5, b"\1\0\1", ""),
            (b"TGR1", 9, b"", ""),
        ])

    def test_invalid_header_is_rejected_before_receiving_points(self):
        source = FragmentedInput(worker.REQUEST.pack(b"TGQ1", 8, worker.MAX_POINTS + 1))
        output = FragmentedOutput()
        self.assertEqual(worker.serve(source, output, None, "CUDA"), 1)
        self.assertEqual(list(responses(output))[-1], (b"TGE1", 8, b"", "Invalid TerraSeg request"))

    def test_truncated_payload_is_rejected(self):
        source = FragmentedInput(packet(8, 9)[:-1])
        with self.assertRaisesRegex(EOFError, "Truncated"):
            worker.serve(source, FragmentedOutput(), None, "CUDA")

    def test_bad_prediction_size_does_not_shift_next_reply(self):
        source = FragmentedInput(packet(13, 2) + packet(14, 1))
        output = FragmentedOutput()
        self.assertEqual(worker.serve(source, output, lambda payload, count: b"\1", "CUDA"), 0)
        ready, invalid, valid = list(responses(output))
        self.assertEqual(ready[1], 0)
        self.assertEqual(invalid[:3], (b"TGE1", 13, b""))
        self.assertIn("one label per point", invalid[3])
        self.assertEqual(valid, (b"TGR1", 14, b"\1", ""))

    def test_untrusted_checkpoint_is_rejected_before_loading(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "test.pth"
            path.write_bytes(b"arbitrary input")
            with self.assertRaisesRegex(ValueError, "invalid size"):
                worker.TerraSegWorker(path)
            with patch.object(worker, "CHECKPOINT_BYTES", path.stat().st_size):
                with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
                    worker.TerraSegWorker(path)


if __name__ == "__main__":
    unittest.main()
