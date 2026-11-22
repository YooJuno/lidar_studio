# SPDX-License-Identifier: Apache-2.0
"""Persistent TerraSeg-S inference worker. Standard output carries TGQ1/TGR1 IPC."""

import argparse
import hashlib
import os
from pathlib import Path
import struct
import sys


MODEL_REVISION = "343bac53852b6c7f84f661ff3afade8ac0d32b43"
CHECKPOINT_SHA256 = "221745c957d8dec0f52db4b0905141ca3c420639fbe6dead76a16a20c429de22"
CHECKPOINT_NAME = "terraseg_s.pth"
CHECKPOINT_BYTES = 46_390_738
REQUEST = struct.Struct("<4sQI")
RESPONSE = struct.Struct("<4sQII")
POINT_BYTES = 3 * 4
MAX_POINTS = 4_000_000
MAX_MESSAGE_BYTES = 16_384


def verify_checkpoint(path):
    """Only the pinned original checkpoint is passed to upstream torch.load."""
    path = Path(path)
    if not path.is_file() or path.stat().st_size != CHECKPOINT_BYTES:
        raise ValueError("TerraSeg Small checkpoint is missing or has an invalid size")
    with path.open("rb") as stream:
        digest = hashlib.sha256()
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    if digest.hexdigest() != CHECKPOINT_SHA256:
        raise ValueError("TerraSeg Small checkpoint SHA-256 mismatch")


def read_into(source, target):
    offset = 0
    while offset < len(target):
        read = source.readinto(target[offset:])
        if not read:
            if offset == 0:
                return False
            raise EOFError("Truncated TerraSeg request")
        offset += read
    return True


def write_all(sink, data):
    remaining = memoryview(data)
    while remaining:
        written = sink.write(remaining)
        if not written:
            raise BrokenPipeError("TerraSeg reply pipe closed")
        remaining = remaining[written:]


def reply(sink, kind, revision, *, mask=b"", message=""):
    encoded = message.encode("utf-8", "replace")[:MAX_MESSAGE_BYTES]
    encoded = encoded.decode("utf-8", "ignore").encode("utf-8")
    write_all(sink, RESPONSE.pack(kind, revision, len(mask), len(encoded)))
    write_all(sink, encoded)
    write_all(sink, mask)
    sink.flush()


def serve(source, sink, ground_mask, provider):
    """Return one byte per input point, with 1 for ground."""
    reply(sink, b"TGR1", 0, message=provider)
    header = bytearray(REQUEST.size)
    receive = bytearray()
    while read_into(source, memoryview(header)):
        magic, revision, count = REQUEST.unpack(header)
        if magic != b"TGQ1" or revision == 0 or count > MAX_POINTS:
            reply(sink, b"TGE1", revision, message="Invalid TerraSeg request")
            return 1
        size = count * POINT_BYTES
        if len(receive) < size:
            receive = bytearray(size)
        payload = memoryview(receive)[:size]
        if not read_into(source, payload):
            raise EOFError("Missing TerraSeg point payload")
        try:
            mask = ground_mask(payload, count) if count else b""
            if len(mask) != count:
                raise RuntimeError("TerraSeg did not return one label per point")
        except Exception as error:
            reply(sink, b"TGE1", revision, message=str(error))
        else:
            reply(sink, b"TGR1", revision, mask=mask)
    return 0


class TerraSegWorker:
    def __init__(self, checkpoint):
        verify_checkpoint(checkpoint)
        import numpy as np
        import torch

        if not torch.cuda.is_available():
            raise RuntimeError("TerraSeg needs a CUDA-capable NVIDIA GPU")
        if torch.cuda.get_device_capability()[0] not in (8, 9):
            raise RuntimeError("The pinned TerraSeg runtime requires an Ampere, Ada or Hopper GPU")

        from terraseg import TerraSegPredictor

        self.np = np
        self.torch = torch
        self.model = TerraSegPredictor(variant="S", checkpoint_path=checkpoint, device="cuda:0")
        self.provider = f"CUDA · {torch.cuda.get_device_name()}"
        # Verify the sparse convolution and attention kernels before reporting ready.
        axis = torch.linspace(-8, 8, 32, device="cuda:0")
        x, y = torch.meshgrid(axis, axis, indexing="ij")
        sample = torch.stack((x.flatten(), y.flatten(), .01 * x.flatten()), dim=1)
        self.model.predict(sample)
        torch.cuda.synchronize()

    def ground_mask(self, payload, count):
        points = self.np.frombuffer(payload, dtype="<f4", count=count * 3).reshape(count, 3)
        valid = self.np.isfinite(points).all(axis=1)
        if not valid.any():
            return bytes(count)
        # Move input to CUDA before the upstream predictor converts coordinates.
        coordinates = self.torch.from_numpy(points if valid.all() else points[valid]).to("cuda:0")
        labels = self.model.predict(coordinates).cpu().numpy()
        ground = (labels == 0).astype(self.np.uint8)
        if len(ground) == count:
            return ground.tobytes()
        mask = self.np.zeros(count, dtype=self.np.uint8)
        mask[valid] = ground
        return mask.tobytes()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--check", action="store_true", help="Load the model and verify CUDA kernels")
    args = parser.parse_args()
    if args.check:
        worker = TerraSegWorker(args.checkpoint)
        print(f"TerraSeg Small ready on {worker.provider}", flush=True)
        return 0

    source = sys.stdin.buffer
    # Preserve a private binary stream before redirecting Python and native
    # diagnostics that might otherwise corrupt detector replies.
    sink = os.fdopen(os.dup(sys.stdout.fileno()), "wb", buffering=0)
    os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
    with sink:
        try:
            worker = TerraSegWorker(args.checkpoint)
        except Exception as error:
            reply(sink, b"TGE1", 0, message=str(error))
            return 1
        try:
            return serve(source, sink, worker.ground_mask, worker.provider)
        except (BrokenPipeError, EOFError):
            return 1


if __name__ == "__main__":
    raise SystemExit(main())
