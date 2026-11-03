#!/usr/bin/env python3
"""End-to-end pipeline demo with no hardware:

    mock device -> protocol frames -> parser -> HDF5 recorder -> reader/reconstruction

Records a synthetic multi-channel EPG session to an HDF5 file, then reads it back and
prints a summary, verifying the round-trip and the Vs reconstruction.

Usage:  python scripts/record_mock.py [out.h5] [--seconds N] [--rate HZ] [--channels MASK]
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from epgrig import protocol as P
from epgrig.acquisition import StreamConsumer
from epgrig.mock_device import MockDevice
from epgrig.recorder import HDF5Recorder
from epgrig.summary import print_summary


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out", nargs="?", default="/tmp/epg_mock.h5")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--rate", type=int, default=1000)
    ap.add_argument("--channels", type=lambda x: int(x, 0), default=0xFF)
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()

    dev = MockDevice(seed=args.seed, channel_mask=args.channels, rate_hz=args.rate)
    rec = HDF5Recorder(args.out, dev.info, args.rate, args.channels,
                       metadata={"operator": "mock", "experiment_id": "demo"})
    consumer = StreamConsumer(recorder=rec)

    print(f"Recording {args.seconds}s @ {args.rate} Hz, channels mask={args.channels:#04x} "
          f"({len(P.active_channels(args.channels))} ch) -> {args.out}")
    consumer.feed_all(dev.iter_stream(args.seconds))
    rec.close()

    print(f"  streamed: {consumer.n_blocks} blocks, {consumer.n_samples} samples, "
          f"{consumer.n_events} events  (CRC errs={consumer.parser.crc_errors})")

    print_summary(args.out)
    print("\nOK: full mock->protocol->HDF5->reconstruct pipeline verified.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
