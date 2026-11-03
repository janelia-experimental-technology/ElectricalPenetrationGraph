#!/usr/bin/env python3
"""Launch the live GUI (mock device by default; --port for the real device).
Requires pyqtgraph + a Qt binding."""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from epgrig.gui import run_mock_gui


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--rate", type=int, default=1000)
    ap.add_argument("--channels", type=lambda x: int(x, 0), default=0xFF)
    ap.add_argument("--detrend-fc", type=float, default=0.5, help="display high-pass cutoff (Hz)")
    ap.add_argument("--port", nargs="?", const="auto", default=None,
                    help="use the REAL device: --port COM5, or just --port to auto-detect the Pico")
    args = ap.parse_args()
    if args.port is None:
        run_mock_gui(rate_hz=args.rate, channel_mask=args.channels, detrend_fc=args.detrend_fc)
        return 0
    from epgrig.serial_device import DeviceError, SerialDevice, SerialGuiBridge
    try:
        dev = SerialDevice(None if args.port == "auto" else args.port)
        dev.open()
        bridge = SerialGuiBridge(dev, args.rate, args.channels)
    except DeviceError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1
    try:
        run_mock_gui(rate_hz=args.rate, channel_mask=args.channels, detrend_fc=args.detrend_fc,
                     dev=bridge, title=f"device {dev.info.serial} on {dev.port}")
    finally:
        dev.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
