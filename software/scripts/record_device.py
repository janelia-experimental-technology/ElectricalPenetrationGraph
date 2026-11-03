#!/usr/bin/env python3
"""Record from the REAL EPG device (RP2040 over USB) to HDF5 -- the hardware counterpart of
record_mock.py.

    python scripts/record_device.py peg.h5 --seconds 30 --rate 1000
    python scripts/record_device.py peg.h5 --port COM5 --channels 0x0F --cal
    python scripts/record_device.py --list-ports

If no ADC is fitted the firmware streams a synthetic signal and the file's device_serial ends in
"-SIM" so such files can never be mistaken for real recordings.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from epgrig import protocol as P
from epgrig.recorder import HDF5Recorder
from epgrig.serial_device import DeviceError, SerialDevice, find_ports
from epgrig.summary import print_summary

SERVO = {"off": P.SERVO_OFF, "acquire": P.SERVO_ACQUIRE, "track": P.SERVO_TRACK}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out", nargs="?", default="epg_recording.h5")
    ap.add_argument("--port", help="serial port (COM5, /dev/ttyACM0, ...); default: auto-detect the Pico")
    ap.add_argument("--list-ports", action="store_true")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--rate", type=int, default=1000, help="sample rate in Hz (250/500/1000/2000/4000)")
    ap.add_argument("--channels", type=lambda x: int(x, 0), default=0xFF, help="channel mask, e.g. 0x0F")
    ap.add_argument("--servo", choices=sorted(SERVO), default="acquire",
                    help="Vs servo at start: acquire (centre, then track), track, or off")
    ap.add_argument("--ri", choices=["1g", "10t"], default=None, help="set Ri on all channels before recording")
    ap.add_argument("--cal", action="store_true", help="fire a 500 ms -50 mV calibration pulse ~1 s into the recording")
    ap.add_argument("--operator", default="")
    ap.add_argument("--experiment", default="")
    args = ap.parse_args()

    if args.list_ports:
        ports = find_ports()
        print("\n".join(ports) if ports else "no Pico-like serial ports found")
        return 0

    chans = P.active_channels(args.channels)
    try:
        with SerialDevice(args.port) as dev:
            info = dev.info
            print(f"Connected on {dev.port}: serial {info.serial}  fw {info.fw_major}.{info.fw_minor}.{info.fw_patch}"
                  f"  rates {list(info.supported_rates)}")
            if info.serial.endswith("-SIM"):
                print("  NOTE: no ADC detected -> firmware is streaming SYNTHETIC data (link test).")
            dev.configure(args.rate, args.channels)
            for ch in chans:
                if args.ri:
                    dev.set_ri(ch, P.RI_1G if args.ri == "1g" else P.RI_10T)
                dev.servo(ch, SERVO[args.servo])

            rec = HDF5Recorder(args.out, info, args.rate, args.channels,
                               metadata={"operator": args.operator, "experiment_id": args.experiment})
            dev.consumer.recorder = rec
            print(f"Recording {args.seconds}s @ {args.rate} Hz, {len(chans)} ch -> {args.out}")
            target = int(args.seconds * args.rate)
            dev.start()
            t0 = time.time(); cal_done = not args.cal; last = 0.0
            try:
                while rec.n_samples < target:
                    time.sleep(0.05)
                    el = time.time() - t0
                    if dev.error is not None:
                        raise DeviceError(f"serial link lost: {dev.error}")
                    if not cal_done and el > 1.0:
                        for ch in chans:
                            dev.cal_pulse(ch, True, 500)
                        cal_done = True
                    if el - last >= 1.0:
                        last = el
                        print(f"  {rec.n_samples}/{target} samples  {dev.stats}", flush=True)
                    if el > args.seconds * 3 + 15:
                        raise DeviceError("timed out: device is not delivering samples at the requested rate")
            finally:
                try:
                    dev.stop()
                finally:
                    rec.close()
            st = dev.stats
            print(f"  streamed: {st['blocks']} blocks, {st['samples']} samples, {st['events']} events  "
                  f"(CRC errs={st['crc_errors']}, COBS errs={st['cobs_errors']})")
    except DeviceError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1

    print_summary(args.out)
    print("\nOK: real device -> protocol -> HDF5 -> reconstruct.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
