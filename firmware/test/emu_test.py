#!/usr/bin/env python3
"""Firmware acceptance test: run the REAL firmware.uf2 in the rp2040js emulator (no ADC fitted,
so it streams its built-in plant model) and drive it with the real host code.

Checks the protocol path, command handling + NACKs, the Vs servo (sign, scale, hand-over),
the calibration pulse, and that the logged events let the host reconstruct the true drift.

    # terminal 1 (see README): node emulator exposing the Pico USB serial on tcp 5555
    #   built-in simulator path (no chips):   run_fw.ts --image firmware.uf2 --port 5555
    #   real-ADC path (chip models attached): run_fw.ts --image firmware.uf2 --port 5555 --adc
    # terminal 2:
    python3 emu_test.py                   # simulator path
    EPG_EMU_ADC=1 python3 emu_test.py     # real-ADC path (also checks the chip models' bus statistics)
"""
import os
import sys
import json
import socket
import tempfile
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "software"))

from epgrig import protocol as P                      # noqa: E402
from epgrig import reader                             # noqa: E402
from epgrig.recorder import HDF5Recorder              # noqa: E402
from epgrig.serial_device import DeviceError, SerialDevice  # noqa: E402

PORT = os.environ.get("EPG_EMU_PORT", "socket://127.0.0.1:5555")
ADC = os.environ.get("EPG_EMU_ADC") == "1"           # firmware talks to the modelled ADS131M08/DAC8568
STATS_PORT = int(os.environ.get("EPG_EMU_STATS_PORT", "5557"))
RATE, SECONDS, MASK = 1000, 22, 0x07
# the firmware's built-in plant model: drift (V/s) per channel = +/-(0.008 + 0.002*ch)
DRIFT = {0: +0.008, 1: -0.010, 2: +0.012}
fails = 0


def check(cond, msg):
    global fails
    print(("  PASS " if cond else "  FAIL ") + msg)
    if not cond:
        fails += 1


def expect_nack(dev, mtype, payload, why):
    try:
        dev.command(mtype, payload)
        check(False, f"NACK expected: {why}")
    except DeviceError as e:
        check("rejected" in str(e), f"NACK for {why}  ({e})")


def main():
    with SerialDevice(PORT, timeout=5.0) as dev:
        info = dev.info
        print(f"device: {info.serial} fw {info.fw_major}.{info.fw_minor}.{info.fw_patch}")
        if ADC:
            check(not info.serial.endswith("-SIM"), "ADC found -> serial NOT marked -SIM")
        else:
            check(info.serial.endswith("-SIM"), "no ADC -> serial marked -SIM")
        check(tuple(info.supported_rates) == (250, 500, 1000, 2000, 4000), "INFO rates")
        check((info.adc_vref_mv, info.fixed_gain_x100, info.vs_range_mv) == (1200, 800, 1000), "INFO analog constants")
        dev.ping()
        check(True, "PING -> ACK")

        # ---- error paths ----
        expect_nack(dev, P.T_SET_VS, bytes([9, 0, 0]), "SET_VS bad channel")
        expect_nack(dev, P.T_SET_RI, bytes([0, 7]), "SET_RI bad mode")
        expect_nack(dev, P.T_CONFIGURE, bytes([7, 0xFF]), "CONFIGURE bad rate code")
        expect_nack(dev, P.T_CONFIGURE, bytes([2, 0x00]), "CONFIGURE empty channel mask")
        expect_nack(dev, 0x99, b"", "unknown command")

        dev.configure(RATE, MASK)
        dev.servo(0, P.SERVO_OFF)        # control: raw drift visible, no Vs events
        dev.servo(1, P.SERVO_TRACK)      # keep-in-range
        dev.servo(2, P.SERVO_ACQUIRE)    # centre, then hand over to track
        dev.set_ri(1, P.RI_10T)

        path = os.path.join(tempfile.gettempdir(), "epg_emu_test.h5")
        rec = HDF5Recorder(path, info, RATE, MASK)
        dev.consumer.recorder = rec
        dev.start()
        expect_nack(dev, P.T_CONFIGURE, bytes([2, 0xFF]), "CONFIGURE while streaming (busy)")

        target = SECONDS * RATE
        steps = [(1.0, "vs"), (4.0, "cal"), (6.0, "ri")]
        done = set()
        t0 = time.time()
        while rec.n_samples < target:
            time.sleep(0.05)
            now_s = rec.n_samples / RATE            # drive actions on the DEVICE timeline
            for at, what in steps:
                if now_s >= at and what not in done:
                    done.add(what)
                    if what == "vs":
                        dev.set_vs_mv(0, +20.0)
                    elif what == "cal":
                        dev.cal_pulse(0, True, 500)
                    elif what == "ri":
                        dev.set_ri(2, P.RI_10T)
            if time.time() - t0 > 400:
                raise SystemExit("FAIL: emulator too slow / no data")
        dev.stop()
        rec.close()
        st = dev.stats
        check(st["crc_errors"] == 0 and st["cobs_errors"] == 0, f"no CRC/COBS errors ({st})")

    r = reader.load(path)
    ev = r.events_as_dicts()
    t = r.t
    ir = r.input_referred
    unc = r.uncompensated()
    print(f"recorded {r.n_samples} samples, {len(ev)} events")

    check(abs(r.n_samples - target) <= 25, f"sample count {r.n_samples} ~ {target}")
    check(r.n_samples == len(set(range(r.n_samples))) and
          rec.start_sample_index == 0, "index timeline starts at 0")
    want = "adc=ok" if ADC else "SIMULATED"
    check(any(e["type"] == "COMMENT" and want in e["text"] for e in ev), f"start COMMENT says {want}")

    def vs_events(ch, reason=None):
        return [e for e in ev if e["type"] == "VS_CHANGE" and e["channel"] == ch
                and e["text"] != "start" and (reason is None or e["text"] == reason)]

    # manual Vs step on ch0 (+20 mV) shows up in data and in the log
    man = vs_events(0, "manual")
    check(len(man) == 1, "ch0: one manual VS_CHANGE logged")
    check(len(vs_events(0)) == 1, "ch0 (servo OFF): servo never moved Vs")
    i1 = man[0]["sample_index"]
    # (the plant's test sine on ch0 is 3.0 Hz = 333 samples: average over whole periods)
    W = 333
    pre, post = ir[i1 - W - 7:i1 - 7, 0], ir[i1 + 60:i1 + 60 + W, 0]
    jump = post.mean() - pre.mean()
    expect = 0.020 + DRIFT[0] * ((60 + W / 2) + (7 + W / 2)) / RATE   # + drift between window centres
    check(abs(jump - expect) < 0.004, f"ch0 manual +20 mV step seen in data: {jump*1e3:+.1f} mV (expect {expect*1e3:+.1f})")

    # calibration pulse on ch0: -50 mV for 500 ms
    cal = [e for e in ev if e["type"] == "CAL_PULSE" and e["channel"] == 0]
    check(len(cal) == 2 and cal[0]["a"] == 1 and cal[1]["a"] == 0 and cal[0]["b"] == -50000,
          "ch0: CAL_PULSE on/off events (-50000 uV)")
    a, b = cal[0]["sample_index"], cal[1]["sample_index"]
    check(abs((b - a) - 500) <= 30, f"cal pulse lasted {b-a} samples (~500)")
    pre, during = ir[a - W - 7:a - 7, 0], ir[a + 100:a + 100 + W, 0]
    dip = during.mean() - pre.mean()
    expect = -0.050 + DRIFT[0] * ((100 + W / 2) + (7 + W / 2)) / RATE
    check(abs(dip - expect) < 0.004, f"ch0 cal dip {dip*1e3:+.1f} mV (expect {expect*1e3:+.1f} = -50 + drift)")

    # reconstruction: uncompensated slope == true plant drift, for all three servo modes
    ok_cal = np.ones(r.n_samples, bool)
    ok_cal[a - 20:b + 150] = False
    for ch in (0, 1, 2):
        good = ok_cal & (np.abs(ir[:, ch]) < 0.140)       # ch0 has no servo and rails: skip clipped samples
        good[:300] = False
        slope = np.polyfit(t[good], unc[good, ch], 1)[0]
        check(abs(slope - DRIFT[ch]) < 0.12 * abs(DRIFT[ch]),
              f"ch{ch} reconstructed drift {slope*1e3:+.2f} mV/s (true {DRIFT[ch]*1e3:+.1f})")

    # servo behaviour
    clipfrac = lambda ch: float((np.abs(ir[:, ch]) >= 0.1485).mean())
    check(clipfrac(0) > 0.10, f"control ch0 (servo OFF) rails as expected: {clipfrac(0)*100:.0f}% clipped")
    check(clipfrac(1) < 0.02, f"ch1 (TRACK) kept in range: {clipfrac(1)*100:.1f}% clipped (drift here is ~100x real)")
    trk = vs_events(1, "track")
    check(len(trk) >= 1, f"ch1 keep-in-range stepped Vs {len(trk)}x, first at {trk[0]['sample_index']/RATE:.1f}s" if trk
          else "ch1 keep-in-range stepped Vs")
    if trk:
        e0 = trk[0]
        check(e0["a"] != e0["b"] and (e0["b"] > e0["a"]),
              f"ch1 drifts negative -> Vs stepped UP ({e0['a']}->{e0['b']})")
    acq = vs_events(2, "acquire")
    check(len(acq) >= 3, f"ch2 ACQUIRE stepped Vs {len(acq)}x")
    ss = [e for e in ev if e["type"] == "SERVO_STATE" and e["channel"] == 2 and e["sample_index"] > 0]
    check(len(ss) == 1 and ss[0]["a"] == P.SERVO_TRACK, "ch2: ACQUIRE handed over to TRACK (SERVO_STATE event)")
    h = ss[0]["sample_index"]
    pre = ir[h - 1000:h, 2]                      # the second before hand-over: centred (4 Hz test tone -> whole periods)
    check(abs(pre.mean()) < 0.015, f"ch2 centred before hand-over: mean {pre.mean()*1e3:+.1f} mV")
    check(clipfrac(2) < 0.02, f"ch2 stays in range afterwards (keep-in-range): {clipfrac(2)*100:.1f}% clipped")
    ri = [e for e in ev if e["type"] == "RI_CHANGE" and e["sample_index"] > 0]
    check(any(e["channel"] == 2 and e["a"] == P.RI_10T for e in ri), "mid-stream SET_RI logged as RI_CHANGE")
    errs = [e for e in ev if e["type"] == "ERROR"]
    check(not errs, f"no device ERROR events {errs[:1]}")

    # ---- every rate: the 3.0 Hz test tone on ch0 must land at 3.0 Hz (catches wrong ODR / decimation) ----
    for rate in (250, 500, 1000, 2000, 4000):
        rows, firsts = [], []
        with SerialDevice(PORT, timeout=5.0) as dev:
            dev.consumer.on_block = lambda b: (rows.extend(x[0] for x in b.samples), firsts.append((b.first_sample_index, len(b.samples))))
            dev.configure(rate, 0x01)
            dev.servo(0, P.SERVO_OFF)
            dev.start()
            t0 = time.time()
            while len(rows) < int(2.5 * rate) and time.time() - t0 < 200:
                time.sleep(0.05)
            dev.stop()
        x = np.asarray(rows[:int(2.0 * rate)], float)
        x = x - np.polyval(np.polyfit(np.arange(len(x)), x, 1), np.arange(len(x)))     # remove drift
        spec = np.abs(np.fft.rfft(x * np.hanning(len(x))))
        f = np.fft.rfftfreq(len(x), 1.0 / rate)
        peak = f[1 + np.argmax(spec[1:])]
        contiguous = all(firsts[i + 1][0] == firsts[i][0] + firsts[i][1] for i in range(len(firsts) - 1))
        check(abs(peak - 3.0) < 0.35, f"{rate:4d} Hz: test tone at {peak:.2f} Hz (expect 3.0), {len(firsts)} blocks of {firsts[0][1]} rows")
        check(contiguous, f"{rate:4d} Hz: block first_sample_index contiguous (no lost samples)")

    if ADC:
        with socket.create_connection(("127.0.0.1", STATS_PORT), timeout=5) as c:
            st = json.loads(c.makefile().read())
        print("chip-model bus stats:", st)
        check(st["badLen"] == 0, "every ADC SPI frame was exactly 30 bytes (10 x 24-bit words)")
        check(st["overruns"] == 0, f"no ADC conversion overwritten before the DRDY interrupt read it (4 kHz session) ({st['overruns']})")
        check(st["dacRef"], "DAC internal reference enabled")
        check(st["dacWrites"] >= 6, f"DAC written by servo/commands ({st['dacWrites']} writes)")
        for k in ("RREG", "WREG", "NULL"):
            check(st["cmds"].get(k, 0) > 0, f"ADC saw {k} frames ({st['cmds'].get(k, 0)})")

    print("\nALL PASSED" if not fails else f"\n{fails} CHECK(S) FAILED")
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())
