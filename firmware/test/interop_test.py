"""Cross-language wire-compatibility test: C firmware codec <-> Python host codec.

C emits known frames -> Python parses & asserts; Python emits the same -> C parses &
its printed fields are checked. Proves the firmware and host agree byte-for-byte.

Run: make interop   (or: python3 interop_test.py)
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SOFTWARE = os.path.join(HERE, "..", "..", "software")
sys.path.insert(0, SOFTWARE)

from epgrig import protocol as P  # noqa: E402

C_BIN = os.path.join(HERE, "test_proto")
C_FILE = "/tmp/c_frames.bin"
PY_FILE = "/tmp/py_frames.bin"

# fixtures shared with test_proto.c
RATES = (250, 500, 1000, 2000, 4000)
SERIAL = "EPG-FW-0001"
SAMP = [[100, -200, 300, -400], [1, 2, 3, 4], [8388607, -8388608, 0, -1]]


def _parse_all(blob):
    parser = P.FrameParser()
    return list(parser.feed(blob))


def test_c_to_python():
    subprocess.run([C_BIN, "emit", C_FILE], check=True)
    frames = _parse_all(open(C_FILE, "rb").read())
    assert len(frames) == 4, len(frames)
    by_type = {t: (f, p) for (t, f, p) in frames[:3]}
    assert set(by_type) == {P.T_INFO, P.T_SAMPLES, P.T_EVENT}, by_type.keys()

    info = P.Info.decode(by_type[P.T_INFO][1])
    assert info.serial == SERIAL and info.supported_rates == RATES
    assert info.adc_vref_mv == 1200 and info.fixed_gain_x100 == 800 and info.adc_bits == 24

    blk = P.SampleBlock.decode(by_type[P.T_SAMPLES][1])
    assert blk.block_seq == 7 and blk.first_sample_index == 1000
    assert blk.channel_mask == 0x0F and blk.samples == SAMP

    ev = P.Event.decode(by_type[P.T_EVENT][1])
    assert (ev.event_type, ev.channel, ev.sample_index, ev.a, ev.b, ev.text) == \
           (P.EV_VS_CHANGE, 2, 54321, 32768, 40000, "track")
    print("  PASS C->Python: INFO, SAMPLES, EVENT decoded with matching fields")

    # 4th frame: large 200x8 block (4.8 KB) as the firmware sends at 4 kHz
    t, _f, p = frames[3]
    big = P.SampleBlock.decode(p)
    assert t == P.T_SAMPLES and big.block_seq == 8 and big.first_sample_index == 5000
    assert len(big.samples) == 200 and big.channel_mask == 0xFF
    want = [[((r * 8 + c) * 7919 % 16000000) - 8000000 for c in range(8)] for r in range(200)]
    assert big.samples == want
    print("  PASS C->Python: large 200x8 SAMPLES block (4.8 KB) intact")


def test_python_to_c():
    info = P.Info(serial=SERIAL, supported_rates=RATES, fixed_gain_x100=800,
                  adc_vref_mv=1200, vs_range_mv=1000, vs_dac_bits=16, adc_bits=24,
                  fw_major=0, fw_minor=1, fw_patch=0, n_channels=8)
    blk = P.SampleBlock(7, 1000, 2, 0x0F, 0, SAMP)
    ev = P.Event(P.EV_VS_CHANGE, 2, 54321, a=32768, b=40000, text="track")
    blob = (P.encode_frame(P.T_INFO, info.encode())
            + P.encode_frame(P.T_SAMPLES, blk.encode())
            + P.encode_frame(P.T_EVENT, ev.encode()))
    open(PY_FILE, "wb").write(blob)
    out = subprocess.run([C_BIN, "parse", PY_FILE], check=True,
                         capture_output=True, text=True).stdout
    assert "INFO proto=1 nch=8 nrates=5 vref=1200 gain=800 serial=EPG-FW-0001" in out, out
    assert "SAMPLES seq=7 first=1000 mask=0x0f n=3 s0c0=100 last=-1" in out, out
    assert "EVENT type=1 ch=2 idx=54321 a=32768 b=40000 text=track" in out, out
    print("  PASS Python->C: C parsed INFO, SAMPLES, EVENT with matching fields")


def main():
    if not os.path.exists(C_BIN):
        subprocess.run(["make", "test_proto"], cwd=HERE, check=True)
    test_c_to_python()
    test_python_to_c()
    print("Interop OK: firmware and host codecs are byte-for-byte compatible.")


if __name__ == "__main__":
    main()
