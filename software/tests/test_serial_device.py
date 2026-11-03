"""SerialDevice / SerialGuiBridge tests against a fake serial port wired to MockDevice
(no hardware, no pyserial port needed)."""
import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import pytest  # noqa: E402

from epgrig import protocol as P  # noqa: E402
from epgrig.mock_device import MockDevice  # noqa: E402
from epgrig.recorder import HDF5Recorder  # noqa: E402
from epgrig.serial_device import DeviceError, SerialDevice, SerialGuiBridge  # noqa: E402


class FakeSerial:
    """pyserial-like object: bytes written are parsed as host commands and answered by a
    MockDevice; while the mock is 'running' a pump thread streams sample blocks."""

    def __init__(self, mock=None, nack_all_start=False):
        self.mock = mock or MockDevice(seed=3)
        self.nack_all_start = nack_all_start
        self._parser = P.FrameParser()
        self._rx = bytearray()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._pump, daemon=True)
        self._t.start()
        self.written_stop = 0

    @property
    def in_waiting(self):
        with self._lock:
            return len(self._rx)

    def read(self, n=1):
        t0 = time.time()
        while time.time() - t0 < 0.02:
            with self._lock:
                if self._rx:
                    out = bytes(self._rx[:n]); del self._rx[:n]
                    return out
            time.sleep(0.001)
        return b""

    def write(self, data):
        for mtype, flags, payload in self._parser.feed(bytes(data)):
            if mtype == P.T_STOP:
                self.written_stop += 1
            if mtype == P.T_START and self.nack_all_start:
                frames = [P.encode_frame(P.T_NACK, bytes([mtype, 0, 6]))]
            else:
                frames = self.mock.handle_command(mtype, flags, payload)
            with self._lock:
                for f in frames:
                    self._rx += f
        return len(data)

    def reset_input_buffer(self):
        with self._lock:
            self._rx.clear()

    def close(self):
        self._stop.set()

    def _pump(self):
        while not self._stop.is_set():
            if self.mock.running:
                frames = self.mock.generate_block(max(1, self.mock.rate_hz // 20))
                with self._lock:
                    for f in frames:
                        self._rx += f
            time.sleep(0.005)


def make_dev(**kw):
    fake = FakeSerial(**kw)
    return SerialDevice(opener=lambda port: fake), fake


def test_open_gets_info_and_halts_stale_stream():
    dev, fake = make_dev()
    info = dev.open()
    try:
        assert info.n_channels == 8 and info.supported_rates == (250, 500, 1000, 2000, 4000)
        assert fake.written_stop >= 1          # a leftover stream from a crashed session is stopped
    finally:
        dev.close()


def test_configure_validates_rate():
    dev, _ = make_dev()
    dev.open()
    try:
        dev.configure(2000, 0x0F)
        assert dev.rate_hz == 2000 and dev.channel_mask == 0x0F
        with pytest.raises(DeviceError, match="not supported"):
            dev.configure(1234, 0xFF)
    finally:
        dev.close()


def test_nack_and_unknown_command_raise():
    dev, _ = make_dev(nack_all_start=True)
    dev.open()
    try:
        with pytest.raises(DeviceError, match="no ADC"):
            dev.start()
        with pytest.raises(DeviceError, match="unknown command"):
            dev.command(0x99)
    finally:
        dev.close()


def test_timeout_when_device_silent():
    class Silent(FakeSerial):
        def write(self, data):
            for mtype, _f, _p in self._parser.feed(bytes(data)):
                if mtype == P.T_GET_INFO:       # answer only GET_INFO so open() succeeds
                    with self._lock:
                        self._rx += P.encode_frame(P.T_INFO, self.mock.info.encode())
            return len(data)
    fake = Silent()
    dev = SerialDevice(opener=lambda p: fake, timeout=0.3)
    dev.open()
    try:
        with pytest.raises(DeviceError, match="timeout"):
            dev.ping()
    finally:
        dev.close()


def test_record_through_serial_to_hdf5(tmp_path):
    dev, _ = make_dev()
    info = dev.open()
    path = str(tmp_path / "x.h5")
    try:
        dev.configure(1000, 0x03)
        rec = HDF5Recorder(path, info, 1000, 0x03)
        dev.consumer.recorder = rec
        dev.set_vs_mv(0, 25.0)                  # commands work before and after start
        dev.start()
        dev.cal_pulse(1, True, 100)
        t0 = time.time()
        while rec.n_samples < 800 and time.time() - t0 < 10:
            time.sleep(0.02)
        dev.stop()
        rec.close()
    finally:
        dev.close()
    from epgrig import reader
    r = reader.load(path)
    assert r.n_samples >= 800 and r.codes.shape[1] == 2
    kinds = {e["type"] for e in r.events_as_dicts()}
    assert "CAL_PULSE" in kinds
    assert dev.stats["crc_errors"] == 0


def test_gui_bridge_streams_and_changes_rate():
    dev, fake = make_dev()
    dev.open()
    try:
        bridge = SerialGuiBridge(dev, 1000, 0xFF)
        assert bridge.info is dev.info and bridge.rate_hz == 1000
        time.sleep(0.3)
        chunks = bridge.generate_block(50)
        assert chunks and sum(len(c) for c in chunks) > 100
        # GUI-style command: fire-and-forget, returns no frames
        assert bridge.handle_command(P.T_SET_VS, 0, bytes([0, 0x00, 0x80])) == []
        bridge.rate_hz = 2000                   # stop -> configure -> start
        assert fake.mock.rate_hz == 2000 and fake.mock.running
        time.sleep(0.2)
        assert bridge.generate_block(50)
    finally:
        dev.close()
