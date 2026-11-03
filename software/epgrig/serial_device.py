"""Real-device transport: talk to the RP2040 firmware over USB-CDC with pyserial.

Drop-in counterpart of MockDevice for recording:

    with SerialDevice() as dev:                    # auto-finds the Pico
        dev.configure(rate_hz=1000, channel_mask=0xFF)
        rec = HDF5Recorder(path, dev.info, 1000, 0xFF)
        dev.consumer.recorder = rec
        dev.start(); time.sleep(30); dev.stop()
        rec.close()

A background thread reads the port and feeds a StreamConsumer (so recording/callbacks run
exactly as with the mock). Commands wait for the device's ACK and raise DeviceError on a NACK
or timeout. `port` may be any pyserial URL, e.g. "COM5", "/dev/ttyACM0", "socket://host:5555".
"""
from __future__ import annotations

import collections
import threading
import time
from typing import Callable, Deque, List, Optional

from . import protocol as P
from .acquisition import StreamConsumer

RPI_VID = 0x2E8A  # Raspberry Pi (RP2040) USB vendor id

NACK_NAMES = {1: "unknown command", 2: "bad payload", 3: "busy (stop first)",
              4: "bad channel", 5: "bad value", 6: "no ADC / cannot stream",
              7: "ADC configuration failed"}
CMD_NAMES = {P.T_GET_INFO: "GET_INFO", P.T_CONFIGURE: "CONFIGURE", P.T_START: "START",
             P.T_STOP: "STOP", P.T_SET_VS: "SET_VS", P.T_SET_RI: "SET_RI",
             P.T_CAL_PULSE: "CAL_PULSE", P.T_SERVO: "SERVO", P.T_PING: "PING"}


class DeviceError(RuntimeError):
    pass


def find_ports(vid: int = RPI_VID) -> List[str]:
    """Serial ports that look like the Pico (USB vendor 0x2E8A)."""
    from serial.tools import list_ports
    return [p.device for p in list_ports.comports() if p.vid == vid]


class SerialDevice:
    def __init__(self, port: Optional[str] = None, *, timeout: float = 2.0,
                 consumer: Optional[StreamConsumer] = None, opener: Optional[Callable] = None):
        self.port = port
        self._opener = opener          # opener(port) -> pyserial-like object (tests inject a fake)
        self.timeout = timeout
        self.ser = None
        self.info: Optional[P.Info] = None
        self.rate_hz: Optional[int] = None
        self.channel_mask: Optional[int] = None
        self.streaming = False
        self.raw_sink: Optional[Callable[[bytes], None]] = None  # bypass parsing (GUI bridge)
        self.error: Optional[BaseException] = None

        self.consumer = consumer or StreamConsumer()
        self.consumer.on_info = self._on_info
        self.consumer.on_ack = self._on_ack
        self.consumer.on_nack = self._on_nack

        self._cmd_lock = threading.Lock()
        self._evt = threading.Event()
        self._expect: Optional[int] = None
        self._result: Optional[tuple] = None
        self._info_evt = threading.Event()
        self._stop_evt = threading.Event()
        self._thread: Optional[threading.Thread] = None

    # ---- lifecycle ----------------------------------------------------------
    def open(self) -> P.Info:
        import serial
        port = self.port
        if port is None and self._opener is not None:
            port = self.port = "fake"
        if port is None:
            ports = find_ports()
            if not ports:
                raise DeviceError("no EPG device found (no USB serial port with VID 0x2E8A). "
                                  "Is it plugged in? Pass port='COM5' to choose one.")
            port = ports[0]
            self.port = port
        try:
            self.ser = (self._opener(port) if self._opener
                        else serial.serial_for_url(port, baudrate=115200, timeout=0.02))
        except (OSError, serial.SerialException) as e:
            raise DeviceError(f"cannot open {port}: {e}") from e

        # A previous session may have died while streaming: halt it, drop stale bytes.
        self.ser.write(P.encode_frame(P.T_STOP))
        time.sleep(0.15)
        self.ser.reset_input_buffer()
        self.consumer.parser = P.FrameParser()

        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._reader, name="epg-serial-reader", daemon=True)
        self._thread.start()
        self.info = self.get_info()
        return self.info

    def close(self) -> None:
        try:
            if self.streaming and self.ser is not None:
                self.stop()
        except DeviceError:
            pass
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self.ser is not None:
            try:
                self.ser.close()
            finally:
                self.ser = None

    def __enter__(self) -> "SerialDevice":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ---- reader thread ------------------------------------------------------
    def _reader(self) -> None:
        import serial
        while not self._stop_evt.is_set():
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
            except (OSError, serial.SerialException) as e:
                self.error = e
                break
            if not data:
                continue
            sink = self.raw_sink
            if sink is not None:
                sink(data)
            else:
                self.consumer.feed(data)

    def _on_info(self, info: P.Info) -> None:
        self.info = info
        self._info_evt.set()

    def _on_ack(self, cmd: int) -> None:
        if cmd == self._expect:
            self._result = ("ack", 0)
            self._evt.set()

    def _on_nack(self, cmd: int, err: int) -> None:
        if cmd == self._expect:
            self._result = ("nack", err)
            self._evt.set()

    # ---- commands -----------------------------------------------------------
    def _send_raw(self, mtype: int, payload: bytes = b"") -> None:
        if self.ser is None:
            raise DeviceError("device not open")
        if self.error is not None:
            raise DeviceError(f"serial link failed: {self.error}")
        self.ser.write(P.encode_frame(mtype, payload))

    def command(self, mtype: int, payload: bytes = b"", *, wait: bool = True) -> None:
        """Send a command; by default wait for ACK (raises DeviceError on NACK/timeout)."""
        if not wait:
            self._send_raw(mtype, payload)
            return
        with self._cmd_lock:
            self._evt.clear()
            self._result = None
            self._expect = mtype
            try:
                self._send_raw(mtype, payload)
                if not self._evt.wait(self.timeout):
                    raise DeviceError(f"timeout waiting for ACK of {CMD_NAMES.get(mtype, hex(mtype))}")
            finally:
                self._expect = None
            kind, err = self._result
            if kind == "nack":
                raise DeviceError(f"{CMD_NAMES.get(mtype, hex(mtype))} rejected: "
                                  f"{NACK_NAMES.get(err, f'error {err}')}")

    def get_info(self, retries: int = 3) -> P.Info:
        for _ in range(retries):
            self._info_evt.clear()
            self._send_raw(P.T_GET_INFO)
            if self._info_evt.wait(self.timeout):
                return self.info
        raise DeviceError("device did not answer GET_INFO (wrong port, or not EPG firmware?)")

    def ping(self) -> None:
        self.command(P.T_PING)

    def configure(self, rate_hz: int, channel_mask: int = 0xFF) -> None:
        if self.info is None:
            raise DeviceError("open the device first")
        rates = list(self.info.supported_rates)
        if rate_hz not in rates:
            raise DeviceError(f"rate {rate_hz} Hz not supported; device offers {rates}")
        self.command(P.T_CONFIGURE, bytes([rates.index(rate_hz), channel_mask & 0xFF]))
        self.rate_hz, self.channel_mask = rate_hz, channel_mask & 0xFF

    def start(self) -> None:
        self.command(P.T_START)
        self.streaming = True

    def stop(self) -> None:
        self.command(P.T_STOP)
        self.streaming = False

    def set_vs_dac(self, ch: int, dac_code: int, **kw) -> None:
        self.command(P.T_SET_VS, bytes([ch]) + int(dac_code).to_bytes(2, "little"), **kw)

    def vs_mv_to_dac(self, mv: float) -> int:
        half = self.info.vs_range_mv / 2.0
        top = (1 << self.info.vs_dac_bits) - 1
        return int(min(top, max(0, round((mv + half) / (2 * half) * top))))

    def set_vs_mv(self, ch: int, mv: float, **kw) -> None:
        self.set_vs_dac(ch, self.vs_mv_to_dac(mv), **kw)

    def set_ri(self, ch: int, mode: int, **kw) -> None:
        self.command(P.T_SET_RI, bytes([ch, mode]), **kw)

    def cal_pulse(self, ch: int, on: bool = True, dur_ms: int = 500, **kw) -> None:
        self.command(P.T_CAL_PULSE, bytes([ch, 1 if on else 0]) + int(dur_ms).to_bytes(2, "little"), **kw)

    def servo(self, ch: int, mode: int, target_code: int = 0, deadband_code: int = 0,
              freeze: bool = False, **kw) -> None:
        payload = bytes([ch, mode]) + int(target_code).to_bytes(2, "little") \
            + int(deadband_code).to_bytes(2, "little") + bytes([1 if freeze else 0])
        self.command(P.T_SERVO, payload, **kw)

    # ---- diagnostics --------------------------------------------------------
    @property
    def stats(self) -> dict:
        c = self.consumer
        return {"blocks": c.n_blocks, "samples": c.n_samples, "events": c.n_events,
                "crc_errors": c.parser.crc_errors, "cobs_errors": c.parser.cobs_errors}


class SerialGuiBridge:
    """Presents a SerialDevice through the small interface the live GUI uses for MockDevice
    (info, rate_hz, handle_command, generate_block), so `run_gui.py --port COM5` just works.
    Incoming bytes are queued by the reader thread and handed to the GUI's own consumer."""

    def __init__(self, dev: SerialDevice, rate_hz: int, channel_mask: int):
        self.dev = dev
        self.info = dev.info
        self._chunks: Deque[bytes] = collections.deque()
        self._rate = rate_hz
        dev.configure(rate_hz, channel_mask)
        dev.start()
        dev.raw_sink = self._chunks.append

    @property
    def rate_hz(self) -> int:
        return self._rate

    @rate_hz.setter
    def rate_hz(self, hz: int) -> None:           # GUI changes rate live -> stop/configure/start
        if hz == self._rate:
            return
        d = self.dev
        d.raw_sink = None                          # so ACKs are parsed during the reconfigure
        try:
            d.stop()
            d.configure(hz, d.channel_mask)
            d.start()
            self._rate = hz
        finally:
            d.raw_sink = self._chunks.append

    def handle_command(self, mtype: int, flags: int, payload: bytes) -> list:
        self.dev.command(mtype, payload, wait=False)   # ACK + logged EVENT arrive on the stream
        return []

    def generate_block(self, n_samples: int) -> list:
        out = []
        while self._chunks:
            out.append(self._chunks.popleft())
        return out
