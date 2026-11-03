# epgrig — host software (Python)

Cross-platform acquisition + analysis for the redesigned EPG rig. Implements the
device↔host protocol and HDF5 recording format in [../docs/protocol-and-data-format.md](../docs/protocol-and-data-format.md),
and ships an **in-process mock device** so the entire pipeline runs and is tested
**before any hardware exists**.

## Layout
```
epgrig/
  protocol.py     COBS+CRC framing + message codecs   (pure stdlib — always importable)
  mock_device.py  synthetic device: aphid-like waveforms, electrode drift, Vs servo
  serial_device.py REAL device over USB-CDC (pyserial): commands + ACK/NACK, reader thread, GUI bridge
  summary.py      end-of-recording read-back summary (shared by the record scripts)
  acquisition.py  parse device frames -> recorder + live callbacks (transport-agnostic)
  recorder.py     incremental crash-safe HDF5 writer
  reader.py       HDF5 reader + signal reconstruction (codes->volts, applied-Vs)
  dsp.py          display-side detrend (high-pass) — view only, never recorded
  gui.py          live pyqtgraph display (optional Qt)
scripts/
  record_mock.py  headless end-to-end demo: mock -> frames -> HDF5 -> reconstruct
  record_device.py the same, from the REAL RP2040 device (auto-detects the Pico)
  run_gui.py      launch the live GUI against the mock, or the real device with --port
tests/            protocol + pipeline tests (run with no hardware)
```

## Quick start
```bash
pip install -r requirements.txt          # numpy + h5py minimum; Qt only for the GUI

# headless end-to-end (no hardware, no Qt):
python scripts/record_mock.py /tmp/epg.h5 --seconds 30 --rate 1000

# the REAL device (flash firmware/prebuilt/*.uf2 first; see ../firmware/README.md):
python scripts/record_device.py --list-ports
python scripts/record_device.py C:\temp\peg_dev.h5 --seconds 30 --rate 1000   # add --port COM5 to choose
python scripts/run_gui.py --port                                             # live GUI on the device

# tests:
python tests/test_protocol.py            # pure stdlib
python tests/test_pipeline.py            # needs numpy + h5py
# (or: python -m pytest)

# live GUI against the mock (needs pyqtgraph + a Qt binding):
python scripts/run_gui.py --rate 1000
```

## Design notes
- **Protocol layer is dependency-free** (stdlib only) so framing is trivially testable and
  portable; numpy/h5py are only needed for the data path, Qt only for the GUI.
- **Reconstructability:** recordings store raw 24-bit codes + a timestamped event log; the
  reader recovers input-referred volts and the applied-Vs trace, so the true signal is
  recoverable even though the keep-in-range servo nudges Vs during recording.
- **Real hardware** drops in behind the same protocol: `SerialDevice` replaces the mock's byte
  stream with a `pyserial` USB-CDC port; the parser, recorder and reader are unchanged. Without
  an ADC the firmware streams a synthetic signal and tags the file serial `...-SIM`.
