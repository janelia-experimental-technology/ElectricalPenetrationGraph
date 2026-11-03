"""Live acquisition GUI (pyqtgraph) with on-screen controls.

A minimal but real acquisition app (Stylet+d replacement, in progress):
  * live multi-channel display with a working detrend toggle and sample-rate selector
  * per-channel controls: Vs adjust, servo mode (off/acquire/track), Ri (1 GΩ/10 TΩ),
    calibration pulse — each sent to the device and echoed as a logged event
  * Record / Stop with directory + filename + operator/experiment metadata
  * insert timestamped comment markers into the recording
Driven by the in-process mock device by default; pass --port to drive the real device
behind the same protocol.

Run:  python scripts/run_gui.py
"""
from __future__ import annotations

import datetime as _dt
import os

import numpy as np

from . import protocol as P
from .acquisition import StreamConsumer
from .dsp import moving_average_detrend
from .mock_device import MockDevice
from .recorder import HDF5Recorder


def run_mock_gui(rate_hz: int = 1000, channel_mask: int = 0xFF,
                 detrend_fc: float = 0.5, window_s: float = 10.0,
                 block_ms: float = 50.0, dev=None, title: str = "mock"):
    """Live GUI. `dev` defaults to an in-process MockDevice; pass a SerialGuiBridge
    (epgrig.serial_device) to drive the real RP2040 device instead."""
    import pyqtgraph as pg  # lazy
    from pyqtgraph.Qt import QtCore, QtWidgets

    chans = P.active_channels(channel_mask)
    nch = len(chans)
    dev = dev if dev is not None else MockDevice(channel_mask=channel_mask, rate_hz=rate_hz)
    info = dev.info
    code_to_in = (info.adc_vref_mv / 1000.0) / (1 << 23) / info.fixed_gain
    vs_half_v = info.vs_range_mv / 2000.0
    vs_dac_max = (1 << info.vs_dac_bits) - 1

    def vs_mv_to_dac(mv: float) -> int:
        return int(round((mv / 1000.0 + vs_half_v) / (2 * vs_half_v) * vs_dac_max))

    rt = {"rate": rate_hz,
          "buf": np.zeros((int(window_s * rate_hz), nch)),
          "x": np.arange(int(window_s * rate_hz)) / rate_hz,
          "n_per_block": max(1, int(rate_hz * block_ms / 1000.0))}
    state = {"detrend": True, "dev_index": 0, "recorder": None, "clip": False}
    # GUI-side mirror of per-channel instrument settings
    chstate = {c: {"vs_mv": 0.0, "servo": P.SERVO_TRACK, "ri": P.RI_1G} for c in chans}

    consumer = StreamConsumer(recorder=None)

    def send(mtype: int, payload: bytes) -> None:
        for fr in dev.handle_command(mtype, 0, payload):
            consumer.feed(fr)

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    win = QtWidgets.QMainWindow()
    win.setWindowTitle(f"EPG rig — acquisition ({title})")
    central = QtWidgets.QWidget(); win.setCentralWidget(central)
    root = QtWidgets.QHBoxLayout(central)

    # ---- plots ----
    glw = pg.GraphicsLayoutWidget()
    root.addWidget(glw, stretch=4)
    curves = []
    for i, ch in enumerate(chans):
        pw = glw.addPlot(row=i, col=0)
        pw.setLabel("left", f"ch{ch}", units="V")
        pw.showGrid(x=True, y=True, alpha=0.2)
        if i < nch - 1:
            pw.getAxis("bottom").setStyle(showValues=False)
        curves.append(pw.plot(pen=pg.mkPen((40 + (i * 25) % 215, 170, 255 - i * 18))))
    glw.getItem(0, 0).setTitle("input-referred volts")

    # ---- control panel ----
    panel = QtWidgets.QWidget(); root.addWidget(panel, stretch=1)
    pl = QtWidgets.QVBoxLayout(panel)

    def group(title):
        g = QtWidgets.QGroupBox(title); f = QtWidgets.QFormLayout(g); pl.addWidget(g); return f

    # status
    sf = group("Status")
    lbl_state = QtWidgets.QLabel("idle"); lbl_elapsed = QtWidgets.QLabel("0.0 s")
    lbl_samples = QtWidgets.QLabel("0"); lbl_clip = QtWidgets.QLabel("—")
    lbl_crc = QtWidgets.QLabel("0")
    for k, w in (("State:", lbl_state), ("Elapsed:", lbl_elapsed), ("Samples:", lbl_samples),
                 ("Clip:", lbl_clip), ("CRC errs:", lbl_crc)):
        sf.addRow(k, w)

    # acquisition (rate + display)
    af = group("Acquisition")
    cmb_rate = QtWidgets.QComboBox()
    for r in info.supported_rates:
        cmb_rate.addItem(f"{r} Hz", r)
    cmb_rate.setCurrentIndex(list(info.supported_rates).index(rate_hz))
    cb_detrend = QtWidgets.QCheckBox("Detrend (center baseline)"); cb_detrend.setChecked(True)
    sp_fc = QtWidgets.QDoubleSpinBox(); sp_fc.setRange(0.05, 50.0); sp_fc.setValue(detrend_fc)
    sp_fc.setSuffix(" Hz"); sp_fc.setSingleStep(0.1)
    af.addRow("Sample rate:", cmb_rate); af.addRow(cb_detrend); af.addRow("Detrend cutoff:", sp_fc)

    # per-channel controls
    cf = group("Channel control")
    cmb_chan = QtWidgets.QComboBox()
    cmb_chan.addItem("All channels", -1)          # -1 = apply to every channel
    for c in chans:
        cmb_chan.addItem(f"ch{c}", c)
    cmb_chan.setCurrentIndex(1 if nch else 0)     # default to the first real channel
    sp_vs = QtWidgets.QDoubleSpinBox(); sp_vs.setRange(-500.0, 500.0); sp_vs.setSuffix(" mV")
    sp_vs.setDecimals(1); sp_vs.setSingleStep(1.0)
    cmb_servo = QtWidgets.QComboBox(); cmb_servo.addItems(["off", "acquire", "track"])
    cmb_servo.setCurrentIndex(P.SERVO_TRACK)
    cmb_ri = QtWidgets.QComboBox(); cmb_ri.addItems(["1 GΩ", "10 TΩ"])
    btn_cal = QtWidgets.QPushButton("Cal pulse (−50 mV, 0.5 s)")
    cf.addRow("Channel:", cmb_chan); cf.addRow("Vs:", sp_vs)
    cf.addRow("Servo:", cmb_servo); cf.addRow("Input R:", cmb_ri); cf.addRow(btn_cal)

    # recording
    rf = group("Recording")
    ed_dir = QtWidgets.QLineEdit(os.path.expanduser("~"))
    btn_browse = QtWidgets.QPushButton("Browse…")
    ed_name = QtWidgets.QLineEdit("epg_" + _dt.datetime.now().strftime("%Y%m%d_%H%M%S"))
    ed_op = QtWidgets.QLineEdit(""); ed_exp = QtWidgets.QLineEdit("")
    btn_record = QtWidgets.QPushButton("● Record"); btn_record.setCheckable(True)
    for k, w in (("Folder:", ed_dir), ("", btn_browse), ("File:", ed_name),
                 ("Operator:", ed_op), ("Experiment:", ed_exp)):
        rf.addRow(k, w)
    rf.addRow(btn_record)

    # markers
    mf = group("Marker")
    ed_comment = QtWidgets.QLineEdit(); btn_mark = QtWidgets.QPushButton("Add marker")
    mf.addRow("Comment:", ed_comment); mf.addRow(btn_mark)
    pl.addStretch(1)

    # ---- handlers: acquisition ----
    def change_rate(idx):
        if state["recorder"] is not None:
            return
        r = cmb_rate.itemData(idx)
        rt["rate"] = r; dev.rate_hz = r
        n = int(window_s * r)
        rt["buf"] = np.zeros((n, nch)); rt["x"] = np.arange(n) / r
        rt["n_per_block"] = max(1, int(r * block_ms / 1000.0))
    cmb_rate.currentIndexChanged.connect(change_rate)
    cb_detrend.stateChanged.connect(lambda _=0: state.update(detrend=cb_detrend.isChecked()))

    # ---- handlers: per-channel (target = one channel, or all when "All channels") ----
    def targets():
        d = cmb_chan.currentData()
        return list(chans) if d == -1 else [d]

    def refresh_chan_widgets():
        d = cmb_chan.currentData()
        if d == -1:
            return  # "All channels": keep current widget values; they apply to every channel
        s = chstate[d]
        for w in (sp_vs, cmb_servo, cmb_ri):
            w.blockSignals(True)
        sp_vs.setValue(s["vs_mv"]); cmb_servo.setCurrentIndex(s["servo"]); cmb_ri.setCurrentIndex(s["ri"])
        for w in (sp_vs, cmb_servo, cmb_ri):
            w.blockSignals(False)
    cmb_chan.currentIndexChanged.connect(lambda _=0: refresh_chan_widgets())

    def on_vs(_=0.0):
        dac = int(vs_mv_to_dac(sp_vs.value())).to_bytes(2, "little")
        for c in targets():
            chstate[c]["vs_mv"] = sp_vs.value()
            send(P.T_SET_VS, bytes([c]) + dac)
    sp_vs.valueChanged.connect(on_vs)

    def on_servo(idx):
        for c in targets():
            chstate[c]["servo"] = idx
            send(P.T_SERVO, bytes([c, idx, 0, 0, 0, 0, 0]))
    cmb_servo.currentIndexChanged.connect(on_servo)

    def on_ri(idx):
        for c in targets():
            chstate[c]["ri"] = idx
            send(P.T_SET_RI, bytes([c, idx]))
    cmb_ri.currentIndexChanged.connect(on_ri)

    def on_cal():
        for c in targets():
            send(P.T_CAL_PULSE, bytes([c, 1]) + (500).to_bytes(2, "little"))
    btn_cal.clicked.connect(on_cal)
    refresh_chan_widgets()

    # ---- handlers: recording ----
    def browse():
        d = QtWidgets.QFileDialog.getExistingDirectory(win, "Recording folder", ed_dir.text())
        if d:
            ed_dir.setText(d)
    btn_browse.clicked.connect(browse)

    def on_record(checked):
        if checked:
            try:
                path = os.path.join(ed_dir.text(), ed_name.text())
                if not path.endswith(".h5"):
                    path += ".h5"
                rec = HDF5Recorder(path, info, rt["rate"], channel_mask,
                                   metadata={"operator": ed_op.text(),
                                             "experiment_id": ed_exp.text()})
                state["recorder"] = rec; consumer.recorder = rec
                lbl_state.setText("RECORDING"); lbl_state.setStyleSheet("color:#c00;font-weight:bold")
                btn_record.setText("■ Stop")
                for w in (ed_dir, ed_name, ed_op, ed_exp, btn_browse, cmb_rate):
                    w.setEnabled(False)
            except Exception as e:  # noqa: BLE001
                btn_record.setChecked(False)
                QtWidgets.QMessageBox.critical(win, "Record error", str(e))
        else:
            rec = state["recorder"]; consumer.recorder = None; state["recorder"] = None
            path, n = rec.path, rec.n_samples; rec.close()
            lbl_state.setText(f"saved {os.path.basename(path)} ({n} samp)")
            lbl_state.setStyleSheet(""); btn_record.setText("● Record")
            ed_name.setText("epg_" + _dt.datetime.now().strftime("%Y%m%d_%H%M%S"))
            for w in (ed_dir, ed_name, ed_op, ed_exp, btn_browse, cmb_rate):
                w.setEnabled(True)
    btn_record.toggled.connect(on_record)

    def add_marker():
        rec = state["recorder"]
        if rec is None:
            QtWidgets.QMessageBox.information(win, "Marker", "Start recording first.")
            return
        rec.add_comment(ed_comment.text() or "mark", state["dev_index"]); ed_comment.clear()
    btn_mark.clicked.connect(add_marker)

    # ---- streaming ----
    def on_block(blk: P.SampleBlock):
        arr = np.asarray(blk.samples, dtype=np.float64) * code_to_in
        n = arr.shape[0]; buf = rt["buf"]
        buf[:-n] = buf[n:]; buf[-n:] = arr
        state["dev_index"] = blk.first_sample_index + n
        if blk.status & 1:
            state["clip"] = True
    consumer.on_block = on_block

    def tick():
        for fr in dev.generate_block(rt["n_per_block"]):
            consumer.feed(fr)
        win_samples = max(2, int(rt["rate"] / max(sp_fc.value(), 0.05)))
        buf = rt["buf"]
        disp = moving_average_detrend(buf, win_samples) if state["detrend"] else buf
        x = rt["x"]
        for i in range(nch):
            curves[i].setData(x, disp[:, i])
        rec = state["recorder"]
        if rec is not None:
            lbl_samples.setText(str(rec.n_samples))
            lbl_elapsed.setText(f"{rec.n_samples / rt['rate']:.1f} s")
        lbl_clip.setText("CLIP" if state["clip"] else "ok")
        lbl_clip.setStyleSheet("color:#c00;font-weight:bold" if state["clip"] else "")
        state["clip"] = False
        lbl_crc.setText(str(consumer.parser.crc_errors))

    timer = QtCore.QTimer(); timer.timeout.connect(tick); timer.start(int(block_ms))
    app.aboutToQuit.connect(lambda: state["recorder"] and state["recorder"].close())

    win.resize(1240, 820); win.show()
    (app.exec_ if hasattr(app, "exec_") else app.exec)()
