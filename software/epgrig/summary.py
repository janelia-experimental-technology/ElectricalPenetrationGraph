"""Shared end-of-recording summary: read the HDF5 back and print what was captured."""
from __future__ import annotations

from collections import Counter

from . import reader


def print_summary(path: str) -> None:
    r = reader.load(path)
    print(f"\nRead back {path}:")
    print(f"  format {r.attrs['format_version']}  serial {r.attrs['device_serial']}  "
          f"fw {r.attrs['firmware_version']}")
    print(f"  {r.n_samples} samples x {len(r.channel_ids)} ch @ {r.rate_hz} Hz "
          f"({r.attrs['duration_s']:.2f} s);  gain={r.fixed_gain}")
    ir = r.input_referred
    unc = r.uncompensated()
    print(f"  input-referred ch{int(r.channel_ids[0])}:  mean={ir[:,0].mean()*1e3:+.2f} mV  "
          f"std={ir[:,0].std()*1e3:.2f} mV  range=[{ir[:,0].min()*1e3:+.1f},{ir[:,0].max()*1e3:+.1f}] mV")
    print(f"  uncompensated ch{int(r.channel_ids[0])}:   mean={unc[:,0].mean()*1e3:+.2f} mV  "
          f"std={unc[:,0].std()*1e3:.2f} mV (drift visible since Vs removed)")
    evs = r.events_as_dicts()
    kinds = Counter(e["type"] for e in evs)
    print(f"  events: {dict(kinds)}")
    for e in evs:
        if e["type"] == "COMMENT" and e["text"].startswith("fw"):
            print(f"  device says: {e['text']}")
            break
    vs_evs = [e for e in evs if e["type"] == "VS_CHANGE" and e["text"] != "start"]
    if vs_evs:
        e = vs_evs[0]
        print(f"  first Vs step: sample {e['sample_index']} ch{e['channel']} "
              f"dac {e['a']}->{e['b']} ({e['text']})")
    errs = [e for e in evs if e["type"] == "ERROR"]
    if errs:
        print(f"  WARNING: {len(errs)} device ERROR events, first: {errs[0]}")
