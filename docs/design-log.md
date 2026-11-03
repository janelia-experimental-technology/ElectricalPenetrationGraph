# Design log — locked decisions & rationale

Decision records for the EPG redesign, newest at the bottom. Each entry: the decision,
why, and any constraints it imposes. Open questions tracked at the end.

---

## D1 — Keep the differential 3-op-amp instrumentation-amp front end
**Decision:** Reuse the proven differential in-amp topology (2 electrometer buffers +
difference amp, 50× primary gain) rather than inventing a new front end.
**Why:** It is a textbook, well-characterized circuit; the original's choice is sound.
The hard part is the electrometer op-amp + Gigaohm node, which we keep.

## D2 — Keep the 1 GΩ / 10 TΩ ("emf-mode") switch
**Decision:** Default 1 GΩ (entangled R+emf, the routine aphid mode); retain a switch to
10 TΩ pure-emf mode.
**Why:** The 10 TΩ value is the op-amp's own input resistance once the external 1 GΩ is
switched out — so the feature costs only a **low-leakage switch** (reed relay or guarded
analog switch), given we already use an electrometer op-amp. Near-zero cost, preserves
plant-electrophysiology capability, loses nothing for routine work.
**Constraint:** switch must be genuinely low-leakage (pA); immaculate board cleanliness /
conformal coat around the input node.

## D3 — Per-channel daughtercard architecture; design for 8, build 1 first
**Decision:** Implement the analog front end as a small per-channel daughtercard (carrying
the high-impedance node: in-amp + Ri + low-leakage switch) that connects to a shared
motherboard. Only **low-impedance buffered analog** crosses the connector.
**Why:** Engineering effort (electrometer layout, firmware, software, Vs servo) is nearly
independent of channel count — only the analog front-end BOM scales (~$30/ch vs ~$30–40
fixed shared). Prototype/validate one channel end-to-end, then populate the rest with zero
redesign. Matches the original's individually-replaceable-probe philosophy.
**Rough BOM:** 1-channel ≈ $60, 8-channel ≈ $270 (single-unit, excl. enclosure/assembly).

## D4 — Automatic drift correction = slow hardware Vs servo + logging
**Decision:** Put Vs under DAC control and run a slow firmware servo that corrects only
minutes-scale baseline drift toward a **user-set target level**, and **logs every Vs
change** into the recording.
**Why:** The drift is electrode-potential drift (not op-amp drift). Vs also sets the R/emf
balance and waveform appearance, so correction must be slow and fully reconstructable in
analysis — never a silent fast auto-center.

## D5 — Software: Python + pyqtgraph/Qt; device = USB CDC
**Decision:** Cross-platform acquisition + analysis app in Python (pyqtgraph/Qt). Device
enumerates as a standard **USB CDC serial** device (no custom drivers).
**Why:** Strong scientific ecosystem, fast real-time plotting, runs on any OS. USB CDC
removes the Windows-only driver problem of the original Stylet+.

## D6 — Central digitization with differential cable drive
**Decision:** Digitize **centrally** on the motherboard with one shared
**simultaneous-sampling Σ-Δ ADC** (e.g. ADS131M08 class). Each daughtercard sends its
buffered signal **differentially** over the cable (add one inverting op-amp on the card to
make the complementary line); the ADC's native **differential input** does the A−B
subtraction.
**Why:** Keeps all switching/clocks far from the femtoamp 1 GΩ node (the original
deliberately keeps digital out of the cage). One shared ADC gives free inter-channel
sample synchronization, lower parts count, simpler firmware. Differential drive adds
common-mode rejection of 50/60 Hz cable pickup and breaks ground loops — most of the noise
benefit of per-probe digitizing, without the noise penalty.
**Constraint:** differential swing must stay within ADC input range / card rails; signal
pair should be twisted/paired for good CMRR. Revisit per-probe digital only if long-cable
field use becomes a primary requirement.
**Bonus simplification:** the −50 mV calibration pulse becomes a firmware-commanded step
on the Vs line — no dedicated cal hardware or conductor.

## D7 — Independent probe heads on multi-conductor cables; Lemo/ODU + BNC
**Decision:** Each daughtercard lives in its own probe head on a **flexible
~8-conductor shielded cable** to the motherboard (probes are positioned individually on
stands inside the cage — no backplane). Per-channel cable carries: differential signal
pair A/B (twisted), Vs (with cal pulse riding on it), +V, −V, analog ground, Ri-switch
control, + overall shield ≈ **7 conductors + shield**. Connectors: **BNC** at the probe
tip for the insect electrode; **Lemo/ODU push-pull 8-pin** probe-to-motherboard
(locking, best shielding/durability). Cost-softeners: PCB-mount sockets on the
motherboard side; ODU/Fischer/compatible series can undercut genuine Lemo.
**Why:** Matches how EPG probes are actually used (individually clamped near each insect).
David chose the premium connector for shield quality, locking security, and longevity.
**Bonus:** cal pulse rides on the Vs line (no dedicated conductor); ±V rails sent from the
motherboard so nothing switches near the femtoamp node.

## D8 — Electrometer input op-amp: ADA4530-1; Ri switch: latching reed relay
**Decision:** Input buffers = **ADA4530-1** (Analog Devices femtoampere electrometer amp),
two per channel (both sit across Ri, so both must be electrometer grade). The downstream
difference amp and the differential-drive inverter are ordinary precision op-amps. Ri
(1 GΩ/10 TΩ) switch = **latching reed relay** (>10^14 Ω open isolation; energized only
during the rare mode change → no continuous coil current/field near the node).
**Why:** ADA4530-1's 20 fA max bias and **integrated guard buffer** make the hardest part
of the build (femtoamp board leakage / guard ring) near-turnkey, and it's ideal for the
10 TΩ mode. ~$10–12 ea (~$170 across 8 ch) — small premium to de-risk the make-or-break
subsystem; consistent with the premium/longevity choices already made. A semiconductor
analog switch's pA leakage would compromise the 10 TΩ mode, hence the reed relay.
**Constraint:** PCB guard ring driven by the ADA4530-1 guard pin; conformal coat / clean
input node.

## PRIORITY NOTE (David, 2026-06-10)
Not very cost-sensitive. **Maximize sensitivity / signal quality and operational
simplicity at every step**; cost is secondary. Default to the higher-sensitivity,
lower-noise, more-foolproof option; automate rather than add knobs. (Mirrors memory
`epg-priorities`.)

## D9 — Fixed gain (no programmable-gain stage); sensitivity from oversampling
**Decision:** Single **fixed** primary gain in the probe (≥50×, exact value set by a noise
budget so the front end, not the ADC, is the limiting noise source). No variable/2nd-stage
gain. Rely on the 24-bit ADC's dynamic range + **oversampling & decimation** for effective
resolution and SNR.
**Why:** Best on both of David's axes — operational simplicity (the only per-channel
adjustment left is Vs, which is auto-servoed per D4) and sensitivity (a PGA would add noise
/ switching artifacts; oversampling improves SNR for free and also feeds goal #1). Digital
zoom in software replaces the old manual gain knob.

## D10 — Core motherboard ICs
**Decision:**
- **ADC:** ADS131M08 — 8-ch, 24-bit, simultaneous-sampling Σ-Δ, native differential inputs
  (serves as the differential-cable receiver), SPI, up to 32 kSPS/ch. Baseline; **AD7768-8**
  kept as a higher-performance upgrade path if the noise budget calls for it.
- **Vs DAC:** DAC8568 — 8-ch, 16-bit, internal ref, SPI (~15 µV steps over ±0.5 V).
- **MCU:** **RP2040** (David's choice) — ample for the data rate, USB, great tooling; all
  digital stays on the motherboard.
- **Bipolar power:** LM27762 — clean ±rails from USB 5 V with integrated low-noise LDOs;
  digital 3.3 V from a separate LDO. Stays within USB current budget.

## D11 — Vs auto-servo: two-phase, keep-in-range, software-centered display
**Decision:** Firmware Vs servo with two phases.
- **Acquire** (recording start / on demand): quickly seek Vs to bring each channel's
  baseline to target — automates the start-of-recording adjustment the manual calls
  essential.
- **Track** (during recording): **keep-in-range only** — hardware moves Vs *only* when
  drift threatens to clip the ADC (mirrors the manual's "only re-adjust if off scale").
  Slow loop (τ ~ tens of s, well below waveform content); pauses during the cal pulse;
  per-channel **freeze** + **manual override** always available.
**Logging:** every Vs change recorded (timestamp, old→new) as a side channel so analysis
can add the steps back and recover the true uncorrected trace.
**Display:** centered in **software** (high-pass/detrend), independent of the hardware — so
"looks centered" is decoupled from "Vs moved." Minimizes interference with the R/emf
balance; hardware interventions are rare, discrete, logged events.
**Why:** Best for David's sensitivity + simplicity priorities and scientifically faithful
(doesn't silently eat slow biological level changes like sustained E2). The electrode-drift
correction is still needed because the fixed 50× amplifies DC drift into volts that would
otherwise rail the ADC.

## D12 — Sample rate: runtime-configurable, default 1 kHz (for exploration, not a fixed spec)
**Decision:** Sample rate is a **runtime recording parameter** (e.g. 250 / 500 / 1000 /
2000 / 4000 Hz), default **1000 Hz** (vs the original's 100 Hz). Not a fixed hardware target.
Oversample + decimate for SNR; optionally store a high-rate raw stream.
**Rationale / correction (David, 2026-06-10):** Aphid signal bandwidth is genuinely
*unknown* — prior data sampled at 100 Hz only resolves <50 Hz, so the field has never been
able to look higher; the original instrument may have been self-limiting. Worth exploring,
but we don't know that we need 1 kHz. **Key finding: sample rate is NOT a hardware design
driver** — 500 vs 1000 vs 2000 Hz changes nothing in the ADC chip, front end, MCU, USB,
power, layout, firmware, or software (only the Nyquist ceiling and file size). So we make it
configurable, default high, and let real recordings reveal whether faster aphid content
exists. The genuinely separate lever is *analog bandwidth* (see below), which is independent
of sample rate.
**Analog-bandwidth note:** the ~150–300 Hz front-end ceiling (Ri·C_in, see D13) is what
actually limits resolvable content; pushing past it needs **capacitance neutralization**, for
which a populate-later footprint is reserved. Trigger to populate it = finding real energy
near the roll-off during high-rate exploratory recordings. C_in ≈ 1 pF layout discipline
(guard ring + ADA4530-1 guard buffer) remains the make-or-break spec regardless.

## D13 — Noise & bandwidth budget results (see noise-bandwidth-budget.md)
**Findings:**
- **Noise floor is the 1 GΩ resistor's thermal noise (~30–65 µV RMS in-band)** — op-amp and
  ADC noise are 100×+ below it. Fundamental to the 1 GΩ choice; the original lives here too.
- **Fixed gain G ≈ 8** (not 50×) — maps ±150 mV input FS to the ADS131M08 ±1.2 V range; the
  24-bit depth (not analog gain) preserves resolution. Confirm vs final max-feature spec.
- **ADS131M08 confirmed; AD7768-8 not warranted** (resistor floor masks any ADC-noise gain).
- **Keep 24-bit** for dynamic range / no-gain-knob operation (not for the floor).
- **Realistic front-end bandwidth at 1 GΩ ≈ 150–300 Hz** (probing), set by Ri·C_in. Hitting
  500 Hz needs C_in < ~0.6 pF (impractical). So **C_in ≈ 1 pF is the make-or-break layout
  spec** (guard ring + short traces). 1 kHz sampling is confirmed/beneficial but understood
  as time-resolution/edge/anti-alias gains over a ~200–300 Hz band, *not* 500 Hz of
  bandwidth. Leave a populate-later footprint for **capacitance neutralization** if more
  analog BW is ever needed.
- Σ-Δ decimation filter handles anti-aliasing → **no separate analog AA filter**.

## D14 — Device↔host protocol + open data format (see protocol-and-data-format.md)
**Decision (draft v0.1):**
- **Link:** USB CDC (no drivers), little-endian, **COBS** framing + CRC-16/CCITT, typed
  messages. Device→host: INFO/SAMPLES/EVENT/ACK/NACK/STATUS. Host→device:
  GET_INFO/CONFIGURE/START/STOP/SET_VS/SET_RI/CAL_PULSE/SERVO/PING.
- **Master timeline = `sample_index`** (monotonic from START); absolute time =
  start_time_utc + index/rate. Every state change (Vs, Ri, cal, servo) — host- *or*
  servo-initiated — is emitted as a `sample_index`-stamped **EVENT**, making the event log
  the authoritative, reconstructable record. Device streams **raw 24-bit ADC codes**
  (lossless); calibration lives in metadata.
- **File:** **HDF5** canonical (`/samples` int32 codes chunked+compressed, `/events`
  compound table, root + per-channel metadata), written incrementally with periodic flush
  for crash-safety. Reconstruction conventions defined (code→input-referred volts; applied-Vs
  = Vs0 + Σ logged steps; display detrend is view-only, never written). CSV + future **NWB**
  export.
**Why:** Cross-platform + driver-free (goal #5); full reconstructability protects the science
given the Vs servo (D11); HDF5 is the scientific-time-series standard. **Unblocks the Python
software** to proceed against this contract in parallel with hardware.

## D15 — Python software v0.1 built & tested against a mock device (`software/`)
**Done:** Implemented the host software package `epgrig` and verified the *entire* pipeline
in software, no hardware:
- `protocol.py` — COBS+CRC framing + message codecs (pure stdlib).
- `mock_device.py` — synthetic aphid-like waveforms + electrode drift + keep-in-range Vs
  servo, emitting real protocol frames.
- `acquisition.py` / `recorder.py` / `reader.py` — parse → incremental HDF5 → reconstruct
  (input-referred volts + applied-Vs from the event log).
- `dsp.py` — display detrend (moving-average + streaming one-pole HP), view-only.
- `gui.py` + `scripts/run_gui.py` — live pyqtgraph display (optional Qt).
- `scripts/record_mock.py` — headless end-to-end demo.
**Verified:** 15 tests pass (protocol round-trips incl. COBS/CRC + parser resync; pipeline
mock→HDF5→reconstruct; dsp). Demo: 30 s × 8 ch @ 1 kHz → 843 KB HDF5, 0 CRC errors, servo
fired (VS_CHANGE events logged), input-referred trace bounded while uncompensated shows the
removed drift — i.e. D11/D14 behavior proven. Real hardware later drops in behind the same
protocol (swap mock byte-stream for a pyserial USB-CDC port).
**Env note:** dev verified with system Python 3.9 + user-installed numpy/h5py; Qt not present
here so the GUI is written + parse-checked but not launched.

### D15 follow-ups (GUI + mock command path)
- GUI gained a **sample-rate selector** (from `Info.supported_rates`; reconfigures live,
  locked during recording), **Record/Stop** + file/metadata, **markers**, working **detrend**
  checkbox, and **per-channel controls** (Vs / servo off-acquire-track / Ri 1G-10T / cal pulse).
- Mock device now **acts on** SET_VS/SET_RI/SERVO/CAL_PULSE and emits the matching logged
  events (cal injects −50 mV + auto-offs; servo respects mode). `tests/test_commands.py` added
  → **19 tests pass**.
- **Repo:** local git initialized at `EPG_redesign/` (vendor PDFs/installers excluded),
  initial commit made. GitHub remote `DavidSternLab/ElectricalPenetrationGraph` pending
  David's auth (no gh/token/SSH in the dev environment).

## D16 — Single-channel schematic v0.1 + SPICE verification (`hardware/`)
**Done:** First schematic-level design (`hardware/single-channel-schematic.md`): block diagram,
signal chain, component table, power tree, layout-critical notes. Topology decided:
- Daughtercard: **A1, A2 = ADA4530-1** unity-gain electrometer followers sensing the two ends
  of Ri; **Ri = 1 GΩ + latching reed relay** (open → 10 TΩ intrinsic); Vs_in applied at the S
  node (cal pulse rides on it); **FDA = ADA4940-1** provides the gain *and* the differential
  cable drive in one stage. **Gain G ≈ 8** via Rg=1.0k / Rf=8.06k (D13).
- Motherboard: ADS131M08 diff input (Σ-Δ AA → no analog AA filter), DAC8568→scale/offset
  (±0.5 V)→~10 Hz LP→Vs_in, RP2040, LM27762 ±5 V + 3.3 V LDO, on-card latching-relay driver.
**Verified by SPICE** (`hardware/sim/single_channel.cir`, ngspice): Ohm's-law divider
Vi = 0.500 mV ✓, differential gain Vout = 4.00 mV (=8×) ✓, and **front-end f₋₃dB = 318.3 Hz,
matching the analytic `1/(2π·(Rbe‖Ri)·Cin)` to 100%** — empirically confirming the D13
bandwidth budget and that **Cin ≤ ~1 pF is the dominant layout lever**.

## D17 — RP2040 firmware skeleton + portable C protocol (`firmware/`)
**Done:**
- `firmware/proto/epg_proto.{h,c}` — the protocol codec in **portable C** (COBS, CRC-16,
  i24, frame builders, incremental parser, command decoders), mirroring the Python codec.
- `firmware/test/` — host-compiled **C selftest** (`make check`) and a **cross-language
  interop test** (`make interop`, `interop_test.py`) that builds frames in C and parses them
  in Python and vice-versa. **Proven byte-for-byte compatible both directions** → the
  firmware will talk to the existing host GUI unchanged.
- `firmware/src/` — RP2040 Arduino app skeleton: `main.cpp` (USB-CDC loop, command dispatch,
  DRDY-driven 8-ch acquisition, block streaming, keep-in-range Vs servo, sample-stamped
  EVENTs), `ads131m08.{h,cpp}`, `dac8568.{h,cpp}`, `board.h` pin map; `platformio.ini`
  (arduino-pico) compiling the shared proto/ alongside src/.
**Verified:** C selftest (7 checks) + interop (both directions) pass; all 4 Python host test
files still pass. **Not hardware-tested:** ADC/DAC register sequences + relay/servo scaling
are marked `TODO: verify vs datasheet`; src/ not compiled here (needs RP2040 toolchain).

## D18 — Netlist + KiCad schematic capture (ERC-clean, `hardware/netlist/`)
**Done:**
- `channel_netlist.py` — the single-channel daughtercard as machine-checkable data
  (23 components, 71 pins, 21 nets) + an **ERC-style checker**: 0 errors / 0 warnings
  (every pin connected exactly once; power + diff pair consistent). Generates
  `connection-table.md`.
- `gen_kicad_sch.py` — emits a real **`single_channel.kicad_sch`** from that netlist (box
  symbols + global-label-per-pin connectivity). Validated with **`kicad-cli` (KiCad 10.0.3):
  upgrade + ERC pass with only benign warnings** — 23 "symbol library 'epg' not configured"
  (symbols are embedded/self-contained) and 3 isolated-label (the intentional single-node
  nets SHLD/GUARD1/GUARD2). **Zero connectivity errors.** Rendered to `single_channel.svg` /
  `.pdf` and eyeballed — all parts, pins, and net labels present and correct.
**Note:** box symbols (not op-amp glyphs) and no footprints yet — intended as a verified
first capture; swap to library symbols + assign footprints during manual cleanup in KiCad
(now installed). The netlist data is the source of truth and regenerates the sheet.

## D19 — PCB layout groundwork + hand-off package (`hardware/pcb/`, `pcb-layout.md`)
**Context:** David will recruit a KiCad layout engineer to finish; this is the verifiable
groundwork + hand-off (routing of an fA front end is deliberately a human task).
**Done:**
- **Footprints assigned** to all 23 parts in `channel_netlist.py` (verified present in KiCad
  10 libs; 3 PLACEHOLDERs flagged: Lemo/ODU J2, latching reed relay K1, ADA4940 LFCSP).
  Schematic now carries a `Footprint` property per symbol (so *Update PCB from Schematic*
  pulls nets+footprints). Connection table regenerated with a footprint column.
- **Starting board** `pcb/single_channel.kicad_pcb` generated via the `pcbnew` API
  (`gen_pcb_placement.py`): 23 footprints pre-placed with electrometer-aware intent (BNC/A1
  input cluster left → FDA centre → Lemo output right), 60×40 mm outline, 4 layers.
  Validated with `kicad-cli pcb drc`: **0 unconnected/structural errors**; 40 items are
  placement-refinement only (courtyard/silk/edge/clearance) for the layout engineer.
  Rendered `pcb/board_top.svg` and eyeballed.
- **`hardware/pcb-layout.md`** — the layout spec / hand-off: 4-layer stackup; the make-or-break
  input-node rules (Cin≤1 pF, no plane under M, no vias on M, **driven guard rings** from the
  ADA4530-1 GUARD pins, surface-leakage slot + conformal coat + cleanliness); net classes
  (HighZ/Guard/Power/Signal/Diff); placement intent; grounding/shielding.
- **`pcb/single_channel.kicad_dru`** — custom design rules (HighZ clearance/no-via/thin-track,
  diff-pair skew, fab floors). Confirmed it **parses cleanly** (kicad-cli DRC, 0 rule errors).

## D20 — RP2040 firmware v0.2 + real-device host transport (`firmware/`, `software/`)
**Done:** the firmware skeleton became a complete, compiling Pico H firmware, plus the host side
needed to swap the mock for it.
- **Firmware** (`firmware/src/`): DRDY-interrupt acquisition into a 1024-sample ring (USB stalls
  can't lose samples); ADS131M08 driver (reset, ID check, MODE/CLOCK/GAIN, OSR per rate — 250 Hz
  is OSR 8192 averaged ×2 so every rate is exact at CLKIN 8.192 MHz); DAC8568 driver; 74HC595 Ri
  lines; Vs servo (off / track / acquire→track with target, deadband, freeze), −50 mV cal pulse on
  the Vs line, clip events, sample-stamped EVENTs for every state change, NACK reasons, START
  snapshot events. **Built-in simulator** when no ADC answers, so a bare Pico exercises the whole
  chain (serial suffixed `-SIM`). Compiles with PlatformIO/arduino-pico (45 kB flash, 66 kB RAM).
  `board.h` is the pin/option contract for the (not yet designed) motherboard; RP2040 generates the
  ADC CLKIN by default, an 8.192 MHz oscillator is the recommended final option.
- **Bugs found in the v0.1 skeleton and fixed:** `epg_encode_frame` staged frames in a 512 B stack
  buffer (real 50 ms blocks are 1.2–4.8 kB → stack overflow; now a streaming COBS encoder with
  `EPG_FRAME_MAX`, tested up to 4.8 kB incl. C↔Python interop); `main.cpp` used a 600 B TX buffer
  (same); Arduino `SPI` left on the default pins (GP16–19, not the board's SPI0 GP2–4); DAC init
  word and ADC configuration were TODOs; servo gain was a placeholder; a CS glitch at pin init.
- **Host** (`software/epgrig/serial_device.py`, `scripts/record_device.py`, `run_gui.py --port`):
  pyserial transport with a reader thread, ACK/NACK handling (`DeviceError`), Pico auto-detect,
  stale-stream recovery on open; `SerialGuiBridge` lets the existing GUI drive the device.
- **Verified without hardware:** the real `firmware.uf2` runs in the rp2040js emulator and is driven
  by the real Python host code (`firmware/test/emu_test.py`, both simulator and modelled-ADC
  paths): every command incl. NACKs; step/cal amplitudes to ~0.1 mV; true drift recovered from the
  event log for servo off/track/acquire; all 5 rates deliver a 3 Hz tone at 3.00 Hz with no lost
  samples (0 missed ADC conversions at 4 kHz). 25 host tests + C selftest + interop pass.
- **NOT verified on silicon:** ADS131M08 ID/MODE/CLOCK/OSR/CRC details, DAC8568 mode/reference,
  Vs polarity, relay edge polarity, clock jitter — see the bring-up checklist in `firmware/README.md`.

---

## Open questions / next pivots
- **Layout engineer:** Update-PCB-from-Schematic, refine placement (clear the 40 items),
  implement the guard scheme, route, swap the 3 PLACEHOLDER footprints, DRC-clean.
- Finalize FDA supply/VOCM, relay-driver sub-circuit, ADC RC + CLKIN, Vs scale/offset, USB budget.
- Firmware hardware bring-up (D20 checklist): verify ADS131M08/DAC8568 details on silicon, Vs
  polarity + calibration, relay edge polarity; design the motherboard to `board.h`.
- Mechanical: shielded probe-head enclosure, guard-ring layout, conformal coating.
- Software niceties: analysis/Y-zoom UI, waveform labeling, NWB export, real serial transport.
- Channel count confirm (assume 8).
- USB bipolar-rail generation.
- Vs servo loop details (rate, target, logging format).
- Open data file format definition + converter.
