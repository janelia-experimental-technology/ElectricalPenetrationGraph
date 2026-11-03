# Device↔host protocol & open data file format (draft v0.1)

Defines (A) the USB link between the device (RP2040) and host (Python app), and (B) the
on-disk recording format. Design goals: cross-platform, no custom drivers, robust to byte
loss, and **fully reconstructable** — every state change (Vs, Ri, cal pulse, servo) is
timestamped to an exact sample so the original signal can always be recovered in analysis.

---

# A. Device↔host protocol

## A.1 Transport & framing
- **USB CDC (virtual serial)** — enumerates as COM/tty, no driver install. Baud is ignored
  (USB full/high speed); throughput is ample (8 ch × 4 kHz × 3 B ≈ 96 kB/s ≪ USB CDC limit).
- **Endianness:** little-endian throughout (host + RP2040 are LE).
- **Framing:** **COBS** (Consistent Overhead Byte Stuffing) with a `0x00` frame delimiter,
  so a dropped byte costs at most one frame and the stream resyncs on the next delimiter.
- **Pre-COBS frame:** `type(1) | flags(1) | payload(N) | crc16(2)` where CRC is
  CRC-16/CCITT over `type|flags|payload`. Receivers drop frames with bad CRC.

## A.2 Message catalog

### Device → Host
| Type | Name | Payload |
|---|---|---|
| 0x01 | INFO | capabilities/identity (A.4) |
| 0x02 | SAMPLES | sample block (A.3) |
| 0x03 | EVENT | timestamped event (A.5) |
| 0x04 | ACK | `cmd_type:u8, seq:u8` (command accepted) |
| 0x05 | NACK | `cmd_type:u8, seq:u8, err:u8` |
| 0x06 | STATUS | periodic heartbeat: snapshot of per-channel Vs/Ri/servo + flags |

### Host → Device
| Type | Name | Payload |
|---|---|---|
| 0x80 | GET_INFO | — |
| 0x81 | CONFIGURE | `rate_code:u8, channel_mask:u8` (only when stopped) |
| 0x82 | START | — (begins SAMPLES + EVENT streaming) |
| 0x83 | STOP | — |
| 0x84 | SET_VS | `ch:u8, dac_code:u16` |
| 0x85 | SET_RI | `ch:u8, mode:u8 (0=1GΩ,1=10TΩ)` |
| 0x86 | CAL_PULSE | `ch:u8, action:u8 (0=off,1=on), dur_ms:u16 (0=manual)` |
| 0x87 | SERVO | `ch:u8, mode:u8 (0=off,1=acquire,2=track), target_code:u16, deadband_code:u16, flags:u8 (bit0=freeze)` |
| 0x88 | PING | — (keepalive; device replies STATUS) |

**Principle:** any host command that changes channel state is **ACKed and then echoed as an
EVENT** stamped with the sample_index at which it took effect. So the EVENT log is the single
source of truth, regardless of whether a change was host- or servo-initiated.

## A.3 Sample block (0x02)
```
block_seq         : u32
first_sample_index: u64   # monotonic counter since START; master timeline
rate_code         : u8
channel_mask      : u8    # which of 8 channels are present
n_samples         : u16   # rows in this block
status            : u8    # bit0 = any channel clipped in this block
data              : n_samples × popcount(channel_mask) × int24_le (signed)
```
- `int24_le` = 3-byte signed little-endian (the native ADS131M08 word). Channels appear in
  ascending index order per row. Blocks sized to balance latency vs overhead (e.g. 10–50 ms).

## A.4 INFO (0x01)
`format_proto_version, fw_version(maj,min,patch), n_channels, supported_rate_codes[],
adc_bits=24, adc_vref_mv, fixed_gain_x100, vs_dac_bits, vs_range_mv, serial[12]`.
The host uses this to self-configure and to write file calibration (no hard-coded constants).

## A.5 EVENT (0x03)
```
event_type : u8
channel    : u8   # 0xFF = device-wide
sample_index: u64 # exact sample at which it occurred
a, b       : i32  # type-specific
text_len   : u8 ; text[text_len]  # optional (e.g. comments)
```
| event_type | meaning | a / b |
|---|---|---|
| 1 VS_CHANGE | Vs stepped | a=old_dac, b=new_dac (text=reason: acquire/track/manual) |
| 2 RI_CHANGE | Ri mode switched | a=mode |
| 3 CAL_PULSE | calibration pulse | a=action(0/1), b=amplitude_µV (≈ −50000) |
| 4 CLIP | over-range onset/offset | a=onset(1)/offset(0) |
| 5 SERVO_STATE | servo mode change | a=mode, b=flags |
| 6 COMMENT | host/operator marker | text |
| 7 MODE_MARK | e.g. the manual's "3 pulses = normal, 5 = emf" convention | a=count |
| 8 ERROR | device error | a=code |

## A.6 Session lifecycle
1. Host opens port → `GET_INFO` → device `INFO`.
2. Host `CONFIGURE` (rate, channel mask); initial per-channel `SET_VS`/`SET_RI`/`SERVO`.
3. Host `START` → device streams `SAMPLES` + `EVENT`; periodic `STATUS`.
4. Live control via `SET_*`/`CAL_PULSE`/`SERVO`; each → `ACK` + an `EVENT`.
5. Host `STOP` → device halts, final `ACK`.

Clock: `sample_index` is the master timeline. Absolute time of sample *i* =
`start_time_utc + i / sample_rate`. All events reference `sample_index`, so nothing depends
on host-side timing jitter.

## A.7 Implemented semantics (firmware v0.2 — resolves several "Open items")

**NACK error codes** (`NACK = cmd_type, seq(0), err`): 1 unknown command · 2 bad payload ·
3 busy (e.g. CONFIGURE while streaming) · 4 bad channel · 5 bad value · 6 no ADC / cannot
stream · 7 ADC configuration failed.

**rate_code table:** index into INFO's `supported_rate_codes` = `[250, 500, 1000, 2000, 4000]` Hz.
**Block size:** 50 ms (12/25/50/100/200 rows at the five rates; hard cap 256 rows).
**ERROR events** (`a` = code): 1 sample gap (`b` = samples lost; ring overflow) · 2 USB transmit
drop (`b` = frames) · 3 Vs DAC at its limit (servo cannot correct further).
**SAMPLES `status`:** bit0 = a channel is clipped (|code| ≥ 99 % FS, 97 % to clear).

**START** is ACKed, then the device emits at index 0 a `COMMENT` (firmware, ADC present or
SIMULATED, rate) and, per active channel, `VS_CHANGE(a=b=current, "start")`, `RI_CHANGE`,
`SERVO_STATE`, so a recording started at START is self-contained. Commands received while
stopped are applied but not logged (the START snapshot records the result). Events are sent
before the SAMPLES block that contains their `sample_index`.

**SERVO** payload `ch, mode, target:u16, deadband:u16, flags:u8`: `target` is offset-binary
(`code24 = (target − 32768)·256`; **0 = centre**), `deadband` is in units of 256 ADC codes
(**0 = default 3 % of full scale**), `flags` bit0 = freeze. *Acquire* steps Vs every ≥150 ms
until the baseline has been within the deadband for ~1 s, then switches itself to *track* and
logs a `SERVO_STATE`. *Track* moves Vs only when the baseline (τ = 2 s) passes 80 % of full
scale. Both pause during a calibration pulse and for 1.5 s after any Vs change.

**CAL_PULSE** (`ch, action, dur_ms`): `dur_ms = 0` keeps the pulse on until `action = 0`.
The pulse is **−50 mV on the Vs line** (events `CAL_PULSE a=1/0, b=−50000`); it is *not* logged
as a `VS_CHANGE`, so the reader's applied-Vs excludes it and the pulse is visible in the
uncompensated trace — exclude the logged cal interval when fitting drift.

**Sample timing:** every rate comes from an 8.192 MHz ADC clock; 250 Hz is 500 Hz averaged ×2.
`PING` is answered with `ACK` (no periodic `STATUS` yet).

---

# B. Data file format

## B.1 Container: HDF5
**HDF5** is the canonical recording format — self-describing, handles hour-long multichannel
time series with chunking + compression, and is first-class in Python (`h5py`), MATLAB, R,
Julia. Written **incrementally** (resizable datasets + periodic flush) so a crash leaves a
valid file up to the last flush.

## B.2 Schema
```
/                       (root attributes)
   format_version, device_serial, firmware_version,
   start_time_utc (ISO-8601), sample_rate_hz, n_channels, channel_mask,
   adc_bits=24, adc_vref_v, fixed_gain, vs_range_mv, vs_dac_bits,
   operator, experiment_id, notes

/samples                int32 [n_samples, n_active_channels]   (24-bit codes)
   chunked (e.g. 1 s × channels), gzip or lzf compressed
   attrs: channel_ids[], code_to_volts_adc (LSB→V at ADC),
          adc_to_input_referred (÷ fixed_gain)   # code → input-referred volts

/events                 compound table (one row per event)
   sample_index:u64, time_s:f64, type:enum/str, channel:i8,
   a:i32, b:i32, text:vlen-str
   # VS_CHANGE, RI_CHANGE, CAL_PULSE, CLIP, SERVO_STATE, COMMENT, MODE_MARK, ERROR

/channels/<id>          (optional per-channel metadata group)
   attrs: insect_id, species, plant, electrode notes, initial Vs/Ri, ...
```
Optionally store a convenience `/vs_applied` decimated trace, but it is always derivable from
`/events`.

## B.3 Reconstruction conventions (the point of all this)
- **Code → input-referred volts:** `V_in = code × (adc_vref / 2^23) / fixed_gain`.
- **Applied Vs at sample *i*:** `Vs0 + Σ(VS_CHANGE steps with sample_index ≤ i)`.
  - *As-recorded* (servo-corrected) trace = `/samples` directly.
  - *Uncompensated* trace = add the applied-Vs offset back (input-referred). Because the
    keep-in-range servo (D11) only steps Vs occasionally and every step is logged, the true
    continuous signal is always recoverable.
  - *Display* detrend (software high-pass, D11) is a view only — never written into `/samples`.
- **Calibration pulse:** during a CAL_PULSE on→off interval a known ≈ −50 mV is injected at
  the input; the step response gives the scale check and the insect-resistance estimate (per
  the manual's method). Cal intervals are in `/events`.

## B.4 Crash-safety & live capture
Write `/samples` and `/events` incrementally with periodic `flush()` (e.g. every 1–2 s). A
power loss leaves a valid HDF5 readable up to the last flush. (Optional: also keep a raw
append-only frame log alongside as a belt-and-suspenders capture that can be re-finalized.)

## B.5 Versioning & export
- `format_version` (file) and `format_proto_version` (link, in INFO) are independent and
  checked on open.
- **Exports:** CSV (samples + events) for spreadsheets; **NWB** (Neurodata Without Borders,
  HDF5-based) as a future archival/sharing target to align with the electrophysiology
  ecosystem.

---

## Open items
- ~~Exact rate_code table, block size, NACK error codes~~ → defined in A.7. STATUS cadence still open.
- Whether to compress `/samples` with gzip (smaller) vs lzf (faster) by default.
- Confirm ADS131M08 code scaling (FS, ref) against datasheet at implementation.
- Python reference implementation of the framing + reader (lands in `software/`).
