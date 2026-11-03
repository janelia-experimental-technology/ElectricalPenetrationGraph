# EPG rig — RP2040 firmware (Raspberry Pi Pico H)

Device-side firmware: streams the ADS131M08's 8 channels to the Python host over USB, runs the
Vs servo, drives the Vs DAC (DAC8568) and the Ri relays, and logs every state change as a
sample-stamped EVENT. It speaks exactly the protocol in
[../docs/protocol-and-data-format.md](../docs/protocol-and-data-format.md), so the existing
`software/` package works with it unchanged.

```
firmware/
  proto/        epg_proto.{h,c}   portable C codec (COBS+CRC, builders, parser) — shared with the PC tests
  src/          board.h           PIN MAP + all build-time options  (the motherboard contract)
                acq.{h,cpp}       DRDY interrupt -> ring buffer; built-in simulator when no ADC
                ads131m08.*       ADC driver        dac8568.*   Vs DAC driver     relays.h  Ri select (74HC595)
                main.cpp          commands, Vs servo, cal pulse, clip detect, SAMPLES streaming
  test/         test_proto.c, interop_test.py       codec tests (PC)
                emu_test.py, emulator/              whole-firmware tests in an RP2040 emulator
  prebuilt/     epg-firmware-<ver>-pico.uf2         ready to flash — no toolchain needed
  platformio.ini
```

## Quick start (about 5 minutes, bare Pico — no board needed)

1. **Flash.** Hold the **BOOTSEL** button, plug the Pico into USB, release. A drive `RPI-RP2`
   appears. Copy `prebuilt/epg-firmware-0.2.0-pico.uf2` onto it; the Pico reboots on its own.
   (Or build it yourself: `pip install platformio`, then `pio run -t upload`.)
2. **Find the port.** Windows: Device Manager → Ports → "USB Serial Device (COMx)".
   `python software/scripts/record_device.py --list-ports` also lists it.
3. **Record.**
   ```
   cd software
   pip install -r requirements.txt
   python scripts/record_device.py C:\temp\peg_dev.h5 --seconds 10 --rate 1000
   ```
   With no ADC fitted the firmware streams a **synthetic** plant (the LED blinks fast, the
   device serial ends in `-SIM`, and the HDF5 file says so). That proves the whole chain:
   USB → protocol → recorder → reconstruct. The final lines should read
   `OK: real device -> protocol -> HDF5 -> reconstruct.`
4. **Live GUI** (needs Qt): `python scripts/run_gui.py --port` (auto-detects) or `--port COM5`.

## Wiring the real board

`src/board.h` is the pin contract (there is no motherboard PCB yet — build it to this or edit it):

| Function | Pico pin | Notes |
|---|---|---|
| SPI0 SCK / MOSI / MISO | GP2 / GP3 / GP4 | shared by ADC + DAC, SPI mode 1, 8 MHz |
| ADS131M08 CS / DRDY / RESET | GP5 / GP6 / GP7 | DRDY falling edge triggers the read |
| ADS131M08 CLKIN | GP21 | RP2040 makes 8.192 MHz here, or fit an oscillator (below) |
| DAC8568 SYNC / LDAC | GP8 / GP9 | LDAC held low |
| 74HC595 DATA / CLK / LATCH | GP10 / GP11 / GP12 | Q0..Q7 = RI_CTRL of channels 0..7 |
| LED | GP25 | idle: 1 Hz blink · streaming: solid · simulated data: fast blink |

The ADC rates come from a CLKIN of **8.192 MHz** (ODR = 4.096 MHz / OSR): 4000/2000/1000/500 Hz
are OSR 1024/2048/4096/8192; 250 Hz is OSR 8192 averaged ×2 (exactly 250 Hz). By default the
RP2040 generates CLKIN from its 48 MHz USB PLL (÷5.859375, exact on average, ~20 ns edge
jitter — fine for a <1 kHz signal and for bring-up). For the final board, fit a dedicated
8.192 MHz oscillator and build with `-DEPG_ADC_CLK_FROM_RP2040=0`.

## What the firmware does

* **Acquisition:** the DRDY interrupt reads each ADC frame immediately into a 1024-sample ring
  buffer, so USB stalls cannot cost samples; the main loop drains it into 50 ms `SAMPLES`
  blocks. `sample_index` (the master timeline) counts samples since START; a ring overflow or
  USB drop is reported as an `ERROR` event, never silently.
* **Vs servo (D11), per channel:** `SERVO` modes — *off*; *track* (move Vs only when the slow
  baseline passes 80 % of full scale; every step is a logged `VS_CHANGE`); *acquire* (walk the
  baseline to the target, then hand over to *track* and log a `SERVO_STATE`). `SERVO` also takes
  `target` / `deadband` (offset-binary 16-bit; 0 = default) and a `freeze` flag. Manual `SET_VS`
  always wins and is logged too.
* **Calibration pulse:** `CAL_PULSE` shifts Vs by −50 mV for `dur_ms` (0 = until switched off),
  logged as `CAL_PULSE` on/off events (not as Vs changes); the servo is paused around it.
* **Ri select:** `SET_RI` drives the channel's RI_CTRL level (1 GΩ = high); the probe's latching
  relay driver pulses the coil on the edge. Relays are forced to 1 GΩ at boot.
* **Self-describing recordings:** at START the firmware emits a `COMMENT` (firmware version,
  ADC present/simulated, rate) plus each channel's Vs / Ri / servo state at index 0.
* **Errors:** a command that cannot be done gets a `NACK` with a reason (bad channel, busy,
  no ADC, …) which `SerialDevice` raises as `DeviceError`.

## Tests (no hardware)

```bash
cd test
make check       # C codec self-test (incl. a 4.8 kB sample block)
make interop     # C <-> Python wire compatibility, both directions
```

**Whole-firmware tests.** The real `firmware.uf2` runs in the
[rp2040js](https://github.com/wokwi/rp2040js) emulator, whose USB serial port is exposed on a TCP
socket; `emu_test.py` then drives it with the real Python host code. `--adc` attaches behavioural
models of the ADS131M08 and DAC8568 (`emulator/chips.ts`) so the **real-ADC path** — SPI driver,
interrupt, ring buffer, decimation, DAC word packing — is exercised too.

```bash
git clone https://github.com/wokwi/rp2040js && cd rp2040js && npm install
cp <repo>/firmware/test/emulator/{run_fw.ts,chips.ts} demo/
npx tsx demo/run_fw.ts --image <repo>/firmware/.pio/build/pico/firmware.uf2 --port 5555 --adc
# in another terminal:
EPG_EMU_ADC=1 python3 firmware/test/emu_test.py        # (omit --adc and EPG_EMU_ADC for the simulator path)
```
It checks, among other things: NACK paths; the manual +20 mV step and the −50 mV cal dip as seen
in the data (to ~0.1 mV); that the **true drift is recovered from the event log** (within 5 %,
usually 0.1 %) for servo off / track / acquire; hand-over from acquire to track; and, at every rate, that a known
3 Hz test tone lands at 3.00 Hz with no lost samples (this catches wrong ODR/decimation).

## Bring-up checklist — what has NOT been verified on silicon

The emulator models encode my reading of the datasheets, so they catch firmware bugs but cannot
prove those readings. On first power-up with the real chips, check (all marked `VERIFY` in code):

1. **ADC found?** If the device serial still ends in `-SIM` with the ADC wired, the ID check
   failed: confirm SPI mode 1 / wiring / CLKIN present on GP21, or build with `-DEPG_FORCE_ADC=1`
   and read `id=` in the start `COMMENT` event (expected `0x28xx` for the ADS131M08).
2. **ADC registers:** MODE `0x0110` (24-bit words, DRDY level), CLOCK OSR field values, and that
   the 250/500/1000/2000/4000 Hz settings really give those rates (record a known tone).
   `-DEPG_ADC_CHECK_CRC=1` counts output-CRC mismatches.
3. **DAC8568:** SPI mode, internal-reference enable, mid-scale at power-up, and that code 32768
   gives 0 V at the probe's Vs input.
4. **Vs polarity:** the FDA computes S − M, so the sign of Vs in the measured signal must be
   checked on the bench. If a positive Vs step makes the trace go *down*, build with
   `-DEPG_VS_POLARITY=-1` (servo and cal pulse then push the right way).
5. **Relay edge polarity** (`EPG_RI_LEVEL_1G`) and the 74HC595 chain order.
6. **Clock jitter:** compare noise floors with `EPG_ADC_CLK_FROM_RP2040=1` vs an oscillator.
7. **Servo constants** in `board.h` (`EPG_SERVO_TRACK_THRESH` etc.) against real electrode drift.

## Known limits

* A recording that starts *mid-stream* (GUI "Record" pressed while already streaming) does not
  contain the Vs/Ri/servo snapshot emitted at START, so the reader assumes Vs = 0 at its first
  sample. For exact reconstruction record from START (as `record_device.py` does); a host-side
  "request state snapshot" is a good follow-up.
* `STATUS` heartbeat frames are not sent (`PING` is answered with `ACK`, like the mock).
