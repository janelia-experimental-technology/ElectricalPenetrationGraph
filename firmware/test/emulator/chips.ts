// Behavioural models of the two SPI chips on the EPG motherboard, attached to the rp2040js
// emulator so the REAL-ADC code path of the firmware can be exercised without hardware.
//   ADS131M08: 24-bit-word frames (resp, 8 ch, crc), register map, DRDY timing from the OSR field
//   DAC8568  : 32-bit command words (write/update, reset, internal-ref setup)
//   plant    : the same 8-channel drift + test-sine model as the firmware's built-in simulator,
//              with Vs taken from what the firmware actually wrote to the DAC
// These models encode MY reading of the datasheets: they catch firmware bugs (framing, ISR, ring
// buffer, decimation, DAC word packing) but cannot prove the datasheet assumptions themselves.
import { RP2040 } from '../src/rp2040.js';
import { GPIOPinState } from '../src/gpio-pin.js';

const PIN = { CS_ADC: 5, DRDY: 6, RESET: 7, CS_DAC: 8 };
const OSR = [128, 256, 512, 1024, 2048, 4096, 8192, 16256];
const FS = 8388607, VREF = 1.2, GAIN = 8.0, VS_RANGE = 1.0;

export function attachChips(mcu: RP2040, log = (s: string) => console.log(s)) {
  const regs: Record<number, number> = {};
  const resetRegs = () => {
    for (const k of Object.keys(regs)) delete regs[+k];
    regs[0x00] = 0x2802; regs[0x01] = 0x0300; regs[0x02] = 0x0510; regs[0x03] = 0x0F0E;
    regs[0x04] = 0; regs[0x05] = 0; regs[0x06] = 0;
  };
  resetRegs();

  // ---- plant ----
  const dac = new Array(8).fill(32768);
  const plant = Array.from({ length: 8 }, (_, i) => ({ phase: i * 0.7, drift: 0, rate: (i & 1 ? -1 : 1) * (0.008 + 0.002 * i), vsf: 0 }));
  const plantReset = () => plant.forEach((p, i) => { p.phase = i * 0.7; p.drift = 0; p.vsf = (dac[i] / 65535) * VS_RANGE - VS_RANGE / 2; });
  let cur = new Array(8).fill(0);
  function convert(dt: number) {
    const a = dt / (0.016 + dt);
    cur = plant.map((p, i) => {
      const vs = (dac[i] / 65535) * VS_RANGE - VS_RANGE / 2;
      p.vsf += a * (vs - p.vsf);
      p.drift += p.rate * dt;
      p.phase += 2 * Math.PI * (3 + 0.5 * i) * dt;
      const inV = 0.015 * Math.sin(p.phase) + p.drift + p.vsf;
      return Math.max(-FS - 1, Math.min(FS, Math.round((inV * GAIN) / VREF * FS)));
    });
  }

  // ---- ADS131M08 ----
  const stats = { frames: 0, badLen: 0, overruns: 0, cmds: {} as Record<string, number>, dacWrites: 0, dacRef: false };
  let csAdc = false, rx: number[] = [], tx: number[] = [], pending = 0xff28 << 8, convSinceRead = 0, framesSinceCfg = 0;
  const w24 = (v: number) => [(v >> 16) & 255, (v >> 8) & 255, v & 255];

  mcu.gpio[PIN.CS_ADC].addListener((state) => {
    if (state === GPIOPinState.Low) {
      csAdc = true; rx = [];
      framesSinceCfg++;
      // conversions overwritten before this read; ignore the first frames after configuration
      // (register verify + filter-settling samples, which the firmware discards anyway)
      if (convSinceRead > 1 && framesSinceCfg > 8) stats.overruns += convSinceRead - 1;
      convSinceRead = 0;
      const en = (regs[0x03] >> 8) & 0xff;
      tx = [...w24(pending), ...cur.flatMap((c, i) => w24(en & (1 << i) ? c & 0xffffff : 0)), 0, 0, 0];
      mcu.gpio[PIN.DRDY].setInputValue(true);       // DRDY returns high once the read starts
    } else if (csAdc) {
      csAdc = false;
      stats.frames++;
      if (rx.length === 0) return;                   // CS pulse without clocks: harmless
      if (rx.length !== 30) { stats.badLen++; log(`ADC model: frame of ${rx.length} bytes (expected 30)`); return; }
      const cmd = (rx[0] << 8) | rx[1], addr = (cmd >> 7) & 0x3f;
      if (cmd === 0x0000) { pending = regs[0x01] << 8; stats.cmds.NULL = (stats.cmds.NULL ?? 0) + 1; }
      else if (cmd === 0x0011) { resetRegs(); pending = 0xff28 << 8; stats.cmds.RESET = (stats.cmds.RESET ?? 0) + 1; }
      else if ((cmd & 0xe000) === 0xa000) { pending = (regs[addr] ?? 0) << 8; stats.cmds.RREG = (stats.cmds.RREG ?? 0) + 1; }
      else if ((cmd & 0xe000) === 0x6000) {
        regs[addr] = (rx[3] << 8) | rx[4];
        if (addr === 0x03) { plantReset(); stats.overruns = 0; convSinceRead = 0; framesSinceCfg = 0; }   // a new acquisition session starts
        pending = (0x4000 | (addr << 7)) << 8; stats.cmds.WREG = (stats.cmds.WREG ?? 0) + 1;
      } else log(`ADC model: unknown command 0x${cmd.toString(16)}`);
    }
  });

  // conversions: ODR = 4.096 MHz / OSR (CLKIN 8.192 MHz); period read live from the CLOCK register
  const alarm = mcu.clock.createAlarm(() => {
    const osr = OSR[(regs[0x03] >> 2) & 7];
    const periodNs = (osr / 4.096e6) * 1e9;
    convert(periodNs * 1e-9);
    convSinceRead++;
    mcu.gpio[PIN.DRDY].setInputValue(false);
    alarm.schedule(periodNs);
  });
  alarm.schedule(1e6);

  // ---- DAC8568 ----
  let csDac = false, dbytes: number[] = [];
  mcu.gpio[PIN.CS_DAC].addListener((state) => {
    if (state === GPIOPinState.Low) { csDac = true; dbytes = []; }
    else if (csDac) {
      csDac = false;
      if (dbytes.length === 0) return;
      if (dbytes.length !== 4) { log(`DAC model: ${dbytes.length}-byte frame`); return; }
      const control = dbytes[0] & 0x0f, addr = dbytes[1] >> 4;
      const data = ((dbytes[1] & 0x0f) << 12) | (dbytes[2] << 4) | (dbytes[3] >> 4);
      if (control === 0x3) { stats.dacWrites++; if (addr === 0xf) dac.fill(data); else if (addr < 8) dac[addr] = data; }
      else if (control === 0x7) dac.fill(0);
      else if (control === 0x8) stats.dacRef = (dbytes[3] & 1) === 1;   // DB0 = 1: static internal reference on
      else log(`DAC model: unhandled control 0x${control.toString(16)}`);
    }
  });

  // ---- SPI byte routing ----
  mcu.spi[0].onTransmit = (v: number) => {
    if (csAdc) { const r = tx[rx.length] ?? 0; rx.push(v); mcu.spi[0].completeTransmit(r); }
    else if (csDac) { dbytes.push(v); mcu.spi[0].completeTransmit(0); }
    else mcu.spi[0].completeTransmit(0);
  };

  return { stats, dac, regs };
}
