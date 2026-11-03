// Run a Pico UF2 in the rp2040js emulator and expose its USB-CDC serial port on TCP.
// Used by ../emu_test.py. See ../../README.md ("Testing the firmware without hardware").
import net from 'net';
import minimist from 'minimist';
import { Simulator } from '../src/simulator.js';
import { USBCDC } from '../src/usb/cdc.js';
import { ConsoleLogger, LogLevel } from '../src/utils/logging.js';
import { bootromB1 } from './bootrom.js';
import { loadUF2 } from './load-flash.js';
import { attachChips } from './chips.js';

const args = minimist(process.argv.slice(2), { string: ['image', 'port'], boolean: ['adc'] });
const simulator = new Simulator();
const mcu = simulator.rp2040;
mcu.loadBootrom(bootromB1);
mcu.logger = new ConsoleLogger(LogLevel.Error);
loadUF2(args.image, mcu);

// --adc: attach behavioural ADS131M08 + DAC8568 models (otherwise the Pico sees no chips)
const chips = args.adc ? attachChips(mcu) : null;
// stats side-channel: connect to --stats-port and read one JSON object (used by emu_test.py)
if (chips) net.createServer((s) => s.end(JSON.stringify(chips.stats))).listen(Number(args['stats-port'] ?? 5557), '127.0.0.1');

const cdc = new USBCDC(mcu.usbCtrl);
let sock: net.Socket | null = null;
cdc.onSerialData = (v) => { if (sock) sock.write(Buffer.from(v)); };
cdc.onDeviceConnected = () => console.log('USB CDC connected');

const server = net.createServer((s) => {
  sock = s;
  s.on('data', (chunk) => { for (const b of chunk) cdc.sendSerialByte(b); });
  s.on('close', () => { sock = null; });
});
server.listen(Number(args.port ?? 5555), '127.0.0.1', () => console.log('listening'));

simulator.rp2040.core.PC = 0x10000000;
simulator.execute();
