/* ADS131M08 driver. Register map / command words per the TI datasheet (SBAS950):
 *   RESET 0x0011, NULL 0x0000, RREG 0xA000|addr<<7|(n-1), WREG 0x6000|addr<<7|(n-1)
 *   ID 0x00, STATUS 0x01, MODE 0x02, CLOCK 0x03, GAIN1 0x04, GAIN2 0x05, CFG 0x06
 *   CLOCK: [15:8]=CH7..CH0 enable, [4:2]=OSR, [1:0]=power mode (10 = high-resolution)
 * Items still worth a datasheet glance at bring-up are marked VERIFY.
 */
#include "ads131m08.h"
#include "board.h"
#include <hardware/spi.h>
#include <hardware/gpio.h>

ADS131M08 ads;

namespace {
enum : uint8_t { REG_ID = 0x00, REG_STATUS = 0x01, REG_MODE = 0x02, REG_CLOCK = 0x03,
                 REG_GAIN1 = 0x04, REG_GAIN2 = 0x05, REG_CFG = 0x06 };
const uint16_t CMD_NULL  = 0x0000;
const uint16_t CMD_RESET = 0x0011;
inline uint16_t cmd_rreg(uint8_t a, uint8_t n) { return 0xA000 | ((uint16_t)a << 7) | (n - 1); }
inline uint16_t cmd_wreg(uint8_t a, uint8_t n) { return 0x6000 | ((uint16_t)a << 7) | (n - 1); }

// MODE: 24-bit words (WLENGTH=01), DRDY = active-low level, input CRC off, timeout on. VERIFY
const uint16_t MODE_VALUE = 0x0110;
// CLOCK power mode field: 10 = high-resolution (needs fCLKIN up to 8.192 MHz)
const uint16_t CLOCK_PWR_HR = 0x0002;

const uint8_t ZERO30[30] = {0};

inline int32_t sext24(uint32_t w) {
    return (w & 0x800000) ? (int32_t)(w | 0xFF000000u) : (int32_t)w;
}

#if EPG_ADC_CHECK_CRC
// ADS131M08 output CRC: CCITT, poly 0x1021, init 0xFFFF, over the 9 preceding words. VERIFY
uint16_t crc_ccitt_ffff(const uint8_t *d, size_t n) {
    uint16_t c = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        c ^= (uint16_t)d[i] << 8;
        for (int b = 0; b < 8; b++) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
    }
    return c;
}
#endif
} // namespace

void ADS131M08::frame(const uint32_t tx[10], uint32_t rx[10]) {
    uint8_t t[30], r[30];
    for (int i = 0; i < 10; i++) {
        t[3 * i]     = (tx[i] >> 16) & 0xFF;
        t[3 * i + 1] = (tx[i] >> 8) & 0xFF;
        t[3 * i + 2] = tx[i] & 0xFF;
    }
    gpio_put(PIN_ADC_CS, 0);
    spi_write_read_blocking(spi0, t, r, 30);
    gpio_put(PIN_ADC_CS, 1);
    for (int i = 0; i < 10; i++)
        rx[i] = ((uint32_t)r[3 * i] << 16) | ((uint32_t)r[3 * i + 1] << 8) | r[3 * i + 2];
}

void ADS131M08::readFrame(int32_t ch[8], uint16_t *status) {
    uint8_t r[30];
    gpio_put(PIN_ADC_CS, 0);
    spi_write_read_blocking(spi0, ZERO30, r, 30);   // all-NULL command frame
    gpio_put(PIN_ADC_CS, 1);
    if (status) *status = (uint16_t)(((uint32_t)r[0] << 8) | r[1]);
    for (int i = 0; i < 8; i++) {
        const uint8_t *p = &r[3 * (i + 1)];
        ch[i] = sext24(((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2]);
    }
#if EPG_ADC_CHECK_CRC
    uint16_t got = (uint16_t)(((uint32_t)r[27] << 8) | r[28]);
    if (got != crc_ccitt_ffff(r, 27)) _crcErr++;
#endif
}

uint16_t ADS131M08::readReg(uint8_t addr) {
    uint32_t tx[10] = {0}, rx[10];
    tx[0] = (uint32_t)cmd_rreg(addr, 1) << 8;       // 16-bit command, left-justified in 24-bit word
    frame(tx, rx);                                  // frame 1: send command
    tx[0] = (uint32_t)CMD_NULL << 8;
    frame(tx, rx);                                  // frame 2: response word = register value
    return (uint16_t)(rx[0] >> 8);
}

bool ADS131M08::writeReg(uint8_t addr, uint16_t value) {
    uint32_t tx[10] = {0}, rx[10];
    tx[0] = (uint32_t)cmd_wreg(addr, 1) << 8;
    tx[1] = (uint32_t)value << 8;
    frame(tx, rx);
    tx[0] = tx[1] = 0;
    frame(tx, rx);                                  // flush the ack
    return readReg(addr) == value;
}

bool ADS131M08::begin() {
    epg_spi_begin();
    gpio_init(PIN_ADC_CS);    gpio_put(PIN_ADC_CS, 1);    gpio_set_dir(PIN_ADC_CS, GPIO_OUT);   // level first: no CS glitch
    gpio_init(PIN_ADC_RESET); gpio_put(PIN_ADC_RESET, 1); gpio_set_dir(PIN_ADC_RESET, GPIO_OUT);
    gpio_init(PIN_ADC_DRDY);  gpio_set_dir(PIN_ADC_DRDY, GPIO_IN);   gpio_pull_up(PIN_ADC_DRDY);

    // Hardware reset (needs CLKIN already running -- acq::begin() starts it first).
    gpio_put(PIN_ADC_RESET, 0); sleep_ms(2);
    gpio_put(PIN_ADC_RESET, 1); sleep_ms(20);

    _id = readReg(REG_ID);
    // Upper byte of ID encodes channel count in its low nibble (0x28xx for the M08). VERIFY
    bool ok = ((_id & 0x0F00) == 0x0800) && _id != 0xFFFF;
#if EPG_FORCE_ADC
    ok = true;
#endif
    _present = ok;
    if (!_present) return false;

    // Base configuration: 24-bit words, PGA gain 1 on all channels (the +/-1.2 V range maps to
    // +/-150 mV at the probe input via the fixed G=8), default 1 kSPS until START picks a rate.
    bool good = writeReg(REG_MODE, MODE_VALUE);
    good &= writeReg(REG_GAIN1, 0x0000);
    good &= writeReg(REG_GAIN2, 0x0000);
    good &= configure(5, 0xFF);                     // OSR 4096 -> 1 kSPS @ 8.192 MHz CLKIN
    _present = good;
    return good;
}

bool ADS131M08::configure(uint8_t osr_code, uint8_t ch_mask) {
    uint16_t v = ((uint16_t)ch_mask << 8) | ((uint16_t)(osr_code & 7) << 2) | CLOCK_PWR_HR;
    return writeReg(REG_CLOCK, v);
}
