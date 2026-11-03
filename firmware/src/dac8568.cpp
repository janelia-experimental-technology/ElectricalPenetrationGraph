/* DAC8568 command word (32 bit): [31:28]=0 prefix | [27:24]=control | [23:20]=address |
 * [19:4]=16-bit data | [3:0]=feature.  control 0x3 = write input reg + update DAC n;
 * 0x7 = software reset; 0x8 = internal-reference setup (data bit0 = 1: static enable). */
#include "dac8568.h"
#include "board.h"
#include <hardware/spi.h>
#include <hardware/gpio.h>
#include <hardware/sync.h>

DAC8568 dac;

void DAC8568::write32(uint32_t w) {
    uint8_t b[4] = { (uint8_t)(w >> 24), (uint8_t)(w >> 16), (uint8_t)(w >> 8), (uint8_t)w };
    uint32_t irq = save_and_disable_interrupts();
    gpio_put(PIN_DAC_CS, 0);
    spi_write_blocking(spi0, b, 4);
    gpio_put(PIN_DAC_CS, 1);
    restore_interrupts(irq);
}

void DAC8568::begin() {
    epg_spi_begin();
    gpio_init(PIN_DAC_CS);   gpio_put(PIN_DAC_CS, 1);   gpio_set_dir(PIN_DAC_CS, GPIO_OUT);   // level first: no CS glitch
    gpio_init(PIN_DAC_LDAC); gpio_put(PIN_DAC_LDAC, 0); gpio_set_dir(PIN_DAC_LDAC, GPIO_OUT);
    write32(0x07000000u);                 // software reset
    sleep_ms(1);
    write32(0x08000001u);                 // enable internal reference (static mode)
    sleep_ms(1);
    write32((0x3u << 24) | (0xFu << 20) | ((uint32_t)32768u << 4));   // broadcast: all channels mid-scale
}

void DAC8568::writeChannel(uint8_t ch, uint16_t code) {
    write32((0x3u << 24) | ((uint32_t)(ch & 0xF) << 20) | ((uint32_t)code << 4));
}
