/* Ri (1 GOhm / 10 TOhm) select lines: one logic level per channel through a 74HC595.
 * Each probe has a latching-relay driver that makes a SET pulse on a rising edge of its
 * RI_CTRL line and a RESET pulse on a falling edge, so the relay coil is only energised on
 * change. Bit n of the shift register = channel n. */
#ifndef EPG_RELAYS_H
#define EPG_RELAYS_H

#include "board.h"
#include "epg_proto.h"

class RiRelays {
public:
    void begin() {
        gpio_init(PIN_SR_DATA);  gpio_set_dir(PIN_SR_DATA, GPIO_OUT);  gpio_put(PIN_SR_DATA, 0);
        gpio_init(PIN_SR_CLK);   gpio_set_dir(PIN_SR_CLK, GPIO_OUT);   gpio_put(PIN_SR_CLK, 0);
        gpio_init(PIN_SR_LATCH); gpio_set_dir(PIN_SR_LATCH, GPIO_OUT); gpio_put(PIN_SR_LATCH, 0);
        _bits = 0;
        for (int c = 0; c < N_CHANNELS; c++) _mode[c] = RI_1G;
        shiftOut();
#if EPG_RI_RESYNC_ON_BOOT
        // Relay state is unknown after power-up: force a known 1 GOhm state, one channel at a
        // time (limits coil inrush). Opposite level first, then the target, so the driver
        // always sees at least one edge.
        for (int c = 0; c < N_CHANNELS; c++) {
            set(c, RI_10T); sleep_ms(25);
            set(c, RI_1G);  sleep_ms(25);
        }
#endif
    }
    void set(uint8_t ch, uint8_t mode) {
        if (ch >= N_CHANNELS) return;
        _mode[ch] = mode;
        bool level = (mode == RI_1G) ? (EPG_RI_LEVEL_1G != 0) : (EPG_RI_LEVEL_1G == 0);
        if (level) _bits |= (1u << ch); else _bits &= ~(1u << ch);
        shiftOut();
    }
    uint8_t mode(uint8_t ch) const { return _mode[ch]; }
private:
    uint8_t _bits = 0;
    uint8_t _mode[N_CHANNELS];
    void shiftOut() {
        for (int i = 7; i >= 0; i--) {            // Q7 first -> ends up at output QH (ch7)
            gpio_put(PIN_SR_DATA, (_bits >> i) & 1);
            busy_wait_us_32(1);
            gpio_put(PIN_SR_CLK, 1); busy_wait_us_32(1);
            gpio_put(PIN_SR_CLK, 0);
        }
        gpio_put(PIN_SR_LATCH, 1); busy_wait_us_32(1);
        gpio_put(PIN_SR_LATCH, 0);
    }
};

#endif
