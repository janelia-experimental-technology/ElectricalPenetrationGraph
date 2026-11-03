/* DAC8568 driver (8-ch, 16-bit) for the per-channel Vs supply. Shares SPI0 with the ADC;
 * writes briefly mask interrupts so they can never interleave with the ADC's DRDY ISR. */
#ifndef DAC8568_H
#define DAC8568_H

#include <Arduino.h>
#include <stdint.h>

class DAC8568 {
public:
    void begin();                                   // reset, enable internal ref, all outputs mid-scale
    void writeChannel(uint8_t ch, uint16_t code);   // write input register + update output
private:
    void write32(uint32_t w);
};

extern DAC8568 dac;

#endif
