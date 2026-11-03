/* ADS131M08 driver (8-ch, 24-bit, simultaneous-sampling Sigma-Delta), pico-sdk SPI.
 *
 * Uses the SDK SPI calls directly (not the Arduino SPI class) so readFrame() is safe to call
 * from the DRDY interrupt. Frame format used throughout: 24-bit words, 10 words per frame:
 *   in : COMMAND, 8x 0 (channel slots), 0 (CRC slot)
 *   out: RESPONSE, CH0..CH7, CRC          (RESPONSE to a NULL command = STATUS register)
 * A register read/write response arrives in the NEXT frame's first word.
 */
#ifndef ADS131M08_H
#define ADS131M08_H

#include <Arduino.h>
#include <stdint.h>

class ADS131M08 {
public:
    // Reset the part and read its ID. Returns true if an ADS131M08 answered.
    bool begin();
    bool present() const { return _present; }
    uint16_t id() const { return _id; }

    // Select output rate + enabled channels. osr_code is the CLOCK-register OSR field
    // (0=128 ... 7=16256); ch_mask bit n enables channel n. Verified by read-back.
    bool configure(uint8_t osr_code, uint8_t ch_mask);

    // Read one conversion frame. ISR-safe. status = STATUS register word (16 bit).
    // Never drops data; with EPG_ADC_CHECK_CRC=1 bad output CRCs are only counted (crcErrors()).
    void readFrame(int32_t ch[8], uint16_t *status = nullptr);

    uint16_t readReg(uint8_t addr);
    bool writeReg(uint8_t addr, uint16_t value);   // write + read-back verify

    uint32_t crcErrors() const { return _crcErr; }

private:
    bool _present = false;
    uint16_t _id = 0;
    volatile uint32_t _crcErr = 0;
    void frame(const uint32_t tx[10], uint32_t rx[10]);
};

extern ADS131M08 ads;

#endif
