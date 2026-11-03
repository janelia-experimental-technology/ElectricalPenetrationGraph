/* Acquisition core: ADC (or simulator) -> interrupt-filled ring buffer -> main loop.
 *
 * The DRDY interrupt reads the ADC immediately (so USB or host stalls can never cost a
 * sample) and pushes frames into a 1024-deep ring. The main loop drains the ring.
 * sample index = number of samples produced since start() (the protocol's master timeline).
 */
#ifndef EPG_ACQ_H
#define EPG_ACQ_H

#include <Arduino.h>
#include <stdint.h>

namespace acq {

struct Sample {
    int32_t  ch[8];     // signed 24-bit codes, all 8 channels (sign-extended)
    uint32_t seq;       // low 32 bits of the sample index
};

// Start the ADC master clock, reset + probe the ADC. Returns true if a real ADC is present.
bool begin();
bool adcPresent();
uint16_t adcId();

// True if this build/hardware can stream (real ADC, or simulator allowed).
bool canStream();
bool isSimulated();

// Begin streaming at rate_hz (250/500/1000/2000/4000) with the given channel enable mask.
bool start(uint16_t rate_hz, uint8_t ch_mask);
void stop();
bool running();

// Main-loop side of the ring. Returns false when empty.
bool pop(Sample &out);
uint32_t ringLevel();

// Index of the NEXT sample that will be produced (interrupt-safe 64-bit read). Event stamps.
uint64_t index();
uint32_t dropped();      // ring overflows since start()
uint32_t crcErrors();    // ADC output CRC mismatches (only if EPG_ADC_CHECK_CRC)

// Tell the simulated plant what the Vs DAC is actually outputting (no-op for real ADC).
void setVsOut(uint8_t ch, uint16_t dac_code);

} // namespace acq

#endif
