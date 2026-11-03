/* EPG rig motherboard: pin map + build-time configuration (RP2040 / Raspberry Pi Pico H).
 *
 * There is no motherboard PCB yet, so THIS FILE is the pin contract: wire the board to
 * match it, or edit it to match your board. Everything is overridable from platformio.ini
 * with -DEPG_xxx=value.
 */
#ifndef EPG_BOARD_H
#define EPG_BOARD_H

#include <Arduino.h>
#include <hardware/spi.h>
#include <hardware/gpio.h>

// ---- firmware version (reported in INFO) ----
#define EPG_FW_MAJOR 0
#define EPG_FW_MINOR 2
#define EPG_FW_PATCH 0

// ---- pins (Pico GPIO numbers) ----
// SPI0 shared by the ADS131M08 and DAC8568. GP2/GP3/GP4 are the SPI0 SCK/TX/RX group.
static const int PIN_SPI_SCK   = 2;
static const int PIN_SPI_MOSI  = 3;    // Pico TX  -> ADC DIN, DAC DIN
static const int PIN_SPI_MISO  = 4;    // Pico RX  <- ADC DOUT
// ADS131M08 (8-ch 24-bit simultaneous-sampling Sigma-Delta)
static const int PIN_ADC_CS    = 5;
static const int PIN_ADC_DRDY  = 6;    // active-low data-ready
static const int PIN_ADC_RESET = 7;    // active-low
static const int PIN_ADC_CLKIN = 21;   // GPOUT0: 8.192 MHz master clock (see EPG_ADC_CLK_FROM_RP2040)
// DAC8568 (8-ch 16-bit, one Vs per channel)
static const int PIN_DAC_CS    = 8;    // = SYNC
static const int PIN_DAC_LDAC  = 9;    // held low: outputs update on write
// 74HC595 -> per-channel Ri relay-driver control lines (RI_CTRL on each probe cable)
static const int PIN_SR_DATA   = 10;
static const int PIN_SR_CLK    = 11;
static const int PIN_SR_LATCH  = 12;
// Status LED
static const int PIN_STATUS_LED = 25;           // on-board LED (GP25) on the Pico / Pico H

static const int N_CHANNELS    = 8;

// ---- analog chain constants (reported to the host in INFO; the host derives all
//      calibration from these, so keep them equal to the real hardware) ----
#define EPG_ADC_VREF_MV        1200     // ADS131M08 internal reference
#define EPG_FIXED_GAIN_X100    800      // probe FDA gain G = 8.00 (docs D13/D16)
#define EPG_VS_RANGE_MV        1000     // Vs spans +/-500 mV over the 16-bit DAC range
#define EPG_VS_DAC_BITS        16
#define EPG_ADC_BITS           24

// ---- behaviour switches ----
// Master clock for the ADC. 1 = RP2040 generates 8.192 MHz on PIN_ADC_CLKIN from the 48 MHz
// USB PLL via a fractional divider (exact on average, ~20 ns edge jitter: fine for <1 kHz
// signals, good for bring-up). 0 = a dedicated 8.192 MHz oscillator on the board (quieter;
// recommended for the final design) -- firmware then leaves the pin alone.
#ifndef EPG_ADC_CLK_FROM_RP2040
#define EPG_ADC_CLK_FROM_RP2040 1
#endif
// Sign of Vs in the measured input:  measured_in = signal + EPG_VS_POLARITY * Vs.
// The host/mock convention is +1. If your analog stage inverts (the FDA computes S - M, so
// check on the bench), set -1 and the servo + calibration pulse will push the right way.
#ifndef EPG_VS_POLARITY
#define EPG_VS_POLARITY  (+1)
#endif
// Ri line level for "1 GOhm" (relay closed). The probe driver makes a SET pulse on a rising
// edge and a RESET pulse on a falling edge (docs: single logic line -> set/reset pulse).
#ifndef EPG_RI_LEVEL_1G
#define EPG_RI_LEVEL_1G  1
#endif
// Latching relays remember their state across power cycles but the MCU does not, so on boot
// toggle each line to a known state (1 GOhm). Set 0 to leave relays untouched.
#ifndef EPG_RI_RESYNC_ON_BOOT
#define EPG_RI_RESYNC_ON_BOOT 1
#endif
// If no ADS131M08 answers, stream a synthetic 8-channel plant model instead (serial gets a
// "-SIM" suffix and a COMMENT event says so). Lets you test USB + host on a bare Pico.
#ifndef EPG_ALLOW_SIM
#define EPG_ALLOW_SIM 1
#endif
// Skip the ID-register sanity check (bring-up escape hatch if the ID value differs).
#ifndef EPG_FORCE_ADC
#define EPG_FORCE_ADC 0
#endif
// Count output-CRC errors from the ADC (informational; frames are never dropped for it).
#ifndef EPG_ADC_CHECK_CRC
#define EPG_ADC_CHECK_CRC 0
#endif

// ---- streaming ----
#define EPG_BLOCK_MS     50      // target SAMPLES block duration
#define EPG_MAX_ROWS     256     // hard cap on rows per block (buffer sizing)
#define EPG_RING_SIZE    1024    // ADC->main FIFO depth in samples (power of 2)

// ---- Vs servo (docs D11) ----
#define EPG_SERVO_TRACK_THRESH   0.80f   // track: move Vs only if slow baseline passes this fraction of full scale
#define EPG_SERVO_DEADBAND       0.03f   // acquire: "centred" when within this fraction of full scale (~4.5 mV at the input)
#define EPG_SERVO_TAU_TRACK_S    2.0f    // baseline estimator time constant
#define EPG_SERVO_TAU_ACQUIRE_S  0.5f
#define EPG_CAL_AMPL_UV          50000   // calibration pulse: -50 mV at the input
#define EPG_CLIP_ON_FRAC         0.990f  // clip onset (fraction of full scale)
#define EPG_CLIP_OFF_FRAC        0.970f  // clip offset (hysteresis)

// ---- one-time SPI0 bring-up shared by ADC and DAC (both use SPI mode 1) ----
static inline void epg_spi_begin() {
    static bool done = false;
    if (done) return;
    spi_init(spi0, 8 * 1000 * 1000);
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(PIN_SPI_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_MISO, GPIO_FUNC_SPI);
    gpio_pull_down(PIN_SPI_MISO);   // floating bus reads 0 -> "no ADC" is detectable
    done = true;
}

#endif // EPG_BOARD_H
