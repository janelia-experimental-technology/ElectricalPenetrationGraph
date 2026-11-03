#include "acq.h"
#include "board.h"
#include "ads131m08.h"
#include <math.h>
#include <hardware/clocks.h>
#include <hardware/sync.h>
#include <pico/time.h>

namespace acq {
namespace {

// ---- ring (single producer = ISR/timer, single consumer = main loop) ----
Sample           ring[EPG_RING_SIZE];
volatile uint32_t r_head = 0, r_tail = 0;
volatile uint64_t g_seq = 0;            // next sample index
volatile uint32_t g_dropped = 0;
volatile bool     g_running = false;

bool  g_present = false;
bool  g_sim = false;
uint8_t  g_mask = 0xFF;
uint16_t g_rate = 1000;

// ISR-side decimation / settling
volatile uint8_t  dec_factor = 1, dec_count = 0, discard = 0;
int32_t dec_acc[8];

inline void push(const int32_t c[8]) {
    uint32_t h = r_head, n = (h + 1) & (EPG_RING_SIZE - 1);
    uint64_t s = g_seq;
    g_seq = s + 1;
    if (n == r_tail) { g_dropped++; return; }      // full: lose sample, keep timeline advancing
    for (int i = 0; i < 8; i++) ring[h].ch[i] = c[i];
    ring[h].seq = (uint32_t)s;
    __compiler_memory_barrier();
    r_head = n;
}

// ================= real ADC path =================
void drdy_isr() {
    int32_t c[8];
    uint16_t st;
    ads.readFrame(c, &st);
    if (!g_running) return;
    if (discard) { discard--; return; }             // filter settling after (re)configuration
    if (dec_factor > 1) {                           // e.g. 250 Hz = 2 x average of 500 Hz frames
        for (int i = 0; i < 8; i++) dec_acc[i] += c[i];
        if (++dec_count < dec_factor) return;
        for (int i = 0; i < 8; i++) { c[i] = dec_acc[i] / (int32_t)dec_factor; dec_acc[i] = 0; }
        dec_count = 0;
    }
    push(c);
}

// OSR field for fMOD = 4.096 MHz (CLKIN 8.192 MHz): ODR = 4.096M / OSR
bool rate_to_osr(uint16_t rate, uint8_t &osr_code, uint8_t &dec) {
    dec = 1;
    switch (rate) {
        case 4000: osr_code = 3; return true;   // OSR 1024
        case 2000: osr_code = 4; return true;   // OSR 2048
        case 1000: osr_code = 5; return true;   // OSR 4096
        case  500: osr_code = 6; return true;   // OSR 8192
        case  250: osr_code = 6; dec = 2; return true;  // OSR 8192 then /2: exact 250 Hz
        default: return false;
    }
}

// ================= simulator path (no ADC fitted) =================
// 8 channels of EPG-like waveform + slow electrode drift, summed with the filtered Vs the
// firmware is really driving into the DAC -- so servo, cal pulse and clipping all behave.
struct SimCh { float phase, drift, drift_rate, vs_f; uint32_t rng; };
SimCh sim[8];
volatile uint16_t vs_out[8];
repeating_timer_t sim_timer;
bool sim_timer_on = false;
float sim_dt = 0.001f;

inline float frand(uint32_t &s) { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return ((s & 0xFFFF) / 32768.0f) - 1.0f; }

bool sim_tick(repeating_timer_t *) {
    if (!g_running) return true;
    const float FS = 8388607.0f;
    const float vref = EPG_ADC_VREF_MV / 1000.0f, gain = EPG_FIXED_GAIN_X100 / 100.0f;
    const float vs_range = EPG_VS_RANGE_MV / 1000.0f;
    const float a = sim_dt / (0.016f + sim_dt);     // 16 ms Vs low-pass on the board
    int32_t c[8];
    for (int i = 0; i < 8; i++) {
        SimCh &s = sim[i];
        float vs_v = (vs_out[i] / 65535.0f) * vs_range - vs_range / 2.0f;
        s.vs_f += a * (vs_v - s.vs_f);
        s.drift += s.drift_rate * sim_dt;
        s.phase += 6.2831853f * (3.0f + 0.5f * i) * sim_dt;
        if (s.phase > 6.2831853f) s.phase -= 6.2831853f;
        float in_v = 0.015f * sinf(s.phase) + s.drift + EPG_VS_POLARITY * s.vs_f + 30e-6f * frand(s.rng);
        float code = in_v * gain / vref * FS;
        if (code > FS) code = FS; else if (code < -FS - 1.0f) code = -FS - 1.0f;
        c[i] = (int32_t)code;
    }
    push(c);
    return true;
}

void sim_reset() {                                   // fresh synthetic session each START
    for (int i = 0; i < 8; i++) {
        sim[i].rng = 0x9E3779B9u * (i + 1);
        sim[i].phase = i * 0.7f;
        sim[i].drift = 0.0f;
        sim[i].drift_rate = ((i & 1) ? -1.0f : 1.0f) * (0.008f + 0.002f * i);   // V/s, exaggerated like the mock
        sim[i].vs_f = 0.0f;
    }
}

} // namespace

// =========================== public API ===========================
bool begin() {
#if EPG_ADC_CLK_FROM_RP2040
    // 48 MHz USB PLL / 5.859375 = 8.192 MHz (5 + 220/256: exact on average).
    clock_gpio_init_int_frac(PIN_ADC_CLKIN, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, 5, 220);
    sleep_ms(2);
#endif
    g_present = ads.begin();
    g_sim = !g_present && EPG_ALLOW_SIM;
    sim_reset();
    for (int i = 0; i < 8; i++) vs_out[i] = 32768;
    return g_present;
}

bool adcPresent() { return g_present; }
uint16_t adcId() { return ads.id(); }
bool canStream() { return g_present || g_sim; }
bool isSimulated() { return g_sim; }
bool running() { return g_running; }

bool start(uint16_t rate_hz, uint8_t ch_mask) {
    stop();
    g_rate = rate_hz; g_mask = ch_mask;
    r_head = r_tail = 0; g_seq = 0; g_dropped = 0;
    dec_count = 0; for (int i = 0; i < 8; i++) dec_acc[i] = 0;

    if (g_present) {
        uint8_t osr, dec;
        if (!rate_to_osr(rate_hz, osr, dec)) return false;
        if (!ads.configure(osr, ch_mask)) return false;
        dec_factor = dec;
        discard = 3;                                // let the digital filter settle
        g_running = true;
        attachInterrupt(digitalPinToInterrupt(PIN_ADC_DRDY), drdy_isr, FALLING);
        irq_set_priority(IO_IRQ_BANK0, 0);          // DRDY outranks everything else
        return true;
    }
    if (g_sim) {
        sim_reset();
        for (int i = 0; i < 8; i++) sim[i].vs_f = ((vs_out[i] / 65535.0f) * (EPG_VS_RANGE_MV / 1000.0f)) - (EPG_VS_RANGE_MV / 2000.0f);
        dec_factor = 1;
        sim_dt = 1.0f / rate_hz;
        g_running = true;
        sim_timer_on = add_repeating_timer_us(-(int64_t)(1000000 / rate_hz), sim_tick, nullptr, &sim_timer);
        if (!sim_timer_on) g_running = false;
        return sim_timer_on;
    }
    return false;
}

void stop() {
    if (g_present) detachInterrupt(digitalPinToInterrupt(PIN_ADC_DRDY));
    if (sim_timer_on) { cancel_repeating_timer(&sim_timer); sim_timer_on = false; }
    g_running = false;
}

bool pop(Sample &out) {
    uint32_t t = r_tail;
    if (t == r_head) return false;
    out = ring[t];
    __compiler_memory_barrier();
    r_tail = (t + 1) & (EPG_RING_SIZE - 1);
    return true;
}

uint32_t ringLevel() { return (r_head - r_tail) & (EPG_RING_SIZE - 1); }

uint64_t index() {
    uint32_t irq = save_and_disable_interrupts();
    uint64_t v = g_seq;
    restore_interrupts(irq);
    return v;
}

uint32_t dropped() { return g_dropped; }
uint32_t crcErrors() { return ads.crcErrors(); }

void setVsOut(uint8_t ch, uint16_t code) { if (ch < 8) vs_out[ch] = code; }

} // namespace acq
