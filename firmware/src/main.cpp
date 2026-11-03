/* EPG rig -- RP2040 (Raspberry Pi Pico H) firmware v0.2.
 *
 * Speaks the device side of the protocol in ../proto/epg_proto.{h,c} over USB-CDC; that codec is
 * unit-tested on a PC and cross-checked byte-for-byte against software/epgrig/protocol.py,
 * so this talks to the existing Python host unchanged.
 *
 *   ADS131M08 --DRDY IRQ--> ring buffer --> [main loop] --> Vs servo, clip detect
 *                                                       --> SAMPLES blocks over USB
 *   host commands --> DAC8568 (Vs), relays (Ri), cal pulse, servo mode
 *   every state change -> EVENT stamped with the sample index at which it took effect
 *
 * If no ADC answers on SPI the firmware runs a synthetic plant model instead, so the USB link
 * and the whole host pipeline can be tested on a bare Pico (serial is suffixed "-SIM").
 */
#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <pico/unique_id.h>
extern "C" {
#include "epg_proto.h"
}
#include "board.h"
#include "acq.h"
#include "ads131m08.h"
#include "dac8568.h"
#include "relays.h"

// ---------------------------------------------------------------- constants
static const uint16_t RATES[5] = {250, 500, 1000, 2000, 4000};
static const float FS = 8388607.0f;
static const float VREF_V   = EPG_ADC_VREF_MV / 1000.0f;
static const float GAIN     = EPG_FIXED_GAIN_X100 / 100.0f;
static const float VS_RANGE = EPG_VS_RANGE_MV / 1000.0f;
static const float DAC_MAX  = 65535.0f;

// NACK error codes (documented in docs/protocol-and-data-format.md)
enum { ERR_UNKNOWN_CMD = 1, ERR_BAD_PAYLOAD = 2, ERR_BUSY = 3, ERR_BAD_CHANNEL = 4,
       ERR_BAD_VALUE = 5, ERR_NO_ADC = 6, ERR_ADC_CONFIG = 7 };
// EV_ERROR codes (event.a)
enum { EVERR_GAP = 1, EVERR_TX_DROP = 2, EVERR_VS_RAIL = 3, EVERR_ADC_RESET = 4 };

// ---------------------------------------------------------------- state
static epg_info_t INFO;
static char       serial_str[24];
static epg_parser_t parser;
static RiRelays   relays;

struct Chan {
    uint16_t vs_base;          // commanded Vs DAC code (calibration pulse is added on top)
    uint8_t  servo_mode;
    bool     freeze;
    int32_t  target, deadband; // ADC codes
    float    baseline;         // slow estimate of the channel's DC level (ADC codes)
    bool     base_init;
    uint32_t hold_until;       // ms: servo may not act before this
    uint8_t  settle;
    bool     cal_active, cal_timed;
    uint32_t cal_until;
    bool     clipped;
};
static Chan chs[N_CHANNELS];

static uint8_t  rate_code = 2;                 // 1000 Hz default
static uint8_t  channel_mask = 0xFF;
static bool     streaming = false;

// stream / block assembly
static uint64_t proc_idx = 0;                  // index of the next sample the main loop will process
static uint32_t block_seq = 0;
static uint16_t rows_per_block = 50;
static uint16_t blk_rows = 0;
static uint64_t blk_first = 0;
static uint8_t  blk_status = 0;
static uint32_t servo_div = 20, servo_tick = 0;
static float    alpha_track = 0.0005f, alpha_acquire = 0.004f;
static uint32_t last_tx_drop = 0, last_dropped = 0, last_err_ms = 0;
static uint32_t tx_drops = 0;

static uint8_t  blk_pay[17 + EPG_MAX_ROWS * N_CHANNELS * 3];
static uint8_t  txbuf[EPG_FRAME_MAX(sizeof(blk_pay))];
static int32_t  blk_rows_buf[EPG_MAX_ROWS][N_CHANNELS];

// ---------------------------------------------------------------- USB output
static bool usb_write(const uint8_t *buf, size_t n) {
    uint32_t t0 = millis();
    size_t off = 0;
    while (off < n) {
        int av = Serial.availableForWrite();
        if (av > 0) {
            size_t k = (size_t)av < n - off ? (size_t)av : n - off;
            size_t w = Serial.write(buf + off, k);
            if (w > 0) { off += w; t0 = millis(); continue; }
        }
        if (millis() - t0 > 200) return false;      // host not draining: give up on this frame
        yield();
    }
    return true;
}

static bool send_frame(uint8_t type, const uint8_t *payload, size_t plen) {
    size_t fl = epg_encode_frame(type, 0, payload, plen, txbuf);
    bool ok = usb_write(txbuf, fl);
    if (!ok) tx_drops++;
    return ok;
}

static void send_ack(uint8_t cmd)               { uint8_t p[2] = {cmd, 0};      send_frame(T_ACK, p, 2); }
static void send_nack(uint8_t cmd, uint8_t err) { uint8_t p[3] = {cmd, 0, err}; send_frame(T_NACK, p, 3); }

static void emit_event_at(uint64_t idx, uint8_t etype, uint8_t ch, int32_t a, int32_t b, const char *text) {
    if (!streaming) return;
    uint8_t pay[96];
    uint8_t tl = 0;
    while (text && text[tl] && tl < 60) tl++;
    size_t pl = epg_build_event(etype, ch, idx, a, b, text, tl, pay);
    send_frame(T_EVENT, pay, pl);
}
static void emit_event(uint8_t etype, uint8_t ch, int32_t a, int32_t b, const char *text) {
    emit_event_at(acq::index(), etype, ch, a, b, text);
}

// ---------------------------------------------------------------- Vs / Ri / cal
static inline float codes_per_vs_volt() { return DAC_MAX / VS_RANGE; }

// DAC code actually driven = commanded Vs + (cal pulse offset while a pulse is active)
static void apply_dac(uint8_t ch) {
    int32_t code = chs[ch].vs_base;
    if (chs[ch].cal_active)
        code += (int32_t)lroundf(-(EPG_CAL_AMPL_UV * 1e-6f) * EPG_VS_POLARITY * codes_per_vs_volt());
    if (code < 0) code = 0; else if (code > 65535) code = 65535;
    dac.writeChannel(ch, (uint16_t)code);
    acq::setVsOut(ch, (uint16_t)code);
}

// Change the commanded Vs. Emits VS_CHANGE(old,new,reason) and keeps the servo's baseline
// estimate consistent with the step it just caused.
static void set_vs(uint8_t ch, uint16_t code, const char *reason) {
    Chan &k = chs[ch];
    uint16_t old = k.vs_base;
    k.vs_base = code;
    apply_dac(ch);
    emit_event(EV_VS_CHANGE, ch, old, code, reason);
    float shift = EPG_VS_POLARITY * ((int32_t)code - (int32_t)old) / codes_per_vs_volt() * GAIN / VREF_V * FS;
    k.baseline += shift;
    k.hold_until = millis() + 1500;
}

static void set_servo(uint8_t ch, uint8_t mode, bool freeze) {
    Chan &k = chs[ch];
    bool changed = (k.servo_mode != mode) || (k.freeze != freeze);
    k.servo_mode = mode; k.freeze = freeze; k.settle = 0;
    if (changed) emit_event(EV_SERVO_STATE, ch, mode, freeze ? 1 : 0, "");
}

static void start_cal(uint8_t ch, uint16_t dur_ms) {
    Chan &k = chs[ch];
    k.cal_active = true;
    k.cal_timed = dur_ms != 0;
    k.cal_until = millis() + dur_ms;
    apply_dac(ch);
    emit_event(EV_CAL_PULSE, ch, 1, -(int32_t)EPG_CAL_AMPL_UV, "");
}
static void end_cal(uint8_t ch) {
    Chan &k = chs[ch];
    if (!k.cal_active) return;
    k.cal_active = false;
    apply_dac(ch);
    emit_event(EV_CAL_PULSE, ch, 0, -(int32_t)EPG_CAL_AMPL_UV, "");
    k.hold_until = millis() + 500;                 // let the 10 Hz Vs filter settle
}

// ---------------------------------------------------------------- Vs servo (docs D11)
// Baseline = slow EMA of each channel's output code. TRACK: only move Vs when the baseline nears
// the ADC rail (keep-in-range). ACQUIRE: walk the baseline to `target`, then hand over to TRACK.
static void servo_decide(uint8_t c, uint32_t now) {
    Chan &k = chs[c];
    if (k.servo_mode == SERVO_OFF || k.freeze || k.cal_active || (int32_t)(now - k.hold_until) < 0) return;
    bool acquire = (k.servo_mode == SERVO_ACQUIRE);
    float err = k.baseline - (float)k.target;
    if (acquire) {
        float ae = fabsf(err), db = (float)k.deadband;
        if (ae <= db) {                                       // centred: count ~1 s, then hand over to TRACK
            if (++k.settle >= 50) { k.settle = 0; set_servo(c, SERVO_TRACK, false); }
            return;
        }
        if (ae > 2.0f * db) k.settle = 0;                     // clearly off-centre: restart the count
        // (between 1x and 2x deadband the count is held, but we still step back towards target)
    } else if (fabsf(k.baseline) <= EPG_SERVO_TRACK_THRESH * FS) {
        return;
    }
    float d_in_v = -(err / FS) * (VREF_V / GAIN);                         // input-referred volts to add
    int32_t d = (int32_t)lroundf(EPG_VS_POLARITY * d_in_v * codes_per_vs_volt());
    if (d == 0) d = (err > 0) ? -EPG_VS_POLARITY : EPG_VS_POLARITY;
    int32_t nb = (int32_t)k.vs_base + d;
    if (nb < 0) nb = 0; else if (nb > 65535) nb = 65535;
    if (nb == (int32_t)k.vs_base) {                                       // Vs already at its limit
        if (now - last_err_ms > 2000) { emit_event(EV_ERROR, c, EVERR_VS_RAIL, 0, "vs-rail"); last_err_ms = now; }
        k.hold_until = now + 1000;
        return;
    }
    set_vs(c, (uint16_t)nb, acquire ? "acquire" : "track");
    k.hold_until = now + (acquire ? 150 : 1500);
}

// ---------------------------------------------------------------- sample path
static void flush_block() {
    if (blk_rows == 0) return;
    size_t pl = epg_build_sampleblock_header(block_seq, blk_first, rate_code, channel_mask,
                                             blk_rows, blk_status, blk_pay);
    for (uint16_t r = 0; r < blk_rows; r++)
        for (int c = 0; c < N_CHANNELS; c++)
            if (channel_mask & (1u << c)) { epg_pack_i24(blk_rows_buf[r][c], blk_pay + pl); pl += 3; }
    send_frame(T_SAMPLES, blk_pay, pl);            // a failed write is counted and reported below
    block_seq++;
    blk_rows = 0; blk_status = 0;
}

static void process_sample(const acq::Sample &s) {
    uint32_t diff = s.seq - (uint32_t)proc_idx;    // samples lost in the ring (rare)
    if (diff) {
        flush_block();
        emit_event_at(proc_idx, EV_ERROR, CHAN_DEVICE, EVERR_GAP, (int32_t)diff, "gap");
        proc_idx += diff;
    }
    uint32_t now = millis();
    if (blk_rows == 0) blk_first = proc_idx;
    const int32_t on  = (int32_t)(EPG_CLIP_ON_FRAC * FS), off = (int32_t)(EPG_CLIP_OFF_FRAC * FS);
    bool decide = (++servo_tick >= servo_div);
    if (decide) servo_tick = 0;

    for (int c = 0; c < N_CHANNELS; c++) {
        if (!(channel_mask & (1u << c))) continue;
        Chan &k = chs[c];
        int32_t code = s.ch[c];
        int32_t mag = code < 0 ? -code : code;
        if (!k.clipped && mag >= on)      { k.clipped = true;  emit_event_at(proc_idx, EV_CLIP, c, 1, 0, ""); }
        else if (k.clipped && mag < off)  { k.clipped = false; emit_event_at(proc_idx, EV_CLIP, c, 0, 0, ""); }
        if (k.clipped) blk_status |= 1;

        if (!k.cal_active) {                                    // baseline ignores the cal pulse
            if (!k.base_init) { k.baseline = (float)code; k.base_init = true; }
            else k.baseline += (k.servo_mode == SERVO_ACQUIRE ? alpha_acquire : alpha_track) * ((float)code - k.baseline);
            if (decide) servo_decide(c, now);
        }
        blk_rows_buf[blk_rows][c] = code;
    }
    blk_rows++; proc_idx++;
    if (blk_rows >= rows_per_block) flush_block();
}

// ---------------------------------------------------------------- commands
static void snapshot_events() {                    // makes every recording self-contained
    char txt[64];
    snprintf(txt, sizeof txt, "fw%d.%d.%d adc=%s id=%04X %uHz", EPG_FW_MAJOR, EPG_FW_MINOR, EPG_FW_PATCH,
             acq::isSimulated() ? "SIMULATED" : "ok", acq::adcId(), (unsigned)RATES[rate_code]);
    emit_event_at(0, EV_COMMENT, CHAN_DEVICE, 0, 0, txt);
    for (int c = 0; c < N_CHANNELS; c++) {
        if (!(channel_mask & (1u << c))) continue;
        emit_event_at(0, EV_VS_CHANGE, c, chs[c].vs_base, chs[c].vs_base, "start");
        emit_event_at(0, EV_RI_CHANGE, c, relays.mode(c), 0, "");
        emit_event_at(0, EV_SERVO_STATE, c, chs[c].servo_mode, chs[c].freeze ? 1 : 0, "");
    }
}

static void do_start() {
    uint16_t rate = RATES[rate_code];
    if (!acq::canStream()) { send_nack(T_START, ERR_NO_ADC); return; }
    streaming = false;
    proc_idx = 0; block_seq = 0; blk_rows = 0; blk_status = 0; servo_tick = 0;
    rows_per_block = (uint16_t)((rate * EPG_BLOCK_MS) / 1000);
    if (rows_per_block < 1) rows_per_block = 1;
    if (rows_per_block > EPG_MAX_ROWS) rows_per_block = EPG_MAX_ROWS;
    servo_div = rate / 50 ? rate / 50 : 1;        // servo decisions every ~20 ms
    alpha_track   = 1.0f / (EPG_SERVO_TAU_TRACK_S * rate);
    alpha_acquire = 1.0f / (EPG_SERVO_TAU_ACQUIRE_S * rate);
    for (int c = 0; c < N_CHANNELS; c++) {
        chs[c].base_init = false; chs[c].clipped = false; chs[c].settle = 0;
        chs[c].hold_until = millis() + 500;
    }
    last_dropped = 0; last_tx_drop = tx_drops;
    streaming = true;                              // events from here on are sent
    if (!acq::start(rate, channel_mask)) { streaming = false; send_nack(T_START, ERR_ADC_CONFIG); return; }
    send_ack(T_START);
    snapshot_events();
}

static void do_stop() {
    acq::stop();
    acq::Sample s;
    while (acq::pop(s)) process_sample(s);         // flush what the ADC already delivered
    flush_block();
    streaming = false;
    send_ack(T_STOP);
}

static void on_frame(uint8_t type, uint8_t flags, const uint8_t *p, size_t n, void *u) {
    (void)flags; (void)u;
    switch (type) {
    case T_GET_INFO: {
        uint8_t pay[160];
        size_t pl = epg_build_info(&INFO, pay);
        send_frame(T_INFO, pay, pl);
        break; }
    case T_CONFIGURE:
        if (n < 2)                  { send_nack(type, ERR_BAD_PAYLOAD); break; }
        if (streaming)              { send_nack(type, ERR_BUSY); break; }
        if (p[0] >= 5 || p[1] == 0) { send_nack(type, ERR_BAD_VALUE); break; }
        rate_code = p[0]; channel_mask = p[1];
        send_ack(type);
        break;
    case T_START:
        if (streaming) { send_ack(type); break; }  // idempotent
        do_start();
        break;
    case T_STOP:
        if (streaming) do_stop(); else send_ack(type);
        break;
    case T_SET_VS: {
        epg_set_vs_t c;
        if (!epg_parse_set_vs(p, n, &c)) { send_nack(type, ERR_BAD_PAYLOAD); break; }
        if (c.ch >= N_CHANNELS)          { send_nack(type, ERR_BAD_CHANNEL); break; }
        set_vs(c.ch, c.dac, "manual");
        send_ack(type);
        break; }
    case T_SET_RI: {
        epg_set_ri_t c;
        if (!epg_parse_set_ri(p, n, &c)) { send_nack(type, ERR_BAD_PAYLOAD); break; }
        if (c.ch >= N_CHANNELS)          { send_nack(type, ERR_BAD_CHANNEL); break; }
        if (c.mode > RI_10T)             { send_nack(type, ERR_BAD_VALUE); break; }
        relays.set(c.ch, c.mode);
        emit_event(EV_RI_CHANGE, c.ch, c.mode, 0, "");
        send_ack(type);
        break; }
    case T_CAL_PULSE: {
        epg_cal_t c;
        if (!epg_parse_cal(p, n, &c)) { send_nack(type, ERR_BAD_PAYLOAD); break; }
        if (c.ch >= N_CHANNELS)       { send_nack(type, ERR_BAD_CHANNEL); break; }
        if (c.action) start_cal(c.ch, c.dur_ms); else end_cal(c.ch);
        send_ack(type);
        break; }
    case T_SERVO: {
        epg_servo_t c;
        if (!epg_parse_servo(p, n, &c)) { send_nack(type, ERR_BAD_PAYLOAD); break; }
        if (c.ch >= N_CHANNELS)         { send_nack(type, ERR_BAD_CHANNEL); break; }
        if (c.mode > SERVO_TRACK)       { send_nack(type, ERR_BAD_VALUE); break; }
        // target/deadband are offset-binary 16-bit (top 16 of the 24-bit ADC code); 0 = default
        chs[c.ch].target   = c.target   ? ((int32_t)c.target - 32768) * 256 : 0;
        chs[c.ch].deadband = c.deadband ? (int32_t)c.deadband * 256 : (int32_t)(EPG_SERVO_DEADBAND * FS);
        set_servo(c.ch, c.mode, (c.flags & 1) != 0);
        send_ack(type);
        break; }
    case T_PING:
        send_ack(type);
        break;
    default:
        send_nack(type, ERR_UNKNOWN_CMD);
    }
}

// ---------------------------------------------------------------- setup / loop
void setup() {
    Serial.begin(115200);                          // USB CDC; baud rate is ignored
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, HIGH);

    for (int c = 0; c < N_CHANNELS; c++) {
        chs[c] = Chan();
        chs[c].vs_base = 32768;
        chs[c].servo_mode = SERVO_TRACK;
        chs[c].deadband = (int32_t)(EPG_SERVO_DEADBAND * FS);
    }
    dac.begin();
    relays.begin();
    acq::begin();                                  // starts CLKIN, resets + probes the ADC

    pico_unique_board_id_t uid;
    pico_get_unique_board_id(&uid);
    snprintf(serial_str, sizeof serial_str, "EPG-%02X%02X%02X%02X%s",
             uid.id[4], uid.id[5], uid.id[6], uid.id[7], acq::adcPresent() ? "" : "-SIM");
    INFO = { EPG_PROTO_VERSION, EPG_FW_MAJOR, EPG_FW_MINOR, EPG_FW_PATCH, N_CHANNELS,
             EPG_ADC_VREF_MV, EPG_FIXED_GAIN_X100, EPG_VS_RANGE_MV, EPG_VS_DAC_BITS, EPG_ADC_BITS,
             RATES, 5, serial_str };
    epg_parser_init(&parser, on_frame, nullptr);
}

void loop() {
    // 1) host commands
    for (int guard = 0; guard < 256 && Serial.available() > 0; guard++) {
        uint8_t b = (uint8_t)Serial.read();
        epg_parser_feed(&parser, &b, 1);
    }

    uint32_t now = millis();
    if (streaming) {
        // 2) drain the ADC ring (bounded per pass so commands stay responsive)
        acq::Sample s;
        for (int n = 0; n < 128 && acq::pop(s); n++) process_sample(s);

        // 3) timed calibration pulses
        for (int c = 0; c < N_CHANNELS; c++)
            if (chs[c].cal_active && chs[c].cal_timed && (int32_t)(now - chs[c].cal_until) >= 0) end_cal(c);

        // 4) report transmit trouble once, outside the hot path
        if (tx_drops != last_tx_drop) {
            emit_event(EV_ERROR, CHAN_DEVICE, EVERR_TX_DROP, (int32_t)(tx_drops - last_tx_drop), "usb-drop");
            last_tx_drop = tx_drops;
        }
    }

    // 5) status LED: idle 1 Hz blink; streaming solid (real ADC) or 4 Hz (simulated)
    bool led;
    if (!streaming)               led = (now / 500) & 1;
    else if (acq::isSimulated())  led = (now / 125) & 1;
    else                          led = true;
    digitalWrite(PIN_STATUS_LED, led);
}
