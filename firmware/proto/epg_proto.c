/* EPG device<->host protocol — portable C implementation. See epg_proto.h. */
#include "epg_proto.h"
#include <string.h>

/* ---- CRC-16/CCITT (XModem): poly 0x1021, init 0x0000, MSB-first ---- */
uint16_t epg_crc16(const uint8_t *data, size_t n) {
    uint16_t crc = 0x0000;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

/* ---- COBS ---- */
size_t epg_cobs_encode(const uint8_t *in, size_t n, uint8_t *out) {
    size_t rd = 0, code_idx = 0, wr = 1;
    uint8_t code = 1;
    out[code_idx] = 0;  /* placeholder */
    for (rd = 0; rd < n; rd++) {
        if (in[rd] == 0) {
            out[code_idx] = code; code_idx = wr++; code = 1;
        } else {
            out[wr++] = in[rd];
            if (++code == 0xFF) { out[code_idx] = code; code_idx = wr++; code = 1; }
        }
    }
    out[code_idx] = code;
    return wr;
}

size_t epg_cobs_decode(const uint8_t *in, size_t n, uint8_t *out, int *ok) {
    size_t rd = 0, wr = 0;
    *ok = 1;
    while (rd < n) {
        uint8_t code = in[rd++];
        if (code == 0) { *ok = 0; return 0; }
        uint8_t len = code - 1;
        if (rd + len > n) { *ok = 0; return 0; }
        for (uint8_t i = 0; i < len; i++) out[wr++] = in[rd++];
        if (code != 0xFF && rd < n) out[wr++] = 0;
    }
    return wr;
}

/* ---- i24 LE ---- */
void epg_pack_i24(int32_t v, uint8_t *o) {
    uint32_t u = (uint32_t)v & 0xFFFFFF;
    o[0] = u & 0xFF; o[1] = (u >> 8) & 0xFF; o[2] = (u >> 16) & 0xFF;
}
int32_t epg_unpack_i24(const uint8_t *in) {
    uint32_t u = (uint32_t)in[0] | ((uint32_t)in[1] << 8) | ((uint32_t)in[2] << 16);
    return (u & 0x800000) ? (int32_t)(u - 0x1000000) : (int32_t)u;
}

/* ---- little-endian writers ---- */
static size_t w8(uint8_t *p, uint8_t v) { p[0] = v; return 1; }
static size_t w16(uint8_t *p, uint16_t v) { p[0]=v&0xFF; p[1]=(v>>8)&0xFF; return 2; }
static size_t w32(uint8_t *p, uint32_t v) { for(int i=0;i<4;i++) p[i]=(v>>(8*i))&0xFF; return 4; }
static size_t w64(uint8_t *p, uint64_t v) { for(int i=0;i<8;i++) p[i]=(v>>(8*i))&0xFF; return 8; }

/* ---- frame ----
 * Streaming COBS+CRC encoder: no intermediate body buffer, so payloads of any size fit
 * as long as `frame` holds EPG_FRAME_MAX(plen) bytes. (The original version staged the
 * body in a 512-byte stack array, which overflowed on real sample blocks of 1-6 KB.)
 * Output is byte-identical to Python's cobs_encode(type|flags|payload|crc16) + 0x00. */
typedef struct { uint8_t *out; size_t wr, code_idx; uint8_t code; uint16_t crc; } enc_t;

static void enc_put(enc_t *e, uint8_t b) {
    if (b == 0) {
        e->out[e->code_idx] = e->code; e->code_idx = e->wr++; e->code = 1;
    } else {
        e->out[e->wr++] = b;
        if (++e->code == 0xFF) { e->out[e->code_idx] = e->code; e->code_idx = e->wr++; e->code = 1; }
    }
}
static void enc_put_crc(enc_t *e, uint8_t b) {   /* byte covered by the CRC */
    e->crc ^= (uint16_t)b << 8;
    for (int k = 0; k < 8; k++)
        e->crc = (e->crc & 0x8000) ? (uint16_t)((e->crc << 1) ^ 0x1021) : (uint16_t)(e->crc << 1);
    enc_put(e, b);
}

size_t epg_encode_frame(uint8_t type, uint8_t flags,
                        const uint8_t *payload, size_t plen, uint8_t *frame) {
    enc_t e; e.out = frame; e.wr = 1; e.code_idx = 0; e.code = 1; e.crc = 0x0000;
    enc_put_crc(&e, type);
    enc_put_crc(&e, flags);
    for (size_t i = 0; i < plen; i++) enc_put_crc(&e, payload[i]);
    uint16_t crc = e.crc;
    enc_put(&e, (uint8_t)(crc & 0xFF));
    enc_put(&e, (uint8_t)(crc >> 8));
    e.out[e.code_idx] = e.code;
    frame[e.wr++] = 0x00;  /* delimiter */
    return e.wr;
}

/* ---- payload builders ---- */
size_t epg_build_info(const epg_info_t *i, uint8_t *p) {
    size_t b = 0;
    b += w8(p+b, i->proto_version); b += w8(p+b, i->fw_major);
    b += w8(p+b, i->fw_minor); b += w8(p+b, i->fw_patch);
    b += w8(p+b, i->n_channels); b += w8(p+b, i->n_rates);
    b += w16(p+b, i->adc_vref_mv); b += w16(p+b, i->fixed_gain_x100);
    b += w16(p+b, i->vs_range_mv); b += w8(p+b, i->vs_dac_bits);
    b += w16(p+b, i->adc_bits);
    for (uint8_t k = 0; k < i->n_rates; k++) b += w16(p+b, i->rates[k]);
    uint8_t slen = 0; while (i->serial && i->serial[slen] && slen < 32) slen++;
    b += w8(p+b, slen);
    memcpy(p+b, i->serial, slen); b += slen;
    return b;
}

size_t epg_build_sampleblock_header(uint32_t block_seq, uint64_t first_sample_index,
                                    uint8_t rate_code, uint8_t channel_mask,
                                    uint16_t n_samples, uint8_t status, uint8_t *p) {
    size_t b = 0;
    b += w32(p+b, block_seq); b += w64(p+b, first_sample_index);
    b += w8(p+b, rate_code); b += w8(p+b, channel_mask);
    b += w16(p+b, n_samples); b += w8(p+b, status);
    return b;  /* caller appends n_samples*popcount(mask) packed i24 */
}

size_t epg_build_event(uint8_t event_type, uint8_t channel, uint64_t sample_index,
                       int32_t a, int32_t b_, const char *text, uint8_t text_len,
                       uint8_t *p) {
    size_t b = 0;
    b += w8(p+b, event_type); b += w8(p+b, channel);
    b += w64(p+b, sample_index);
    b += w32(p+b, (uint32_t)a); b += w32(p+b, (uint32_t)b_);
    b += w8(p+b, text_len);
    if (text_len) { memcpy(p+b, text, text_len); b += text_len; }
    return b;
}

/* ---- parser ---- */
void epg_parser_init(epg_parser_t *p, epg_frame_cb cb, void *user) {
    p->len = 0; p->crc_errors = 0; p->cobs_errors = 0; p->cb = cb; p->user = user;
}

static void parse_one(epg_parser_t *p, const uint8_t *frame, size_t n) {
    uint8_t body[512]; int ok = 0;
    size_t blen = epg_cobs_decode(frame, n, body, &ok);
    if (!ok) { p->cobs_errors++; return; }
    if (blen < 4) { p->cobs_errors++; return; }
    uint16_t got = (uint16_t)body[blen-2] | ((uint16_t)body[blen-1] << 8);
    uint16_t want = epg_crc16(body, blen - 2);
    if (got != want) { p->crc_errors++; return; }
    if (p->cb) p->cb(body[0], body[1], body + 2, blen - 4, p->user);
}

void epg_parser_feed(epg_parser_t *p, const uint8_t *data, size_t n) {
    for (size_t i = 0; i < n; i++) {
        uint8_t c = data[i];
        if (c == 0x00) {
            if (p->len) parse_one(p, p->buf, p->len);
            p->len = 0;
        } else if (p->len < sizeof(p->buf)) {
            p->buf[p->len++] = c;
        } else {
            p->len = 0;  /* overflow: drop & resync */
        }
    }
}

/* ---- command decoders ---- */
int epg_parse_set_vs(const uint8_t *p, size_t n, epg_set_vs_t *o) {
    if (n < 3) return 0;
    o->ch = p[0]; o->dac = (uint16_t)p[1] | ((uint16_t)p[2] << 8);
    return 1;
}
int epg_parse_set_ri(const uint8_t *p, size_t n, epg_set_ri_t *o) {
    if (n < 2) return 0;
    o->ch = p[0]; o->mode = p[1];
    return 1;
}
int epg_parse_cal(const uint8_t *p, size_t n, epg_cal_t *o) {
    if (n < 2) return 0;
    o->ch = p[0]; o->action = p[1];
    o->dur_ms = (n >= 4) ? ((uint16_t)p[2] | ((uint16_t)p[3] << 8)) : 0;
    return 1;
}
int epg_parse_servo(const uint8_t *p, size_t n, epg_servo_t *o) {
    if (n < 2) return 0;
    o->ch = p[0]; o->mode = p[1];
    o->target   = (n >= 4) ? ((uint16_t)p[2] | ((uint16_t)p[3] << 8)) : 0;
    o->deadband = (n >= 6) ? ((uint16_t)p[4] | ((uint16_t)p[5] << 8)) : 0;
    o->flags    = (n >= 7) ? p[6] : 0;
    return 1;
}
