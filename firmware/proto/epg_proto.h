/* EPG device<->host protocol — portable C (draft v0.1).
 *
 * Byte-for-byte compatible with the Python codec in software/epgrig/protocol.py.
 * No hardware or libc-IO dependencies (only <stdint.h>/<string.h>), so it compiles
 * both on the host (for tests) and on the RP2040 firmware.
 *
 * Wire frame (pre-COBS): type(1) | flags(1) | payload(N) | crc16_le(2)
 * then COBS-encoded with a trailing 0x00 delimiter. Little-endian throughout.
 */
#ifndef EPG_PROTO_H
#define EPG_PROTO_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EPG_PROTO_VERSION 1

/* message types: device->host */
enum { T_INFO=0x01, T_SAMPLES=0x02, T_EVENT=0x03, T_ACK=0x04, T_NACK=0x05, T_STATUS=0x06 };
/* message types: host->device */
enum { T_GET_INFO=0x80, T_CONFIGURE=0x81, T_START=0x82, T_STOP=0x83,
       T_SET_VS=0x84, T_SET_RI=0x85, T_CAL_PULSE=0x86, T_SERVO=0x87, T_PING=0x88 };
/* event types */
enum { EV_VS_CHANGE=1, EV_RI_CHANGE=2, EV_CAL_PULSE=3, EV_CLIP=4,
       EV_SERVO_STATE=5, EV_COMMENT=6, EV_MODE_MARK=7, EV_ERROR=8 };
/* servo modes / Ri modes */
enum { SERVO_OFF=0, SERVO_ACQUIRE=1, SERVO_TRACK=2 };
enum { RI_1G=0, RI_10T=1 };
#define CHAN_DEVICE 0xFF

/* ---- primitives ---- */
uint16_t epg_crc16(const uint8_t *data, size_t n);
/* COBS. out buffers must hold worst case: encode <= n + n/254 + 2; decode <= n. */
size_t epg_cobs_encode(const uint8_t *in, size_t n, uint8_t *out);
size_t epg_cobs_decode(const uint8_t *in, size_t n, uint8_t *out, int *ok);

/* signed 24-bit little-endian */
void   epg_pack_i24(int32_t v, uint8_t *out3);
int32_t epg_unpack_i24(const uint8_t *in3);

/* Worst-case encoded frame size for a payload of `plen` bytes (type+flags+crc = 4 extra,
 * COBS overhead 1 per 254, +1 leading code, +1 delimiter). */
#define EPG_FRAME_MAX(plen) ((size_t)(plen) + 4u + ((size_t)(plen) + 4u) / 254u + 2u)

/* Build a complete framed message (COBS + CRC + 0x00 delimiter).
 * Returns total bytes written to `frame`, which must hold EPG_FRAME_MAX(plen) bytes.
 * Any payload size is safe (streaming encode; no internal size limit). */
size_t epg_encode_frame(uint8_t type, uint8_t flags,
                        const uint8_t *payload, size_t plen, uint8_t *frame);

/* ---- message payload builders (return payload length) ---- */
typedef struct {
    uint8_t  proto_version, fw_major, fw_minor, fw_patch, n_channels;
    uint16_t adc_vref_mv, fixed_gain_x100, vs_range_mv;
    uint8_t  vs_dac_bits, adc_bits;
    const uint16_t *rates; uint8_t n_rates;
    const char *serial;
} epg_info_t;
size_t epg_build_info(const epg_info_t *i, uint8_t *payload);

/* sample block header (samples appended separately as packed i24) */
size_t epg_build_sampleblock_header(uint32_t block_seq, uint64_t first_sample_index,
                                    uint8_t rate_code, uint8_t channel_mask,
                                    uint16_t n_samples, uint8_t status, uint8_t *payload);

size_t epg_build_event(uint8_t event_type, uint8_t channel, uint64_t sample_index,
                       int32_t a, int32_t b, const char *text, uint8_t text_len,
                       uint8_t *payload);

/* ---- incremental frame parser (byte-loss tolerant; resync on 0x00) ---- */
typedef void (*epg_frame_cb)(uint8_t type, uint8_t flags,
                             const uint8_t *payload, size_t plen, void *user);
typedef struct {
    uint8_t  buf[512];
    size_t   len;
    uint32_t crc_errors, cobs_errors;
    epg_frame_cb cb;
    void *user;
} epg_parser_t;
void epg_parser_init(epg_parser_t *p, epg_frame_cb cb, void *user);
void epg_parser_feed(epg_parser_t *p, const uint8_t *data, size_t n);

/* ---- command payload decoders (host->device) ---- */
typedef struct { uint8_t ch; uint16_t dac; } epg_set_vs_t;
typedef struct { uint8_t ch, mode; } epg_set_ri_t;
typedef struct { uint8_t ch, action; uint16_t dur_ms; } epg_cal_t;
typedef struct { uint8_t ch, mode; uint16_t target, deadband; uint8_t flags; } epg_servo_t;
int epg_parse_set_vs(const uint8_t *p, size_t n, epg_set_vs_t *o);
int epg_parse_set_ri(const uint8_t *p, size_t n, epg_set_ri_t *o);
int epg_parse_cal(const uint8_t *p, size_t n, epg_cal_t *o);
int epg_parse_servo(const uint8_t *p, size_t n, epg_servo_t *o);

#ifdef __cplusplus
}
#endif
#endif /* EPG_PROTO_H */
