// lora_proto.c — pack/unpack, CRC, KISS framing for LifeTrac v25.
//
// DRAFT FOR REVIEW. Not compiled or tested.

#include "lora_proto.h"
#include <string.h>

#define KISS_FEND  0xC0
#define KISS_FESC  0xDB
#define KISS_TFEND 0xDC
#define KISS_TFESC 0xDD

// CRC-16/CCITT (false): poly 0x1021, init 0xFFFF, no reflection, no xor-out.
uint16_t lp_crc16(const uint8_t* buf, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)buf[i]) << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

size_t lp_kiss_encode(const uint8_t* in, size_t in_len,
                      uint8_t* out, size_t out_max) {
    size_t o = 0;
    if (out_max < 2) return 0;
    out[o++] = KISS_FEND;
    for (size_t i = 0; i < in_len; i++) {
        if (o + 2 >= out_max) return 0;
        uint8_t b = in[i];
        if (b == KISS_FEND) {
            out[o++] = KISS_FESC;
            out[o++] = KISS_TFEND;
        } else if (b == KISS_FESC) {
            out[o++] = KISS_FESC;
            out[o++] = KISS_TFESC;
        } else {
            out[o++] = b;
        }
    }
    if (o >= out_max) return 0;
    out[o++] = KISS_FEND;
    return o;
}

bool lp_kiss_feed(KissDecoder* dec, uint8_t b,
                  uint8_t** frame_out, size_t* frame_len) {
    if (b == KISS_FEND) {
        if (dec->in_frame && dec->len > 0) {
            // End of frame — hand it up.
            *frame_out = dec->buf;
            *frame_len = dec->len;
            dec->len = 0;
            dec->in_frame = false;
            dec->escape_next = false;
            return true;
        }
        // Either a leading FEND or an empty frame — start fresh.
        dec->in_frame = true;
        dec->len = 0;
        dec->escape_next = false;
        return false;
    }
    if (!dec->in_frame) return false;

    if (dec->escape_next) {
        if (b == KISS_TFEND) b = KISS_FEND;
        else if (b == KISS_TFESC) b = KISS_FESC;
        // else: protocol error — passed through verbatim, caller's CRC will catch it.
        dec->escape_next = false;
    } else if (b == KISS_FESC) {
        dec->escape_next = true;
        return false;
    }

    if (dec->len < sizeof(dec->buf)) {
        dec->buf[dec->len++] = b;
    } else {
        // Overflow — drop frame.
        dec->len = 0;
        dec->in_frame = false;
        dec->escape_next = false;
    }
    return false;
}

void lp_make_control(ControlFrame* f,
                     uint8_t source_id, uint16_t seq,
                     int8_t lhx, int8_t lhy, int8_t rhx, int8_t rhy,
                     uint16_t buttons, uint8_t flags, uint8_t hb_ctr) {
    f->hdr.version      = LIFETRAC_PROTO_VERSION;
    f->hdr.source_id    = source_id;
    f->hdr.frame_type   = FT_CONTROL;
    f->hdr.sequence_num = seq;
    f->axis_lh_x        = lhx;
    f->axis_lh_y        = lhy;
    f->axis_rh_x        = rhx;
    f->axis_rh_y        = rhy;
    f->buttons          = buttons;
    f->flags            = flags;
    f->heartbeat_ctr    = hb_ctr;
    f->reserved         = 0;
    f->crc16            = lp_crc16((const uint8_t*)f,
                                   sizeof(ControlFrame) - sizeof(uint16_t));
}

void lp_make_heartbeat(HeartbeatFrame* f,
                       uint8_t source_id, uint16_t seq,
                       uint8_t priority_request, uint8_t flags) {
    f->hdr.version          = LIFETRAC_PROTO_VERSION;
    f->hdr.source_id        = source_id;
    f->hdr.frame_type       = FT_HEARTBEAT;
    f->hdr.sequence_num     = seq;
    f->priority_request     = priority_request;
    f->flags                = flags;
    f->reserved             = 0;
    f->crc16                = lp_crc16((const uint8_t*)f,
                                       sizeof(HeartbeatFrame) - sizeof(uint16_t));
}
