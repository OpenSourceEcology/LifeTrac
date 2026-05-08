// lora_proto.c — pack/unpack, CRC, KISS framing for LifeTrac v25.
//
// DRAFT FOR REVIEW. Not compiled or tested.

#include "lora_proto.h"
#include <string.h>
#include <math.h>

#define KISS_FEND  0xC0
#define KISS_FESC  0xDB
#define KISS_TFEND 0xDC
#define KISS_TFESC 0xDD

// PHY profiles per DECISIONS.md D-A2 + D-A3 (2026-04-27):
//   * Control SF7 runs at BW250 so the encrypted 44 B ControlFrame fits a
//     50 ms (20 Hz) cadence with margin (~46 ms airtime). SF8/SF9 fallback
//     rungs stay BW125 for link-budget headroom — the ladder accepts the
//     slower effective cadence as graceful degradation.
//   * Image PHY runs at BW500 so a 32 B fragment stays under the 25 ms
//     per-fragment cap that protects P0 preemption.
const LoraPhyProfile LP_PHY_CONTROL_SF7 = { 7, 250, 5, 8 };
const LoraPhyProfile LP_PHY_CONTROL_SF8 = { 8, 125, 5, 8 };
const LoraPhyProfile LP_PHY_CONTROL_SF9 = { 9, 125, 5, 8 };
const LoraPhyProfile LP_PHY_TELEMETRY   = { 9, 250, 8, 12 };
const LoraPhyProfile LP_PHY_IMAGE       = { 7, 500, 5, 8 };

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
    // IP-305: bound exactly how many bytes each iteration writes (2 for an
    // escape, 1 for a regular byte) instead of assuming the worst case for
    // every byte; the old check refused buffers that were 1 byte short of
    // fitting an escape-free payload.
    size_t o = 0;
    if (out_max < 2) return 0;
    out[o++] = KISS_FEND;
    for (size_t i = 0; i < in_len; i++) {
        uint8_t b = in[i];
        size_t need = (b == KISS_FEND || b == KISS_FESC) ? 2 : 1;
        // Reserve one trailing byte for the closing FEND.
        if (o + need + 1 > out_max) return 0;
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

size_t lp_make_command(CommandFrame* f,
                       uint8_t source_id, uint16_t seq,
                       uint8_t opcode, const uint8_t* arg, uint8_t arg_len) {
    // IP-108: see CommandFrame docblock in lora_proto.h. The on-wire
    // layout packs CRC immediately after the actual arg bytes, which for
    // short commands lands inside the struct's ``arg[]`` storage and
    // leaves the trailing struct ``crc16`` field undefined. We zero the
    // whole struct first so any debugger / serializer that inspects
    // unused tail bytes sees deterministic 0x00 instead of stack garbage.
    if (arg_len > sizeof(f->arg)) arg_len = sizeof(f->arg);
    memset(f, 0, sizeof(*f));
    f->hdr.version      = LIFETRAC_PROTO_VERSION;
    f->hdr.source_id    = source_id;
    f->hdr.frame_type   = FT_COMMAND;
    f->hdr.sequence_num = seq;
    f->opcode           = opcode;
    if (arg_len > 0 && arg != 0) memcpy(f->arg, arg, arg_len);
    size_t prefix = sizeof(LoraHeader) + 1 + arg_len;
    uint16_t crc = lp_crc16((const uint8_t*)f, prefix);
    uint8_t* raw = (uint8_t*)f;
    raw[prefix]     = (uint8_t)(crc & 0xFF);
    raw[prefix + 1] = (uint8_t)(crc >> 8);
    return prefix + 2;
}

uint32_t lp_lora_airtime_ms(uint8_t payload_len,
                            uint8_t sf,
                            uint16_t bw_khz,
                            uint8_t cr_den,
                            uint8_t preamble_len) {
    if (sf < 6 || sf > 12 || bw_khz == 0 || cr_den < 5 || cr_den > 8) return 0;
    double bw_hz = (double)bw_khz * 1000.0;
    double tsym_ms = ((double)(1UL << sf) / bw_hz) * 1000.0;
    uint8_t low_data_rate_opt = (sf >= 11 && bw_khz <= 125) ? 1 : 0;
    uint8_t cr = (uint8_t)(cr_den - 4);  // RadioLib denominator 5..8 -> formula 1..4
    int32_t numerator = (int32_t)(8u * payload_len) - (int32_t)(4u * sf) + 28 + 16;
    int32_t denominator = 4 * ((int32_t)sf - (2 * low_data_rate_opt));
    int32_t payload_sym = 8;
    if (numerator > 0 && denominator > 0) {
        payload_sym += (int32_t)ceil((double)numerator / (double)denominator) * (cr + 4);
    }
    double total_ms = ((double)preamble_len + 4.25 + (double)payload_sym) * tsym_ms;
    return (uint32_t)ceil(total_ms);
}

static uint32_t xorshift32(uint32_t x) {
    if (x == 0) x = 0x6D2B79F5UL;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return x;
}

uint8_t lp_fhss_channel_index(uint32_t key_id, uint32_t hop_counter) {
    uint8_t perm[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    uint32_t cycle = hop_counter / 8u;
    uint32_t rnd = xorshift32(key_id ^ (cycle * 0x9E3779B9UL));
    for (int i = 7; i > 0; i--) {
        rnd = xorshift32(rnd);
        uint8_t j = (uint8_t)(rnd % (uint32_t)(i + 1));
        uint8_t tmp = perm[i];
        perm[i] = perm[j];
        perm[j] = tmp;
    }
    return perm[hop_counter & 0x07u];
}

uint32_t lp_fhss_channel_hz(uint32_t key_id, uint32_t hop_counter) {
    return 902000000UL + ((uint32_t)lp_fhss_channel_index(key_id, hop_counter) * 3250000UL);
}

// -------------------------------------------------------------------------
// CSMA skip-busy wrapper. See lora_proto.h for the protocol contract.
// -------------------------------------------------------------------------
uint32_t lp_csma_pick_hop(uint32_t key_id,
                          uint32_t start_hop,
                          lp_rssi_sampler_fn sample,
                          void* ctx,
                          int16_t busy_threshold_dbm,
                          uint8_t max_skips,
                          uint8_t* skips_out) {
    if (sample == NULL) {
        if (skips_out) *skips_out = 0;
        return start_hop;
    }

    uint32_t hop = start_hop;
    for (uint8_t skips = 0; skips <= max_skips; skips++) {
        int16_t rssi = sample(lp_fhss_channel_hz(key_id, hop), ctx);
        if (rssi <= busy_threshold_dbm || rssi == INT16_MIN) {
            if (skips_out) *skips_out = skips;
            return hop;
        }
        hop++;
    }
    // Every candidate within budget was busy. Caller must audit-log.
    if (skips_out) *skips_out = max_skips;
    return hop - 1u;
}

// -------------------------------------------------------------------------
// Replay-defence sliding window. See lora_proto.h for the protocol contract.
// -------------------------------------------------------------------------
void lp_replay_init(LpReplayWindow* w) {
    w->high_water  = 0;
    w->bitmap      = 0;
    w->primed      = false;
}

bool lp_replay_check_and_update(LpReplayWindow* w, uint16_t seq) {
    if (!w->primed) {
        w->high_water = seq;
        w->bitmap     = 1u;     // bit 0 = seq itself, marked seen
        w->primed     = true;
        return true;
    }

    // Signed 16-bit delta handles wrap correctly: delta > 0 means "newer",
    // delta <= 0 means "same or older".
    int16_t delta = (int16_t)(seq - w->high_water);

    if (delta > 0) {
        // Newer than anything we've seen: shift bitmap, mark new top.
        if ((uint32_t)delta >= LP_REPLAY_WINDOW_BITS) {
            w->bitmap = 1u;     // gap exceeds window; reset, accept new top
        } else {
            w->bitmap = (w->bitmap << (uint32_t)delta) | 1u;
        }
        w->high_water = seq;
        return true;
    }

    // delta <= 0: candidate is at or behind high_water.
    uint32_t age = (uint32_t)(-delta);
    if (age >= LP_REPLAY_WINDOW_BITS) {
        return false;          // too old to track; reject as replay
    }
    uint64_t mask = (uint64_t)1u << age;
    if (w->bitmap & mask) {
        return false;          // already seen
    }
    w->bitmap |= mask;
    return true;
}
