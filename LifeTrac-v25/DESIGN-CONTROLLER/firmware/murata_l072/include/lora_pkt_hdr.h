#ifndef LIFETRAC_MURATA_L072_LORA_PKT_HDR_H
#define LIFETRAC_MURATA_L072_LORA_PKT_HDR_H

/*
 * LoRa link-layer packet header used for FHSS hop-sync (FCC-A6a, plan
 * §14.2 step 10). Every TX prepends this 8-byte header so the receiver
 * can extract (profile_id, epoch, hop_idx) and either confirm or
 * recover its hop alignment.
 *
 * Wire layout (8 bytes, little-endian):
 *
 *   off  size  field
 *   ---  ----  ---------------------------------------------------
 *     0   1    schema_ver   (== LORA_PKT_HDR_SCHEMA_VER)
 *     1   1    profile_id   (REG_PROFILE_* from host_cfg_keys.h)
 *     2   1    hop_idx      (0 .. SX1276_FHSS_CHANNEL_COUNT-1)
 *     3   1    _reserved    (must be 0 on emit, ignored on parse)
 *     4   4    epoch_le     (uint32 FHSS scheduler epoch)
 *
 * Schema evolution (FCC notes §17.3 #4 additive-only rule):
 *
 *   schema_ver=1 (this revision): 8 bytes, no MIC. The wire format
 *     is intentionally tight so it adds ~0.5 ms ToA at SF7/BW250.
 *     Parsers MUST refuse any unfamiliar schema_ver rather than
 *     silently mis-decoding the tail.
 *
 *   schema_ver=2 (planned, gated on Track C1 MIC integration): adds
 *     a trailing N-byte MIC computed over [schema_ver .. epoch] with
 *     a per-link key. The 8-byte v1 prefix layout is preserved
 *     byte-for-byte; older parsers will simply refuse v2 frames.
 *
 * Naming discipline: this header is the LORA-link header. Do NOT
 * confuse it with the X8↔L072 UART URC framing in host_uart.[ch] —
 * those are two independent transports with two independent schemas.
 */

#include <stdbool.h>
#include <stdint.h>

#define LORA_PKT_HDR_SCHEMA_VER   1U
#define LORA_PKT_HDR_LEN          8U

typedef struct lora_pkt_hdr_s {
    uint8_t  profile_id;
    uint8_t  hop_idx;
    uint32_t epoch;
} lora_pkt_hdr_t;

typedef enum {
    LORA_PKT_HDR_OK              = 0,
    LORA_PKT_HDR_ERR_NULL_ARG    = 1,
    LORA_PKT_HDR_ERR_SHORT_INPUT = 2,
    LORA_PKT_HDR_ERR_BAD_SCHEMA  = 3,
} lora_pkt_hdr_status_t;

/*
 * Pack `in` into the 8 leading bytes of `out`. `out` MUST have at
 * least LORA_PKT_HDR_LEN bytes available. Writes schema_ver and
 * zeroes the reserved byte. Returns false on NULL input.
 */
bool lora_pkt_hdr_pack(const lora_pkt_hdr_t *in, uint8_t *out);

/*
 * Unpack the 8 leading bytes of `in` into `out`. `in_len` MUST be
 * >= LORA_PKT_HDR_LEN; shorter buffers return SHORT_INPUT. An
 * unrecognised schema_ver returns BAD_SCHEMA without mutating *out.
 * The reserved byte is not checked (additive-evolution friendly).
 */
lora_pkt_hdr_status_t lora_pkt_hdr_unpack(const uint8_t *in,
                                          uint16_t in_len,
                                          lora_pkt_hdr_t *out);

#endif /* LIFETRAC_MURATA_L072_LORA_PKT_HDR_H */
