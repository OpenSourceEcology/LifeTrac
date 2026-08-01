/*
 * F8 (2026-07-30): golden vectors for the RX_FRAME_URC payload packer
 * (host_rx_wire.c). Pins:
 *   (a) the first 8+len bytes stay BYTE-IDENTICAL to the pre-F8 layout
 *       — that is the additive-evolution contract every legacy parser
 *       relies on;
 *   (b) payload[0] (len) excludes the tail;
 *   (c) hdr_valid=false -> tail all-zero, flags=0 (pins the
 *       garbage-hdr guard for unrouted builds);
 *   (d) hdr_valid=true -> flags=0x01, epoch little-endian, and a
 *       slot_offset of 255 passes through (TX saturation value);
 *   (e) len=255 -> 271 bytes total, no overflow at out_cap 272;
 *   (f) len=0 -> 16-byte payload;
 *   (g) cap/NULL refusals return 0.
 *
 * This TU builds with -DLIFETRAC_FHSS_TX_ROUTED so the hdr-valid path
 * is live (mirroring the ARM app CFLAGS); case (c) still exercises the
 * zero-tail path via hdr_valid=false.
 */
#include <stdio.h>
#include <string.h>

#include "host_rx_wire.h"

static int g_failures = 0;

#define CHECK(cond, ...) do {                                          \
    if (!(cond)) {                                                     \
        ++g_failures;                                                  \
        fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__);           \
        fprintf(stderr, __VA_ARGS__);                                  \
        fputc('\n', stderr);                                           \
    }                                                                  \
} while (0)

static sx1276_rx_frame_t mk_frame(uint8_t len) {
    sx1276_rx_frame_t f;
    memset(&f, 0, sizeof(f));
    f.length = len;
    f.snr_db = -3;
    f.rssi_dbm = -91;
    f.timestamp_us = 0x11223344UL;
    for (uint16_t i = 0; i < len; ++i) {
        f.payload[i] = (uint8_t)(0xA0U + i);
    }
    return f;
}

static void test_legacy_prefix_and_tail_valid(void) {
    uint8_t out[272];
    sx1276_rx_frame_t f = mk_frame(4U);
    f.hdr_valid = true;
    f.hdr.profile_id = 1U;
    f.hdr.hop_idx = 37U;
    f.hdr.slot_offset_ms = 255U;   /* TX saturation value passes through */
    f.hdr.epoch = 0x0A0B0C0DUL;

    const uint16_t n = host_rx_frame_urc_pack(&f, out, sizeof(out));
    CHECK(n == 8U + 4U + 8U, "(a) total 20, got %u", (unsigned)n);
    /* (a) legacy prefix byte-identical to the pre-F8 serializer */
    CHECK(out[0] == 4U, "(b) len byte excludes the tail");
    CHECK(out[1] == (uint8_t)-3, "(a) snr");
    CHECK(out[2] == (uint8_t)(-91 & 0xFF) &&
          out[3] == (uint8_t)(((uint16_t)(int16_t)-91) >> 8),
          "(a) rssi i16 LE");
    CHECK(out[4] == 0x44U && out[5] == 0x33U &&
          out[6] == 0x22U && out[7] == 0x11U, "(a) timestamp u32 LE");
    CHECK(out[8] == 0xA0U && out[11] == 0xA3U, "(a) payload bytes");
    /* (d) tail */
    CHECK(out[12] == HOST_RX_FRAME_URC_PHASE_VALID, "(d) flags bit0");
    CHECK(out[13] == 1U, "(d) profile_id");
    CHECK(out[14] == 37U, "(d) hop_idx");
    CHECK(out[15] == 255U, "(d) slot_offset 255 passthrough");
    CHECK(out[16] == 0x0DU && out[17] == 0x0CU &&
          out[18] == 0x0BU && out[19] == 0x0AU, "(d) epoch LE");
}

static void test_invalid_hdr_zero_tail(void) {
    uint8_t out[272];
    sx1276_rx_frame_t f = mk_frame(2U);
    /* poison hdr to prove the gate, not the memset, zeroes the tail */
    f.hdr.profile_id = 0xEEU;
    f.hdr.epoch = 0xDEADBEEFUL;
    f.hdr_valid = false;

    const uint16_t n = host_rx_frame_urc_pack(&f, out, sizeof(out));
    CHECK(n == 8U + 2U + 8U, "(c) total 18");
    for (uint16_t i = 10U; i < 18U; ++i) {
        CHECK(out[i] == 0U, "(c) tail byte %u zero", (unsigned)i);
    }
}

static void test_max_len_and_caps(void) {
    uint8_t out[272];
    sx1276_rx_frame_t f = mk_frame(255U);
    f.hdr_valid = true;
    f.hdr.epoch = 7UL;

    const uint16_t n = host_rx_frame_urc_pack(&f, out, sizeof(out));
    CHECK(n == 8U + 255U + 8U, "(e) 271 at len=255");
    CHECK(out[0] == 255U, "(e) len byte 255");
    CHECK(out[263] == HOST_RX_FRAME_URC_PHASE_VALID, "(e) tail after 255B");
    CHECK(out[267] == 7U, "(e) epoch lsb");

    /* (g) refusals */
    CHECK(host_rx_frame_urc_pack(&f, out, 270U) == 0U,
          "(g) cap 270 refused (needs 271)");
    CHECK(host_rx_frame_urc_pack(NULL, out, sizeof(out)) == 0U,
          "(g) NULL frame refused");
    CHECK(host_rx_frame_urc_pack(&f, NULL, sizeof(out)) == 0U,
          "(g) NULL out refused");
}

static void test_zero_len(void) {
    uint8_t out[272];
    sx1276_rx_frame_t f = mk_frame(0U);
    f.hdr_valid = false;
    const uint16_t n = host_rx_frame_urc_pack(&f, out, sizeof(out));
    CHECK(n == 16U, "(f) len=0 -> 16 bytes");
    CHECK(out[0] == 0U, "(f) len byte 0");
    CHECK(out[8] == 0U, "(f) flags 0");
}

int main(void) {
    test_legacy_prefix_and_tail_valid();
    test_invalid_hdr_zero_tail();
    test_max_len_and_caps();
    test_zero_len();

    if (g_failures != 0) {
        fprintf(stderr, "[FAIL] rx_frame_urc: %d failure(s)\n", g_failures);
        return 1;
    }
    printf("[PASS] rx_frame_urc: 4 test fns, 27 checks\n");
    return 0;
}
