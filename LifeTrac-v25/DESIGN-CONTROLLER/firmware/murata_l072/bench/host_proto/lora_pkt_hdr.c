/*
 * FCC-A6a host-side test for the LoRa link-layer packet header.
 *
 * Coverage:
 *   1. Golden vector — nominal mid-range values pack to a known
 *      8-byte sequence. Verifies field offsets, little-endian
 *      epoch layout, and the hard-coded schema_ver byte.
 *   2. Golden vector — all-zero input still emits schema_ver=1.
 *   3. Golden vector — edge values (profile_id=0xFF, hop_idx=49,
 *      epoch=UINT32_MAX) pack without truncation.
 *   4. NULL pointer rejection on pack() (returns false, no writes).
 *   5. Round-trip: pack(in) then unpack() reproduces in.
 *   6. Short input on unpack() returns SHORT_INPUT, leaves *out
 *      untouched.
 *   7. Bad schema byte on unpack() returns BAD_SCHEMA, leaves *out
 *      untouched.
 *   8. NULL pointer rejection on unpack() (returns NULL_ARG).
 *   9. Reserved byte ignored by unpack() (additive-evolution rule).
 *  10. Constants: schema_ver==1, header_len==8.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>

#include "lora_pkt_hdr.h"

static int g_failures = 0;

#define CHECK(cond, ...) do {                                          \
    if (!(cond)) {                                                     \
        ++g_failures;                                                  \
        fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__);           \
        fprintf(stderr, __VA_ARGS__);                                  \
        fputc('\n', stderr);                                           \
    }                                                                  \
} while (0)

static void dump_hex(const char *label, const uint8_t *p, size_t n) {
    fprintf(stderr, "  %s:", label);
    for (size_t i = 0; i < n; ++i) { fprintf(stderr, " %02X", p[i]); }
    fputc('\n', stderr);
}

static void test_constants(void) {
    CHECK(LORA_PKT_HDR_SCHEMA_VER == 1U, "schema_ver should be 1");
    CHECK(LORA_PKT_HDR_LEN == 8U, "header length should be 8");
}

static void test_nominal_golden(void) {
    /*
     * profile_id = 1 (FCC_15_247_FHSS_50CH_BW250)
     * hop_idx    = 17
     * epoch      = 0x12345678
     *
     * Expected bytes (LE):
     *   00: 01 schema_ver
     *   01: 01 profile_id
     *   02: 11 hop_idx
     *   03: 00 _reserved
     *   04: 78 56 34 12 epoch_le
     */
    const lora_pkt_hdr_t in = {
        .profile_id = 1U,
        .hop_idx    = 17U,
        .epoch      = 0x12345678U,
    };
    const uint8_t expected[LORA_PKT_HDR_LEN] = {
        0x01, 0x01, 0x11, 0x00, 0x78, 0x56, 0x34, 0x12
    };
    uint8_t actual[LORA_PKT_HDR_LEN];
    memset(actual, 0xCC, sizeof(actual));

    CHECK(lora_pkt_hdr_pack(&in, actual) == true, "pack should succeed");
    if (memcmp(actual, expected, LORA_PKT_HDR_LEN) != 0) {
        ++g_failures;
        fprintf(stderr, "FAIL %s:%d: nominal golden mismatch\n", __FILE__, __LINE__);
        dump_hex("expected", expected, LORA_PKT_HDR_LEN);
        dump_hex("actual  ", actual, LORA_PKT_HDR_LEN);
    }
}

static void test_all_zero_golden(void) {
    const lora_pkt_hdr_t in = {0};
    const uint8_t expected[LORA_PKT_HDR_LEN] = {
        0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    uint8_t actual[LORA_PKT_HDR_LEN] = {0};
    CHECK(lora_pkt_hdr_pack(&in, actual) == true, "pack all-zero");
    CHECK(memcmp(actual, expected, LORA_PKT_HDR_LEN) == 0,
          "all-zero input must still emit schema_ver=1");
}

static void test_edge_values(void) {
    const lora_pkt_hdr_t in = {
        .profile_id = 0xFFU,
        .hop_idx    = 49U,
        .epoch      = UINT32_MAX,
    };
    const uint8_t expected[LORA_PKT_HDR_LEN] = {
        0x01, 0xFF, 0x31, 0x00, 0xFF, 0xFF, 0xFF, 0xFF
    };
    uint8_t actual[LORA_PKT_HDR_LEN];
    CHECK(lora_pkt_hdr_pack(&in, actual) == true, "pack edge");
    CHECK(memcmp(actual, expected, LORA_PKT_HDR_LEN) == 0,
          "edge values must round-trip without truncation");
}

static void test_slot_offset_byte(void) {
    /* v25.0.7 slot-clock: byte 3 (former _reserved) now carries
     * slot_offset_ms. Packing must place it verbatim; unpack must
     * surface it; a zero value keeps the pre-v25.0.7 wire image
     * byte-for-byte (additive evolution, schema_ver stays 1). */
    const lora_pkt_hdr_t in = {
        .profile_id     = 1U,
        .hop_idx        = 5U,
        .epoch          = 0x00000002U,
        .slot_offset_ms = 0xABU,
    };
    const uint8_t expected[LORA_PKT_HDR_LEN] = {
        0x01, 0x01, 0x05, 0xAB, 0x02, 0x00, 0x00, 0x00
    };
    uint8_t actual[LORA_PKT_HDR_LEN];
    lora_pkt_hdr_t back;
    CHECK(lora_pkt_hdr_pack(&in, actual) == true, "pack slot_offset");
    CHECK(memcmp(actual, expected, LORA_PKT_HDR_LEN) == 0,
          "slot_offset_ms must land in byte 3");
    CHECK(lora_pkt_hdr_unpack(actual, LORA_PKT_HDR_LEN, &back)
              == LORA_PKT_HDR_OK,
          "unpack slot_offset");
    CHECK(back.slot_offset_ms == 0xABU,
          "slot_offset_ms must round-trip");
}

static void test_pack_null_rejected(void) {
    uint8_t buf[LORA_PKT_HDR_LEN];
    memset(buf, 0xAA, sizeof(buf));
    const lora_pkt_hdr_t in = {0};
    CHECK(lora_pkt_hdr_pack(NULL, buf) == false, "pack(NULL,buf) should reject");
    CHECK(lora_pkt_hdr_pack(&in, NULL) == false, "pack(in,NULL) should reject");
    /* Verify pack(NULL,...) did not mutate the buffer. */
    for (size_t i = 0; i < sizeof(buf); ++i) {
        CHECK(buf[i] == 0xAAU, "pack(NULL) must not write to out");
    }
}

static void test_round_trip(void) {
    const lora_pkt_hdr_t in = {
        .profile_id     = 2U,
        .hop_idx        = 33U,
        .epoch          = 0xCAFEBABEU,
        .slot_offset_ms = 199U,
    };
    uint8_t buf[LORA_PKT_HDR_LEN];
    lora_pkt_hdr_t out = {0};

    CHECK(lora_pkt_hdr_pack(&in, buf) == true, "round-trip pack");
    CHECK(lora_pkt_hdr_unpack(buf, LORA_PKT_HDR_LEN, &out) == LORA_PKT_HDR_OK,
          "round-trip unpack");
    CHECK(out.profile_id == in.profile_id, "round-trip profile_id");
    CHECK(out.hop_idx == in.hop_idx, "round-trip hop_idx");
    CHECK(out.epoch == in.epoch, "round-trip epoch");
    CHECK(out.slot_offset_ms == in.slot_offset_ms,
          "round-trip slot_offset_ms");
}

static void test_unpack_short_input(void) {
    const uint8_t buf[7] = { 0x01, 0x01, 0x11, 0x00, 0x78, 0x56, 0x34 };
    lora_pkt_hdr_t out = {
        .profile_id = 0xAAU,
        .hop_idx    = 0xBBU,
        .epoch      = 0xCCCCCCCCU,
    };
    const lora_pkt_hdr_t before = out;

    CHECK(lora_pkt_hdr_unpack(buf, 7U, &out) == LORA_PKT_HDR_ERR_SHORT_INPUT,
          "short input must return SHORT_INPUT");
    CHECK(out.profile_id == before.profile_id, "short unpack must not mutate out.profile_id");
    CHECK(out.hop_idx == before.hop_idx, "short unpack must not mutate out.hop_idx");
    CHECK(out.epoch == before.epoch, "short unpack must not mutate out.epoch");
}

static void test_unpack_bad_schema(void) {
    const uint8_t buf[LORA_PKT_HDR_LEN] = {
        0x02, /* schema_ver=2 — not yet defined */
        0x01, 0x11, 0x00, 0x78, 0x56, 0x34, 0x12
    };
    lora_pkt_hdr_t out = {
        .profile_id = 0xAAU,
        .hop_idx    = 0xBBU,
        .epoch      = 0xCCCCCCCCU,
    };
    const lora_pkt_hdr_t before = out;

    CHECK(lora_pkt_hdr_unpack(buf, LORA_PKT_HDR_LEN, &out)
              == LORA_PKT_HDR_ERR_BAD_SCHEMA,
          "unknown schema must return BAD_SCHEMA");
    CHECK(out.profile_id == before.profile_id, "BAD_SCHEMA must not mutate out");
    CHECK(out.hop_idx == before.hop_idx, "BAD_SCHEMA must not mutate out");
    CHECK(out.epoch == before.epoch, "BAD_SCHEMA must not mutate out");
}

static void test_unpack_null_rejected(void) {
    const uint8_t buf[LORA_PKT_HDR_LEN] = {
        0x01, 0x01, 0x11, 0x00, 0x78, 0x56, 0x34, 0x12
    };
    lora_pkt_hdr_t out;
    CHECK(lora_pkt_hdr_unpack(NULL, LORA_PKT_HDR_LEN, &out)
              == LORA_PKT_HDR_ERR_NULL_ARG, "unpack(NULL,_,_) → NULL_ARG");
    CHECK(lora_pkt_hdr_unpack(buf, LORA_PKT_HDR_LEN, NULL)
              == LORA_PKT_HDR_ERR_NULL_ARG, "unpack(_,_,NULL) → NULL_ARG");
}

static void test_unpack_ignores_reserved(void) {
    /* Same as nominal golden but with reserved byte forced to 0xAB.
     * v25.0.7: byte 3 is now surfaced as slot_offset_ms — the case
     * doubles as proof that pre-slot-clock parsers' "ignore" rule and
     * the new field are the same wire byte. */
    const uint8_t buf[LORA_PKT_HDR_LEN] = {
        0x01, 0x01, 0x11, 0xAB, 0x78, 0x56, 0x34, 0x12
    };
    lora_pkt_hdr_t out = {0};
    CHECK(lora_pkt_hdr_unpack(buf, LORA_PKT_HDR_LEN, &out) == LORA_PKT_HDR_OK,
          "non-zero reserved byte must NOT fail unpack");
    CHECK(out.profile_id == 1U, "profile_id after reserved-byte ignore");
    CHECK(out.hop_idx == 17U, "hop_idx after reserved-byte ignore");
    CHECK(out.epoch == 0x12345678U, "epoch after reserved-byte ignore");
}

int main(void) {
    test_constants();
    test_nominal_golden();
    test_all_zero_golden();
    test_edge_values();
    test_slot_offset_byte();
    test_pack_null_rejected();
    test_round_trip();
    test_unpack_short_input();
    test_unpack_bad_schema();
    test_unpack_null_rejected();
    test_unpack_ignores_reserved();

    if (g_failures != 0) {
        fprintf(stderr, "[FAIL] lora_pkt_hdr: %d failure(s)\n", g_failures);
        return 1;
    }
    printf("[PASS] lora_pkt_hdr: 11 cases\n");
    return 0;
}
