/*
 * FCC-B1-SUMMARY-b-2 host-side test for the per-minute RFCO_SUMMARY
 * URC pack helper.
 *
 * Coverage (17 cases):
 *   CRC primitive:
 *     1. CRC-16/CCITT-FALSE KAT: crc("123456789") == 0x29B1.
 *
 *   Pack helper API + invariants:
 *     2. NULL `in` rejected; out buffer untouched.
 *     3. NULL `out` rejected.
 *     4. Payload length constant is exactly 191.
 *     5. Schema byte (offset 0) is HARD-PINNED to SCHEMA_VER even when
 *        the struct field contains a different value.
 *     6. pertx_schema_ver_at_emit (offset 1) stamps caller value.
 *     7. _reserved_align (offsets 6..7) is HARD-ZEROED even when the
 *        struct field is 0xDEAD (catches uninit-struct leaks).
 *
 *   Field encoding:
 *     8. All-zero snapshot: byte 0 = SCHEMA_VER, bytes 1..188 = 0,
 *        bytes 189..190 = CRC over (1, 0×188).
 *     9. u32 LE encoding at offsets 8/12/16
 *        (uptime_ms / summary_seq / window_elapsed_ms).
 *    10. per_channel_hop_count[50] round-trips byte-for-byte;
 *        saturated 0xFF value encodes verbatim.
 *    11. per_channel_dwell_max_ms[50] u16 LE round-trip; saturated
 *        0xFFFF encodes as FF FF.
 *    12. blocked_attempts_by_reason[8] u16 LE round-trip across all
 *        8 slots; saturated 0xFFFF encodes as FF FF.
 *    13. Tail bytes (offsets 186/187/188) round-trip.
 *    14. flags FIRST_SINCE_BOOT bit round-trips.
 *
 *   CRC integrity:
 *    15. CRC at offsets 189..190 is LE (low byte first).
 *    16. Flipping any byte in [0..188] changes the CRC.
 *    17. Flipping a CRC byte does NOT match a re-pack (the helper
 *        always overwrites the CRC).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>

#include "host_rfco_summary.h"

/* ---- test harness ---- */

static int g_failures = 0;
static int g_cases    = 0;

#define CHECK(cond, ...) do {                                          \
    if (!(cond)) {                                                     \
        ++g_failures;                                                  \
        fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__);           \
        fprintf(stderr, __VA_ARGS__);                                  \
        fprintf(stderr, "\n");                                         \
    }                                                                  \
} while (0)

#define CASE(name) do { ++g_cases; fprintf(stderr, "[CASE] %s\n", name); } while (0)

/* ---- independent CRC-16/CCITT-FALSE reference ---- */
/* Same algorithm as host/host_rfco_summary.c. The KAT in case 1 pins
 * the algorithm itself; subsequent cases use this reference to predict
 * the CRC field. */
static uint16_t crc_ref(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (uint8_t b = 0; b < 8; ++b) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

/* ---- helpers ---- */

static uint16_t read_u16_le(const uint8_t *p) {
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t read_u32_le(const uint8_t *p) {
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

static void dump_bytes(const char *label, const uint8_t *b, size_t n) {
    fprintf(stderr, "  %s:", label);
    for (size_t i = 0; i < n; ++i) fprintf(stderr, " %02X", b[i]);
    fprintf(stderr, "\n");
}

/* ============================================================
 * 1. CRC primitive KAT
 * ============================================================ */
static void test_crc_kat(void) {
    CASE("CRC-16/CCITT-FALSE KAT: crc(\"123456789\") == 0x29B1");
    const uint8_t kat[] = "123456789";
    uint16_t got = crc_ref(kat, 9);
    CHECK(got == 0x29B1U, "got 0x%04X, want 0x29B1", got);
}

/* ============================================================
 * 2-7. API + invariants
 * ============================================================ */

static void test_null_in_rejected(void) {
    CASE("NULL in rejected, out buffer untouched");
    uint8_t out[HOST_RFCO_SUMMARY_PAYLOAD_LEN];
    memset(out, 0xAA, sizeof(out));
    bool ok = host_rfco_summary_pack(NULL, out);
    CHECK(!ok, "pack(NULL, out) should return false");
    for (size_t i = 0; i < sizeof(out); ++i) {
        CHECK(out[i] == 0xAAU, "out[%zu] = 0x%02X, want 0xAA (untouched)",
              i, out[i]);
    }
}

static void test_null_out_rejected(void) {
    CASE("NULL out rejected");
    host_rfco_summary_t in = {0};
    bool ok = host_rfco_summary_pack(&in, NULL);
    CHECK(!ok, "pack(in, NULL) should return false");
}

static void test_payload_len_constant(void) {
    CASE("HOST_RFCO_SUMMARY_PAYLOAD_LEN == 191");
    CHECK(HOST_RFCO_SUMMARY_PAYLOAD_LEN == 191U,
          "payload_len = %u, want 191", (unsigned)HOST_RFCO_SUMMARY_PAYLOAD_LEN);
}

static void test_schema_byte_hard_pinned(void) {
    CASE("schema byte (off 0) hard-pinned to SCHEMA_VER even when struct differs");
    host_rfco_summary_t in = {0};
    in.schema_ver = 0xAA;  /* lie */
    uint8_t out[HOST_RFCO_SUMMARY_PAYLOAD_LEN];
    bool ok = host_rfco_summary_pack(&in, out);
    CHECK(ok, "pack should succeed");
    CHECK(out[HOST_RFCO_SUMMARY_OFF_SCHEMA_VER] == HOST_RFCO_SUMMARY_SCHEMA_VER,
          "byte 0 = 0x%02X, want 0x%02X", out[0],
          (unsigned)HOST_RFCO_SUMMARY_SCHEMA_VER);
}

static void test_pertx_schema_stamps(void) {
    CASE("pertx_schema_ver_at_emit (off 1) stamps caller value");
    host_rfco_summary_t in = {0};
    in.pertx_schema_ver_at_emit = 0x42U;
    uint8_t out[HOST_RFCO_SUMMARY_PAYLOAD_LEN];
    (void)host_rfco_summary_pack(&in, out);
    CHECK(out[HOST_RFCO_SUMMARY_OFF_PERTX_SCHEMA_VER] == 0x42U,
          "byte 1 = 0x%02X, want 0x42", out[1]);
}

static void test_reserved_align_hard_zeroed(void) {
    CASE("_reserved_align (off 6..7) hard-zeroed even when struct field is 0xDEAD");
    host_rfco_summary_t in = {0};
    in._reserved_align = 0xDEADU;
    uint8_t out[HOST_RFCO_SUMMARY_PAYLOAD_LEN];
    memset(out, 0xCC, sizeof(out));
    (void)host_rfco_summary_pack(&in, out);
    CHECK(out[HOST_RFCO_SUMMARY_OFF_RESERVED_ALIGN] == 0x00U,
          "byte 6 = 0x%02X, want 0x00", out[6]);
    CHECK(out[HOST_RFCO_SUMMARY_OFF_RESERVED_ALIGN + 1U] == 0x00U,
          "byte 7 = 0x%02X, want 0x00", out[7]);
}

/* ============================================================
 * 8. All-zero snapshot golden vector
 * ============================================================ */
static void test_all_zero_snapshot(void) {
    CASE("all-zero snapshot: only schema byte + CRC are non-zero");
    host_rfco_summary_t in = {0};
    uint8_t out[HOST_RFCO_SUMMARY_PAYLOAD_LEN];
    bool ok = host_rfco_summary_pack(&in, out);
    CHECK(ok, "pack should succeed");

    CHECK(out[0] == HOST_RFCO_SUMMARY_SCHEMA_VER,
          "byte 0 = 0x%02X, want 0x%02X", out[0],
          (unsigned)HOST_RFCO_SUMMARY_SCHEMA_VER);
    for (size_t i = 1; i < HOST_RFCO_SUMMARY_OFF_PAYLOAD_CRC16; ++i) {
        CHECK(out[i] == 0x00U, "byte %zu = 0x%02X, want 0x00", i, out[i]);
    }
    /* CRC is over (schema_ver, 0×188). */
    uint8_t expected_hdr[HOST_RFCO_SUMMARY_OFF_PAYLOAD_CRC16] = {0};
    expected_hdr[0] = HOST_RFCO_SUMMARY_SCHEMA_VER;
    uint16_t want_crc = crc_ref(expected_hdr, HOST_RFCO_SUMMARY_OFF_PAYLOAD_CRC16);
    uint16_t got_crc  = read_u16_le(&out[HOST_RFCO_SUMMARY_OFF_PAYLOAD_CRC16]);
    CHECK(got_crc == want_crc, "CRC = 0x%04X, want 0x%04X", got_crc, want_crc);
}

/* ============================================================
 * 9. u32 LE encoding
 * ============================================================ */
static void test_u32_le_encoding(void) {
    CASE("u32 LE: uptime_ms / summary_seq / window_elapsed_ms");
    host_rfco_summary_t in = {0};
    in.uptime_ms         = 0x12345678UL;
    in.summary_seq       = 0xDEADBEEFUL;
    in.window_elapsed_ms = 60000UL;
    uint8_t out[HOST_RFCO_SUMMARY_PAYLOAD_LEN];
    (void)host_rfco_summary_pack(&in, out);

    CHECK(read_u32_le(&out[HOST_RFCO_SUMMARY_OFF_UPTIME_MS]) == 0x12345678UL,
          "uptime_ms got 0x%08" PRIX32, read_u32_le(&out[8]));
    CHECK(read_u32_le(&out[HOST_RFCO_SUMMARY_OFF_SUMMARY_SEQ]) == 0xDEADBEEFUL,
          "summary_seq got 0x%08" PRIX32, read_u32_le(&out[12]));
    CHECK(read_u32_le(&out[HOST_RFCO_SUMMARY_OFF_WINDOW_ELAPSED_MS]) == 60000UL,
          "window_elapsed_ms got %" PRIu32, read_u32_le(&out[16]));

    /* Spot-check byte order at offset 8: 0x12345678 -> 78 56 34 12. */
    CHECK(out[8]  == 0x78U, "uptime byte0 = 0x%02X", out[8]);
    CHECK(out[9]  == 0x56U, "uptime byte1 = 0x%02X", out[9]);
    CHECK(out[10] == 0x34U, "uptime byte2 = 0x%02X", out[10]);
    CHECK(out[11] == 0x12U, "uptime byte3 = 0x%02X", out[11]);
}

/* ============================================================
 * 10. per_channel_hop_count[50] round-trip + saturation
 * ============================================================ */
static void test_hop_count_round_trip(void) {
    CASE("per_channel_hop_count[50] round-trips byte-for-byte; 0xFF passes through");
    host_rfco_summary_t in = {0};
    for (uint8_t i = 0; i < HOST_RFCO_SUMMARY_CHANNEL_COUNT; ++i) {
        in.per_channel_hop_count[i] = (uint8_t)(i + 1U); /* 1..50 */
    }
    in.per_channel_hop_count[7]  = 0xFFU;
    in.per_channel_hop_count[49] = 0xFFU;
    uint8_t out[HOST_RFCO_SUMMARY_PAYLOAD_LEN];
    (void)host_rfco_summary_pack(&in, out);
    for (uint8_t i = 0; i < HOST_RFCO_SUMMARY_CHANNEL_COUNT; ++i) {
        const uint8_t got = out[HOST_RFCO_SUMMARY_OFF_PER_CHANNEL_HOP_COUNT + i];
        CHECK(got == in.per_channel_hop_count[i],
              "hop ch %u: got 0x%02X, want 0x%02X",
              i, got, in.per_channel_hop_count[i]);
    }
}

/* ============================================================
 * 11. per_channel_dwell_max_ms[50] u16 LE round-trip
 * ============================================================ */
static void test_dwell_max_round_trip(void) {
    CASE("per_channel_dwell_max_ms[50] u16 LE; 0xFFFF passes through");
    host_rfco_summary_t in = {0};
    for (uint8_t i = 0; i < HOST_RFCO_SUMMARY_CHANNEL_COUNT; ++i) {
        in.per_channel_dwell_max_ms[i] = (uint16_t)(100U + i);
    }
    in.per_channel_dwell_max_ms[3]  = 0xFFFFU;
    in.per_channel_dwell_max_ms[49] = 0xFFFFU;
    uint8_t out[HOST_RFCO_SUMMARY_PAYLOAD_LEN];
    (void)host_rfco_summary_pack(&in, out);
    for (uint8_t i = 0; i < HOST_RFCO_SUMMARY_CHANNEL_COUNT; ++i) {
        const uint16_t got = read_u16_le(
            &out[HOST_RFCO_SUMMARY_OFF_PER_CHANNEL_DWELL_MAX + (uint16_t)(i * 2U)]);
        CHECK(got == in.per_channel_dwell_max_ms[i],
              "dwell ch %u: got %u, want %u",
              i, got, in.per_channel_dwell_max_ms[i]);
    }
    /* Spot-check LE byte order for ch 0 (= 100 = 0x0064 -> 64 00). */
    CHECK(out[HOST_RFCO_SUMMARY_OFF_PER_CHANNEL_DWELL_MAX]      == 0x64U,
          "dwell ch0 byte0 = 0x%02X", out[70]);
    CHECK(out[HOST_RFCO_SUMMARY_OFF_PER_CHANNEL_DWELL_MAX + 1U] == 0x00U,
          "dwell ch0 byte1 = 0x%02X", out[71]);
}

/* ============================================================
 * 12. blocked_attempts_by_reason[8] u16 LE round-trip
 * ============================================================ */
static void test_blocked_attempts_round_trip(void) {
    CASE("blocked_attempts_by_reason[8] u16 LE; 0xFFFF passes through");
    host_rfco_summary_t in = {0};
    in.blocked_attempts_by_reason[0] = 0U;       /* OK slot, reserved */
    in.blocked_attempts_by_reason[1] = 11U;
    in.blocked_attempts_by_reason[2] = 222U;
    in.blocked_attempts_by_reason[3] = 0x0F0FU;
    in.blocked_attempts_by_reason[4] = 0xABCDU;
    in.blocked_attempts_by_reason[5] = 5U;
    in.blocked_attempts_by_reason[6] = 6U;
    in.blocked_attempts_by_reason[7] = 0xFFFFU;  /* INTERNAL saturated */
    uint8_t out[HOST_RFCO_SUMMARY_PAYLOAD_LEN];
    (void)host_rfco_summary_pack(&in, out);
    for (uint8_t i = 0; i < HOST_RFCO_SUMMARY_REASON_SLOTS; ++i) {
        const uint16_t got = read_u16_le(
            &out[HOST_RFCO_SUMMARY_OFF_BLOCKED_ATTEMPTS + (uint16_t)(i * 2U)]);
        CHECK(got == in.blocked_attempts_by_reason[i],
              "blocked slot %u: got 0x%04X, want 0x%04X",
              i, got, in.blocked_attempts_by_reason[i]);
    }
    /* Spot-check LE byte order for slot 4 (0xABCD -> CD AB). */
    const size_t off4 = HOST_RFCO_SUMMARY_OFF_BLOCKED_ATTEMPTS + (4U * 2U);
    CHECK(out[off4]      == 0xCDU, "blocked slot4 byte0 = 0x%02X", out[off4]);
    CHECK(out[off4 + 1U] == 0xABU, "blocked slot4 byte1 = 0x%02X", out[off4 + 1U]);
}

/* ============================================================
 * 13. Tail bytes round-trip
 * ============================================================ */
static void test_tail_bytes(void) {
    CASE("tail bytes (off 186/187/188) round-trip");
    host_rfco_summary_t in = {0};
    in.pertx_count_in_window = 0xA5U;
    in.summary_emit_count    = 0x5AU;
    in.flags                 = 0x00U;
    uint8_t out[HOST_RFCO_SUMMARY_PAYLOAD_LEN];
    (void)host_rfco_summary_pack(&in, out);
    CHECK(out[HOST_RFCO_SUMMARY_OFF_PERTX_COUNT_IN_WINDOW] == 0xA5U,
          "pertx_count_in_window got 0x%02X", out[186]);
    CHECK(out[HOST_RFCO_SUMMARY_OFF_SUMMARY_EMIT_COUNT] == 0x5AU,
          "summary_emit_count got 0x%02X", out[187]);
    CHECK(out[HOST_RFCO_SUMMARY_OFF_FLAGS] == 0x00U,
          "flags got 0x%02X", out[188]);
}

/* ============================================================
 * 14. flags FIRST_SINCE_BOOT bit
 * ============================================================ */
static void test_flags_first_since_boot(void) {
    CASE("flags FIRST_SINCE_BOOT bit round-trips");
    host_rfco_summary_t in = {0};
    in.flags = HOST_RFCO_SUMMARY_FLAG_FIRST_SINCE_BOOT;
    uint8_t out[HOST_RFCO_SUMMARY_PAYLOAD_LEN];
    (void)host_rfco_summary_pack(&in, out);
    CHECK((out[HOST_RFCO_SUMMARY_OFF_FLAGS] & HOST_RFCO_SUMMARY_FLAG_FIRST_SINCE_BOOT) != 0U,
          "FIRST_SINCE_BOOT bit should be set; flags = 0x%02X", out[188]);
}

/* ============================================================
 * 15. CRC byte order is LE
 * ============================================================ */
static void test_crc_le_byte_order(void) {
    CASE("CRC at offsets 189..190 is little-endian");
    host_rfco_summary_t in = {0};
    in.uptime_ms = 0x01020304UL;  /* something deterministic */
    uint8_t out[HOST_RFCO_SUMMARY_PAYLOAD_LEN];
    (void)host_rfco_summary_pack(&in, out);
    uint16_t want = crc_ref(out, HOST_RFCO_SUMMARY_OFF_PAYLOAD_CRC16);
    CHECK(out[HOST_RFCO_SUMMARY_OFF_PAYLOAD_CRC16] == (uint8_t)(want & 0xFFU),
          "CRC low byte: got 0x%02X, want 0x%02X",
          out[189], (uint8_t)(want & 0xFFU));
    CHECK(out[HOST_RFCO_SUMMARY_OFF_PAYLOAD_CRC16 + 1U] == (uint8_t)((want >> 8) & 0xFFU),
          "CRC high byte: got 0x%02X, want 0x%02X",
          out[190], (uint8_t)((want >> 8) & 0xFFU));
}

/* ============================================================
 * 16. Flipping any covered byte changes the CRC
 * ============================================================ */
static void test_crc_sensitivity(void) {
    CASE("flipping any byte in [0..188] changes the CRC");
    host_rfco_summary_t in = {0};
    in.uptime_ms = 0xCAFEBABEUL;
    in.summary_seq = 7U;
    for (uint8_t i = 0; i < HOST_RFCO_SUMMARY_CHANNEL_COUNT; ++i) {
        in.per_channel_hop_count[i] = (uint8_t)(i * 3U);
    }
    uint8_t out0[HOST_RFCO_SUMMARY_PAYLOAD_LEN];
    (void)host_rfco_summary_pack(&in, out0);
    uint16_t crc0 = read_u16_le(&out0[HOST_RFCO_SUMMARY_OFF_PAYLOAD_CRC16]);

    /* Probe a sparse set of representative bytes spanning the payload. */
    static const size_t probes[] = {0, 1, 2, 6, 7, 8, 15, 20, 69, 70, 169, 170, 185, 186, 188};
    for (size_t pi = 0; pi < sizeof(probes) / sizeof(probes[0]); ++pi) {
        uint8_t scratch[HOST_RFCO_SUMMARY_PAYLOAD_LEN];
        memcpy(scratch, out0, sizeof(scratch));
        scratch[probes[pi]] ^= 0x01U;  /* flip one bit */
        uint16_t crc_new = crc_ref(scratch, HOST_RFCO_SUMMARY_OFF_PAYLOAD_CRC16);
        CHECK(crc_new != crc0,
              "flipping byte %zu should change CRC (was 0x%04X)",
              probes[pi], crc0);
    }
}

/* ============================================================
 * 17. Repack overwrites the CRC bytes
 * ============================================================ */
static void test_repack_overwrites_crc(void) {
    CASE("pack always overwrites CRC bytes (caller-poisoned input ignored)");
    host_rfco_summary_t in = {0};
    in.summary_seq = 42UL;
    uint8_t out[HOST_RFCO_SUMMARY_PAYLOAD_LEN];
    memset(out, 0x00, sizeof(out));
    /* Pre-poison CRC region. */
    out[HOST_RFCO_SUMMARY_OFF_PAYLOAD_CRC16]      = 0xAAU;
    out[HOST_RFCO_SUMMARY_OFF_PAYLOAD_CRC16 + 1U] = 0xBBU;
    (void)host_rfco_summary_pack(&in, out);

    uint16_t want = crc_ref(out, HOST_RFCO_SUMMARY_OFF_PAYLOAD_CRC16);
    uint16_t got  = read_u16_le(&out[HOST_RFCO_SUMMARY_OFF_PAYLOAD_CRC16]);
    CHECK(got == want, "CRC after pack got 0x%04X, want 0x%04X (poison not cleared?)",
          got, want);
    CHECK(got != 0xBBAAU, "CRC must not equal pre-poisoned bytes");
}

/* ============================================================ */

int main(void) {
    test_crc_kat();
    test_null_in_rejected();
    test_null_out_rejected();
    test_payload_len_constant();
    test_schema_byte_hard_pinned();
    test_pertx_schema_stamps();
    test_reserved_align_hard_zeroed();
    test_all_zero_snapshot();
    test_u32_le_encoding();
    test_hop_count_round_trip();
    test_dwell_max_round_trip();
    test_blocked_attempts_round_trip();
    test_tail_bytes();
    test_flags_first_since_boot();
    test_crc_le_byte_order();
    test_crc_sensitivity();
    test_repack_overwrites_crc();

    if (g_failures != 0) {
        fprintf(stderr, "[FAIL] rfco_summary: %d failures across %d cases\n",
                g_failures, g_cases);
        return 1;
    }
    printf("[PASS] rfco_summary: %d cases\n", g_cases);
    (void)dump_bytes; /* silence unused-warning if no failure path uses it */
    return 0;
}
