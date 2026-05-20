/*
 * FCC-B1-PERTX host-side test for the per-TX RFCO URC packer + emitter.
 *
 * Coverage:
 *   1. Golden vector: nominal mid-range values pack to a known 21-byte
 *      sequence. Verifies field offsets and little-endian layout.
 *   2. Golden vector: all-zero snapshot still emits schema_ver=1 in
 *      byte 0 (schema byte is hard-coded, not field-driven).
 *   3. Golden vector: edge values (UINT32_MAX, profile_id=0xFF,
 *      channel_idx=63) pack without truncation.
 *   4. NULL input rejected by pack() and emit() without writing to out.
 *   5. emit() forwards (HOST_TYPE_RFCO_PERTX_URC, seq, flags, payload,
 *      HOST_RFCO_PERTX_PAYLOAD_LEN) to host_uart_send_urc(), and the
 *      payload matches what pack() produces.
 *   6. Status enum sentinel: INTERNAL == 0xFF survives the pack round
 *      trip (catches signed-byte mistakes).
 *   7. Schema version byte is exactly 1 (catches accidental bumps).
 *   8. Payload length constant is exactly 21 (catches layout drift).
 *
 * FCC-B1-SUMMARY-b-3 (sidecar wiring at the per-TX emit boundary):
 *   9.  emit(status=OK, hop_idx=N) bumps per-channel hop_count[N] by 1
 *       and leaves blocked_attempts[] all zero.
 *  10. emit(status=ABORT_LBT, hop_idx=N) bumps blocked_attempts[slot=2]
 *       by 1 and does NOT bump hop_count[N] (a blocked TX never went
 *       on air).
 *  11. emit(status=INTERNAL) bumps blocked_attempts[slot=7] (0xFF is
 *       remapped per the documented slot table).
 *  12. Repeated emits accumulate across snapshots: 5x emit(OK,hop=22)
 *       leaves hop_count[22]==5, all other channels zero.
 *  13. emit(NULL) does NOT touch either counter (no spurious bumps
 *       from rejected emits).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>

#include "host_rfco.h"
#include "host_types.h"
#include "sx1276_fhss.h"

/* ---- host_uart_send_urc stub: captures the last call ---- */

static struct {
    int      called;
    uint8_t  type;
    uint16_t seq;
    uint8_t  flags;
    uint8_t  payload[64];
    uint16_t payload_len;
} g_last_urc;

void host_uart_send_urc(uint8_t type,
                        uint16_t seq,
                        uint8_t flags,
                        const uint8_t *payload,
                        uint16_t payload_len) {
    g_last_urc.called      = 1;
    g_last_urc.type        = type;
    g_last_urc.seq         = seq;
    g_last_urc.flags       = flags;
    g_last_urc.payload_len = payload_len;
    if ((payload != NULL) && (payload_len <= sizeof(g_last_urc.payload))) {
        memcpy(g_last_urc.payload, payload, payload_len);
    }
}

/* ---- test harness ---- */

static int g_failures = 0;

#define CHECK(cond, ...) do {                                          \
    if (!(cond)) {                                                     \
        ++g_failures;                                                  \
        fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__);           \
        fprintf(stderr, __VA_ARGS__);                                  \
        fprintf(stderr, "\n");                                         \
    }                                                                  \
} while (0)

static void dump_bytes(const char *label, const uint8_t *b, size_t n) {
    fprintf(stderr, "  %s:", label);
    for (size_t i = 0; i < n; ++i) fprintf(stderr, " %02X", b[i]);
    fprintf(stderr, "\n");
}

static void check_bytes_eq(const char *label,
                           const uint8_t *got,
                           const uint8_t *want,
                           size_t n) {
    if (memcmp(got, want, n) != 0) {
        ++g_failures;
        fprintf(stderr, "FAIL %s: byte mismatch (n=%zu)\n", label, n);
        dump_bytes("got ", got,  n);
        dump_bytes("want", want, n);
    }
}

/* ---- 1. Nominal golden vector ----
 *
 * profile_id=1 (FCC_15_247_FHSS_50CH_BW250),
 * tx_status=0 (OK),
 * hop_idx=7,
 * channel_idx=23,
 * epoch=0x00112233,
 * freq_hz=915500000 = 0x36916BE0,
 * pkt_toa_us=12_345 = 0x00003039,
 * legal_dwell_used_us_10s=200_000 = 0x00030D40.
 *
 * Expected 21 bytes (little-endian):
 *   00: 01                          schema_ver
 *   01: 01                          profile_id
 *   02: 00                          tx_status
 *   03: 07                          hop_idx
 *   04: 17                          channel_idx (=23)
 *   05: 33 22 11 00                 epoch
 *   09: E0 6B 91 36                 freq_hz
 *   13: 39 30 00 00                 pkt_toa_us
 *   17: 40 0D 03 00                 legal_dwell_used_us_10s
 */
static void test_nominal_golden_vector(void) {
    const host_rfco_pertx_t snap = {
        .profile_id              = 1U,
        .tx_status               = HOST_RFCO_TX_STATUS_OK,
        .hop_idx                 = 7U,
        .channel_idx             = 23U,
        .epoch                   = 0x00112233UL,
        .freq_hz                 = 915500000UL,
        .pkt_toa_us              = 12345UL,
        .legal_dwell_used_us_10s = 200000UL,
    };
    const uint8_t expected[HOST_RFCO_PERTX_PAYLOAD_LEN] = {
        0x01,                          /* schema_ver */
        0x01,                          /* profile_id */
        0x00,                          /* tx_status */
        0x07,                          /* hop_idx */
        0x17,                          /* channel_idx */
        0x33, 0x22, 0x11, 0x00,        /* epoch */
        0xE0, 0x6B, 0x91, 0x36,        /* freq_hz = 915500000 */
        0x39, 0x30, 0x00, 0x00,        /* pkt_toa_us = 12345 */
        0x40, 0x0D, 0x03, 0x00,        /* legal_dwell_used_us_10s = 200000 */
    };
    uint8_t out[HOST_RFCO_PERTX_PAYLOAD_LEN] = {0};
    const bool ok = host_rfco_pertx_pack(&snap, out);
    CHECK(ok, "pack must succeed for nominal vector");
    check_bytes_eq("nominal", out, expected, HOST_RFCO_PERTX_PAYLOAD_LEN);
}

/* ---- 2. All-zero snapshot — schema_ver byte must still be 1 ---- */
static void test_all_zero_still_has_schema_ver(void) {
    const host_rfco_pertx_t snap = {0};
    uint8_t out[HOST_RFCO_PERTX_PAYLOAD_LEN];
    memset(out, 0xAA, sizeof(out));  /* poison */
    const bool ok = host_rfco_pertx_pack(&snap, out);
    CHECK(ok, "pack must succeed on zero snapshot");
    CHECK(out[0] == HOST_RFCO_PERTX_SCHEMA_VER,
          "byte 0 must be schema_ver=1, got 0x%02X", out[0]);
    for (size_t i = 1; i < HOST_RFCO_PERTX_PAYLOAD_LEN; ++i) {
        CHECK(out[i] == 0x00,
              "byte %zu must be 0 for zero snapshot, got 0x%02X", i, out[i]);
    }
}

/* ---- 3. Edge values pack without truncation ---- */
static void test_edge_values(void) {
    const host_rfco_pertx_t snap = {
        .profile_id              = 0xFFU,
        .tx_status               = HOST_RFCO_TX_STATUS_INTERNAL,
        .hop_idx                 = 49U,
        .channel_idx             = 63U,
        .epoch                   = UINT32_MAX,
        .freq_hz                 = UINT32_MAX,
        .pkt_toa_us              = UINT32_MAX,
        .legal_dwell_used_us_10s = UINT32_MAX,
    };
    uint8_t out[HOST_RFCO_PERTX_PAYLOAD_LEN] = {0};
    CHECK(host_rfco_pertx_pack(&snap, out), "pack must succeed for edge vector");
    CHECK(out[0] == 0x01U, "schema_ver");
    CHECK(out[1] == 0xFFU, "profile_id");
    CHECK(out[2] == 0xFFU, "tx_status INTERNAL must survive round-trip as 0xFF");
    CHECK(out[3] == 49U,   "hop_idx");
    CHECK(out[4] == 63U,   "channel_idx");
    for (size_t base = 5; base <= 17; base += 4) {
        for (size_t i = 0; i < 4; ++i) {
            CHECK(out[base + i] == 0xFFU,
                  "u32 field at off=%zu byte %zu expected 0xFF, got 0x%02X",
                  base, i, out[base + i]);
        }
    }
}

/* ---- 4. NULL pointers rejected ---- */
static void test_null_input_rejected(void) {
    uint8_t out[HOST_RFCO_PERTX_PAYLOAD_LEN];
    memset(out, 0xCC, sizeof(out));
    CHECK(!host_rfco_pertx_pack(NULL, out),  "pack(NULL in) must return false");
    CHECK(!host_rfco_pertx_pack(&(host_rfco_pertx_t){0}, NULL),
          "pack(NULL out) must return false");
    /* Out buffer must NOT have been written when in==NULL. */
    for (size_t i = 0; i < sizeof(out); ++i) {
        CHECK(out[i] == 0xCCU, "out must be untouched on NULL in");
    }
    g_last_urc.called = 0;
    CHECK(!host_rfco_pertx_emit(0, 0, NULL),
          "emit(NULL snapshot) must return false");
    CHECK(g_last_urc.called == 0,
          "emit(NULL) must NOT call host_uart_send_urc");
}

/* ---- 5. emit() forwards correct (type, payload, len) ---- */
static void test_emit_forwards_to_send_urc(void) {
    const host_rfco_pertx_t snap = {
        .profile_id              = 1U,
        .tx_status               = HOST_RFCO_TX_STATUS_ABORT_LBT,
        .hop_idx                 = 12U,
        .channel_idx             = 5U,
        .epoch                   = 0xDEADBEEFUL,
        .freq_hz                 = 903250000UL,
        .pkt_toa_us              = 5000UL,
        .legal_dwell_used_us_10s = 350000UL,
    };
    memset(&g_last_urc, 0, sizeof(g_last_urc));
    const bool ok = host_rfco_pertx_emit(0x1234U, 0x42U, &snap);
    CHECK(ok, "emit must succeed for valid snapshot");
    CHECK(g_last_urc.called == 1,           "send_urc must have been called");
    CHECK(g_last_urc.type == HOST_TYPE_RFCO_PERTX_URC,
          "URC type must be 0xC3, got 0x%02X", g_last_urc.type);
    CHECK(g_last_urc.seq == 0x1234U,        "seq must be forwarded");
    CHECK(g_last_urc.flags == 0x42U,        "flags must be forwarded");
    CHECK(g_last_urc.payload_len == HOST_RFCO_PERTX_PAYLOAD_LEN,
          "payload_len must be 21, got %u", g_last_urc.payload_len);

    /* Re-pack independently and compare. */
    uint8_t expected[HOST_RFCO_PERTX_PAYLOAD_LEN];
    (void)host_rfco_pertx_pack(&snap, expected);
    check_bytes_eq("emit-forwards-payload",
                   g_last_urc.payload, expected,
                   HOST_RFCO_PERTX_PAYLOAD_LEN);
}

/* ---- 6, 7, 8. Constants are exactly what bench post-processing expects ---- */
static void test_constants(void) {
    CHECK(HOST_RFCO_PERTX_SCHEMA_VER  == 1U,
          "schema ver must be 1 until additive payload extension");
    CHECK(HOST_RFCO_PERTX_PAYLOAD_LEN == 21U,
          "payload len must be 21; layout drift requires a schema bump");
    CHECK(HOST_TYPE_RFCO_PERTX_URC    == 0xC3U,
          "URC type must be 0xC3 (next free slot after 0xC2 invariant reject)");
    /* Status enum sentinels — bench parsers depend on these. */
    CHECK(HOST_RFCO_TX_STATUS_OK                       == 0U,   "OK==0");
    CHECK(HOST_RFCO_TX_STATUS_ABORT_AIRTIME_INVARIANT  == 1U,   "ABORT_AIRTIME==1");
    CHECK(HOST_RFCO_TX_STATUS_ABORT_LBT                == 2U,   "ABORT_LBT==2");
    CHECK(HOST_RFCO_TX_STATUS_ABORT_LEGAL_DWELL        == 3U,   "ABORT_LEGAL_DWELL==3");
    CHECK(HOST_RFCO_TX_STATUS_ABORT_QOS                == 4U,   "ABORT_QOS==4");
    CHECK(HOST_RFCO_TX_STATUS_TX_TIMEOUT               == 5U,   "TX_TIMEOUT==5");
    CHECK(HOST_RFCO_TX_STATUS_TX_FAIL                  == 6U,   "TX_FAIL==6");
    CHECK(HOST_RFCO_TX_STATUS_INTERNAL                 == 0xFFU,"INTERNAL==0xFF");
}

/* ---- 9..13. FCC-B1-SUMMARY-b-3 sidecar wiring at emit boundary ----
 *
 * host_rfco_pertx_emit() must (a) call host_rfco_blocked_attempts_record
 * with snapshot->tx_status on every accepted call (record() internally
 * skips OK), and (b) call sx1276_fhss_record_hop(snapshot->hop_idx)
 * iff tx_status == OK. NULL snapshots must short-circuit before either
 * counter is touched. These tests pin that wiring contract — without
 * them, a refactor could silently drop the per-minute SUMMARY URC
 * sidecar inputs.
 */

static void reset_sidecar_counters(void) {
    host_rfco_blocked_attempts_reset();
    uint8_t  drain_hop[SX1276_FHSS_CHANNEL_COUNT];
    sx1276_fhss_hop_count_snapshot_and_clear(drain_hop);
    uint16_t drain_blk[HOST_RFCO_BLOCKED_ATTEMPTS_SLOTS];
    host_rfco_blocked_attempts_snapshot_and_clear(drain_blk);
    memset(&g_last_urc, 0, sizeof(g_last_urc));
}

static host_rfco_pertx_t make_snap(uint8_t tx_status, uint8_t hop_idx) {
    host_rfco_pertx_t snap = {
        .profile_id              = 1U,
        .tx_status               = tx_status,
        .hop_idx                 = hop_idx,
        .channel_idx             = hop_idx,
        .epoch                   = 0U,
        .freq_hz                 = 915000000UL,
        .pkt_toa_us              = 1000UL,
        .legal_dwell_used_us_10s = 0U,
    };
    return snap;
}

static void test_emit_wires_hop_count_on_ok(void) {
    reset_sidecar_counters();
    host_rfco_pertx_t snap = make_snap(HOST_RFCO_TX_STATUS_OK, 7U);
    CHECK(host_rfco_pertx_emit(0U, 0U, &snap),
          "emit(OK) must succeed");
    uint8_t  hop[SX1276_FHSS_CHANNEL_COUNT];
    sx1276_fhss_hop_count_snapshot_and_clear(hop);
    CHECK(hop[7] == 1U, "hop_count[7] must be 1 after emit(OK,hop=7), got %u",
          hop[7]);
    for (uint8_t i = 0; i < SX1276_FHSS_CHANNEL_COUNT; ++i) {
        if (i == 7U) continue;
        CHECK(hop[i] == 0U, "hop_count[%u] must be 0, got %u", i, hop[i]);
    }
    uint16_t blk[HOST_RFCO_BLOCKED_ATTEMPTS_SLOTS];
    host_rfco_blocked_attempts_snapshot_and_clear(blk);
    for (uint8_t i = 0; i < HOST_RFCO_BLOCKED_ATTEMPTS_SLOTS; ++i) {
        CHECK(blk[i] == 0U,
              "blocked_attempts[%u] must be 0 after emit(OK), got %u",
              i, blk[i]);
    }
}

static void test_emit_wires_blocked_on_abort_lbt(void) {
    reset_sidecar_counters();
    host_rfco_pertx_t snap = make_snap(HOST_RFCO_TX_STATUS_ABORT_LBT, 12U);
    CHECK(host_rfco_pertx_emit(0U, 0U, &snap),
          "emit(ABORT_LBT) must succeed");
    uint16_t blk[HOST_RFCO_BLOCKED_ATTEMPTS_SLOTS];
    host_rfco_blocked_attempts_snapshot_and_clear(blk);
    CHECK(blk[2] == 1U, "blocked_attempts[slot=2/LBT] must be 1, got %u", blk[2]);
    for (uint8_t i = 0; i < HOST_RFCO_BLOCKED_ATTEMPTS_SLOTS; ++i) {
        if (i == 2U) continue;
        CHECK(blk[i] == 0U, "blocked_attempts[%u] must be 0, got %u", i, blk[i]);
    }
    uint8_t hop[SX1276_FHSS_CHANNEL_COUNT];
    sx1276_fhss_hop_count_snapshot_and_clear(hop);
    CHECK(hop[12] == 0U,
          "hop_count[12] must be 0 (blocked TX must not inflate histogram), got %u",
          hop[12]);
}

static void test_emit_wires_blocked_on_internal(void) {
    reset_sidecar_counters();
    host_rfco_pertx_t snap = make_snap(HOST_RFCO_TX_STATUS_INTERNAL, 3U);
    CHECK(host_rfco_pertx_emit(0U, 0U, &snap),
          "emit(INTERNAL) must succeed");
    uint16_t blk[HOST_RFCO_BLOCKED_ATTEMPTS_SLOTS];
    host_rfco_blocked_attempts_snapshot_and_clear(blk);
    CHECK(blk[7] == 1U,
          "blocked_attempts[slot=7/INTERNAL] must be 1, got %u", blk[7]);
    for (uint8_t i = 0; i < 7U; ++i) {
        CHECK(blk[i] == 0U, "blocked_attempts[%u] must be 0, got %u", i, blk[i]);
    }
}

static void test_emit_accumulates_repeated_ok(void) {
    reset_sidecar_counters();
    host_rfco_pertx_t snap = make_snap(HOST_RFCO_TX_STATUS_OK, 22U);
    for (uint8_t i = 0; i < 5U; ++i) {
        CHECK(host_rfco_pertx_emit(i, 0U, &snap),
              "emit #%u must succeed", i);
    }
    uint8_t hop[SX1276_FHSS_CHANNEL_COUNT];
    sx1276_fhss_hop_count_snapshot_and_clear(hop);
    CHECK(hop[22] == 5U, "hop_count[22] must be 5 after 5x emit, got %u",
          hop[22]);
    for (uint8_t i = 0; i < SX1276_FHSS_CHANNEL_COUNT; ++i) {
        if (i == 22U) continue;
        CHECK(hop[i] == 0U, "hop_count[%u] must be 0, got %u", i, hop[i]);
    }
}

static void test_emit_null_does_not_bump_counters(void) {
    reset_sidecar_counters();
    CHECK(!host_rfco_pertx_emit(0U, 0U, NULL),
          "emit(NULL) must return false");
    uint8_t  hop[SX1276_FHSS_CHANNEL_COUNT];
    sx1276_fhss_hop_count_snapshot_and_clear(hop);
    for (uint8_t i = 0; i < SX1276_FHSS_CHANNEL_COUNT; ++i) {
        CHECK(hop[i] == 0U,
              "hop_count[%u] must remain 0 after emit(NULL), got %u",
              i, hop[i]);
    }
    uint16_t blk[HOST_RFCO_BLOCKED_ATTEMPTS_SLOTS];
    host_rfco_blocked_attempts_snapshot_and_clear(blk);
    for (uint8_t i = 0; i < HOST_RFCO_BLOCKED_ATTEMPTS_SLOTS; ++i) {
        CHECK(blk[i] == 0U,
              "blocked_attempts[%u] must remain 0 after emit(NULL), got %u",
              i, blk[i]);
    }
}

int main(void) {
    test_nominal_golden_vector();
    test_all_zero_still_has_schema_ver();
    test_edge_values();
    test_null_input_rejected();
    test_emit_forwards_to_send_urc();
    test_constants();
    test_emit_wires_hop_count_on_ok();
    test_emit_wires_blocked_on_abort_lbt();
    test_emit_wires_blocked_on_internal();
    test_emit_accumulates_repeated_ok();
    test_emit_null_does_not_bump_counters();

    if (g_failures != 0) {
        fprintf(stderr, "[FAIL] rfco_pertx: %d failure(s)\n", g_failures);
        return EXIT_FAILURE;
    }
    printf("[PASS] rfco_pertx: 11 cases\n");
    return EXIT_SUCCESS;
}
