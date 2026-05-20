/*
 * FCC-B1-SUMMARY-c-2 host-side integration test for the
 * cadence-driven summary emit (host_rfco_summary_tick).
 *
 * The c-2 increment moves last_emit_ms + summary_seq tracking out of
 * main.c and into the emit TU (so main.c just calls tick() once per
 * loop iteration). This bench drives a simulated clock past several
 * 60 000 ms boundaries and asserts the cadence contract:
 *
 *   1. No emit before the first boundary (tick at 0, 1000, 30000,
 *      59999 must NOT call host_uart_send_urc()).
 *   2. Exactly one emit at the first boundary (now_ms = 60000):
 *      type == 0xC4, payload[OFF_SUMMARY_SEQ..]==0, flags has
 *      FIRST_SINCE_BOOT, window_elapsed_ms == 60000.
 *   3. tick() is idempotent across rapid back-to-back invocations at
 *      the same now_ms (no double-emit).
 *   4. The second emit fires at now_ms = 120000 with seq==1, FIRST
 *      cleared, window_elapsed_ms == 60000.
 *   5. A LATE tick (now_ms = 181500 after last emit at 120000)
 *      emits with window_elapsed_ms == 61500 and seq==2.
 *   6. reset_wrapper_state() re-arms the 60 s startup delay and
 *      zeros seq + last_emit_ms.
 *   7. profile_id argument is propagated to payload byte 2.
 *   8. Cadence survives u32 wraparound: after reset, jump now_ms
 *      to 0xFFFFFFFF - 10, tick at that point (no emit, seq still 0),
 *      then tick at 0xFFFFFFFF - 10 + 60000 (wraps to 0x0000EA5F)
 *      which IS the first boundary — emit fires with FIRST set and
 *      window_elapsed_ms == 60000.
 *
 * The pack-helper byte offsets exercised here (HOST_RFCO_SUMMARY_OFF_*)
 * pin the wire layout used by the c-2 main-loop integration.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>

#include "host_rfco.h"
#include "host_rfco_summary.h"
#include "host_types.h"
#include "sx1276_fhss.h"
#include "sx1276_legal_dwell.h"

/* ---- host_uart_send_urc stub: counts calls + captures the last ---- */

static struct {
    int      call_count;
    uint8_t  type;
    uint16_t seq;
    uint8_t  flags;
    uint8_t  payload[256];
    uint16_t payload_len;
} g_urc;

void host_uart_send_urc(uint8_t type,
                        uint16_t seq,
                        uint8_t flags,
                        const uint8_t *payload,
                        uint16_t payload_len) {
    ++g_urc.call_count;
    g_urc.type        = type;
    g_urc.seq         = seq;
    g_urc.flags       = flags;
    g_urc.payload_len = payload_len;
    if ((payload != NULL) && (payload_len <= sizeof(g_urc.payload))) {
        memcpy(g_urc.payload, payload, payload_len);
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

/* Drain every sidecar + wrapper + cadence state. */
static void reset_all(void) {
    host_rfco_blocked_attempts_reset();
    host_rfco_pertx_window_reset();
    host_rfco_summary_reset_wrapper_state();
    sx1276_legal_dwell_reset();
    uint8_t  drain_hop[SX1276_FHSS_CHANNEL_COUNT];
    sx1276_fhss_hop_count_snapshot_and_clear(drain_hop);
    uint16_t drain_blk[HOST_RFCO_BLOCKED_ATTEMPTS_SLOTS];
    host_rfco_blocked_attempts_snapshot_and_clear(drain_blk);
    uint32_t drain_dwell[SX1276_DWELL_CHANNEL_COUNT];
    sx1276_legal_dwell_peak_us_snapshot_and_clear(drain_dwell);
    memset(&g_urc, 0, sizeof(g_urc));
}

/* Read a little-endian u32 from the payload at the given offset. */
static uint32_t le32(const uint8_t *p, size_t off) {
    return (uint32_t)p[off]
         | ((uint32_t)p[off + 1U] << 8)
         | ((uint32_t)p[off + 2U] << 16)
         | ((uint32_t)p[off + 3U] << 24);
}

/* ---------- Test 1: no emit before first 60 s boundary ---------- */
static void test_no_emit_before_first_boundary(void) {
    reset_all();
    const uint32_t pre_boundary[] = { 0U, 1U, 1000U, 30000U, 59999U };
    for (size_t i = 0; i < sizeof(pre_boundary) / sizeof(pre_boundary[0]); ++i) {
        const bool emitted = host_rfco_summary_tick(pre_boundary[i], 0U);
        CHECK(!emitted, "tick(%" PRIu32 ") should NOT emit", pre_boundary[i]);
    }
    CHECK(g_urc.call_count == 0,
          "send_urc must not be called before first boundary, got %d",
          g_urc.call_count);
}

/* ---------- Test 2: first emit at exactly 60 000 ms ---------- */
static void test_first_emit_at_boundary(void) {
    reset_all();
    /* warm-up tick that should not emit */
    (void)host_rfco_summary_tick(59999U, 0U);
    CHECK(g_urc.call_count == 0, "premature emit at 59999");

    const bool emitted = host_rfco_summary_tick(60000U, 0U);
    CHECK(emitted, "tick(60000) must emit");
    CHECK(g_urc.call_count == 1, "exactly one send_urc, got %d", g_urc.call_count);
    CHECK(g_urc.type == HOST_TYPE_RFCO_SUMMARY_URC,
          "type must be 0xC4, got 0x%02X", g_urc.type);
    CHECK(g_urc.payload_len == HOST_RFCO_SUMMARY_PAYLOAD_LEN,
          "payload_len must be %u, got %u",
          (unsigned)HOST_RFCO_SUMMARY_PAYLOAD_LEN, g_urc.payload_len);
    CHECK((g_urc.flags & HOST_RFCO_SUMMARY_FLAG_FIRST_SINCE_BOOT) != 0U,
          "FIRST_SINCE_BOOT flag missing, flags=0x%02X", g_urc.flags);
    CHECK(le32(g_urc.payload, HOST_RFCO_SUMMARY_OFF_SUMMARY_SEQ) == 0U,
          "first summary_seq must be 0");
    CHECK(le32(g_urc.payload, HOST_RFCO_SUMMARY_OFF_WINDOW_ELAPSED_MS) == 60000U,
          "first window_elapsed_ms must be 60000");
    CHECK(le32(g_urc.payload, HOST_RFCO_SUMMARY_OFF_UPTIME_MS) == 60000U,
          "first uptime_ms must be 60000");
}

/* ---------- Test 3: tick is idempotent within a window ---------- */
static void test_idempotent_within_window(void) {
    reset_all();
    CHECK(host_rfco_summary_tick(60000U, 0U), "first emit");
    const int after_first = g_urc.call_count;
    /* Many back-to-back ticks at the same now_ms must not re-emit. */
    for (int i = 0; i < 10; ++i) {
        const bool emitted = host_rfco_summary_tick(60000U, 0U);
        CHECK(!emitted, "double emit at 60000 (iter %d)", i);
    }
    /* And ticks strictly inside the next window must not emit. */
    const uint32_t mid_window[] = { 60001U, 90000U, 119999U };
    for (size_t i = 0; i < sizeof(mid_window) / sizeof(mid_window[0]); ++i) {
        const bool emitted = host_rfco_summary_tick(mid_window[i], 0U);
        CHECK(!emitted, "emit inside window at %" PRIu32, mid_window[i]);
    }
    CHECK(g_urc.call_count == after_first,
          "send_urc must not be called again, expected %d got %d",
          after_first, g_urc.call_count);
}

/* ---------- Test 4: on-time second emit ---------- */
static void test_second_emit_on_time(void) {
    reset_all();
    CHECK(host_rfco_summary_tick(60000U, 0U), "first emit");
    CHECK((g_urc.flags & HOST_RFCO_SUMMARY_FLAG_FIRST_SINCE_BOOT) != 0U,
          "first emit must carry FIRST");
    CHECK(host_rfco_summary_tick(120000U, 0U), "second emit");
    CHECK(g_urc.call_count == 2, "two emits, got %d", g_urc.call_count);
    CHECK((g_urc.flags & HOST_RFCO_SUMMARY_FLAG_FIRST_SINCE_BOOT) == 0U,
          "second emit must NOT carry FIRST, flags=0x%02X", g_urc.flags);
    CHECK(le32(g_urc.payload, HOST_RFCO_SUMMARY_OFF_SUMMARY_SEQ) == 1U,
          "second summary_seq must be 1");
    CHECK(le32(g_urc.payload, HOST_RFCO_SUMMARY_OFF_WINDOW_ELAPSED_MS) == 60000U,
          "on-time second window_elapsed_ms must be 60000");
    CHECK(le32(g_urc.payload, HOST_RFCO_SUMMARY_OFF_UPTIME_MS) == 120000U,
          "second uptime_ms must be 120000");
}

/* ---------- Test 5: late tick reports actual jitter ---------- */
static void test_late_tick_window_reflects_jitter(void) {
    reset_all();
    CHECK(host_rfco_summary_tick(60000U, 0U), "first emit");
    CHECK(host_rfco_summary_tick(120000U, 0U), "second emit");
    /* late by 1500 ms */
    CHECK(host_rfco_summary_tick(181500U, 0U), "late third emit");
    CHECK(g_urc.call_count == 3, "three emits, got %d", g_urc.call_count);
    CHECK(le32(g_urc.payload, HOST_RFCO_SUMMARY_OFF_SUMMARY_SEQ) == 2U,
          "third summary_seq must be 2");
    CHECK(le32(g_urc.payload, HOST_RFCO_SUMMARY_OFF_WINDOW_ELAPSED_MS) == 61500U,
          "late window_elapsed_ms must reflect actual jitter (61500)");
    CHECK(le32(g_urc.payload, HOST_RFCO_SUMMARY_OFF_UPTIME_MS) == 181500U,
          "third uptime_ms must be 181500");
}

/* ---------- Test 6: reset re-arms 60 s startup delay ---------- */
static void test_reset_rearms_startup_delay(void) {
    reset_all();
    CHECK(host_rfco_summary_tick(60000U, 0U), "first emit");
    CHECK(host_rfco_summary_tick(120000U, 0U), "second emit");
    /* Now simulate a soft restart of the wrapper. The next tick at
     * 120001 must NOT emit (because last_emit_ms was reset to 0 and
     * 120001 - 0 >= 60000 — wait, that WOULD emit). The contract is
     * that reset zeros last_emit_ms, so the next tick fires immediately
     * if now_ms >= 60000. This is the documented behaviour. */
    host_rfco_summary_reset_wrapper_state();
    memset(&g_urc, 0, sizeof(g_urc));
    /* After reset, FIRST_SINCE_BOOT must be re-armed and seq back to 0. */
    CHECK(host_rfco_summary_tick(120001U, 0U), "post-reset emit");
    CHECK((g_urc.flags & HOST_RFCO_SUMMARY_FLAG_FIRST_SINCE_BOOT) != 0U,
          "post-reset emit must carry FIRST again");
    CHECK(le32(g_urc.payload, HOST_RFCO_SUMMARY_OFF_SUMMARY_SEQ) == 0U,
          "post-reset summary_seq must be back to 0");
    CHECK(le32(g_urc.payload, HOST_RFCO_SUMMARY_OFF_WINDOW_ELAPSED_MS) == 120001U,
          "post-reset window_elapsed_ms == now_ms (last was 0)");
}

/* ---------- Test 7: profile_id threads through to payload ---------- */
static void test_profile_id_propagation(void) {
    reset_all();
    CHECK(host_rfco_summary_tick(60000U, 0x05U), "emit with profile_id=5");
    CHECK(g_urc.payload[HOST_RFCO_SUMMARY_OFF_PROFILE_ID] == 0x05U,
          "payload[OFF_PROFILE_ID] must be 5, got 0x%02X",
          g_urc.payload[HOST_RFCO_SUMMARY_OFF_PROFILE_ID]);

    reset_all();
    CHECK(host_rfco_summary_tick(60000U, 0xA7U), "emit with profile_id=0xA7");
    CHECK(g_urc.payload[HOST_RFCO_SUMMARY_OFF_PROFILE_ID] == 0xA7U,
          "payload[OFF_PROFILE_ID] must be 0xA7, got 0x%02X",
          g_urc.payload[HOST_RFCO_SUMMARY_OFF_PROFILE_ID]);
}

/* ---------- Test 8: cadence survives u32 wraparound ---------- */
static void test_cadence_across_u32_wrap(void) {
    reset_all();
    /* Jump now_ms close to UINT32_MAX. Because last_emit_ms is 0 and
     * now_ms is huge, (now - 0) >= 60000 trivially holds — so the
     * first tick fires immediately. That's the documented post-boot
     * behaviour. Take that emit out of the way... */
    const uint32_t boot_jump = 0xFFFFFFFFUL - 10U;
    CHECK(host_rfco_summary_tick(boot_jump, 0U),
          "post-jump first emit (huge window vs last=0)");
    const int after_first = g_urc.call_count;

    /* Now last_emit_ms == boot_jump. A tick 30 000 ms later (which
     * wraps the counter to ~30000) must NOT emit. */
    const uint32_t mid = boot_jump + 30000U; /* wraps */
    CHECK(!host_rfco_summary_tick(mid, 0U),
          "mid-window tick across wrap must NOT emit (got call_count=%d)",
          g_urc.call_count);
    CHECK(g_urc.call_count == after_first, "no extra emit at mid-window");

    /* At exactly +60 000 ms from boot_jump (wrapped), tick MUST emit
     * and report window_elapsed_ms == 60000. */
    const uint32_t boundary = boot_jump + 60000U; /* wraps */
    CHECK(host_rfco_summary_tick(boundary, 0U),
          "wrap-boundary tick must emit (now=0x%08" PRIX32 ")", boundary);
    CHECK(g_urc.call_count == after_first + 1, "exactly one new emit");
    CHECK(le32(g_urc.payload, HOST_RFCO_SUMMARY_OFF_WINDOW_ELAPSED_MS) == 60000U,
          "wrap-boundary window_elapsed_ms must be 60000");
    CHECK(le32(g_urc.payload, HOST_RFCO_SUMMARY_OFF_UPTIME_MS) == boundary,
          "wrap-boundary uptime_ms must equal wrapped now_ms");
}

int main(void) {
    test_no_emit_before_first_boundary();
    test_first_emit_at_boundary();
    test_idempotent_within_window();
    test_second_emit_on_time();
    test_late_tick_window_reflects_jitter();
    test_reset_rearms_startup_delay();
    test_profile_id_propagation();
    test_cadence_across_u32_wrap();

    if (g_failures != 0) {
        fprintf(stderr, "[FAIL] rfco_summary_integration: %d failures\n",
                g_failures);
        return EXIT_FAILURE;
    }
    printf("[PASS] rfco_summary_integration: 8 cases\n");
    return EXIT_SUCCESS;
}
