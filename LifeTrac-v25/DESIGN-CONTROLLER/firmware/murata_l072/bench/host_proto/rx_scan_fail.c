/*
 * FCC-A6c-3-c: host golden-vector test for the RX-scan FAIL eval
 * helper (radio/sx1276_rx_scan_fail.c).
 *
 * Locks the v5.0 §13.1 #2 sub-byte bit layout and the retry-vs-
 * final gate against unintended drift — scan_dispatch_action(FAIL)
 * in radio/sx1276_rx.c relies on this helper to decide whether to
 * emit a final URC + absorb in FAILED or emit a non-final URC +
 * re-enter SCANNING. Cross-decoders on the host side will index
 * dashboards by these bits.
 *
 * Cases:
 *   (1)  NULL input → {sub=0x80, is_final=true} (defensive).
 *   (2)  Cold first attempt, silent radio (no IRQ, no CRC, no warm,
 *        max=3) → sub = hw_suspect|attempt(0) = 0x20, final=false.
 *   (3)  Cold first attempt, CRC-only window → sub = crc_seen|
 *        attempt(0) = 0x10, final=false. hw_suspect MUST be 0.
 *   (4)  Cold first attempt, IRQs but no CRC → sub = attempt(0)
 *        = 0x00, final=false. (No CRCs and IRQs seen ⇒ neither
 *        hw_suspect nor crc_seen.)
 *   (5)  Cold first attempt, IRQs AND CRC → sub = crc_seen|
 *        attempt(0) = 0x10, final=false.
 *   (6)  Cold attempt 2 (= max-1), silent → sub = hw_suspect|
 *        attempt(2) = 0x22, final=false.
 *   (7)  Cold attempt 3 (= max), silent → sub = final|hw_suspect|
 *        attempt(3) = 0xA3, final=true.
 *   (8)  Warm boot, first FAIL, silent → sub = warm|hw_suspect|
 *        attempt(0) = 0x60, final=false.
 *   (9)  Warm boot, final FAIL, CRC seen → sub = final|warm|
 *        crc_seen|attempt(3) = 0xD3, final=true.
 *   (10) Attempt-nibble saturation: retries=255, max=255 → final
 *        true, attempt nibble pinned at 0x0F regardless of the
 *        high bits of retries.
 *   (11) max_retries=0 boundary: very first FAIL is also the
 *        final one → sub = final|hw_suspect|attempt(0) = 0xA0,
 *        is_final=true.
 *   (12) DIM/constants: SX1276_RX_SCAN_MAX_RETRIES == 3U (the
 *        v5.0 §13.1 lock) and every bit position is unique.
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "sx1276_rx_scan_fail.h"

static int g_failures = 0;

#define CHECK(cond, ...)                                                \
    do {                                                                \
        if (!(cond)) {                                                  \
            ++g_failures;                                               \
            fprintf(stderr, "[FAIL] %s:%d: ", __FILE__, __LINE__);      \
            fprintf(stderr, __VA_ARGS__);                               \
            fprintf(stderr, "\n");                                      \
        }                                                               \
    } while (0)

static void check_eval(const char *label,
                       sx1276_rx_scan_fail_eval_t got,
                       uint8_t want_sub,
                       bool    want_final) {
    CHECK(got.sub == want_sub,
          "%s: sub expected 0x%02X got 0x%02X",
          label, want_sub, got.sub);
    CHECK(got.is_final == want_final,
          "%s: is_final expected %d got %d",
          label, (int)want_final, (int)got.is_final);
}

static void test_null_input(void) {
    sx1276_rx_scan_fail_eval_t d = sx1276_rx_scan_fail_eval(NULL);
    check_eval("(1) NULL input", d, 0x80U, true);
}

static void test_cold_silent_first(void) {
    sx1276_rx_scan_fail_input_t in = {
        .retries     = 0U,
        .max_retries = 3U,
        .got_any_irq = false,
        .crc_seen    = false,
        .ever_locked = false,
    };
    check_eval("(2) cold silent first",
               sx1276_rx_scan_fail_eval(&in),
               0x20U, false);
}

static void test_cold_crc_only_first(void) {
    sx1276_rx_scan_fail_input_t in = {
        .retries     = 0U,
        .max_retries = 3U,
        .got_any_irq = true,    /* CRC IRQ counts as a DIO0 IRQ */
        .crc_seen    = true,
        .ever_locked = false,
    };
    check_eval("(3) cold crc-only first",
               sx1276_rx_scan_fail_eval(&in),
               0x10U, false);
}

static void test_cold_irq_only_first(void) {
    sx1276_rx_scan_fail_input_t in = {
        .retries     = 0U,
        .max_retries = 3U,
        .got_any_irq = true,
        .crc_seen    = false,
        .ever_locked = false,
    };
    check_eval("(4) cold irq-only first",
               sx1276_rx_scan_fail_eval(&in),
               0x00U, false);
}

static void test_cold_irq_and_crc_first(void) {
    sx1276_rx_scan_fail_input_t in = {
        .retries     = 0U,
        .max_retries = 3U,
        .got_any_irq = true,
        .crc_seen    = true,
        .ever_locked = false,
    };
    check_eval("(5) cold irq+crc first",
               sx1276_rx_scan_fail_eval(&in),
               0x10U, false);
}

static void test_cold_silent_attempt2(void) {
    sx1276_rx_scan_fail_input_t in = {
        .retries     = 2U,
        .max_retries = 3U,
        .got_any_irq = false,
        .crc_seen    = false,
        .ever_locked = false,
    };
    check_eval("(6) cold silent attempt 2",
               sx1276_rx_scan_fail_eval(&in),
               0x22U, false);
}

static void test_cold_silent_final(void) {
    sx1276_rx_scan_fail_input_t in = {
        .retries     = 3U,
        .max_retries = 3U,
        .got_any_irq = false,
        .crc_seen    = false,
        .ever_locked = false,
    };
    check_eval("(7) cold silent final",
               sx1276_rx_scan_fail_eval(&in),
               0xA3U, true);
}

static void test_warm_silent_first(void) {
    sx1276_rx_scan_fail_input_t in = {
        .retries     = 0U,
        .max_retries = 3U,
        .got_any_irq = false,
        .crc_seen    = false,
        .ever_locked = true,
    };
    check_eval("(8) warm silent first",
               sx1276_rx_scan_fail_eval(&in),
               0x60U, false);
}

static void test_warm_crc_final(void) {
    sx1276_rx_scan_fail_input_t in = {
        .retries     = 3U,
        .max_retries = 3U,
        .got_any_irq = true,
        .crc_seen    = true,
        .ever_locked = true,
    };
    check_eval("(9) warm crc final",
               sx1276_rx_scan_fail_eval(&in),
               0xD3U, true);
}

static void test_attempt_saturation(void) {
    sx1276_rx_scan_fail_input_t in = {
        .retries     = 0xFFU,
        .max_retries = 0xFFU,
        .got_any_irq = false,
        .crc_seen    = false,
        .ever_locked = false,
    };
    sx1276_rx_scan_fail_eval_t d = sx1276_rx_scan_fail_eval(&in);
    CHECK(d.is_final == true,
          "(10) retries=max=255 must be final, got is_final=%d",
          (int)d.is_final);
    CHECK((d.sub & SX1276_RX_SCAN_FAIL_SUB_MASK_ATTEMPT) == 0x0FU,
          "(10) attempt nibble must saturate at 0x0F, got 0x%02X",
          d.sub & SX1276_RX_SCAN_FAIL_SUB_MASK_ATTEMPT);
    /* sub upper nibble: final(0x80) | hw_suspect(0x20) = 0xA0;
     * combined with saturated attempt(0x0F) ⇒ 0xAF. */
    CHECK(d.sub == 0xAFU,
          "(10) saturated silent-final sub expected 0xAF got 0x%02X",
          d.sub);
}

static void test_max_zero_boundary(void) {
    sx1276_rx_scan_fail_input_t in = {
        .retries     = 0U,
        .max_retries = 0U,
        .got_any_irq = false,
        .crc_seen    = false,
        .ever_locked = false,
    };
    check_eval("(11) max=0 first-is-final boundary",
               sx1276_rx_scan_fail_eval(&in),
               0xA0U, true);
}

static void test_constants(void) {
    /* v5.0 §13.1 lock: 3 retries (4 total attempts). */
    CHECK(SX1276_RX_SCAN_MAX_RETRIES == 3U,
          "(12a) SX1276_RX_SCAN_MAX_RETRIES expected 3 got %u",
          (unsigned)SX1276_RX_SCAN_MAX_RETRIES);
    /* Bit positions must be unique (no collisions). */
    const uint8_t bits[] = {
        SX1276_RX_SCAN_FAIL_SUB_BIT_FINAL,
        SX1276_RX_SCAN_FAIL_SUB_BIT_WARM,
        SX1276_RX_SCAN_FAIL_SUB_BIT_HW_SUSPECT,
        SX1276_RX_SCAN_FAIL_SUB_BIT_CRC_SEEN,
    };
    uint8_t mask = 0U;
    for (unsigned i = 0; i < sizeof(bits) / sizeof(bits[0]); ++i) {
        const uint8_t b = (uint8_t)(1U << bits[i]);
        CHECK((mask & b) == 0U,
              "(12b) status bit position %u collides with mask 0x%02X",
              (unsigned)bits[i], mask);
        mask = (uint8_t)(mask | b);
    }
    /* And none of them must overlap the attempt nibble. */
    CHECK((mask & SX1276_RX_SCAN_FAIL_SUB_MASK_ATTEMPT) == 0U,
          "(12c) status bits 0x%02X overlap attempt nibble 0x%02X",
          mask, SX1276_RX_SCAN_FAIL_SUB_MASK_ATTEMPT);
}

int main(void) {
    test_null_input();
    test_cold_silent_first();
    test_cold_crc_only_first();
    test_cold_irq_only_first();
    test_cold_irq_and_crc_first();
    test_cold_silent_attempt2();
    test_cold_silent_final();
    test_warm_silent_first();
    test_warm_crc_final();
    test_attempt_saturation();
    test_max_zero_boundary();
    test_constants();

    if (g_failures != 0) {
        fprintf(stderr, "[FAIL] rx_scan_fail: %d case(s) failed\n",
                g_failures);
        return 1;
    }
    printf("[PASS] rx_scan_fail: 12 cases\n");
    return 0;
}
