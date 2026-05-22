/*
 * P3 / Phase 3.2 — frames_per_dwell() host<->firmware parity test.
 *
 * Plan ref: LifeTrac-v25/AI NOTES/2026-05-21_Open_Problems_To_Fix_v1_0.md P3.
 *
 * Asserts that the C mirror lora_frames_per_dwell() produces the same
 * values as the Python authority frames_per_dwell() in
 * LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/
 * w2_02_host_pipeline.py, for the anchor cases pinned in that file's
 * docstring AND for the boundary/edge cases the Python function
 * documents.
 *
 * Computed reference values (verified manually against the Python
 * integer arithmetic; all use LEGAL_DWELL_US=400_000, HEADROOM=85%,
 * so budget = 340_000 us):
 *
 *   toa_us =       0     -> 1   (degenerate)
 *   toa_us =   1_000     -> 8   (cap binds; 340 floor)
 *   toa_us =  28_000     -> 8   (SF7/BW250/64B anchor; 340_000/28_000=12 -> cap 8)
 *   toa_us =  42_500     -> 8   (boundary just at cap; 340_000/42_500=8)
 *   toa_us =  42_501     -> 7   (one above boundary; floor drops below cap)
 *   toa_us =  60_000     -> 5
 *   toa_us = 113_333     -> 3   (just at "3" floor; 340_000/113_333=3)
 *   toa_us = 115_000     -> 2   (SF9/BW250/64B anchor; 340_000/115_000=2.95 -> 2)
 *   toa_us = 170_000     -> 2   (340_000/170_000=2)
 *   toa_us = 170_001     -> 1   (just above the 2-floor)
 *   toa_us = 340_000     -> 1   (340_000/340_000=1)
 *   toa_us = 340_001     -> 1   (degenerate-large; clamped up to 1)
 *   toa_us = 1_000_000   -> 1   (ToA already over budget)
 *   toa_us = UINT32_MAX  -> 1   (no overflow; floor(0)->clamped to 1)
 *
 * The Python anchors in w2_02_host_pipeline.py (SF7 -> 8, SF9 -> 2-3)
 * are covered by the 28_000 and 113_333/115_000 cases.
 */

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>

#include "lora_frames_per_dwell.h"

static int g_failures = 0;

static void check_eq(uint32_t toa_us, uint8_t expected) {
    const uint8_t got = lora_frames_per_dwell(toa_us);
    if (got != expected) {
        ++g_failures;
        fprintf(stderr,
                "FAIL frames_per_dwell(toa_us=%" PRIu32 "): expected %u, got %u\n",
                toa_us, (unsigned)expected, (unsigned)got);
    }
}

static void check_constants(void) {
    /* Parity constants must match the Python authority. */
    if (LORA_FPD_LEGAL_DWELL_US != 400000UL) {
        ++g_failures;
        fprintf(stderr, "FAIL LORA_FPD_LEGAL_DWELL_US != 400000\n");
    }
    if (LORA_FPD_DWELL_HEADROOM_PCT != 85U) {
        ++g_failures;
        fprintf(stderr, "FAIL LORA_FPD_DWELL_HEADROOM_PCT != 85\n");
    }
    if (LORA_FPD_MAX_FRAMES_PER_DWELL_CAP != 8U) {
        ++g_failures;
        fprintf(stderr, "FAIL LORA_FPD_MAX_FRAMES_PER_DWELL_CAP != 8\n");
    }
    /* Min inter-cycle pacing (host walk_power matrix verdict 2026-05-21):
     * 50 ms == 0.05 s, matching Python's MIN_LORA_HOST_INTER_CYCLE_S. */
    if (LORA_FPD_MIN_INTER_CYCLE_MS != 50U) {
        ++g_failures;
        fprintf(stderr, "FAIL LORA_FPD_MIN_INTER_CYCLE_MS != 50\n");
    }
}

int main(void) {
    check_constants();

    /* Degenerate / clamp-to-1 cases. */
    check_eq(0U,           1U);
    check_eq(340001U,      1U);
    check_eq(1000000U,     1U);
    check_eq(UINT32_MAX,   1U);

    /* Cap-binds region. */
    check_eq(1000U,        8U);
    check_eq(28000U,       8U); /* SF7/BW250/64B host anchor */
    check_eq(42500U,       8U); /* boundary at cap */
    check_eq(42501U,       7U); /* just past cap boundary */

    /* ToA-binds region. */
    check_eq(60000U,       5U);
    check_eq(113333U,      3U); /* 340000/113333 = 3 */
    check_eq(115000U,      2U); /* SF9/BW250/64B host anchor */
    check_eq(170000U,      2U);
    check_eq(170001U,      1U);
    check_eq(340000U,      1U);

    if (g_failures != 0) {
        fprintf(stderr, "frames_per_dwell parity FAIL: %d cases\n", g_failures);
        return 1;
    }
    printf("[OK] frames_per_dwell parity (14 cases + 4 constants)\n");
    return 0;
}
