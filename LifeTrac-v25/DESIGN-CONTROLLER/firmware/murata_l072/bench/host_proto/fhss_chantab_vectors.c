/*
 * Host-side unit test + golden vectors for sx1276_fhss_chantab.
 *
 * Built and run by `make check-fhss-chantab` (see Makefile). Confirms:
 *   - count == 50
 *   - first / last centers exactly 902.750 / 927.250 MHz
 *   - uniform 500 kHz spacing across all 49 intervals
 *   - all centers inside 902-928 MHz with >= 750 kHz edge guard
 *   - out-of-range indices return 0
 *
 * Golden vectors (idx -> Hz) below are the cross-side parity contract
 * with bench/host_proto/fhss_chantab.py and any X8-side Python tooling.
 * If any of these change, both this C test and the Python mirror must
 * change in the same commit.
 */

#include "sx1276_fhss_chantab.h"

#include <stdint.h>
#include <stdio.h>

static uint32_t g_failures;

static void check(const char *name, int cond) {
    if (!cond) {
        printf("[FAIL] %s\n", name);
        g_failures++;
    }
}

/* Golden vectors. Same numbers as bench/host_proto/fhss_chantab.py. */
typedef struct {
    uint8_t  idx;
    uint32_t hz;
} golden_t;

static const golden_t k_golden[] = {
    {  0U, 902750000UL },
    {  1U, 903250000UL },
    {  9U, 907250000UL },
    { 24U, 914750000UL }, /* mid-band */
    { 25U, 915250000UL },
    { 49U, 927250000UL },
};

int main(void) {
    /* Count. */
    check("count == 50", sx1276_fhss_chantab_count() == 50U);

    /* Golden vectors. */
    for (unsigned i = 0; i < sizeof(k_golden) / sizeof(k_golden[0]); ++i) {
        uint32_t got = sx1276_fhss_chantab_center_hz(k_golden[i].idx);
        if (got != k_golden[i].hz) {
            printf("[FAIL] golden idx=%u expected=%lu got=%lu\n",
                   (unsigned)k_golden[i].idx,
                   (unsigned long)k_golden[i].hz,
                   (unsigned long)got);
            g_failures++;
        }
    }

    /* Uniform spacing. */
    for (uint8_t i = 1U; i < SX1276_FHSS_CHANNEL_COUNT; ++i) {
        uint32_t a = sx1276_fhss_chantab_center_hz(i);
        uint32_t b = sx1276_fhss_chantab_center_hz((uint8_t)(i - 1U));
        if ((a - b) != SX1276_FHSS_CHANNEL_SPACING_HZ) {
            printf("[FAIL] spacing at idx=%u: %lu Hz (expected %lu)\n",
                   (unsigned)i,
                   (unsigned long)(a - b),
                   (unsigned long)SX1276_FHSS_CHANNEL_SPACING_HZ);
            g_failures++;
        }
    }

    /* All centers inside band with edge guard. */
    for (uint8_t i = 0U; i < SX1276_FHSS_CHANNEL_COUNT; ++i) {
        uint32_t hz = sx1276_fhss_chantab_center_hz(i);
        check("center >= 902.000 MHz", hz >= SX1276_FHSS_BAND_LOWER_HZ);
        check("center <= 928.000 MHz", hz <= SX1276_FHSS_BAND_UPPER_HZ);
        /* 750 kHz guard from each band edge (intentional, see plan §14). */
        check("center >= 902.750 MHz lower-guard",
              hz >= (SX1276_FHSS_BAND_LOWER_HZ + 750000UL));
        check("center <= 927.250 MHz upper-guard",
              hz <= (SX1276_FHSS_BAND_UPPER_HZ - 750000UL));
    }

    /* Out-of-range. */
    check("idx 50 OOR returns 0", sx1276_fhss_chantab_center_hz(50U) == 0UL);
    check("idx 200 OOR returns 0", sx1276_fhss_chantab_center_hz(200U) == 0UL);
    check("idx 255 OOR returns 0", sx1276_fhss_chantab_center_hz(255U) == 0UL);

    if (g_failures == 0U) {
        printf("[OK] sx1276_fhss_chantab: all checks passed (count=%u, span=%lu..%lu Hz)\n",
               (unsigned)sx1276_fhss_chantab_count(),
               (unsigned long)sx1276_fhss_chantab_center_hz(0U),
               (unsigned long)sx1276_fhss_chantab_center_hz(49U));
        return 0;
    }
    printf("[FAIL] sx1276_fhss_chantab: %u check(s) failed\n",
           (unsigned)g_failures);
    return 1;
}
