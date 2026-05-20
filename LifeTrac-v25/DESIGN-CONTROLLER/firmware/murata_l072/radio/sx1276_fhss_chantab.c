#include "sx1276_fhss_chantab.h"

/*
 * Compile-time invariants per FCC §15.247(a)(1)(i) narrowband FHSS and
 * implementation plan §14 deltas #6 / #8.
 *
 * Any future edit to the macros in sx1276_fhss_chantab.h that violates
 * one of these asserts will fail the host `check` build before any
 * cross-compile is attempted.
 */
_Static_assert(SX1276_FHSS_CHANNEL_COUNT == 50U,
               "FCC 15.247(a)(1)(i) narrowband requires exactly 50 channels");

_Static_assert(SX1276_FHSS_CHANNEL_SPACING_HZ >= 25000UL,
               "FCC 15.247(a)(1) requires channel spacing >= 25 kHz");

_Static_assert(SX1276_FHSS_FIRST_CENTER_HZ >= SX1276_FHSS_BAND_LOWER_HZ,
               "first channel center must be inside the 902-928 MHz ISM band");

_Static_assert(SX1276_FHSS_LAST_CENTER_HZ <= SX1276_FHSS_BAND_UPPER_HZ,
               "last channel center must be inside the 902-928 MHz ISM band");

/*
 * TODO(FCC-EVID-D2): after the bench occupied-BW measurement lands,
 * convert the line below into a real _Static_assert against the measured
 * 20 dB OBW, and adjust SX1276_FHSS_CHANNEL_SPACING_HZ if the measured
 * value would otherwise violate it.
 *
 *   _Static_assert(SX1276_FHSS_CHANNEL_SPACING_HZ >= MEASURED_20DB_OBW_HZ, ...)
 */

uint32_t sx1276_fhss_chantab_center_hz(uint8_t idx) {
    if (idx >= SX1276_FHSS_CHANNEL_COUNT) {
        return 0UL;
    }
    return SX1276_FHSS_FIRST_CENTER_HZ +
           ((uint32_t)idx * SX1276_FHSS_CHANNEL_SPACING_HZ);
}

uint8_t sx1276_fhss_chantab_count(void) {
    return SX1276_FHSS_CHANNEL_COUNT;
}
