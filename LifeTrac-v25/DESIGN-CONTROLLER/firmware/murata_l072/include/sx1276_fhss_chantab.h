#ifndef LIFETRAC_MURATA_L072_SX1276_FHSS_CHANTAB_H
#define LIFETRAC_MURATA_L072_SX1276_FHSS_CHANTAB_H

/*
 * 50-channel FHSS center-frequency table for FCC §15.247(a)(1)(i)
 * narrowband (BW < 250 kHz) production profile
 * `FCC_15_247_FHSS_50CH_BW250`.
 *
 * Single source of truth for the L072 firmware and the X8-side Python
 * tooling (mirror at
 * bench/host_proto/fhss_chantab.py — must remain bit-identical).
 *
 * Generator formula (interim — final list gated on bench occupied-BW
 * measurement, see implementation plan §14 delta #6 and bench gate
 * FCC-EVID-D2):
 *
 *   center_hz = SX1276_FHSS_FIRST_CENTER_HZ
 *             + idx * SX1276_FHSS_CHANNEL_SPACING_HZ,   idx = 0..49
 *
 *   span:    902.750 MHz .. 927.250 MHz
 *   spacing: 500 kHz
 *   band:    902.000 .. 928.000 MHz (ISM)
 *   guard from each band edge: 750 kHz
 *
 * References:
 *   - LifeTrac-v25/AI NOTES/2026-05-19_FCC_Part15_902-928_Compliance_Notes_Copilot_v1_0.md §19-§20
 *   - LifeTrac-v25/AI NOTES/2026-05-19_LoRa_FCC_50CH_FHSS_Implementation_Plan_Copilot_v1_0.md §14
 */

#include <stdint.h>

#define SX1276_FHSS_CHANNEL_COUNT        50U
#define SX1276_FHSS_CHANNEL_SPACING_HZ   500000UL
#define SX1276_FHSS_FIRST_CENTER_HZ      902750000UL
#define SX1276_FHSS_LAST_CENTER_HZ \
    (SX1276_FHSS_FIRST_CENTER_HZ + \
     ((uint32_t)(SX1276_FHSS_CHANNEL_COUNT - 1U) * SX1276_FHSS_CHANNEL_SPACING_HZ))

#define SX1276_FHSS_BAND_LOWER_HZ        902000000UL
#define SX1276_FHSS_BAND_UPPER_HZ        928000000UL

/* Return the center frequency in Hz for channel index 0..49.
 * Returns 0 for out-of-range indices (caller must treat as fault). */
uint32_t sx1276_fhss_chantab_center_hz(uint8_t idx);

/* Return SX1276_FHSS_CHANNEL_COUNT. Wrapper exists so host tooling and
 * tests can probe the count through the API rather than the macro. */
uint8_t sx1276_fhss_chantab_count(void);

#endif /* LIFETRAC_MURATA_L072_SX1276_FHSS_CHANTAB_H */
