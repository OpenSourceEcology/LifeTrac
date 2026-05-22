#ifndef LIFETRAC_MURATA_L072_LORA_FRAMES_PER_DWELL_H
#define LIFETRAC_MURATA_L072_LORA_FRAMES_PER_DWELL_H

/*
 * P3 / Phase 3.2 — firmware mirror of the host-side
 * frames_per_dwell() pacing-hint helper.
 *
 * Cross-references:
 *   - LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/
 *     w2_02_host_pipeline.py
 *       constants:  MIN_LORA_HOST_INTER_CYCLE_S, LEGAL_DWELL_US,
 *                   DWELL_HEADROOM_PCT, MAX_FRAMES_PER_DWELL_CAP
 *       function :  frames_per_dwell(toa_us)
 *   - LifeTrac-v25/AI NOTES/2026-05-21_Open_Problems_To_Fix_v1_0.md
 *     P3 (PARTIAL -> CLOSED via this mirror).
 *
 * SEPARATE from sx1276_legal_dwell (radio/sx1276_legal_dwell.c):
 *   - sx1276_legal_dwell is the FCC §15.247(a)(1)(i) per-channel
 *     airtime accountant — a hard regulatory gate enforced PER FRAME
 *     at TX-start, in microseconds, with rollback-free booking.
 *   - lora_frames_per_dwell is a PACING HINT: "given the modem's
 *     current per-fragment ToA, how many fragments may be queued
 *     back-to-back inside one 400 ms dwell, leaving DWELL_HEADROOM_PCT
 *     of slack for retransmits/slop?" It does not enforce anything;
 *     it lets a TX scheduler size its burst without bumping into the
 *     legal-dwell ceiling on the very next frame.
 *
 * Parity contract:
 *   The unit-test anchors below MUST stay byte-for-byte identical to
 *   the Python docstring anchors in w2_02_host_pipeline.py:
 *     - SF7 / BW250 / 64 B  (~28_000 us)  -> 8   (cap binds)
 *     - SF9 / BW250 / 64 B  (~115_000 us) -> 2-3 (ToA binds)
 *
 * Integer-arithmetic equivalence:
 *   Python: n = (LEGAL_DWELL_US * DWELL_HEADROOM_PCT // 100) // toa_us
 *   C    : n = (LEGAL_DWELL_US * DWELL_HEADROOM_PCT /  100) /  toa_us
 *   These match for all positive integer operands within uint32_t
 *   range because Python's // and C's / on positive ints both
 *   truncate toward zero.
 */

#include <stdint.h>

#define LORA_FPD_MIN_INTER_CYCLE_MS         50U
#define LORA_FPD_LEGAL_DWELL_US             400000UL
#define LORA_FPD_DWELL_HEADROOM_PCT         85U
#define LORA_FPD_MAX_FRAMES_PER_DWELL_CAP   8U

/*
 * Return the legal number of fragments per dwell for a given per-
 * fragment ToA in microseconds.
 *
 * Contract:
 *   - toa_us == 0      -> 1 (degenerate / unknown ToA: yield one frame,
 *                            never zero, so the caller always makes
 *                            forward progress).
 *   - 0 < toa_us  large -> 1 (ToA already at/over the dwell budget).
 *   - 0 < toa_us  small -> min(MAX_FRAMES_PER_DWELL_CAP, floor(
 *                            LEGAL_DWELL_US * HEADROOM_PCT/100 / toa_us)).
 *   - Return value is always in the closed interval [1, CAP].
 *
 * Pure function: no side effects, no clock reads, no allocations.
 * Safe to call from ISR context.
 */
uint8_t lora_frames_per_dwell(uint32_t toa_us);

#endif /* LIFETRAC_MURATA_L072_LORA_FRAMES_PER_DWELL_H */
