#ifndef LIFETRAC_MURATA_L072_SX1276_RX_SCAN_WALKER_H
#define LIFETRAC_MURATA_L072_SX1276_RX_SCAN_WALKER_H

/*
 * FCC-A6c-2-c-i-α': RX cold-start scan channel walker.
 *
 * Single source of truth for the channel order the receiver visits
 * while in the A6c-1 SCANNING state. Walks the 50-channel FHSS table
 * (sx1276_fhss_chantab) in a deterministic linear order 0..49, then
 * wraps to 0. Maintains its own file-static index — DOES NOT consult
 * or mutate the shared sx1276_fhss scheduler state (`s_fhss`) that
 * the TX path and the γ-1 RX retune loop share.
 *
 * Why a separate walker (FCC notes 2026-05-27, RX scan analysis v1.0
 * §1.3 / §8.1): the pre-α' implementation called
 * sx1276_fhss_next_channel() to pick scan channels. That call mutates
 * the shared scheduler's slot pointer, which γ-1 reads on every post-
 * lock retune tick. Under wide-mask FHSS the two paths would silently
 * desynchronize, so when sx1276_fhss_consider_remote() finally snapped
 * the scheduler to the remote's (epoch, hop_idx), the snap had to
 * unwind whatever drift scan introduced in the same shared struct.
 * Bench evidence on 2026-05-27 showed rx_frames climbing only
 * ~6/min instead of v25.0.1's ~168/min — caller is observe-only on
 * the synth, so scan must be observe-only on s_fhss.
 *
 * Linear 0..49 was chosen over a pseudo-random scan permutation because
 *   (a) RX scan is listen-only — no FCC hop-randomness rule applies,
 *   (b) the TX-side schedule IS already pseudo-random (Fisher-Yates
 *       per FCC §15.247), so the RX-vs-TX collision distribution is
 *       essentially uniform whether RX walks linearly or permuted,
 *   (c) linear walk is trivially testable (golden vector =
 *       0,1,2,...,49,0,1,...) and trivially debuggable on the bench
 *       (the channel index is the wall-clock dwell number mod 50).
 * If bench evidence shows pathological aliasing, swap the walker to a
 * fixed pseudo-random permutation (seed=0 via
 * sx1276_fhss_compute_permutation) in a follow-up increment — the
 * API surface here does not change.
 *
 * No HW deps. Includes only <stdbool.h> + <stdint.h>.
 */

#include <stdbool.h>
#include <stdint.h>

/* Reset the walker to its starting channel (idx=0). Idempotent.
 * Called by the A6c-2-c dispatcher on every BEGIN_SCAN edge so a
 * recovery re-scan (A6c-3 non-final retry) starts cleanly. */
void sx1276_rx_scan_walker_reset(void);

/* Return the next (channel_idx, center_hz) to listen on and advance
 * the walker. Walks 0..(SX1276_FHSS_CHANNEL_COUNT - 1) then wraps.
 *
 *   out_idx — receives the channel index that was just selected
 *             (0..49). Must be non-NULL.
 *   returns — center frequency in Hz for the selected channel.
 *             Returns 0UL on NULL out_idx or on a chantab corruption
 *             (caller must treat 0 as a fault and NOT call
 *             sx1276_set_frequency_hz(0)). */
uint32_t sx1276_rx_scan_walker_next(uint8_t *out_idx);

/* Read the next channel index the walker WOULD return on the next
 * call to sx1276_rx_scan_walker_next(), without advancing. Used by
 * tests and by FCC-B1-SUMMARY observability hooks. Returns 0 if the
 * walker is fresh-from-reset (next call will yield idx=0). */
uint8_t sx1276_rx_scan_walker_peek_next(void);

#endif /* LIFETRAC_MURATA_L072_SX1276_RX_SCAN_WALKER_H */
