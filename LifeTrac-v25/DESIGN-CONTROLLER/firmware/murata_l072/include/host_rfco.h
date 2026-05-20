#ifndef LIFETRAC_MURATA_L072_HOST_RFCO_H
#define LIFETRAC_MURATA_L072_HOST_RFCO_H

/*
 * Per-TX RFCO (Radio Frequency Compliance Observability) URC
 * ----------------------------------------------------------
 *
 * Plan ref: AI NOTES/2026-05-19_LoRa_FCC_50CH_FHSS_Implementation_Plan_Copilot_v1_0.md
 *           §14.2 step 7 (FCC-B1-PERTX), §13.3 (RFCO bandwidth budget).
 *
 * Emitted by the L072 to the X8 (over the host UART) once per TX
 * "result" — meaning every attempted TX, whether it was actually
 * transmitted (status OK), refused at a fail-closed gate (status
 * ABORT_*) or failed downstream (status TX_*). This is the LIGHT
 * snapshot per §13.3; the HEAVY per-minute `RFCO_SUMMARY` URC (with
 * the 50-bucket histogram and per-channel max) is FCC-B1-SUMMARY,
 * landed separately.
 *
 * Naming discipline (plan delta #11): the dwell field is explicitly
 * `legal_dwell_used_us_10s` so bench post-processing can NEVER
 * conflate it with `qos_used_us_1s` from the existing 1 s QoS gate.
 * If we ever ship a 20 s variant, it gets a separate URC field with
 * the `_20s` suffix — never overload the same slot.
 *
 * The 1-byte `schema_ver` lets us extend the payload additively
 * (FCC notes §17.3 #4): bench scripts MUST refuse to parse an
 * unfamiliar schema_ver rather than silently mis-decoding the tail.
 *
 * Payload wire layout (21 bytes, little-endian throughout):
 *
 *   off  size  field
 *   ---- ----  ---------------------------------------------------
 *     0   1    schema_ver                  (== HOST_RFCO_PERTX_SCHEMA_VER)
 *     1   1    profile_id                  (REG_PROFILE_* enum from host_cfg_keys.h)
 *     2   1    tx_status                   (host_rfco_tx_status_t)
 *     3   1    hop_idx                     (0..49 within the 50-channel permutation)
 *     4   1    channel_idx                 (0..63 — the index used by sx1276_legal_dwell)
 *     5   4    epoch_le                    (uint32 FHSS scheduler epoch)
 *     9   4    freq_hz_le                  (uint32 channel center freq, Hz)
 *    13   4    pkt_toa_us_le               (uint32 packet ToA in microseconds)
 *    17   4    legal_dwell_used_us_10s_le  (uint32 us booked on this channel in the
 *                                          10 s window AT THE MOMENT this URC is
 *                                          emitted — i.e. AFTER pessimistic
 *                                          reserve was applied for this TX, but
 *                                          BEFORE reconcile-down. Bench post-
 *                                          processing must treat this as the
 *                                          worst-case high-water mark.)
 *    21   --   (end of payload)
 *
 * Status semantics (`host_rfco_tx_status_t`):
 *
 *   OK                          TX was emitted to completion (TX_DONE IRQ fired).
 *                              `pkt_toa_us` is the post-reconcile actual airtime.
 *   ABORT_AIRTIME_INVARIANT     FCC-A1b rejected the (SF,BW,CR,payload) tuple
 *                              BEFORE TX-start. No RF energy emitted. The
 *                              channel/freq fields reflect the *intended*
 *                              channel; `pkt_toa_us` is the predicted ToA that
 *                              tripped the cap.
 *   ABORT_LBT                   LBT (CAD) detected a busy channel. No RF
 *                              energy emitted. `legal_dwell_used_us_10s` is
 *                              UNCHANGED from prior state (no reservation
 *                              made — the LBT gate runs before reserve).
 *   ABORT_LEGAL_DWELL           Legal-dwell accountant refused the reservation
 *                              (would exceed 400 ms/10 s cap). No RF energy
 *                              emitted. Booking unchanged.
 *   ABORT_QOS                   Existing 1 s / 400 ms QoS gate refused. No RF.
 *   TX_TIMEOUT                  TX-start succeeded but TX_DONE never fired
 *                              within the watchdog window. RF energy WAS
 *                              emitted; legal-dwell reservation stands at the
 *                              pessimistic value (never rolled back per A2).
 *   TX_FAIL                     TX_DONE fired but downstream NACK / link
 *                              error. RF emitted; legal-dwell reservation
 *                              stands (no rollback on NACK — A2 contract).
 *   INTERNAL                    Bug bucket (e.g. invalid profile_id slipped
 *                              past A5 validation). Used for fail-loud.
 */

#include <stdbool.h>
#include <stdint.h>

#define HOST_RFCO_PERTX_SCHEMA_VER   1U
#define HOST_RFCO_PERTX_PAYLOAD_LEN  21U

typedef enum {
    HOST_RFCO_TX_STATUS_OK                       = 0U,
    HOST_RFCO_TX_STATUS_ABORT_AIRTIME_INVARIANT  = 1U,
    HOST_RFCO_TX_STATUS_ABORT_LBT                = 2U,
    HOST_RFCO_TX_STATUS_ABORT_LEGAL_DWELL        = 3U,
    HOST_RFCO_TX_STATUS_ABORT_QOS                = 4U,
    HOST_RFCO_TX_STATUS_TX_TIMEOUT               = 5U,
    HOST_RFCO_TX_STATUS_TX_FAIL                  = 6U,
    HOST_RFCO_TX_STATUS_INTERNAL                 = 0xFFU,
} host_rfco_tx_status_t;

typedef struct host_rfco_pertx_s {
    uint8_t  profile_id;
    uint8_t  tx_status;
    uint8_t  hop_idx;
    uint8_t  channel_idx;
    uint32_t epoch;
    uint32_t freq_hz;
    uint32_t pkt_toa_us;
    uint32_t legal_dwell_used_us_10s;
} host_rfco_pertx_t;

/*
 * Pack a per-TX RFCO snapshot into the on-wire byte layout described
 * above. `out` MUST point to at least HOST_RFCO_PERTX_PAYLOAD_LEN
 * (= 21) bytes. The function never reads past `*in` and never writes
 * outside `out[0 .. HOST_RFCO_PERTX_PAYLOAD_LEN-1]`.
 *
 * Returns true on success, false if either pointer is NULL.
 */
bool host_rfco_pertx_pack(const host_rfco_pertx_t *in, uint8_t *out);

/*
 * Build + emit a per-TX RFCO URC by calling host_uart_send_urc() with
 * HOST_TYPE_RFCO_PERTX_URC and the packed payload. `seq` and `flags`
 * are passed straight through to host_uart_send_urc(). Returns true
 * if the URC was queued for transmission, false on bad input.
 *
 * NOTE: this is the ONLY public path that wraps host_uart_send_urc()
 * for RFCO. Radio-layer code (sx1276_tx.c) MUST call this function
 * rather than poking the URC type directly, so the schema_ver and
 * payload-length invariants stay in one place.
 */
bool host_rfco_pertx_emit(uint16_t seq,
                          uint8_t flags,
                          const host_rfco_pertx_t *snapshot);

/*
 * FCC-B1-SUMMARY-b sidecar: blocked-attempts-by-reason 8-slot counter.
 *
 * The summary URC payload (see include/host_rfco_summary.h §3)
 * carries `blocked_attempts_by_reason[8]` indexed by the reason a TX
 * attempt was blocked. Slot mapping (locked by the wire-layout doc):
 *
 *   slot 0  HOST_RFCO_TX_STATUS_OK                       — NEVER recorded
 *                                                          (OK is not a
 *                                                          "blocked" outcome;
 *                                                          slot reserved for
 *                                                          future-proofing)
 *   slot 1  HOST_RFCO_TX_STATUS_ABORT_AIRTIME_INVARIANT
 *   slot 2  HOST_RFCO_TX_STATUS_ABORT_LBT
 *   slot 3  HOST_RFCO_TX_STATUS_ABORT_LEGAL_DWELL
 *   slot 4  HOST_RFCO_TX_STATUS_ABORT_QOS
 *   slot 5  HOST_RFCO_TX_STATUS_TX_TIMEOUT
 *   slot 6  HOST_RFCO_TX_STATUS_TX_FAIL
 *   slot 7  HOST_RFCO_TX_STATUS_INTERNAL (= 0xFF)         — mapped here
 *                                                          since the enum
 *                                                          value 0xFF does
 *                                                          not fit in 8 slots
 *
 * Any other reason value is silently dropped (defensive against enum
 * additions that have not yet been allocated a slot — add a slot +
 * bump HOST_RFCO_SUMMARY_SCHEMA_VER in that case).
 *
 * Counters saturate at UINT16_MAX. snapshot_and_clear copies and
 * zeroes; NULL `out` is a no-op (counters preserved).
 *
 * reset() is exposed for tests; production callers should rely on
 * snapshot_and_clear() at SUMMARY emit time. */
#define HOST_RFCO_BLOCKED_ATTEMPTS_SLOTS  8U

void host_rfco_blocked_attempts_record(host_rfco_tx_status_t reason);
void host_rfco_blocked_attempts_snapshot_and_clear(
    uint16_t out[HOST_RFCO_BLOCKED_ATTEMPTS_SLOTS]);
void host_rfco_blocked_attempts_reset(void);

/* FCC-B1-SUMMARY-c-1 per-window sidecars (consumed by the SUMMARY
 * emit wrapper, fed at the per-TX emit boundary). Two pieces of
 * per-window state collapsed into one record() because both are
 * derived from the same tx_status seen at host_rfco_pertx_emit():
 *
 *   - pertx_count_in_window: u8 saturated count of per-TX URCs since
 *     the last SUMMARY snapshot. Internal store is u16 so the
 *     boundary between "saturated at 255" and overflow rollover is
 *     unambiguous (the wire byte is u8).
 *   - last_clamp_reason: u8 holding the most-recent non-OK
 *     tx_status seen in the current window. Initial / cleared value
 *     is 0xFF (== HOST_RFCO_SUMMARY_LAST_CLAMP_REASON_NONE). OK
 *     statuses do NOT update this — only blocked/failed TX outcomes
 *     are "clamps" the operator cares about.
 *
 * record() is called once per accepted host_rfco_pertx_emit() (after
 * the existing blocked_attempts_record + record_hop wiring). The
 * snapshot_and_clear pair is called by host_rfco_summary_emit() once
 * per cadence tick. reset() is exposed for tests. */
void host_rfco_pertx_window_record(uint8_t tx_status);
uint8_t host_rfco_pertx_count_in_window_snapshot_and_clear(void);
uint8_t host_rfco_last_clamp_reason_snapshot_and_clear(void);
void host_rfco_pertx_window_reset(void);

#endif /* LIFETRAC_MURATA_L072_HOST_RFCO_H */
