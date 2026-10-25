#ifndef LIFETRAC_MURATA_L072_HOST_TYPES_H
#define LIFETRAC_MURATA_L072_HOST_TYPES_H

#include <stdint.h>

/* Version contract (RS-12.21 note, 2026-10-24). HOST_PROTOCOL_VER is the
 * frame header's `ver` byte, refused on mismatch by host_cmd_dispatch
 * (BAD_VERSION), so it changes only with the framing. HOST_WIRE_SCHEMA_VER
 * is reported in BOOT/VER/READY and CFG key 0x81 (1 -> 2 on 2026-05-20,
 * d4dfcb86, with the FHSS batch). Additive changes have kept both since: a
 * new message type (0xC5 RX_CRC_DUMP_URC, 2026-08-02), a tail appended to a
 * payload (the F8 RX_FRAME_URC tail, the STATS_URC tails), a new fault code
 * or a fault code's sub-byte (the 2026-05-19 RX-scan-FAILED analysis §13.1
 * #7 contract: the FAULT_URC format, host_stats and schema_ver untouched).
 * The 2026-10-24 RADIO_PARK_REQ/URC and the 0x0D split are such changes:
 * no bump. A new command is advertised in the capability bitmap instead,
 * and an older image answers it with ERR_PROTO UNKNOWN_TYPE (detail = the
 * type). */
#define HOST_PROTOCOL_VER                    1U
#define HOST_WIRE_SCHEMA_VER                 2U

#define HOST_CAP_PING                        (1UL << 0)
#define HOST_CAP_VERSION                     (1UL << 1)
#define HOST_CAP_UID                         (1UL << 2)
#define HOST_CAP_REG_IO                      (1UL << 3)
#define HOST_CAP_STATS                       (1UL << 4)
#define HOST_CAP_RESET                       (1UL << 5)
#define HOST_CAP_TX_FRAME                    (1UL << 6)
/* RS-12.21 (2026-10-24): RADIO_PARK_REQ/URC below. The image that added it
 * also moved TX_FIFO_RB_MISMATCH to 0x0F, so a host may read this bit as
 * "0x0D is RX_SCAN_FAILED only" (the 0x0D split, fault codes below). */
#define HOST_CAP_RADIO_PARK                  (1UL << 7)

#define HOST_CAPABILITY_BITMAP               (HOST_CAP_PING | \
                                             HOST_CAP_VERSION | \
                                             HOST_CAP_UID | \
                                             HOST_CAP_REG_IO | \
                                             HOST_CAP_STATS | \
                                             HOST_CAP_RESET | \
                                             HOST_CAP_TX_FRAME | \
                                             HOST_CAP_RADIO_PARK)

/* Host-to-L072 commands. */
#define HOST_TYPE_PING_REQ                   0x00U
#define HOST_TYPE_VER_REQ                    0x01U
#define HOST_TYPE_UID_REQ                    0x02U
#define HOST_TYPE_RESET_REQ                  0x03U
/*
 * RADIO_PARK_REQ (RS-12.21, 2026-10-24): the firmware radio park, the bench's
 * persistent radio-off state (FHSS_SCAN_FAILED_OPTIONS.md section 5 item 2,
 * adopted with scan option (b): a node without originator authority now
 * scans on after its scan budget, re-arming RX every dwell, so a host SLEEP
 * write does not hold once the daemons stop). Payload: none (ignored).
 *   - Sets the host_parked flag (radio/sx1276_rx.c, sx1276_rx_park()) and
 *     puts the modem to LoRa SLEEP. While it is set nothing arms the
 *     receiver: the scan machine is frozen (no walker step, no FAIL fault,
 *     no (b) restart), the slot follower, the gamma-1 retune and the post-TX
 *     re-arm do nothing, and sx1276_rx_arm() refuses.
 *   - A TX_FRAME_REQ while parked is refused (ERR_PROTO FORBIDDEN, detail
 *     0, as any refused TX): nothing goes on air.
 *   - Ends at a host raw RegOpMode RX write (REG_WRITE 0x01 = 0x85 RXCONT,
 *     the daemons' arm; 0x86 RXSINGLE in a diag build), a profile
 *     activation (CFG_SET REG_PROFILE) or any reset. The scan machine then
 *     resumes as it stood (an activation restarts it): a SCANNING node's
 *     dwell and 30 s attempt do not count the parked time; a LOCKED node's
 *     2 s loss timer does, as the FHSS clock's freshness does, so a park
 *     longer than that demotes it at the first tick (a fresh acquisition,
 *     not a drifted grid).
 * Reply: RADIO_PARK_URC, same seq, HOST_RADIO_PARK_URC_PAYLOAD_LEN bytes:
 *   u8 status  HOST_RADIO_PARK_STATUS_*: 0 PARKED; 1 ALREADY_PARKED (SLEEP
 *              re-applied); 2 REFUSED -- a TX is on air or queued, nothing
 *              changed (retry after its TX_DONE)
 *   u8 opmode  RegOpMode read back after the request (0x80 = LoRa SLEEP)
 * Bench check: check-radio-park (bench/host_proto/radio_park.c).
 */
#define HOST_TYPE_RADIO_PARK_REQ             0x04U

#define HOST_TYPE_TX_FRAME_REQ               0x10U

#define HOST_TYPE_CFG_SET_REQ                0x20U
#define HOST_TYPE_CFG_GET_REQ                0x21U

#define HOST_TYPE_REG_READ_REQ               0x30U
#define HOST_TYPE_REG_WRITE_REQ              0x31U

#define HOST_TYPE_STATS_RESET_REQ            0x40U
#define HOST_TYPE_STATS_DUMP_REQ             0x41U

/* L072-to-host URCs/responses. */
#define HOST_TYPE_VER_URC                    0x81U
#define HOST_TYPE_UID_URC                    0x82U
/* RS-12.21 (2026-10-24): reply to RADIO_PARK_REQ, layout above. */
#define HOST_TYPE_RADIO_PARK_URC             0x84U
#define HOST_RADIO_PARK_URC_PAYLOAD_LEN      2U
#define HOST_RADIO_PARK_STATUS_PARKED        0U
#define HOST_RADIO_PARK_STATUS_ALREADY       1U
#define HOST_RADIO_PARK_STATUS_REFUSED       2U

#define HOST_TYPE_TX_DONE_URC                0x90U
/* TX_DONE_URC payload: {u8 tx_id, u8 status, u32 time_on_air_us_le, u8 tx_power_dbm} */
/* RX_FRAME_URC payload: {u8 len, i8 snr_db, i16 rssi_dbm_le,
 * u32 timestamp_us_le, payload[len],
 * F8 additive tail (always present, +8 B): u8 phase_flags (bit0 = hop
 * header valid), u8 profile_id, u8 hop_idx, u8 slot_offset_ms,
 * u32 epoch_le. `len` EXCLUDES the tail; pre-F8 parsers that slice by
 * it keep working. Constants in host_rx_wire.h; layout packed by
 * host_rx_frame_urc_pack() and pinned by check-rx-frame-urc. */
#define HOST_TYPE_RX_FRAME_URC               0x91U
#define HOST_RX_FRAME_URC_TAIL_LEN           8U
#define HOST_RX_FRAME_URC_PHASE_VALID        0x01U

#define HOST_TYPE_CFG_OK_URC                 0xA0U
#define HOST_TYPE_CFG_DATA_URC               0xA1U

#define HOST_TYPE_REG_DATA_URC               0xB0U
#define HOST_TYPE_REG_WRITE_ACK_URC          0xB1U

#define HOST_TYPE_RADIO_IRQ_URC              0xC0U
#define HOST_TYPE_STATS_URC                  0xC1U
#define HOST_TYPE_AIRTIME_INVARIANT_REJECT_URC 0xC2U
/*
 * AIRTIME_INVARIANT_REJECT_URC (a.k.a. "__AIRTIME_INVARIANT_REJECT__" in
 * plan §14.2 step 3) signals that a modem-config setter refused a
 * (SF, BW, CR) tuple whose worst-case ToA at max payload exceeds the
 * per-channel dwell cap (currently 380 ms = 400 ms dwell - 20 ms guard).
 *
 * Payload layout (15 bytes, additive-only):
 *   u8  sf                     spreading factor 6..12
 *   u8  cr_den                  coding-rate denominator 5..8 (CR = 4/cr_den)
 *   u32 bw_hz_le                bandwidth in Hz (e.g. 125000 / 250000 / 500000)
 *   u8  payload_len             payload byte count used in the worst-case calc
 *   u32 computed_toa_us_le      ToA the helper computed for this tuple
 *   u32 cap_us_le               dwell cap that was exceeded (e.g. 380000)
 *
 * Cross-refs: AI NOTES/2026-05-19_LoRa_FCC_50CH_FHSS_Implementation_Plan_Copilot_v1_0.md
 *             §14 / §14.2 step 3, TODO.md Stage S1.5 FCC-A1a.
 */
#define HOST_AIRTIME_REJECT_PAYLOAD_LEN      15U

/*
 * RFCO_PERTX_URC (plan §14.2 step 7, FCC-B1-PERTX) — per-TX Radio
 * Frequency Compliance Observability snapshot. Emitted on EVERY TX
 * result (OK, abort-at-gate, or downstream failure). Full payload
 * documentation and helper API live in include/host_rfco.h; do NOT
 * pack this URC inline — call host_rfco_pertx_emit() so the
 * schema_ver byte and payload length stay in one place.
 *
 * Payload length is fixed at HOST_RFCO_PERTX_PAYLOAD_LEN (= 21).
 * Schema version is HOST_RFCO_PERTX_SCHEMA_VER (= 1); future field
 * additions MUST bump this. The 50-bucket per-channel histogram + per-
 * channel `legal_dwell_max_us` snapshot is the SEPARATE per-minute
 * RFCO_SUMMARY URC (FCC-B1-SUMMARY), not this one.
 */
#define HOST_TYPE_RFCO_PERTX_URC             0xC3U

/*
 * RFCO_SUMMARY_URC (plan §14.2, FCC-B1-SUMMARY) — per-minute Radio
 * Frequency Compliance Observability snapshot. Carries the
 * 50-bucket per-channel hop histogram, per-channel legal-dwell peak,
 * blocked-attempts-by-reason histogram, active_count / blacklist_size,
 * and last_clamp_reason. Complements the LIGHT per-TX RFCO_PERTX URC
 * (one per TX result, 0xC3 above).
 *
 * Wire layout, schema_ver, constants, and snapshot struct live in
 * include/host_rfco_summary.h. Payload length is fixed at
 * HOST_RFCO_SUMMARY_PAYLOAD_LEN (= 191). Schema version is
 * HOST_RFCO_SUMMARY_SCHEMA_VER (= 1); future additions MUST bump it.
 * Cadence: HOST_RFCO_SUMMARY_PERIOD_MS (60 000 ms), polled from the
 * main loop against platform_now_ms() — no new HAL timer.
 *
 * Design doc:
 *   AI NOTES/2026-05-19_FCC_B1_SUMMARY_Wire_Layout_Design_Copilot_v1_0.md
 *
 * This is a DECLARATION-ONLY type-code allocation (FCC-B1-SUMMARY-a);
 * pack helper, emit wrapper, and main-loop cadence land in
 * FCC-B1-SUMMARY-b and FCC-B1-SUMMARY-c.
 */
#define HOST_TYPE_RFCO_SUMMARY_URC           0xC4U
/* RS-11.5 (2026-08-02) RX corrupt-frame capture: emitted on every LoRa
 * payload-CRC failure so the host can characterize the damage the radio
 * currently throws away. Payload:
 *   {u8 ver=1, u8 irq_flags, u8 rx_len, u8 pkt_snr_q4, u8 pkt_rssi_raw,
 *    u8 payload[min(rx_len, 251)]}
 * pkt_rssi_raw converts as dBm = raw - 157 (HF port); snr_db = q4/4.
 * Diagnostic-rate traffic (~0.3/s at the observed 4% corruption). */
#define HOST_TYPE_RX_CRC_DUMP_URC            0xC5U

/* STATS_URC payload layout is additive-only; parsers must tolerate trailing bytes. */
#define HOST_STATS_OFFSET_HOST_DROPPED       0U
#define HOST_STATS_OFFSET_HOST_ERRORS        4U
#define HOST_STATS_OFFSET_HOST_QUEUE_FULL    8U
#define HOST_STATS_OFFSET_HOST_IRQ_IDLE      12U
#define HOST_STATS_OFFSET_HOST_IRQ_HT        16U
#define HOST_STATS_OFFSET_HOST_IRQ_TC        20U
#define HOST_STATS_OFFSET_HOST_IRQ_TE        24U
#define HOST_STATS_OFFSET_RADIO_DIO0         28U
#define HOST_STATS_OFFSET_RADIO_DIO1         32U
#define HOST_STATS_OFFSET_RADIO_DIO2         36U
#define HOST_STATS_OFFSET_RADIO_DIO3         40U
#define HOST_STATS_OFFSET_RADIO_CRC_ERR      44U
#define HOST_STATS_OFFSET_RADIO_RX_OK        48U
#define HOST_STATS_OFFSET_RADIO_TX_OK        52U
#define HOST_STATS_OFFSET_RADIO_TX_ABORT_LBT 56U
#define HOST_STATS_OFFSET_RADIO_TX_ABORT_AIRTIME 60U
#define HOST_STATS_OFFSET_RADIO_STATE        64U
#define HOST_STATS_OFFSET_HOST_RX_BYTES      68U
#define HOST_STATS_OFFSET_HOST_RX_LPUART_BYTES 72U
#define HOST_STATS_OFFSET_HOST_RX_USART1_BYTES 76U
#define HOST_STATS_OFFSET_HOST_PARSE_OK      80U
#define HOST_STATS_OFFSET_HOST_PARSE_ERR     84U
#define HOST_STATS_OFFSET_HOST_UART_ERR_LPUART 88U
#define HOST_STATS_OFFSET_HOST_UART_ERR_USART1 92U
/* v1.2 (W1-7 RX): per-flag UART error counters + RX ring overflow.
 * Additive only — older parsers tolerate this growth (see plan v1.2 §3.8). */
#define HOST_STATS_OFFSET_HOST_UART_PE_LPUART  96U
#define HOST_STATS_OFFSET_HOST_UART_FE_LPUART  100U
#define HOST_STATS_OFFSET_HOST_UART_NE_LPUART  104U
#define HOST_STATS_OFFSET_HOST_UART_ORE_LPUART 108U
#define HOST_STATS_OFFSET_HOST_UART_PE_USART1  112U
#define HOST_STATS_OFFSET_HOST_UART_FE_USART1  116U
#define HOST_STATS_OFFSET_HOST_UART_NE_USART1  120U
#define HOST_STATS_OFFSET_HOST_UART_ORE_USART1 124U
#define HOST_STATS_OFFSET_HOST_RX_RING_OVF     128U
/* RS-11.5 (2026-08-02) additive tail: TX FIFO readback discriminator.
 * Older host parsers tolerate the growth (offset-guarded label list). */
#define HOST_STATS_OFFSET_TX_FIFO_RB_OK        132U
#define HOST_STATS_OFFSET_TX_FIFO_RB_BAD       136U
#define HOST_STATS_OFFSET_TX_DONE_EARLY       140U
/* RS-12 (2026-09-12) additive tail: URC-path loss accounting.
 * rx_urc_lost      = DIO0 edges the main loop never serviced (coalesced
 *                    RxDones, or a frame clobbered by the TX FIFO load).
 * rx_pretx_drained = frames rescued by the drain-before-TX step.
 * Older host parsers tolerate the growth (offset-guarded label list). */
#define HOST_STATS_OFFSET_RX_URC_LOST         144U
#define HOST_STATS_OFFSET_RX_PRETX_DRAINED    148U
/* RS-12.10 (2026-09-12) additive tail: where the loss actually is.
 * rx_fifo_skip            = packets that completed while the previous one
 *                           was still unserviced (FifoRxCurrentAddr jumped
 *                           past a hole). DIO0 is level-held across such a
 *                           pair, so rx_urc_lost above cannot see them.
 * tx_deaf_max_us / sum_us = RX disarm -> RX re-arm around each TX: the
 *                           receiver's deaf window (max, and total).
 * tx_done_to_rearm_max_us = TX_DONE seen -> RX re-armed: the firmware's
 *                           own turnaround, airtime excluded. */
#define HOST_STATS_OFFSET_RX_FIFO_SKIP              152U
#define HOST_STATS_OFFSET_TX_DEAF_MAX_US            156U
#define HOST_STATS_OFFSET_TX_DEAF_SUM_US            160U
#define HOST_STATS_OFFSET_TX_DONE_TO_REARM_MAX_US   164U
/* RS-12.15 v2 (2026-09-14) additive tail: FHSS clock authority.
 * fhss_dec_*              = consider_remote() decisions since reset, one
 *                           counter per sx1276_fhss_snap_decision_t value
 *                           (ALIGNED, SNAPPED, REJ_NOT_INIT, REJ_BAD_HOP,
 *                           REJ_EPOCH_DRIFT, REJ_LOCKED_OUT). An originator
 *                           refusing its follower's echo shows as LOCKED_OUT.
 * clk_demotion_reset      = LOCKED->SCANNING demotions that reset an ADOPTED
 *                           clock (follower re-acquisition, as before).
 * clk_demotion_kept       = demotions where a self-anchored clock was KEPT
 *                           (the streaming node's grid survives -- the fix).
 * tx_first_anchor         = own-TX lazy clock anchors ("slot starts now");
 *                           more than one per session = a phase restart.
 * tx_stream_streak_max    = longest run of own FHSS TXs each less than 1 s
 *                           apart. A CADENCE measure only: the streak is
 *                           advanced on every own TX_DONE, including on a
 *                           node whose grid is ADOPTED (an ordinary
 *                           follower streaming hard reaches 8 too), so
 *                           >= 8 does NOT by itself mean authority was
 *                           held. sx1276_fhss_authority_is_originator()
 *                           additionally requires a VALID clock, an
 *                           UNADOPTED grid, and a last own TX less than
 *                           1 s ago. Read it alongside clk_demotion_kept
 *                           (authority exercised) and tx_first_anchor. */
#define HOST_STATS_OFFSET_FHSS_DEC_ALIGNED          168U
#define HOST_STATS_OFFSET_FHSS_DEC_SNAPPED          172U
#define HOST_STATS_OFFSET_FHSS_DEC_REJ_NOT_INIT     176U
#define HOST_STATS_OFFSET_FHSS_DEC_REJ_BAD_HOP      180U
#define HOST_STATS_OFFSET_FHSS_DEC_REJ_EPOCH_DRIFT  184U
#define HOST_STATS_OFFSET_FHSS_DEC_REJ_LOCKED_OUT   188U
#define HOST_STATS_OFFSET_CLK_DEMOTION_RESET        192U
#define HOST_STATS_OFFSET_CLK_DEMOTION_KEPT         196U
#define HOST_STATS_OFFSET_TX_FIRST_ANCHOR           200U
#define HOST_STATS_OFFSET_TX_STREAM_STREAK_MAX      204U
#define HOST_STATS_PAYLOAD_LEN               208U

#define HOST_TYPE_BOOT_URC                   0xF0U
/* FAULT_URC payload: {u8 code, u8 sub, u16 reserved, u32 pc, u32 lr, u32 psr, u32 bfar, u32 uptime_ms} */
#define HOST_TYPE_FAULT_URC                  0xF1U
#define HOST_TYPE_READY_URC                  0xF2U
#define HOST_TYPE_ERR_PROTO_URC              0xFEU

#define HOST_FAULT_URC_PAYLOAD_LEN           24U

#define HOST_FAULT_CODE_HARDFAULT            0x01U
#define HOST_FAULT_CODE_NMI                  0x02U
#define HOST_FAULT_CODE_RADIO_INIT_FAIL      0x03U
#define HOST_FAULT_CODE_RADIO_OPMODE_DRIFT   0x04U
#define HOST_FAULT_CODE_RX_CRC_DISABLED      0x05U
#define HOST_FAULT_CODE_HOST_DMA_OVERRUN     0x06U
#define HOST_FAULT_CODE_STACK_GUARD          0x07U
#define HOST_FAULT_CODE_CLOCK_HSE_FAILED     0x08U
#define HOST_FAULT_CODE_HOST_RX_SEEN         0x09U
#define HOST_FAULT_CODE_HOST_RX_INACTIVE     0x0AU
#define HOST_FAULT_CODE_HOST_PARSE_ERROR     0x0BU
#define HOST_FAULT_CODE_HOST_DIAG_MARK       0x0CU
/*
 * RX_SCAN_FAILED (0x0D) -- emitted by the scan FAIL dispatch in
 * radio/sx1276_rx.c (scan_dispatch_action, FCC-A6c-3-c) at the end of every
 * 30 s scan attempt that did not LOCK (SX1276_RX_SCAN_REDESIGN_MS,
 * radio/sx1276_rx_scan_policy.c). The `sub` byte, built by
 * sx1276_rx_scan_fail_eval() (radio/sx1276_rx_scan_fail.c), carries a
 * four-class disambiguation encoding so the host can tell the root cause at
 * the fault edge without a wider URC payload (AI NOTES/2026-05-19_RX_Scan_
 * FAILED_State_Analysis §13.1 #2, A6c-3-a):
 *
 *   bit 7 : final       — 1 == the retries are spent (attempt nibble at
 *                         SX1276_RX_SCAN_MAX_RETRIES). Since RS-12.21 (b)
 *                         (2026-10-24) this does NOT mean "FAILED for
 *                         good": only a node with originator authority
 *                         holds FAILED; any other node scans on, and its
 *                         final fault repeats every 30 s until a LOCK.
 *   bit 6 : warm        — 1 == this boot has previously LOCKED
 *   bit 5 : hw_suspect  — 1 == no DIO0 IRQ observed during scan
 *   bit 4 : crc_seen    — 1 == >=1 CRC-error IRQ observed during scan
 *   bits 3..0 : attempt — 0..15 saturating, this attempt's index
 *
 * History (FHSS_SCAN_FAILED_OPTIONS.md section 7): this define took 0x0D on
 * 2026-05-20 as a declaration-only step whose note read "no code emits it
 * yet"; the emitter landed with A6c-3-c, the note stayed, and RS-11.5 took
 * 0x0D again for TX_FIFO_RB_MISMATCH on 2026-08-02. The 2026-10-24 split
 * keeps 0x0D for this fault (every logged 0x0D is one) and moves the FIFO
 * fault to 0x0F. A host decoding an older image: a 0x0D whose sub is not a
 * valid scan encoding is that image's FIFO fault (tx_id = sub); an image
 * advertising HOST_CAP_RADIO_PARK has the split.
 */
#define HOST_FAULT_CODE_RX_SCAN_FAILED       0x0DU
/* RS-11.5 (2026-08-02) TX-path discriminators for the slot-(total-2)
 * on-air CRC corruption (radio/sx1276_tx.c). sub = the TX's tx_id (the
 * host's: a fragment index, or a command/PROBE id such as 0xFD/0xFE).
 * 0x0E: TX_DONE arrived >5 ms EARLIER than the expected ToA — the
 *       radiation was truncated.
 * 0x0F: post-TX FIFO readback checksum != load-time checksum — the FIFO
 *       was altered between load and TX_DONE (corrupt at the source). Was
 *       0x0D until 2026-10-24 (the 0x0D split, above); the counter
 *       tx_fifo_rb_bad (STATS offset 136) stays its primary signal. */
#define HOST_FAULT_CODE_TX_DONE_EARLY        0x0EU
#define HOST_FAULT_CODE_TX_FIFO_RB_MISMATCH  0x0FU

/* CFG status codes used in CFG_OK_URC payload. */
#define HOST_CFG_STATUS_OK                   0U
#define HOST_CFG_STATUS_UNKNOWN_KEY          1U
#define HOST_CFG_STATUS_BAD_LENGTH           2U
#define HOST_CFG_STATUS_OUT_OF_RANGE         3U
#define HOST_CFG_STATUS_APPLY_FAILED         4U
#define HOST_CFG_STATUS_DEFERRED             5U
#define HOST_CFG_STATUS_READ_ONLY            6U

/* ERR_PROTO sub-codes. */
#define HOST_ERR_PROTO_BAD_VERSION           1U
#define HOST_ERR_PROTO_UNKNOWN_TYPE          2U
#define HOST_ERR_PROTO_BAD_LENGTH            3U
#define HOST_ERR_PROTO_BAD_CRC               4U
#define HOST_ERR_PROTO_BAD_COBS              5U
#define HOST_ERR_PROTO_TOO_LARGE             6U
#define HOST_ERR_PROTO_QUEUE_FULL            7U
#define HOST_ERR_PROTO_FORBIDDEN             8U

/* Duplicate case labels produce a compile-time error if any ID collides.
 * RS-12.21 (2026-10-24): RADIO_PARK_REQ/URC added, and the 0xC3-0xC5 URC
 * types the list had missed. */
static inline void host_type_compile_time_uniqueness_check(void) {
    switch (0U) {
        case HOST_TYPE_PING_REQ:
        case HOST_TYPE_VER_REQ:
        case HOST_TYPE_UID_REQ:
        case HOST_TYPE_RESET_REQ:
        case HOST_TYPE_RADIO_PARK_REQ:
        case HOST_TYPE_TX_FRAME_REQ:
        case HOST_TYPE_CFG_SET_REQ:
        case HOST_TYPE_CFG_GET_REQ:
        case HOST_TYPE_REG_READ_REQ:
        case HOST_TYPE_REG_WRITE_REQ:
        case HOST_TYPE_STATS_RESET_REQ:
        case HOST_TYPE_STATS_DUMP_REQ:
        case HOST_TYPE_VER_URC:
        case HOST_TYPE_UID_URC:
        case HOST_TYPE_RADIO_PARK_URC:
        case HOST_TYPE_TX_DONE_URC:
        case HOST_TYPE_RX_FRAME_URC:
        case HOST_TYPE_CFG_OK_URC:
        case HOST_TYPE_CFG_DATA_URC:
        case HOST_TYPE_REG_DATA_URC:
        case HOST_TYPE_REG_WRITE_ACK_URC:
        case HOST_TYPE_RADIO_IRQ_URC:
        case HOST_TYPE_STATS_URC:
        case HOST_TYPE_AIRTIME_INVARIANT_REJECT_URC:
        case HOST_TYPE_RFCO_PERTX_URC:
        case HOST_TYPE_RFCO_SUMMARY_URC:
        case HOST_TYPE_RX_CRC_DUMP_URC:
        case HOST_TYPE_BOOT_URC:
        case HOST_TYPE_FAULT_URC:
        case HOST_TYPE_READY_URC:
        case HOST_TYPE_ERR_PROTO_URC:
            break;
        default:
            break;
    }
}

/* RS-12.21 (2026-10-24, FHSS_SCAN_FAILED_OPTIONS.md section 7): the same
 * guard for the FAULT_URC code space. RX_SCAN_FAILED and TX_FIFO_RB_MISMATCH
 * shared 0x0D from 2026-08-02 to 2026-10-24 because nothing checked fault
 * codes; every HOST_FAULT_CODE_* belongs in this list. Called from
 * host_cmd_init() next to the type check. */
static inline void host_fault_code_compile_time_uniqueness_check(void) {
    switch (0U) {
        case HOST_FAULT_CODE_HARDFAULT:
        case HOST_FAULT_CODE_NMI:
        case HOST_FAULT_CODE_RADIO_INIT_FAIL:
        case HOST_FAULT_CODE_RADIO_OPMODE_DRIFT:
        case HOST_FAULT_CODE_RX_CRC_DISABLED:
        case HOST_FAULT_CODE_HOST_DMA_OVERRUN:
        case HOST_FAULT_CODE_STACK_GUARD:
        case HOST_FAULT_CODE_CLOCK_HSE_FAILED:
        case HOST_FAULT_CODE_HOST_RX_SEEN:
        case HOST_FAULT_CODE_HOST_RX_INACTIVE:
        case HOST_FAULT_CODE_HOST_PARSE_ERROR:
        case HOST_FAULT_CODE_HOST_DIAG_MARK:
        case HOST_FAULT_CODE_RX_SCAN_FAILED:
        case HOST_FAULT_CODE_TX_DONE_EARLY:
        case HOST_FAULT_CODE_TX_FIFO_RB_MISMATCH:
            break;
        default:
            break;
    }
}

#endif /* LIFETRAC_MURATA_L072_HOST_TYPES_H */
