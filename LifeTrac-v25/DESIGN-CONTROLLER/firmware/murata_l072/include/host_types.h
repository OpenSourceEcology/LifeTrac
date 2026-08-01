#ifndef LIFETRAC_MURATA_L072_HOST_TYPES_H
#define LIFETRAC_MURATA_L072_HOST_TYPES_H

#include <stdint.h>

#define HOST_PROTOCOL_VER                    1U
#define HOST_WIRE_SCHEMA_VER                 2U

#define HOST_CAP_PING                        (1UL << 0)
#define HOST_CAP_VERSION                     (1UL << 1)
#define HOST_CAP_UID                         (1UL << 2)
#define HOST_CAP_REG_IO                      (1UL << 3)
#define HOST_CAP_STATS                       (1UL << 4)
#define HOST_CAP_RESET                       (1UL << 5)
#define HOST_CAP_TX_FRAME                    (1UL << 6)

#define HOST_CAPABILITY_BITMAP               (HOST_CAP_PING | \
                                             HOST_CAP_VERSION | \
                                             HOST_CAP_UID | \
                                             HOST_CAP_REG_IO | \
                                             HOST_CAP_STATS | \
                                             HOST_CAP_RESET | \
                                             HOST_CAP_TX_FRAME)

/* Host-to-L072 commands. */
#define HOST_TYPE_PING_REQ                   0x00U
#define HOST_TYPE_VER_REQ                    0x01U
#define HOST_TYPE_UID_REQ                    0x02U
#define HOST_TYPE_RESET_REQ                  0x03U

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
#define HOST_STATS_PAYLOAD_LEN               132U

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
 * RX_SCAN_FAILED — emitted by the SCANNING-state failure handler in
 * radio/sx1276_rx.c when the FHSS rendezvous scan has failed to lock
 * within REDESIGN_MS (per FCC-A6c-1 policy, sx1276_rx_scan_policy.c).
 *
 * The `sub` byte carries a four-class disambiguation encoding so the
 * host can distinguish root cause at the fault edge without a wider
 * URC payload (per AI NOTES/2026-05-19_RX_Scan_FAILED_State_Analysis
 * §13.1 #2, A6c-3-a):
 *
 *   bit 7 : final       — 1 == retries exhausted, FAILED entered
 *   bit 6 : warm        — 1 == this boot has previously LOCKED
 *   bit 5 : hw_suspect  — 1 == no DIO0 IRQ observed during scan
 *   bit 4 : crc_seen    — 1 == >=1 CRC-error IRQ observed during scan
 *   bits 3..0 : attempt — 0..15 saturating, this attempt's index
 *
 * Emission semantics (B-prime, per §13.1 #6) and the file-statics that
 * back the `sub` bits land in FCC-A6c-3-b / A6c-3-c. This define is
 * declaration-only (FCC-A6c-3-a); no code emits it yet.
 */
#define HOST_FAULT_CODE_RX_SCAN_FAILED       0x0DU

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

/* Duplicate case labels produce a compile-time error if any ID collides. */
static inline void host_type_compile_time_uniqueness_check(void) {
    switch (0U) {
        case HOST_TYPE_PING_REQ:
        case HOST_TYPE_VER_REQ:
        case HOST_TYPE_UID_REQ:
        case HOST_TYPE_RESET_REQ:
        case HOST_TYPE_TX_FRAME_REQ:
        case HOST_TYPE_CFG_SET_REQ:
        case HOST_TYPE_CFG_GET_REQ:
        case HOST_TYPE_REG_READ_REQ:
        case HOST_TYPE_REG_WRITE_REQ:
        case HOST_TYPE_STATS_RESET_REQ:
        case HOST_TYPE_STATS_DUMP_REQ:
        case HOST_TYPE_VER_URC:
        case HOST_TYPE_UID_URC:
        case HOST_TYPE_TX_DONE_URC:
        case HOST_TYPE_RX_FRAME_URC:
        case HOST_TYPE_CFG_OK_URC:
        case HOST_TYPE_CFG_DATA_URC:
        case HOST_TYPE_REG_DATA_URC:
        case HOST_TYPE_REG_WRITE_ACK_URC:
        case HOST_TYPE_RADIO_IRQ_URC:
        case HOST_TYPE_STATS_URC:
        case HOST_TYPE_AIRTIME_INVARIANT_REJECT_URC:
        case HOST_TYPE_BOOT_URC:
        case HOST_TYPE_FAULT_URC:
        case HOST_TYPE_READY_URC:
        case HOST_TYPE_ERR_PROTO_URC:
            break;
        default:
            break;
    }
}

#endif /* LIFETRAC_MURATA_L072_HOST_TYPES_H */
