#ifndef LIFETRAC_MURATA_L072_HOST_TYPES_H
#define LIFETRAC_MURATA_L072_HOST_TYPES_H

#include <stdint.h>

#define HOST_PROTOCOL_VER                    1U
#define HOST_WIRE_SCHEMA_VER                 1U

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
/* RX_FRAME_URC payload: {u8 len, i8 snr_db, i16 rssi_dbm_le, u32 timestamp_us_le, payload[len]} */
#define HOST_TYPE_RX_FRAME_URC               0x91U

#define HOST_TYPE_CFG_OK_URC                 0xA0U
#define HOST_TYPE_CFG_DATA_URC               0xA1U

#define HOST_TYPE_REG_DATA_URC               0xB0U
#define HOST_TYPE_REG_WRITE_ACK_URC          0xB1U

#define HOST_TYPE_RADIO_IRQ_URC              0xC0U
#define HOST_TYPE_STATS_URC                  0xC1U

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
#define HOST_STATS_PAYLOAD_LEN               96U

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
