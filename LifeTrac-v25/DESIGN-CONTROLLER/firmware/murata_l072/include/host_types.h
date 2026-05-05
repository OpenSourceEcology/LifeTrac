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

#define HOST_CAPABILITY_BITMAP               (HOST_CAP_PING | \
                                             HOST_CAP_VERSION | \
                                             HOST_CAP_UID | \
                                             HOST_CAP_REG_IO | \
                                             HOST_CAP_STATS | \
                                             HOST_CAP_RESET)

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
/* RX_FRAME_URC payload: {u8 len, i8 snr_db, i16 rssi_dbm_le, u32 timestamp_us_le, payload[len]} */
#define HOST_TYPE_RX_FRAME_URC               0x91U

#define HOST_TYPE_CFG_OK_URC                 0xA0U
#define HOST_TYPE_CFG_DATA_URC               0xA1U

#define HOST_TYPE_REG_DATA_URC               0xB0U
#define HOST_TYPE_REG_WRITE_ACK_URC          0xB1U

#define HOST_TYPE_RADIO_IRQ_URC              0xC0U
#define HOST_TYPE_STATS_URC                  0xC1U

#define HOST_TYPE_BOOT_URC                   0xF0U
#define HOST_TYPE_FAULT_URC                  0xF1U
#define HOST_TYPE_ERR_PROTO_URC              0xFEU

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
        case HOST_TYPE_ERR_PROTO_URC:
            break;
        default:
            break;
    }
}

#endif /* LIFETRAC_MURATA_L072_HOST_TYPES_H */
