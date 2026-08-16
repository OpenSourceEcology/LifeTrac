#ifndef LIFETRAC_TRACTOR_H7_MH_WIRE_H
#define LIFETRAC_TRACTOR_H7_MH_WIRE_H

#include <stdint.h>

/* Keep this mirror aligned with firmware/murata_l072/include/host_types.h. */

#define HOST_PROTOCOL_VER                    1U
#define HOST_WIRE_SCHEMA_VER                 2U

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

#define HOST_TYPE_VER_URC                    0x81U
#define HOST_TYPE_UID_URC                    0x82U
#define HOST_TYPE_TX_DONE_URC                0x90U
#define HOST_TYPE_RX_FRAME_URC               0x91U
#define HOST_RX_FRAME_URC_TAIL_LEN           8U
#define HOST_RX_FRAME_URC_PHASE_VALID        0x01U
#define HOST_TYPE_CFG_OK_URC                 0xA0U
#define HOST_TYPE_CFG_DATA_URC               0xA1U
#define HOST_TYPE_REG_DATA_URC               0xB0U
#define HOST_TYPE_REG_WRITE_ACK_URC          0xB1U
#define HOST_TYPE_RADIO_IRQ_URC              0xC0U
#define HOST_TYPE_STATS_URC                  0xC1U
#define HOST_TYPE_BOOT_URC                   0xF0U
#define HOST_TYPE_FAULT_URC                  0xF1U
#define HOST_TYPE_ERR_PROTO_URC              0xFEU

#define HOST_FAULT_CODE_CLOCK_HSE_FAILED     0x08U

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
/* v1.2 (W1-7 RX) additive tail: byte/parse counters and per-flag UART errors. */
#define HOST_STATS_OFFSET_HOST_RX_BYTES        68U
#define HOST_STATS_OFFSET_HOST_RX_LPUART_BYTES 72U
#define HOST_STATS_OFFSET_HOST_RX_USART1_BYTES 76U
#define HOST_STATS_OFFSET_HOST_PARSE_OK        80U
#define HOST_STATS_OFFSET_HOST_PARSE_ERR       84U
#define HOST_STATS_OFFSET_HOST_UART_ERR_LPUART 88U
#define HOST_STATS_OFFSET_HOST_UART_ERR_USART1 92U
#define HOST_STATS_OFFSET_HOST_UART_PE_LPUART  96U
#define HOST_STATS_OFFSET_HOST_UART_FE_LPUART  100U
#define HOST_STATS_OFFSET_HOST_UART_NE_LPUART  104U
#define HOST_STATS_OFFSET_HOST_UART_ORE_LPUART 108U
#define HOST_STATS_OFFSET_HOST_UART_PE_USART1  112U
#define HOST_STATS_OFFSET_HOST_UART_FE_USART1  116U
#define HOST_STATS_OFFSET_HOST_UART_NE_USART1  120U
#define HOST_STATS_OFFSET_HOST_UART_ORE_USART1 124U
#define HOST_STATS_OFFSET_HOST_RX_RING_OVF     128U
/* RS-11.5 (2026-08-02) additive tail: TX FIFO readback discriminator. */
#define HOST_STATS_OFFSET_TX_FIFO_RB_OK        132U
#define HOST_STATS_OFFSET_TX_FIFO_RB_BAD       136U
#define HOST_STATS_OFFSET_TX_DONE_EARLY        140U
#define HOST_STATS_PAYLOAD_LEN               144U

/* Legacy v1 payload emitted before radio_tx_abort_airtime was appended. */
#define HOST_STATS_LEGACY_OFFSET_RADIO_STATE 60U
#define HOST_STATS_LEGACY_PAYLOAD_LEN        64U

/* Smallest payload that carries radio_tx_abort_airtime@60 and radio_state@64,
 * i.e. the v1.1 layout. This used to be spelled HOST_STATS_PAYLOAD_LEN because
 * the two happened to be equal at 68 — but PAYLOAD_LEN tracks the CURRENT
 * layout and has since grown to 144, while the threshold at which those two
 * fields start existing has not moved. Keeping them as one constant meant that
 * mirroring the L072's growth would silently push a genuine 68-byte v1.1
 * payload down the legacy branch, where radio_state is read from offset 60 and
 * would come back wrong. They are separate constants now. */
#define HOST_STATS_MIN_LEN_WITH_AIRTIME      68U

#define HOST_HEADER_LEN                      7U
#define HOST_CRC_LEN                         2U
#define HOST_MAX_PAYLOAD_LEN                 320U
#define HOST_MAX_INNER_LEN                   (HOST_HEADER_LEN + HOST_MAX_PAYLOAD_LEN + HOST_CRC_LEN)

#endif /* LIFETRAC_TRACTOR_H7_MH_WIRE_H */
