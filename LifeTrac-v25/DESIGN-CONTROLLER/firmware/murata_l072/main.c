/*
 * main.c - Murata L072 firmware entry point.
 *
 * This increment wires real bare-metal primitives:
 *   - BOOT-region safe-mode listener
 *   - HSI16 + SysTick platform bring-up
 *   - UART2 DMA-on-IDLE host transport
 *   - SX1276 register-level SPI driver + DIO IRQ capture
 */

#include "config.h"
#include "host_uart.h"
#include "platform.h"
#include "sx1276.h"
#include "stm32l072_regs.h"

#include <stdbool.h>
#include <stdint.h>

extern void safe_mode_listen(void);

extern const unsigned long _mm_static_asserts_anchor;
const unsigned long * const _mm_anchor_ref = &_mm_static_asserts_anchor;

#define HOST_TYPE_PING_REQ           0x00U
#define HOST_TYPE_VER_REQ            0x01U
#define HOST_TYPE_REG_READ_REQ       0x30U
#define HOST_TYPE_REG_WRITE_REQ      0x31U

#define HOST_TYPE_VER_RSP            0x81U
#define HOST_TYPE_REG_WRITE_RSP      0xA0U
#define HOST_TYPE_REG_READ_RSP       0xB0U

#define HOST_TYPE_BOOT_URC           0xF0U
#define HOST_TYPE_RADIO_IRQ_URC      0xF1U

static void handle_host_frame(const host_frame_t *frame) {
    if (frame->ver != 1U) {
        host_uart_send_err_proto(frame->seq, frame->type, frame->ver);
        return;
    }

    switch (frame->type) {
        case HOST_TYPE_PING_REQ:
            host_uart_send_urc(HOST_TYPE_PING_REQ,
                               frame->seq,
                               frame->flags,
                               frame->payload,
                               frame->payload_len);
            break;

        case HOST_TYPE_VER_REQ: {
            static const uint8_t kVer[] = "murata_l072 baremetal v0.1.0";
            host_uart_send_urc(HOST_TYPE_VER_RSP,
                               frame->seq,
                               0U,
                               kVer,
                               (uint16_t)(sizeof(kVer) - 1U));
            break;
        }

        case HOST_TYPE_REG_READ_REQ: {
            uint8_t rsp[2];
            if (frame->payload_len != 1U) {
                host_uart_send_err_proto(frame->seq, frame->type, frame->ver);
                break;
            }

            rsp[0] = frame->payload[0];
            rsp[1] = sx1276_read_reg(frame->payload[0]);
            host_uart_send_urc(HOST_TYPE_REG_READ_RSP,
                               frame->seq,
                               0U,
                               rsp,
                               (uint16_t)sizeof(rsp));
            break;
        }

        case HOST_TYPE_REG_WRITE_REQ: {
            if (frame->payload_len != 2U) {
                host_uart_send_err_proto(frame->seq, frame->type, frame->ver);
                break;
            }

            sx1276_write_reg(frame->payload[0], frame->payload[1]);
            host_uart_send_urc(HOST_TYPE_REG_WRITE_RSP,
                               frame->seq,
                               0U,
                               frame->payload,
                               2U);
            break;
        }

        default:
            host_uart_send_err_proto(frame->seq, frame->type, frame->ver);
            break;
    }
}

int main(void) {
    host_frame_t frame;
    bool radio_ok;

    /* Brick-recovery Layer 2 must run before any normal init. */
    safe_mode_listen();

    platform_clock_init_hsi16();
    platform_systick_init_1ms();
    host_uart_init(HOST_BAUD_DEFAULT);

    radio_ok = sx1276_init();

    {
        uint8_t boot_payload[2];
        boot_payload[0] = radio_ok ? 1U : 0U;
        boot_payload[1] = sx1276_read_version();
        host_uart_send_urc(HOST_TYPE_BOOT_URC, 0U, 0U, boot_payload, 2U);
    }

    for (;;) {
        host_uart_poll_dma();

        while (host_uart_pop_frame(&frame)) {
            handle_host_frame(&frame);
        }

        {
            const uint32_t radio_events = sx1276_take_irq_events();
            if (radio_events != 0U) {
                uint8_t payload[4];
                payload[0] = (uint8_t)(radio_events & 0xFFU);
                payload[1] = (uint8_t)((radio_events >> 8) & 0xFFU);
                payload[2] = (uint8_t)((radio_events >> 16) & 0xFFU);
                payload[3] = (uint8_t)((radio_events >> 24) & 0xFFU);
                host_uart_send_urc(HOST_TYPE_RADIO_IRQ_URC, 0U, 0U, payload, 4U);
            }
        }

        cpu_wfi();
    }
}
