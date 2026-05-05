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
#include "host_cmd.h"
#include "host_uart.h"
#include "platform.h"
#include "sx1276.h"
#include "sx1276_rx.h"
#include "stm32l072_regs.h"

#include <stdbool.h>
#include <stdint.h>

extern void safe_mode_listen(void);

extern const unsigned long _mm_static_asserts_anchor;
const unsigned long * const _mm_anchor_ref = &_mm_static_asserts_anchor;

int main(void) {
    host_frame_t frame;
    sx1276_rx_frame_t rx_frame;
    bool radio_ok;
    uint8_t radio_version;

    /* Brick-recovery Layer 2 must run before any normal init. */
    safe_mode_listen();

    platform_clock_init_hsi16();
    platform_systick_init_1ms();
    host_uart_init(HOST_BAUD_DEFAULT);

    radio_ok = sx1276_init();
    if (radio_ok) {
        radio_ok = sx1276_rx_arm();
    }
    radio_version = sx1276_read_version();
    host_cmd_init(radio_ok, radio_version);

    for (;;) {
        host_uart_poll_dma();

        while (host_uart_pop_frame(&frame)) {
            host_cmd_dispatch(&frame);
        }

        {
            const uint32_t radio_events = sx1276_take_irq_events();
            host_cmd_on_radio_events(radio_events);

            if (sx1276_rx_service(radio_events, &rx_frame)) {
                host_cmd_emit_rx_frame(&rx_frame);
            }
        }

        cpu_wfi();
    }
}
