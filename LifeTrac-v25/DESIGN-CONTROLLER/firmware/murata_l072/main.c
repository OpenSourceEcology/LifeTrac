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
#include "sx1276_tx.h"
#include "stm32l072_regs.h"

#include <stdbool.h>
#include <stdint.h>

extern void safe_mode_listen(void);

extern const unsigned long _mm_static_asserts_anchor;
const unsigned long * const _mm_anchor_ref = &_mm_static_asserts_anchor;

int main(void) {
    host_frame_t frame;
    sx1276_rx_frame_t rx_frame;
    sx1276_tx_result_t tx_result;
    bool radio_ok;
    uint8_t radio_version;
    uint32_t boot_ms;
    uint8_t rx_seen_latch = 0U;
    uint8_t diag_marks_latch = 0U;
    bool emitted_rx_seen = false;
    bool emitted_rx_inactive = false;
    bool emitted_parse_error = false;
    bool emitted_diag_mark = false;

    platform_reset_cause_capture_early();

    /* Brick-recovery Layer 2 must run before any normal init. */
    safe_mode_listen();

    platform_clock_init_hsi16();
    platform_systick_init_1ms();
    host_uart_init(HOST_BAUD_DEFAULT);
#if LIFETRAC_BENCH_BOOT_HEARTBEAT_ENABLE
    host_uart_send_ascii("LT_BOOT_HEARTBEAT stage=post_uart_init\\r\\n");
#endif

    radio_ok = sx1276_init();
    if (!radio_ok) {
        host_cmd_emit_fault(HOST_FAULT_CODE_RADIO_INIT_FAIL, 0U);
        /* TEMPORARY FOR DEBUGGING: Continue even if radio fails to test binary protocol path */
        platform_diag_trace("M:RADIO_BYPASS\r\n");
    }
    if (radio_ok) {
        radio_ok = sx1276_rx_arm();
    }
    /* TEMPORARY: Continue even if RX arm fails to test binary protocol path */
#if LIFETRAC_BENCH_BOOT_HEARTBEAT_ENABLE
    if (radio_ok) {
        host_uart_send_ascii("LT_BOOT_HEARTBEAT stage=radio_ready\\r\\n");
    } else {
        host_uart_send_ascii("LT_BOOT_HEARTBEAT stage=radio_fault\\r\\n");
    }
#endif
    radio_version = sx1276_read_version();
    host_cmd_init(radio_ok, radio_version);
    boot_ms = platform_now_ms();

    if (platform_clock_source_id() != PLATFORM_CLOCK_SOURCE_HSE_OK) {
        host_cmd_emit_fault(HOST_FAULT_CODE_CLOCK_HSE_FAILED,
                            platform_clock_source_id());
    }

    for (;;) {
        const uint32_t now_ms = platform_now_ms();

        host_uart_poll_dma();
        host_uart_service_rx();
        host_uart_flush_diag_traces();

        rx_seen_latch = (uint8_t)(rx_seen_latch | host_uart_take_rx_seen_flags());
        diag_marks_latch = (uint8_t)(diag_marks_latch | host_uart_take_diag_marks());

        if (!emitted_rx_seen && rx_seen_latch != 0U) {
            host_cmd_emit_fault(HOST_FAULT_CODE_HOST_RX_SEEN, rx_seen_latch);
            host_cmd_emit_stats_snapshot();
            emitted_rx_seen = true;
        }

        if (!emitted_diag_mark && diag_marks_latch != 0U) {
            host_cmd_emit_fault(HOST_FAULT_CODE_HOST_DIAG_MARK, diag_marks_latch);
            host_cmd_emit_stats_snapshot();
            emitted_diag_mark = true;
        }

        if (!emitted_parse_error && host_uart_stats_parse_err() != 0U) {
            uint32_t parse_err = host_uart_stats_parse_err();
            uint8_t parse_sub = (parse_err > 0xFFU) ? 0xFFU : (uint8_t)parse_err;
            host_cmd_emit_fault(HOST_FAULT_CODE_HOST_PARSE_ERROR, parse_sub);
            host_cmd_emit_stats_snapshot();
            emitted_parse_error = true;
        }

        if (!emitted_rx_inactive && !emitted_rx_seen &&
            ((now_ms - boot_ms) >= 3000U) &&
            host_uart_stats_rx_bytes() == 0U) {
            host_cmd_emit_fault(HOST_FAULT_CODE_HOST_RX_INACTIVE, 0U);
            host_cmd_emit_stats_snapshot();
            emitted_rx_inactive = true;
        }

        if (host_uart_take_dma_te_events() != 0U) {
            host_cmd_emit_fault(HOST_FAULT_CODE_HOST_DMA_OVERRUN, 0U);
        }

        while (host_uart_pop_frame(&frame)) {
            host_cmd_dispatch(&frame);
        }

        {
            const uint32_t radio_events = sx1276_take_irq_events();
            host_cmd_on_radio_events(radio_events);

            if (sx1276_tx_poll(radio_events, &tx_result)) {
                host_cmd_emit_tx_done(&tx_result);
            }

            if (!sx1276_tx_busy() && sx1276_rx_service(radio_events, &rx_frame)) {
                host_cmd_emit_rx_frame(&rx_frame);
            }
        }

        cpu_wfi();
    }
}
