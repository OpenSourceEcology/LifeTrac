#ifndef LIFETRAC_MURATA_L072_HOST_CMD_H
#define LIFETRAC_MURATA_L072_HOST_CMD_H

#include "host_uart.h"
#include "sx1276_rx.h"
#include "sx1276_tx.h"

#include <stdbool.h>
#include <stdint.h>

void host_cmd_init(bool radio_ok, uint8_t radio_version);
void host_cmd_dispatch(const host_frame_t *frame);
void host_cmd_dispatch_at_line(const char *line, uint16_t len);
void host_cmd_on_radio_events(uint32_t radio_events);
void host_cmd_emit_rx_frame(const sx1276_rx_frame_t *frame);
void host_cmd_emit_tx_done(const sx1276_tx_result_t *result);
void host_cmd_emit_fault(uint8_t code, uint8_t sub);
void host_cmd_emit_stats_snapshot(void);

#endif /* LIFETRAC_MURATA_L072_HOST_CMD_H */
