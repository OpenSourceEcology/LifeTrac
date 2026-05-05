#ifndef LIFETRAC_MURATA_L072_SX1276_RX_H
#define LIFETRAC_MURATA_L072_SX1276_RX_H

#include <stdbool.h>
#include <stdint.h>

typedef struct sx1276_rx_frame_s {
    uint8_t payload[256];
    uint8_t length;
    int8_t snr_db;
    int16_t rssi_dbm;
    uint32_t timestamp_us;
} sx1276_rx_frame_t;

bool sx1276_rx_arm(void);
void sx1276_rx_disarm(void);
bool sx1276_rx_service(uint32_t events, sx1276_rx_frame_t *out_frame);

#endif /* LIFETRAC_MURATA_L072_SX1276_RX_H */
