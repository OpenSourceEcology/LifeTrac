#ifndef LIFETRAC_MURATA_L072_SX1276_TX_H
#define LIFETRAC_MURATA_L072_SX1276_TX_H

#include <stdbool.h>
#include <stdint.h>

#define SX1276_TX_STATUS_OK          0U
#define SX1276_TX_STATUS_TIMEOUT     1U
#define SX1276_TX_STATUS_LBT_ABORT   2U
#define SX1276_TX_STATUS_BUSY        3U

typedef struct sx1276_tx_request_s {
    uint8_t tx_id;
    uint8_t payload[255];
    uint8_t length;
} sx1276_tx_request_t;

typedef struct sx1276_tx_result_s {
    uint8_t tx_id;
    uint8_t status;
    uint8_t tx_power_dbm;
    uint32_t time_on_air_us;
} sx1276_tx_result_t;

bool sx1276_tx_begin(const sx1276_tx_request_t *req);
bool sx1276_tx_poll(uint32_t events, sx1276_tx_result_t *out_result);
bool sx1276_tx_busy(void);

#endif /* LIFETRAC_MURATA_L072_SX1276_TX_H */