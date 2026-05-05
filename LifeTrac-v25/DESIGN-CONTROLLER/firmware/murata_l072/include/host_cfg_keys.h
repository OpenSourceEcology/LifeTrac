#ifndef LIFETRAC_MURATA_L072_HOST_CFG_KEYS_H
#define LIFETRAC_MURATA_L072_HOST_CFG_KEYS_H

#include <stdint.h>

#define CFG_KEY_TX_POWER_DBM                 0x01U
#define CFG_KEY_TX_POWER_ADAPT_ENABLE        0x02U
#define CFG_KEY_LBT_ENABLE                   0x03U
#define CFG_KEY_LBT_THRESHOLD_DBM            0x04U
#define CFG_KEY_FHSS_ENABLE                  0x05U
#define CFG_KEY_FHSS_QUALITY_AWARE           0x06U
#define CFG_KEY_FHSS_CHANNEL_MASK            0x07U
#define CFG_KEY_DEEP_SLEEP_ENABLE            0x08U
#define CFG_KEY_BEACON_ENABLE                0x09U
#define CFG_KEY_BEACON_CHANNEL_IDX           0x0AU
#define CFG_KEY_HOST_BAUD                    0x0BU
#define CFG_KEY_REPLAY_WINDOW                0x0CU
#define CFG_KEY_IWDG_WINDOW_MS               0x0DU
#define CFG_KEY_CRYPTO_IN_L072               0x0EU

#define CFG_KEY_PROTOCOL_VERSION             0x80U
#define CFG_KEY_WIRE_SCHEMA_VERSION          0x81U
#define CFG_KEY_CFG_DIRTY                    0x82U

#define CFG_KEY_MAX_VALUE_LEN                8U
#define CFG_KEY_COUNT                        17U

#endif /* LIFETRAC_MURATA_L072_HOST_CFG_KEYS_H */
