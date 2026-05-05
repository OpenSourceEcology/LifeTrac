#include "mh_crc16.h"

uint16_t mh_crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFFU;
    size_t i;

    if (data == 0) {
        return crc;
    }

    for (i = 0U; i < len; ++i) {
        uint8_t bit;
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }

    return crc;
}
