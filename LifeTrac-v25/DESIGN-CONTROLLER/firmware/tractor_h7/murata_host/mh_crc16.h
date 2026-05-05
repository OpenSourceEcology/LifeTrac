#ifndef LIFETRAC_TRACTOR_H7_MH_CRC16_H
#define LIFETRAC_TRACTOR_H7_MH_CRC16_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint16_t mh_crc16_ccitt(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* LIFETRAC_TRACTOR_H7_MH_CRC16_H */
