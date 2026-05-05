#ifndef LIFETRAC_TRACTOR_H7_MH_COBS_H
#define LIFETRAC_TRACTOR_H7_MH_COBS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

size_t mh_cobs_encode(const uint8_t *in, size_t in_len, uint8_t *out, size_t out_cap);
size_t mh_cobs_decode(const uint8_t *in, size_t in_len, uint8_t *out, size_t out_cap);

#ifdef __cplusplus
}
#endif

#endif /* LIFETRAC_TRACTOR_H7_MH_COBS_H */
