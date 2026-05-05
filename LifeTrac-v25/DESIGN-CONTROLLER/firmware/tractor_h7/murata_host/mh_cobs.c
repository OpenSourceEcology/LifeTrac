#include "mh_cobs.h"

#include <stddef.h>

size_t mh_cobs_encode(const uint8_t *in, size_t in_len, uint8_t *out, size_t out_cap) {
    size_t read_idx = 0U;
    size_t write_idx = 1U;
    size_t code_idx = 0U;
    uint8_t code = 1U;

    if (out == NULL) {
        return 0U;
    }

    if (in_len > 0U && in == NULL) {
        return 0U;
    }

    if (out_cap == 0U) {
        return 0U;
    }

    while (read_idx < in_len) {
        if (in[read_idx] == 0U) {
            if (code_idx >= out_cap) {
                return 0U;
            }
            out[code_idx] = code;
            code = 1U;
            code_idx = write_idx;
            write_idx++;
            if (write_idx > out_cap) {
                return 0U;
            }
        } else {
            if (write_idx >= out_cap) {
                return 0U;
            }
            out[write_idx++] = in[read_idx];
            code++;
            if (code == 0xFFU) {
                if (code_idx >= out_cap) {
                    return 0U;
                }
                out[code_idx] = code;
                code = 1U;
                code_idx = write_idx;
                write_idx++;
                if (write_idx > out_cap) {
                    return 0U;
                }
            }
        }
        read_idx++;
    }

    if (code_idx >= out_cap) {
        return 0U;
    }

    out[code_idx] = code;
    return write_idx;
}

size_t mh_cobs_decode(const uint8_t *in, size_t in_len, uint8_t *out, size_t out_cap) {
    size_t read_idx = 0U;
    size_t write_idx = 0U;

    if (out == NULL) {
        return 0U;
    }

    if (in_len > 0U && in == NULL) {
        return 0U;
    }

    while (read_idx < in_len) {
        uint8_t code;
        uint8_t i;

        code = in[read_idx++];
        if (code == 0U) {
            return 0U;
        }

        for (i = 1U; i < code; ++i) {
            if (read_idx >= in_len || write_idx >= out_cap) {
                return 0U;
            }
            out[write_idx++] = in[read_idx++];
        }

        if (code < 0xFFU && read_idx < in_len) {
            if (write_idx >= out_cap) {
                return 0U;
            }
            out[write_idx++] = 0U;
        }
    }

    return write_idx;
}
