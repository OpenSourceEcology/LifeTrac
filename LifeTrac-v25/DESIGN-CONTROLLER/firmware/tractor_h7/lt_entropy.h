#ifndef LIFETRAC_TRACTOR_H7_LT_ENTROPY_H
#define LIFETRAC_TRACTOR_H7_LT_ENTROPY_H

/*
 * H7-13: random bytes for AES-GCM nonces, drawn from a hardware entropy
 * source (the STM32H747 RNG through mbed's trng_api on target, a fake in
 * the host tests).
 *
 * The source is read in 32-bit words with a continuous test: a word equal
 * to the one before it latches the pool as failed for good (a stuck RNG),
 * as in the FIPS 140-2 CRNGT. A source error fails only that call. After
 * any failure lt_entropy_get() returns false and the caller must not send
 * the frame: a predictable nonce tail is worse than a dropped frame.
 *
 * Pure C with no I/O so the host vectors can pin it.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Fill out[0..len) with hardware entropy; false on a source error. */
typedef bool (*lt_entropy_source_t)(uint8_t *out, size_t len, void *ctx);

#define LT_ENTROPY_POOL_LEN 32U

typedef struct lt_entropy_s {
    lt_entropy_source_t source;
    void *ctx;
    uint8_t pool[LT_ENTROPY_POOL_LEN];
    uint8_t pos;            /* bytes of pool already handed out */
    bool have_last;
    uint32_t last_word;
    bool failed;            /* latched by the continuous test */

    uint32_t refills;
    uint32_t source_errors;
    uint32_t repeat_errors;
} lt_entropy_t;

void lt_entropy_init(lt_entropy_t *e, lt_entropy_source_t source, void *ctx);

/* Copy len fresh bytes into out. False if the source failed or the pool is
 * latched failed; out is then not to be used. */
bool lt_entropy_get(lt_entropy_t *e, uint8_t *out, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* LIFETRAC_TRACTOR_H7_LT_ENTROPY_H */
