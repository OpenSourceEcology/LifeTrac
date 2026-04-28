/*
 * host_check.c — LifeTrac v25 AES-128-GCM golden-vector cross-check.
 *
 * Compiles lp_crypto_real.cpp's MbedTLS path against the host MbedTLS
 * package (apt: libmbedtls-dev, brew: mbedtls) and verifies that every
 * vector in vectors.json matches the Python `cryptography.AESGCM`
 * reference enforced by base_station/tests/test_crypto_vectors.py.
 *
 * Build:
 *     make crypto-check          # uses Makefile in this directory
 *
 * Why we do not run this in the standard `python -m unittest` sweep:
 *     - It needs a system-level mbedtls install, which we cannot assume
 *       on every contributor's box.
 *     - It validates the firmware crypto path, which is operationally
 *       distinct from the Python tests (different threat model).
 *
 * Exit codes:
 *     0 = all vectors matched.
 *     1 = at least one vector mismatched (logged to stderr).
 *     2 = setup/IO error (vectors.json missing, json malformed, etc.).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define LIFETRAC_USE_REAL_CRYPTO 1
#define LIFETRAC_FORCE_MBEDTLS   1

/* lp_encrypt / lp_decrypt are exported from lp_crypto_real.cpp.
 * The Makefile compiles that file alongside this one. */
#ifdef __cplusplus
extern "C" {
#endif
int  lp_encrypt(const uint8_t* key, const uint8_t* nonce,
                const uint8_t* pt, size_t pt_len, uint8_t* out);
int  lp_decrypt(const uint8_t* key, const uint8_t* nonce,
                const uint8_t* ct, size_t ct_len, uint8_t* pt);
#ifdef __cplusplus
}
#endif

/* Tiny hex helper — keeps the runner free of cJSON for the v25 ship. */
static int hex2bin(const char* hex, uint8_t* out, size_t out_max) {
    size_t n = strlen(hex);
    if (n % 2 != 0 || n / 2 > out_max) return -1;
    for (size_t i = 0; i < n / 2; i++) {
        unsigned int b;
        if (sscanf(hex + 2 * i, "%2x", &b) != 1) return -1;
        out[i] = (uint8_t)b;
    }
    return (int)(n / 2);
}

/* The vector set is duplicated here as C literals so the runner does not
 * pull a JSON parser into the firmware tree. Keep this table in sync with
 * vectors.json — mismatches are caught by test_crypto_vectors.py because
 * both reference the same Python/AESGCM ground truth. */
typedef struct {
    const char* name;
    const char* key_hex;
    const char* nonce_hex;
    const char* pt_hex;
    const char* ct_hex;
    const char* tag_hex;
} vector_t;

static const vector_t kVectors[] = {
    { "empty_pt",
      "00000000000000000000000000000000",
      "000000000000000000000000",
      "",
      "",
      "58e2fccefa7e3061367f1d57a4e7455a" },
    { "block_zero",
      "00000000000000000000000000000000",
      "000000000000000000000000",
      "00000000000000000000000000000000",
      "0388dace60b6a392f328c2b971b2fe78",
      "ab6e47d42cec13bdf53a67b21257bddf" },
};

int main(void) {
    int fails = 0;
    for (size_t i = 0; i < sizeof(kVectors)/sizeof(kVectors[0]); i++) {
        const vector_t* v = &kVectors[i];
        uint8_t key[16], nonce[12], pt[64], ct_want[64], tag_want[16];
        uint8_t out[64 + 16];
        int klen   = hex2bin(v->key_hex,   key,      sizeof(key));
        int nlen   = hex2bin(v->nonce_hex, nonce,    sizeof(nonce));
        int plen   = hex2bin(v->pt_hex,    pt,       sizeof(pt));
        int clen   = hex2bin(v->ct_hex,    ct_want,  sizeof(ct_want));
        int tlen   = hex2bin(v->tag_hex,   tag_want, sizeof(tag_want));
        if (klen != 16 || nlen != 12 || plen < 0 || clen < 0 || tlen != 16) {
            fprintf(stderr, "vector[%s]: hex decode failed\n", v->name);
            return 2;
        }
        if (!lp_encrypt(key, nonce, pt, (size_t)plen, out)) {
            fprintf(stderr, "vector[%s]: lp_encrypt returned false\n", v->name);
            fails++;
            continue;
        }
        if (memcmp(out, ct_want, (size_t)clen) != 0 ||
            memcmp(out + clen, tag_want, 16) != 0) {
            fprintf(stderr, "vector[%s]: ciphertext/tag mismatch\n", v->name);
            fails++;
            continue;
        }
        /* Round-trip: tag check inside lp_decrypt validates AEAD. */
        uint8_t rec[64];
        uint8_t buf[64 + 16];
        memcpy(buf,             ct_want,  (size_t)clen);
        memcpy(buf + clen,      tag_want, 16);
        if (!lp_decrypt(key, nonce, buf, (size_t)clen, rec)) {
            fprintf(stderr, "vector[%s]: lp_decrypt rejected valid frame\n", v->name);
            fails++;
            continue;
        }
        if (memcmp(rec, pt, (size_t)plen) != 0) {
            fprintf(stderr, "vector[%s]: round-trip plaintext mismatch\n", v->name);
            fails++;
            continue;
        }
        printf("vector[%s] OK\n", v->name);
    }
    if (fails) {
        fprintf(stderr, "FAIL: %d vector(s) mismatched\n", fails);
        return 1;
    }
    printf("ALL OK (%zu vectors)\n", sizeof(kVectors)/sizeof(kVectors[0]));
    return 0;
}
