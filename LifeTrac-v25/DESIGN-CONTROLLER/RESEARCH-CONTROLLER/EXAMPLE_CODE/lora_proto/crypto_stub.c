// crypto_stub.c — placeholder AES-128-GCM bindings.
//
// DRAFT FOR REVIEW. Not compiled or tested.
//
// REPLACE BEFORE FIELD USE. This stub copies plaintext through and writes a
// zero auth tag. It exists so the rest of the chain can be exercised end-to-end
// without pulling MbedTLS / ArduinoBearSSL in yet.
//
// Real implementations to swap in once we choose:
//   - Portenta H7 / X8 (M7+M4 firmware): MbedTLS — already part of Mbed OS, see
//     mbedtls_gcm_setkey() / mbedtls_gcm_crypt_and_tag() / mbedtls_gcm_auth_decrypt().
//   - MKR WAN 1310 (Cortex-M0+): ArduinoBearSSL br_aes_*  — smaller footprint
//     than MbedTLS, fits in ~10 kB flash, AES-GCM at ~80 kB/s on the SAMD21.
//   - Base station Python bridge: cryptography.hazmat.primitives.ciphers.aead.AESGCM.

#include "lora_proto.h"
#include <string.h>

// Build guard: opting into the insecure stub must be explicit. The Python
// bridge uses real AES-128-GCM, so silently building the C side with the stub
// would produce an *unauthenticated* radio link that *appears* to work in
// loopback against itself but fails AEAD verification against the real Python
// peer (and offers zero confidentiality on the air).
#ifndef LIFETRAC_ALLOW_STUB_CRYPTO
#  error "crypto_stub.c is a no-op AES placeholder. Either link a real "       \
         "AES-GCM (MbedTLS / ArduinoBearSSL) or define LIFETRAC_ALLOW_STUB_CRYPTO " \
         "to acknowledge that this build is for benchtop/sim use only."
#endif

bool lp_encrypt(const uint8_t* key, const uint8_t* nonce,
                const uint8_t* pt, size_t pt_len,
                uint8_t* out) {
    (void)key; (void)nonce;
    // STUB: copy plaintext, then zero 16-byte tag. NOT SECURE.
    memcpy(out, pt, pt_len);
    memset(out + pt_len, 0, 16);
    return true;
}

bool lp_decrypt(const uint8_t* key, const uint8_t* nonce,
                const uint8_t* ct, size_t ct_len,
                uint8_t* pt) {
    (void)key; (void)nonce;
    if (ct_len < 16) return false;
    // STUB: ignore tag, copy ciphertext as plaintext. NOT SECURE.
    memcpy(pt, ct, ct_len - 16);
    return true;
}
