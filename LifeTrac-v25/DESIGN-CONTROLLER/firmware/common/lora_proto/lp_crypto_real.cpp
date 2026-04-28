// lp_crypto_real.cpp — real AES-128-GCM wrapper for LifeTrac v25 firmware.
//
// Per DECISIONS.md D-C1/C2:
//   * Portenta H7 (mbed-os core, ARDUINO_ARCH_MBED) → MbedTLS (mbedtls/gcm.h).
//   * MKR WAN 1310 (SAMD21, ARDUINO_SAMD_MKRWAN1310) → rweather/Crypto
//     (AES128 + GCM<AES128>).
//
// Build flag: define LIFETRAC_USE_REAL_CRYPTO at the sketch level (or via
// arduino-cli --build-property "build.extra_flags=-DLIFETRAC_USE_REAL_CRYPTO")
// to compile this file's lp_encrypt/lp_decrypt instead of the crypto_stub.c
// versions. Production firmware MUST set this flag; CI builds may continue to
// use crypto_stub.c by also defining LIFETRAC_ALLOW_STUB_CRYPTO.
//
// Golden-vector cross-check vs cryptography.AESGCM lives in
// base_station/tests/test_crypto_vectors.py (TODO once a board is on the
// bench). Until then the stub remains the default for unit tests.

#include "lora_proto.h"

#ifdef LIFETRAC_USE_REAL_CRYPTO

#include <string.h>

// ===== Portenta H7 path (mbed-os MbedTLS) ====================================
#if defined(ARDUINO_ARCH_MBED) || defined(LIFETRAC_FORCE_MBEDTLS)
  #include "mbedtls/gcm.h"

  static int lp__gcm_run(int mode,
                         const uint8_t* key, const uint8_t* nonce,
                         const uint8_t* in, size_t in_len,
                         uint8_t* out_buf, uint8_t* tag_io) {
      mbedtls_gcm_context ctx;
      mbedtls_gcm_init(&ctx);
      int rc = mbedtls_gcm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, key, 128);
      if (rc != 0) { mbedtls_gcm_free(&ctx); return rc; }
      if (mode == MBEDTLS_GCM_ENCRYPT) {
          rc = mbedtls_gcm_crypt_and_tag(&ctx, MBEDTLS_GCM_ENCRYPT, in_len,
                                         nonce, 12, NULL, 0,
                                         in, out_buf, 16, tag_io);
      } else {
          rc = mbedtls_gcm_auth_decrypt(&ctx, in_len,
                                        nonce, 12, NULL, 0,
                                        tag_io, 16, in, out_buf);
      }
      mbedtls_gcm_free(&ctx);
      return rc;
  }

  bool lp_encrypt(const uint8_t* key, const uint8_t* nonce,
                  const uint8_t* pt, size_t pt_len, uint8_t* out) {
      // Layout written by us: ciphertext[pt_len] | tag[16].
      uint8_t* tag = out + pt_len;
      return lp__gcm_run(MBEDTLS_GCM_ENCRYPT, key, nonce, pt, pt_len, out, tag) == 0;
  }

  bool lp_decrypt(const uint8_t* key, const uint8_t* nonce,
                  const uint8_t* ct, size_t ct_len, uint8_t* pt) {
      // ct buffer carries ct_len + 16-byte tag at the end.
      uint8_t tag_copy[16];
      memcpy(tag_copy, ct + ct_len, 16);
      return lp__gcm_run(MBEDTLS_GCM_DECRYPT, key, nonce, ct, ct_len, pt, tag_copy) == 0;
  }

// ===== MKR WAN 1310 path (rweather/Crypto) ===================================
#elif defined(ARDUINO_SAMD_MKRWAN1310) || defined(LIFETRAC_FORCE_RWEATHER_CRYPTO)
  #include <Crypto.h>
  #include <AES.h>
  #include <GCM.h>

  bool lp_encrypt(const uint8_t* key, const uint8_t* nonce,
                  const uint8_t* pt, size_t pt_len, uint8_t* out) {
      GCM<AES128> gcm;
      if (!gcm.setKey(key, 16)) return false;
      if (!gcm.setIV(nonce, 12)) return false;
      gcm.encrypt(out, pt, pt_len);
      gcm.computeTag(out + pt_len, 16);
      return true;
  }

  bool lp_decrypt(const uint8_t* key, const uint8_t* nonce,
                  const uint8_t* ct, size_t ct_len, uint8_t* pt) {
      GCM<AES128> gcm;
      if (!gcm.setKey(key, 16)) return false;
      if (!gcm.setIV(nonce, 12)) return false;
      gcm.decrypt(pt, ct, ct_len);
      return gcm.checkTag(ct + ct_len, 16);
  }

#else
  #error "lp_crypto_real.cpp: no AES-GCM backend selected for this MCU. \
Define LIFETRAC_FORCE_MBEDTLS or LIFETRAC_FORCE_RWEATHER_CRYPTO, or build \
without LIFETRAC_USE_REAL_CRYPTO and accept the stub (CI/sim only)."
#endif

#endif  // LIFETRAC_USE_REAL_CRYPTO
