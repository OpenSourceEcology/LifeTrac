/*
 * config.h — Build-time feature flags for the Murata L072 firmware.
 *
 * Per DESIGN-LORAFIRMWARE/02 §2 ("Notes on layout choices") and review-driven
 * decisions in DESIGN-LORAFIRMWARE/05 §5.4, the launch profile is:
 *
 *   - Single-slot Flash layout (A/B deferred to Phase 6 / N-26).
 *   - Crypto Profile A: H7 owns AES-GCM; L072 transports authenticated bytes.
 *   - 8-channel FHSS baseline (R-01); quality-aware FHSS (N-06) post-launch.
 *   - Thin-modem L072 role (Copilot review §7.3).
 */

#ifndef LIFETRAC_MURATA_L072_CONFIG_H
#define LIFETRAC_MURATA_L072_CONFIG_H

/* ------------------------------------------------------------------------ */
/* Launch profile selectors — change these only via a documented decision. */
/* ------------------------------------------------------------------------ */

/* A/B firmware slots (N-26). 0 = single-slot launch; 1 = A/B (Phase 6+). */
#define LORA_FW_AB_SLOTS              0

/* Crypto location. 0 = Profile A (H7 owns GCM); 1 = Profile B (L072 owns). */
#define LORA_FW_CRYPTO_IN_L072        0

/* Quality-aware FHSS (N-06). 0 = fixed 8-channel cycle (R-01); 1 = N-06. */
#define LORA_FW_QUALITY_AWARE_FHSS    0

/* Listen-before-talk (N-04). Default ON for launch per DESIGN-LORAFIRMWARE/04 §6. */
#define LORA_FW_LBT_ENABLE            1

/* Per-frame TX-power adaptation (N-07). Default ON for launch. */
#define LORA_FW_TX_POWER_ADAPT        1

/* Deep-sleep RX scheduler (N-09). Recommended in launch set per Claude
 * review §1.4; tractor leaves OFF, handheld turns ON via runtime CFG. */
#define LORA_FW_DEEP_SLEEP_BUILD      1

/* Autonomous emergency beacon (N-30). Default ON; runtime-disable via CFG. */
#define LORA_FW_BEACON_ENABLE         1

/* ------------------------------------------------------------------------ */
/* Host UART transport sizing (DESIGN-LORAFIRMWARE/04 §2 + 05 §C/D).        */
/* Hard limits chosen to match the planned RAM budget in 02 §4.2.           */
/* ------------------------------------------------------------------------ */

#define HOST_BAUD_DEFAULT             921600UL
#define HOST_INNER_MAX_LEN            320U      /* max decoded inner frame */
#define HOST_COBS_MAX_LEN             325U      /* worst-case encoded + framing */
#define HOST_TXQ_DEPTH                8U
#define HOST_TXQ_P0_RESERVED          1U        /* per Claude §H5: keep one P0 */

/* Diagnostic/prototyping controls. Keep these conservative in production. */
#define HOST_ALLOW_REG_WRITE_DIAG     1
#define HOST_EMIT_RADIO_IRQ_DEBUG_URC 1
#define HOST_DEBUG_OPMODE_GUARD       0
#ifndef HOST_AT_SHELL_ENABLE
#define HOST_AT_SHELL_ENABLE          1
#endif
#ifndef HOST_AT_LINE_MAX_LEN
#define HOST_AT_LINE_MAX_LEN          96U
#endif

/* ------------------------------------------------------------------------ */
/* Watchdog (N-20). Initial window per Claude review §5.5: 500 ms during    */
/* boot/init, tightened to 100 ms after radio init succeeds.                */
/* ------------------------------------------------------------------------ */

#define IWDG_BOOT_WINDOW_MS           500U
#define IWDG_RUN_WINDOW_MS            100U

#endif /* LIFETRAC_MURATA_L072_CONFIG_H */
