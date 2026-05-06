/*
 * memory_map.h — Single source of truth for STM32L072CZ Flash & RAM layout.
 *
 * SCOPE: this header is consumed by BOTH C/C++ source AND the linker script
 *        (ld/stm32l072cz_flash.ld) via `arm-none-eabi-cpp`. Do NOT mirror any
 *        of these constants anywhere else. If a value needs to change, change
 *        it here and the static_asserts in include/static_asserts.c will tell
 *        you whether the change is still self-consistent.
 *
 * Decision references:
 *   - DESIGN-LORAFIRMWARE/02 §4.3 (memory-map narrative, with the arithmetic
 *     errors flagged in DESIGN-LORAFIRMWARE/05 §5.1 fixed here).
 *   - DESIGN-LORAFIRMWARE/05 §5.1 (Claude Opus 4.7) — defer A/B slots to
 *     Phase 6 (N-26); launch is single-slot. Layout below preserves
 *     A/B-friendly alignment so the Phase 6 split is a #define swap.
 *   - DESIGN-LORAFIRMWARE/05 §5.4 (Codex E) — single header, asserts at build.
 *
 * IMPORTANT: this file MUST stay valid C preprocessor input — no C statements,
 * no typedefs, no inline functions. ld scans it via the preprocessor.
 */

#ifndef LIFETRAC_MURATA_L072_MEMORY_MAP_H
#define LIFETRAC_MURATA_L072_MEMORY_MAP_H

/* ------------------------------------------------------------------------ */
/* Chip totals — STM32L072CZ                                                */
/* ------------------------------------------------------------------------ */

#define MM_FLASH_BASE          0x08000000
#define MM_FLASH_SIZE          (192 * 1024)     /* 0x30000 */
#define MM_FLASH_END           (MM_FLASH_BASE + MM_FLASH_SIZE)

#define MM_RAM_BASE            0x20000000
#define MM_RAM_SIZE            (20 * 1024)      /* 0x5000  */
#define MM_RAM_END             (MM_RAM_BASE + MM_RAM_SIZE)

/*
 * STM32L0 Flash erase granularity is 128 bytes (one "page"). We align region
 * boundaries to 4 KB anyway because:
 *   (a) it leaves headroom for the future A/B split (N-26) which sizes slots
 *       in 4 KB increments,
 *   (b) it matches the conceptual "logical sector" used throughout the
 *       design docs, and
 *   (c) ST documents 4 KB as a safe granularity for option-byte and
 *       calibration storage.
 */
#define MM_FLASH_PAGE_SIZE     128
#define MM_REGION_ALIGN        (4 * 1024)       /* 0x1000 */

/* ------------------------------------------------------------------------ */
/* Launch (single-slot) Flash layout                                        */
/*                                                                          */
/*  0x08000000  BOOT     4 KB    resident: vectors + safe-mode + slot stub  */
/*  0x08001000  APP    180 KB    single launch slot                         */
/*  0x0802E000  CFG      8 KB    calibration + persisted config + flags     */
/*  0x08030000  --- end of Flash (192 KB) ---                               */
/* ------------------------------------------------------------------------ */

#define MM_BOOT_BASE           MM_FLASH_BASE
#define MM_BOOT_SIZE           (4 * 1024)       /* 0x01000 */

#define MM_APP_BASE            (MM_BOOT_BASE + MM_BOOT_SIZE)
#define MM_APP_SIZE            (180 * 1024)     /* 0x2D000 */

#define MM_CFG_BASE            (MM_APP_BASE + MM_APP_SIZE)
#define MM_CFG_SIZE            (8 * 1024)       /* 0x02000 */

/* ------------------------------------------------------------------------ */
/* Future A/B slot layout (Phase 6 / N-26) — DO NOT consume in Phase 1.     */
/* Provided here so the Phase 6 patch is a single #define swap, not a       */
/* re-derivation of every address. APP region splits into Slot A + Slot B,  */
/* boot and cfg regions are unchanged.                                      */
/* ------------------------------------------------------------------------ */

#define MM_FUTURE_SLOT_A_BASE  MM_APP_BASE
#define MM_FUTURE_SLOT_A_SIZE  (88 * 1024)      /* 0x16000 */

#define MM_FUTURE_SLOT_B_BASE  (MM_FUTURE_SLOT_A_BASE + MM_FUTURE_SLOT_A_SIZE)
#define MM_FUTURE_SLOT_B_SIZE  (88 * 1024)      /* 0x16000 */

/* MM_APP_SIZE - (2 * MM_FUTURE_SLOT_*_SIZE) = 4 KB unused gap.  When N-26  */
/* lands, that gap moves into either slot or into an enlarged CFG region —  */
/* the static_asserts file documents the available choices.                 */

/* ------------------------------------------------------------------------ */
/* RAM layout                                                               */
/*                                                                          */
/*  0x20000000  .data / .bss / pools (grow up)                              */
/*  ...                                                                     */
/*  stack (grows down from MM_RAM_END)                                      */
/*  0x20005000  --- end of RAM (20 KB) ---                                  */
/*                                                                          */
/* Stack reservation is enforced via _stack_size in the linker script and   */
/* via a static_assert that the runtime free area still leaves margin.      */
/* ------------------------------------------------------------------------ */

#define MM_STACK_SIZE          (2560)             /* 2.5 KB; Claude rev §2.3 */
#define MM_STACK_TOP           MM_RAM_END
#define MM_STACK_BASE          (MM_RAM_END - MM_STACK_SIZE)

/* Minimum free heap/.bss headroom we want left after stack reservation.    */
/* Tripping this is a "the firmware grew too much" early-warning signal     */
/* rather than a chip-capacity error.                                       */
#define MM_RAM_HEADROOM_MIN    (4 * 1024)

/* ------------------------------------------------------------------------ */
/* STM32 system memory (ROM bootloader entry point) — used by safe_mode.c   */
/* See AN2606. The L0-series bootloader lives at 0x1FF00000.                */
/* ------------------------------------------------------------------------ */

#define MM_SYSMEM_BASE         0x1FF00000

#endif /* LIFETRAC_MURATA_L072_MEMORY_MAP_H */
