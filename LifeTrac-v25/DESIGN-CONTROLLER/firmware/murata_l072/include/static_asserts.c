/*
 * static_asserts.c — Build-time validation of the memory map.
 *
 * This translation unit contains no runtime code. Its sole purpose is to
 * fail compilation if any constant in include/memory_map.h drifts into an
 * inconsistent state. The reviews in DESIGN-LORAFIRMWARE/05 documented
 * arithmetic errors in the original 02 §4.3 memory map; those errors would
 * have shipped silently without enforcement like this.
 *
 * Every assert below MUST stay true. If you change a region size, the
 * compiler will tell you which other constraint you broke.
 */

#include "memory_map.h"

/* C11 _Static_assert is supported on arm-none-eabi-gcc with -std=gnu11. */

/* ------------------------------------------------------------------------ */
/* Chip-capacity invariants                                                 */
/* ------------------------------------------------------------------------ */

_Static_assert(MM_FLASH_SIZE == (192UL * 1024UL),
               "L072CZ has exactly 192 KB Flash");

_Static_assert(MM_RAM_SIZE == (20UL * 1024UL),
               "L072CZ has exactly 20 KB RAM");

/* ------------------------------------------------------------------------ */
/* Launch layout fits the chip exactly                                      */
/* ------------------------------------------------------------------------ */

_Static_assert(MM_BOOT_SIZE + MM_APP_SIZE + MM_CFG_SIZE == MM_FLASH_SIZE,
               "Launch Flash regions must sum to exactly 192 KB");

_Static_assert(MM_BOOT_BASE == MM_FLASH_BASE,
               "BOOT region must start at Flash base (vector table location)");

_Static_assert(MM_APP_BASE == MM_BOOT_BASE + MM_BOOT_SIZE,
               "APP region must immediately follow BOOT region (no gap)");

_Static_assert(MM_CFG_BASE == MM_APP_BASE + MM_APP_SIZE,
               "CFG region must immediately follow APP region (no gap)");

_Static_assert(MM_CFG_BASE + MM_CFG_SIZE == MM_FLASH_END,
               "CFG region must end exactly at Flash end");

/* ------------------------------------------------------------------------ */
/* Alignment — every region must start on a 4 KB logical-sector boundary   */
/* and be a multiple of the 128-byte Flash page erase granularity.         */
/* ------------------------------------------------------------------------ */

_Static_assert((MM_BOOT_BASE % MM_REGION_ALIGN) == 0,
               "BOOT base must be 4 KB aligned");
_Static_assert((MM_APP_BASE  % MM_REGION_ALIGN) == 0,
               "APP base must be 4 KB aligned");
_Static_assert((MM_CFG_BASE  % MM_REGION_ALIGN) == 0,
               "CFG base must be 4 KB aligned");

_Static_assert((MM_BOOT_SIZE % MM_FLASH_PAGE_SIZE) == 0,
               "BOOT size must be a multiple of the 128-byte Flash page");
_Static_assert((MM_APP_SIZE  % MM_FLASH_PAGE_SIZE) == 0,
               "APP size must be a multiple of the 128-byte Flash page");
_Static_assert((MM_CFG_SIZE  % MM_FLASH_PAGE_SIZE) == 0,
               "CFG size must be a multiple of the 128-byte Flash page");

/* ------------------------------------------------------------------------ */
/* BOOT region must be large enough to actually hold a vector table + the   */
/* safe-mode listener. Vector table on M0+ with 32 IRQs is ~192 bytes; the  */
/* listener is ~512 bytes worst case. Reserve at least 1 KB.                */
/* ------------------------------------------------------------------------ */

_Static_assert(MM_BOOT_SIZE >= 1024UL,
               "BOOT region needs at least 1 KB for vector table + safe-mode");

/* ------------------------------------------------------------------------ */
/* Future A/B layout invariants — DO NOT remove when N-26 lands; expand    */
/* them. These ensure the Phase 6 patch can be a clean #define swap.       */
/* ------------------------------------------------------------------------ */

_Static_assert(MM_FUTURE_SLOT_A_BASE == MM_APP_BASE,
               "Future Slot A must start at APP base (no further movement)");

_Static_assert(MM_FUTURE_SLOT_A_SIZE == MM_FUTURE_SLOT_B_SIZE,
               "Future A/B slots must be the same size for symmetric updates");

_Static_assert(MM_FUTURE_SLOT_A_SIZE + MM_FUTURE_SLOT_B_SIZE <= MM_APP_SIZE,
               "Future A/B slots must fit inside the launch APP region");

_Static_assert((MM_FUTURE_SLOT_A_BASE % MM_REGION_ALIGN) == 0,
               "Future Slot A base must be 4 KB aligned");
_Static_assert((MM_FUTURE_SLOT_B_BASE % MM_REGION_ALIGN) == 0,
               "Future Slot B base must be 4 KB aligned");

/* ------------------------------------------------------------------------ */
/* RAM stack invariants                                                     */
/* ------------------------------------------------------------------------ */

_Static_assert(MM_STACK_SIZE >= 1024UL,
               "Stack must be at least 1 KB (HardFault handler needs ~500 B)");

_Static_assert(MM_STACK_SIZE < MM_RAM_SIZE,
               "Stack cannot be larger than total RAM");

_Static_assert(MM_STACK_BASE > MM_RAM_BASE,
               "Stack must leave at least one byte for .data/.bss");

_Static_assert(MM_RAM_SIZE - MM_STACK_SIZE >= MM_RAM_HEADROOM_MIN,
               "Stack reservation leaves less than minimum data/bss headroom");

/* ------------------------------------------------------------------------ */
/* System memory address sanity                                             */
/* ------------------------------------------------------------------------ */

_Static_assert(MM_SYSMEM_BASE == 0x1FF00000UL,
               "L0-series ROM bootloader entry is at 0x1FF00000 per AN2606");

/* ------------------------------------------------------------------------ */
/* This file emits no symbols. Reference one constant so it is not stripped */
/* by --gc-sections in case the build links it into nothing.                */
/* ------------------------------------------------------------------------ */

const unsigned long _mm_static_asserts_anchor = MM_FLASH_BASE;
