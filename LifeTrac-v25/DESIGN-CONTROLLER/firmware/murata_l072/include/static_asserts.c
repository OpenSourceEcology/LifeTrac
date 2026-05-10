/*
 * static_asserts.c — Build-time validation of the memory map.
 *
 * This translation unit contains no runtime code. Its sole purpose is to
 * fail compilation if any constant in include/memory_map.h drifts into an
 * inconsistent state.
 */

#include "memory_map.h"

/* C11 _Static_assert is supported on arm-none-eabi-gcc with -std=gnu11. */

/* Basic chip capacity checks */
_Static_assert(MM_FLASH_SIZE == (192UL * 1024UL), "L072CZ has 192 KB Flash");
_Static_assert(MM_RAM_SIZE == (20UL * 1024UL), "L072CZ has 20 KB RAM");

/* Unified Flash layout checks */
_Static_assert(MM_APP_SIZE == MM_FLASH_SIZE, "APP covers full Flash");
_Static_assert(MM_APP_BASE == MM_FLASH_BASE, "APP starts at Flash base");
_Static_assert(MM_CFG_BASE + MM_CFG_SIZE == MM_FLASH_END, "CFG ends at Flash end");
_Static_assert((MM_FLASH_BASE % MM_REGION_ALIGN) == 0, "Flash base 4KB aligned");
_Static_assert((MM_CFG_BASE % MM_REGION_ALIGN) == 0, "CFG base 4KB aligned");
_Static_assert((MM_CFG_SIZE % MM_FLASH_PAGE_SIZE) == 0, "CFG is page-aligned");

/* RAM stack checks */
_Static_assert(MM_STACK_SIZE >= 1024UL, "Stack at least 1 KB");
_Static_assert(MM_STACK_SIZE < MM_RAM_SIZE, "Stack fits in RAM");
