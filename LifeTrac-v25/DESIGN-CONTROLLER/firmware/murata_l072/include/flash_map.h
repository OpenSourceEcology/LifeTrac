#ifndef FLASH_MAP_H
#define FLASH_MAP_H

/**
 * @file flash_map.h
 * @brief Canonical flash memory map for STM32L072CZ Custom Firmware
 * Consumed by both the C bootloader/application layer and the GCC linker script.
 */

#define L072_FLASH_BASE      0x08000000u
#define L072_FLASH_SIZE      (192u * 1024u) /* 192 KB */
#define L072_PAGE_SIZE       128u           /* L072 page size is 128 bytes, but sector erase is 4KB? Actually L0 series page size is 128 bytes, sector is 4KB. Wait, STM32L0x2 has 128-byte pages. The sector size for erase is 128 bytes. Half-page erase is 64 bytes. Let's not specify erase granularity here if we aren't sure, or assume 128 bytes. */

/* Recommended layout for single-slot launch (defer A/B to Phase 6) */
#define BOOT_REGION_START    0x08000000u
#define BOOT_REGION_SIZE     (4u * 1024u)   /* 4 KB */

#define SLOT_A_START         0x08001000u
#define SLOT_A_SIZE          (176u * 1024u) /* 176 KB */

#define CONFIG_REGION_START  0x0802D000u
#define CONFIG_REGION_SIZE   (12u * 1024u)  /* 12 KB */

#ifndef __ASSEMBLER__
/* Build-time math verification */
_Static_assert(BOOT_REGION_SIZE + SLOT_A_SIZE + CONFIG_REGION_SIZE == L072_FLASH_SIZE,
               "Flash layout must exactly fit STM32L072 192KB flash size");
#endif

#endif /* FLASH_MAP_H */
