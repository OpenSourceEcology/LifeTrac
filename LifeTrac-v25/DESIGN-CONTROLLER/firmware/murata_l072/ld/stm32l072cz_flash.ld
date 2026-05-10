/*
 * stm32l072cz_flash.ld — STM32L072CZ linker script for the launch (single-slot)
 * Method G firmware.
 *
 * IMPORTANT: this file is preprocessed (`arm-none-eabi-cpp -P -x c`) before
 * being passed to the linker. It #include's include/memory_map.h, which is
 * the SINGLE SOURCE OF TRUTH for every Flash/RAM region address and size.
 * Do NOT hand-mirror constants here. If a value looks wrong, fix it in
 * memory_map.h; the static asserts in include/static_asserts.c will catch
 * any inconsistency.
 *
 * Build glue: see Makefile rule for `firmware.ld` — runs the preprocessor
 * to expand this file into a plain linker script before invoking ld.
 */

#include "memory_map.h"

OUTPUT_FORMAT("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(Reset_Handler)

/* ------------------------------------------------------------------------ */
/* MEMORY regions — unified flash layout (no BOOT/APP/CFG split).           */
/*                                                                          */
/* Single FLASH region from 0x08000000 to 0x08030000 (192 KB) with vector   */
/* table, Reset_Handler, and all code continuous. This matches STM32L0 ROM  */
/* bootloader expectations and the Arduino/hardwario proven approach.       */
/*                                                                          */
/* CFG region is declared as read-only to reserve space at the end of       */
/* flash; nothing is placed there at link time — it is written/read by the  */
/* application at runtime.                                                  */
/* ------------------------------------------------------------------------ */

MEMORY
{
    FLASH (rx) : ORIGIN = MM_FLASH_BASE, LENGTH = MM_FLASH_SIZE
    CFG   (r)  : ORIGIN = MM_CFG_BASE,   LENGTH = MM_CFG_SIZE
    RAM  (rwx) : ORIGIN = MM_RAM_BASE,   LENGTH = MM_RAM_SIZE
}

/* Stack symbols consumed by the Reset_Handler / startup code. */
_estack       = MM_STACK_TOP;
_stack_size   = MM_STACK_SIZE;
_stack_bottom = MM_STACK_BASE;

/* Region symbols exposed for runtime sanity checks (e.g. a CRC over CFG).  */
_app_start  = MM_APP_BASE;   _app_end  = MM_APP_BASE  + MM_APP_SIZE;
_cfg_start  = MM_CFG_BASE;   _cfg_end  = MM_CFG_BASE  + MM_CFG_SIZE;

/* ------------------------------------------------------------------------ */
/* Sections                                                                 */
/* ------------------------------------------------------------------------ */

SECTIONS
{
    /* Vector table and code sit at the very base of Flash. Reset_Handler
     * entry immediately follows, with all application code continuous.
     * This standard layout is expected by the STM32L0 ROM bootloader. */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > FLASH

    /* Application code, read-only data, and exception unwind tables go
     * in the main text section following the vector table. */
    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text.*)
        *(.rodata)
        *(.rodata.*)
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame)
        . = ALIGN(4);
        _etext = .;
    } > FLASH

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH

    .ARM :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } > FLASH

    /* Initialised data — load address in FLASH, runtime address in RAM. */
    _sidata = LOADADDR(.data);
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data.*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT> FLASH

    .noinit (NOLOAD) :
    {
        . = ALIGN(4);
        *(.noinit)
        *(.noinit.*)
        . = ALIGN(4);
    } > RAM

    /* Zero-initialised data. */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } > RAM

    /* Static heap pool (no malloc by policy, but a few static pools may
     * land here). Free space between _end and _stack_bottom is what's
     * available; runtime code can verify against MM_RAM_HEADROOM_MIN. */
    ._user_heap :
    {
        . = ALIGN(8);
        _end = .;
        _heap_start = .;
        . = . + 0;                   /* no auto-grow; pools are explicit */
        . = ALIGN(8);
        _heap_end = .;
    } > RAM

    /* Verify the link-time stack reservation actually fits. */
    .stack_check (NOLOAD) :
    {
        . = ALIGN(8);
        . = . + 0;
        ASSERT(_heap_end + _stack_size <= _estack,
               "Linker error: not enough RAM left for the reserved stack");
    } > RAM

    /* CFG region: reserved, not written at link time. We declare an empty
     * NOLOAD section so the linker tracks the region but emits no bytes. */
    .cfg_reserved (NOLOAD) :
    {
        . = . + MM_CFG_SIZE;
    } > CFG

    /DISCARD/ :
    {
        *(.note.GNU-stack)
        *(.comment)
    }
}

/* Belt-and-braces ASSERTs — verify the unified layout is sound. */
ASSERT(MM_FLASH_BASE + MM_FLASH_SIZE == (MM_CFG_BASE + MM_CFG_SIZE),
       "Linker: FLASH + CFG must cover full 192 KB");
ASSERT(MM_CFG_BASE >= MM_FLASH_BASE,
       "Linker: CFG region must fall within overall memory");
