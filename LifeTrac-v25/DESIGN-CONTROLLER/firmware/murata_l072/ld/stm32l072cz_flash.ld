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
/* MEMORY regions — driven entirely by memory_map.h.                        */
/*                                                                          */
/* BOOT and APP are presented to the linker as separate regions so the      */
/* boot code (vector table + safe-mode + slot-select stub) physically lives */
/* in the resident BOOT region and the rest of the app in APP. CFG is       */
/* declared so the linker reserves it (and refuses to spill into it) but    */
/* nothing is placed there at link time — it is written/read by the         */
/* application at runtime.                                                  */
/* ------------------------------------------------------------------------ */

MEMORY
{
    BOOT (rx)  : ORIGIN = MM_BOOT_BASE, LENGTH = MM_BOOT_SIZE
    APP  (rx)  : ORIGIN = MM_APP_BASE,  LENGTH = MM_APP_SIZE
    CFG  (r)   : ORIGIN = MM_CFG_BASE,  LENGTH = MM_CFG_SIZE
    RAM  (rwx) : ORIGIN = MM_RAM_BASE,  LENGTH = MM_RAM_SIZE
}

/* Stack symbols consumed by the Reset_Handler / startup code. */
_estack       = MM_STACK_TOP;
_stack_size   = MM_STACK_SIZE;
_stack_bottom = MM_STACK_BASE;

/* Region symbols exposed for runtime sanity checks (e.g. a CRC over CFG).  */
_boot_start = MM_BOOT_BASE;  _boot_end = MM_BOOT_BASE + MM_BOOT_SIZE;
_app_start  = MM_APP_BASE;   _app_end  = MM_APP_BASE  + MM_APP_SIZE;
_cfg_start  = MM_CFG_BASE;   _cfg_end  = MM_CFG_BASE  + MM_CFG_SIZE;

/* ------------------------------------------------------------------------ */
/* Sections                                                                 */
/* ------------------------------------------------------------------------ */

SECTIONS
{
    /* Vector table sits at the very base of Flash inside BOOT. The boot
     * code (safe-mode listener, slot-select stub once Phase 6 lands) lives
     * here too because it must run before any APP-region code. */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > BOOT

    .boot_text :
    {
        . = ALIGN(4);
        KEEP(*(.boot_text))
        KEEP(*(.boot_text.*))
        . = ALIGN(4);
    } > BOOT

    /* Application code, read-only data, and exception unwind tables go in
     * the APP region. Phase 6 splits this into Slot A + Slot B; until then
     * the whole APP region is one slot. */
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
    } > APP

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > APP

    .ARM :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } > APP

    /* Initialised data — load address in APP, runtime address in RAM. */
    _sidata = LOADADDR(.data);
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data.*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT> APP

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

/* Belt-and-braces ASSERTs — duplicate of the C static_asserts so that even
 * a misconfigured build (e.g. someone running ld without preprocessing
 * static_asserts.c) cannot produce a binary with a broken layout. */
ASSERT(MM_BOOT_SIZE + MM_APP_SIZE + MM_CFG_SIZE == MM_FLASH_SIZE,
       "Linker: Flash regions must sum to 192 KB");
ASSERT(MM_BOOT_BASE == MM_FLASH_BASE,
       "Linker: BOOT must start at Flash base");
ASSERT(MM_CFG_BASE + MM_CFG_SIZE == MM_FLASH_END,
       "Linker: CFG must end exactly at Flash end");
