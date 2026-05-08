OUTPUT_FORMAT("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(Reset_Handler)
MEMORY
{
    BOOT (rx) : ORIGIN = 0x08000000, LENGTH = (4 * 1024)
    APP (rx) : ORIGIN = (0x08000000 + (4 * 1024)), LENGTH = (180 * 1024)
    CFG (r) : ORIGIN = ((0x08000000 + (4 * 1024)) + (180 * 1024)), LENGTH = (8 * 1024)
    RAM (rwx) : ORIGIN = 0x20000000, LENGTH = (20 * 1024)
}
_estack = (0x20000000 + (20 * 1024));
_stack_size = (2560);
_stack_bottom = ((0x20000000 + (20 * 1024)) - (2560));
_boot_start = 0x08000000; _boot_end = 0x08000000 + (4 * 1024);
_app_start = (0x08000000 + (4 * 1024)); _app_end = (0x08000000 + (4 * 1024)) + (180 * 1024);
_cfg_start = ((0x08000000 + (4 * 1024)) + (180 * 1024)); _cfg_end = ((0x08000000 + (4 * 1024)) + (180 * 1024)) + (8 * 1024);
SECTIONS
{
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
    .boot_rodata :
    {
        . = ALIGN(4);
        KEEP(*(.boot_rodata))
        KEEP(*(.boot_rodata.*))
        . = ALIGN(4);
    } > BOOT
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
    ._user_heap :
    {
        . = ALIGN(8);
        _end = .;
        _heap_start = .;
        . = . + 0;
        . = ALIGN(8);
        _heap_end = .;
    } > RAM
    .stack_check (NOLOAD) :
    {
        . = ALIGN(8);
        . = . + 0;
        ASSERT(_heap_end + _stack_size <= _estack,
               "Linker error: not enough RAM left for the reserved stack");
    } > RAM
    .cfg_reserved (NOLOAD) :
    {
        . = . + (8 * 1024);
    } > CFG
    /DISCARD/ :
    {
        *(.note.GNU-stack)
        *(.comment)
    }
}
ASSERT((4 * 1024) + (180 * 1024) + (8 * 1024) == (192 * 1024),
       "Linker: Flash regions must sum to 192 KB");
ASSERT(0x08000000 == 0x08000000,
       "Linker: BOOT must start at Flash base");
ASSERT(((0x08000000 + (4 * 1024)) + (180 * 1024)) + (8 * 1024) == (0x08000000 + (192 * 1024)),
       "Linker: CFG must end exactly at Flash end");
