OUTPUT_FORMAT("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(Reset_Handler)
MEMORY
{
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = (192 * 1024)
    CFG (r) : ORIGIN = (0x08000000 + 0x2E000), LENGTH = (8 * 1024)
    RAM (rwx) : ORIGIN = 0x20000000, LENGTH = (20 * 1024)
}
_estack = (0x20000000 + (20 * 1024));
_stack_size = (2560);
_stack_bottom = ((0x20000000 + (20 * 1024)) - (2560));
_app_start = 0x08000000; _app_end = 0x08000000 + (192 * 1024);
_cfg_start = (0x08000000 + 0x2E000); _cfg_end = (0x08000000 + 0x2E000) + (8 * 1024);
SECTIONS
{
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > FLASH
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
ASSERT(0x08000000 + (192 * 1024) == ((0x08000000 + 0x2E000) + (8 * 1024)),
       "Linker: FLASH + CFG must cover full 192 KB");
ASSERT((0x08000000 + 0x2E000) >= 0x08000000,
       "Linker: CFG region must fall within overall memory");
