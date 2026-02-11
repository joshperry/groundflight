/*
 * GroundFlight Linker Script
 * Target: STM32F722RET6 (RadioMaster Nexus)
 * 
 * Memory Map:
 *   FLASH:  512KB @ 0x08000000
 *   DTCM:    64KB @ 0x20000000 (Tightly-coupled, fastest for data)
 *   SRAM1:  176KB @ 0x20010000 (Main RAM)
 *   SRAM2:   16KB @ 0x2003C000 (Can be retained in standby)
 *
 * Note: Using DTCM for stack/fast data, SRAM1 for heap/general
 */

ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(DTCM) + LENGTH(DTCM);

/* Minimum heap and stack sizes */
_Min_Heap_Size  = 0x4000;  /* 16KB heap */
_Min_Stack_Size = 0x2000;  /* 8KB stack (nested ISRs + float math) */

MEMORY
{
    FLASH (rx)     : ORIGIN = 0x08000000, LENGTH = 512K
    DTCM  (rwx)    : ORIGIN = 0x20000000, LENGTH = 64K
    SRAM1 (rwx)    : ORIGIN = 0x20010000, LENGTH = 176K
    SRAM2 (rwx)    : ORIGIN = 0x2003C000, LENGTH = 16K
}

SECTIONS
{
    /* Interrupt Vector Table - must be at start of flash */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >FLASH

    /* Program code and read-only data */
    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.glue_7)         /* ARM/Thumb glue code */
        *(.glue_7t)
        *(.eh_frame)

        KEEP(*(.init))
        KEEP(*(.fini))

        . = ALIGN(4);
        _etext = .;
    } >FLASH

    /* Constant data (strings, tables, etc) */
    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } >FLASH

    /* ARM exception tables */
    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } >FLASH

    .ARM :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } >FLASH

    /* C++ constructors/destructors */
    .preinit_array :
    {
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array*))
        PROVIDE_HIDDEN(__preinit_array_end = .);
    } >FLASH

    .init_array :
    {
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array*))
        PROVIDE_HIDDEN(__init_array_end = .);
    } >FLASH

    .fini_array :
    {
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array*))
        PROVIDE_HIDDEN(__fini_array_end = .);
    } >FLASH

    /* Used by startup to initialize data */
    _sidata = LOADADDR(.data);

    /* Initialized data - loaded from flash, runs from DTCM for speed */
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } >DTCM AT> FLASH

    /* Uninitialized data (zeroed by startup) */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } >DTCM

    /* Provide 'end' symbol for _sbrk (heap allocation) */
    PROVIDE(end = _ebss);
    PROVIDE(_end = _ebss);

    /* User heap - in SRAM1 for larger allocations */
    .heap (NOLOAD) :
    {
        . = ALIGN(8);
        PROVIDE(__heap_start = .);
        . = . + _Min_Heap_Size;
        . = ALIGN(8);
        PROVIDE(__heap_end = .);
    } >SRAM1

    /* Stack - top of DTCM for fastest access */
    ._user_stack (NOLOAD) :
    {
        . = ALIGN(8);
        PROVIDE(__stack_start = .);
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
        PROVIDE(__stack_end = .);
    } >DTCM

    /* Discard standard library debug sections */
    /DISCARD/ :
    {
        libc.a(*)
        libm.a(*)
        libgcc.a(*)
    }

    /* ARM attributes */
    .ARM.attributes 0 : { *(.ARM.attributes) }
}
