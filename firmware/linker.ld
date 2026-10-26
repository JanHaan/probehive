/* The whole image in main RAM, where the CPU starts. */
OUTPUT_ARCH(riscv)
ENTRY(_start)

MEMORY { ram : ORIGIN = 0x40000000, LENGTH = 0x8000 }

SECTIONS
{
    .text : { *(.text.start) *(.text .text.*) } > ram
    .rodata : { *(.rodata .rodata.* .srodata .srodata.*) } > ram
    .data : { *(.data .data.* .sdata .sdata.*) } > ram
    .bss (NOLOAD) : {
        _bss_start = .;
        *(.bss .bss.* .sbss .sbss.* COMMON)
        . = ALIGN(4);
        _bss_end = .;
    } > ram
    _stack_top = ORIGIN(ram) + LENGTH(ram);
}
