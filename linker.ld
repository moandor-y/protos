OUTPUT_FORMAT(elf64-x86-64)
OUTPUT_ARCH(i386:x86-64)
ENTRY(_start)

SECTIONS
{
    /* Load kernel at physical address 1 MiB (0x100000) */
    . = 1M;
    g_kernel_start = .;

    /* Ensure Multiboot headers appear at the very start of the binary (within first 8 KiB) */
    .text ALIGN(4K) : AT(ADDR(.text))
    {
        KEEP(*(.multiboot))
        *(.text .text.*)
    }

    .rodata ALIGN(4K) : AT(ADDR(.rodata))
    {
        *(.rodata .rodata.*)
    }

    .data ALIGN(4K) : AT(ADDR(.data))
    {
        *(.data .data.*)
    }

    .bss ALIGN(4K) : AT(ADDR(.bss))
    {
        g_bss_start = .;
        *(COMMON)
        *(.bss .bss.*)
        . = ALIGN(4K);
        g_bss_end = .;
    }

    . = ALIGN(4K);
    g_kernel_end = .;

    /DISCARD/ :
    {
        *(.comment)
        *(.note*)
        *(.eh_frame*)
    }
}
