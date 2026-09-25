/* boot.S — x86-64 Multiboot1/Multiboot2 Bootstrap Assembly (GNU as) */

.section .multiboot, "a"
.align 8

/* =========================================================================
 * Multiboot2 Header (Specification version 2.0)
 * Must be 8-byte aligned and within first 32 KiB of file.
 * ========================================================================= */
multiboot2_header_start:
    .long 0xE85250D6                                      /* Magic number (Multiboot2) */
    .long 0                                               /* Architecture: 0 (i386 32-bit protected mode) */
    .long multiboot2_header_end - multiboot2_header_start /* Header length */
    .long -(0xE85250D6 + 0 + (multiboot2_header_end - multiboot2_header_start)) /* Checksum */

    /* End tag (type = 0, flags = 0, size = 8) */
    .align 8
    .word 0
    .word 0
    .long 8
multiboot2_header_end:

/* =========================================================================
 * Multiboot1 Header (Specification version 1)
 * Must be 4-byte aligned and within first 8 KiB of file.
 * ========================================================================= */
.align 4
multiboot1_header_start:
    .set MB1_MAGIC,    0x1BADB002
    .set MB1_FLAGS,    0x00000003                         /* Align modules on 4KB + provide mem info */
    .set MB1_CHECKSUM, -(MB1_MAGIC + MB1_FLAGS)
    .long MB1_MAGIC
    .long MB1_FLAGS
    .long MB1_CHECKSUM
multiboot1_header_end:


/* =========================================================================
 * 32-Bit Bootstrap Code (.code32)
 * Executed immediately after GRUB hands off control in 32-bit flat mode.
 * ========================================================================= */
.section .text
.global _start
.type _start, @function

.code32
_start:
    /* 1. Disable interrupts and clear direction flag */
    cli
    cld

    /* 2. Zero the 3 page tables (PML4, PDPT, PD = 3 * 4096 = 12288 bytes) */
    xorl %eax, %eax
    movl $pml4_table, %edi
    movl $3072, %ecx            /* 12288 / 4 = 3072 dwords */
    rep stosl

    /* 3. Set up identity paging using 2 MiB huge pages for first 1 GiB
     *    PML4[0] -> PDPT
     *    PDPT[0] -> PD
     *    PD[0..511] -> 0x00000000 .. 0x3FFFFFFF (2 MiB pages, Present + RW + PS)
     */
    movl $pdpt_table, %eax
    orl  $0x03, %eax            /* Present (bit 0) | Read/Write (bit 1) */
    movl %eax, pml4_table

    movl $pd_table, %eax
    orl  $0x03, %eax            /* Present (bit 0) | Read/Write (bit 1) */
    movl %eax, pdpt_table

    movl $0, %ecx
.Lmap_pd_loop:
    movl %ecx, %eax
    shll $21, %eax              /* eax = ecx * 2 MiB (0x200000) */
    orl  $0x83, %eax            /* Present (0x01) | RW (0x02) | Page Size 2MiB (0x80) */
    movl %eax, pd_table(,%ecx,8)
    movl $0, pd_table+4(,%ecx,8)
    incl %ecx
    cmpl $512, %ecx
    jne  .Lmap_pd_loop

    /* 4. Load CR3 with physical base address of PML4 */
    movl $pml4_table, %eax
    movl %eax, %cr3

    /* 5. Enable PAE (Physical Address Extension) in CR4 (bit 5) */
    movl %cr4, %eax
    orl  $(1 << 5), %eax
    movl %eax, %cr4

    /* 6. Enable Long Mode in IA32_EFER MSR (0xC0000080, bit 8 = LME) */
    movl $0xC0000080, %ecx
    rdmsr
    orl  $(1 << 8), %eax
    wrmsr

    /* 7. Enable Paging (bit 31 = PG) and Protected Mode (bit 0 = PE) in CR0 */
    movl %cr0, %eax
    orl  $((1 << 31) | (1 << 0)), %eax
    movl %eax, %cr0

    /* 8. Load 64-bit Global Descriptor Table (GDT) */
    lgdt gdt64_pointer

    /* 9. Far jump to 64-bit Kernel Code Segment (selector 0x08) to enter 64-bit mode */
    ljmp $0x08, $long_mode_start


/* =========================================================================
 * 64-Bit Long Mode Entry Point (.code64)
 * ========================================================================= */
.code64
long_mode_start:
    /* 10. Reload all data segment registers with Kernel Data Segment selector (0x10) */
    movw $0x10, %ax
    movw %ax, %ds
    movw %ax, %es
    movw %ax, %fs
    movw %ax, %gs
    movw %ax, %ss

    /* 11. Set up 64-bit stack pointer (16-byte aligned) */
    movq $stack_top, %rsp
    xorq %rbp, %rbp

    /* 12. Call freestanding C++ kernel entry point */
    .extern kernel_main
    call kernel_main

    /* 13. Clean CPU halt loop if kernel_main returns */
.Lhalt_loop:
    cli
    hlt
    jmp .Lhalt_loop


/* =========================================================================
 * 64-Bit Global Descriptor Table (Read-Only Data)
 * ========================================================================= */
.section .rodata
.align 16
gdt64:
    .quad 0x0000000000000000    /* 0x00: Null Descriptor */
    .quad 0x00AF9A000000FFFF    /* 0x08: 64-bit Kernel Code (P=1, DPL=0, S=1, Exec=1, R=1, L=1, D=0, G=1) */
    .quad 0x00CF92000000FFFF    /* 0x10: 64-bit Kernel Data (P=1, DPL=0, S=1, Exec=0, W=1, L=0, D/B=1, G=1) */
gdt64_end:

.align 16
gdt64_pointer:
    .word gdt64_end - gdt64 - 1 /* 16-bit Limit */
    .quad gdt64                 /* 64-bit Base Address */


/* =========================================================================
 * BSS Section: Page Tables (4 KiB aligned) and Kernel Stack (16-byte aligned)
 * ========================================================================= */
.section .bss
.align 4096
pml4_table:
    .skip 4096
pdpt_table:
    .skip 4096
pd_table:
    .skip 4096

.align 16
stack_bottom:
    .skip 16384                 /* 16 KiB Kernel Stack */
stack_top:

.section .note.GNU-stack,"",@progbits
