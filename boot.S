/* boot.S — x86-64 Multiboot1/Multiboot2 Bootstrap Assembly (GNU as) */

.intel_syntax noprefix

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
    xor eax, eax
    mov edi, offset pml4_table
    mov ecx, 3072               /* 12288 / 4 = 3072 dwords */
    rep stosd

    /* 3. Set up identity paging using 2 MiB huge pages for first 1 GiB
     *    PML4[0] -> PDPT
     *    PDPT[0] -> PD
     *    PD[0..511] -> 0x00000000 .. 0x3FFFFFFF (2 MiB pages, Present + RW + PS)
     */
    mov eax, offset pdpt_table
    or  eax, 0x03               /* Present (bit 0) | Read/Write (bit 1) */
    mov dword ptr [pml4_table], eax

    mov eax, offset pd_table
    or  eax, 0x03               /* Present (bit 0) | Read/Write (bit 1) */
    mov dword ptr [pdpt_table], eax

    mov ecx, 0
.Lmap_pd_loop:
    mov eax, ecx
    shl eax, 21                 /* eax = ecx * 2 MiB (0x200000) */
    or  eax, 0x83               /* Present (0x01) | RW (0x02) | Page Size 2MiB (0x80) */
    mov dword ptr [pd_table + ecx * 8], eax
    mov dword ptr [pd_table + ecx * 8 + 4], 0
    inc ecx
    cmp ecx, 512
    jne .Lmap_pd_loop

    /* 4. Load CR3 with physical base address of PML4 */
    mov eax, offset pml4_table
    mov cr3, eax

    /* 5. Enable PAE (Physical Address Extension) in CR4 (bit 5) */
    mov eax, cr4
    or  eax, (1 << 5)
    mov cr4, eax

    /* 6. Enable Long Mode in IA32_EFER MSR (0xC0000080, bit 8 = LME) */
    mov ecx, 0xC0000080
    rdmsr
    or  eax, (1 << 8)
    wrmsr

    /* 7. Enable Paging (bit 31 = PG) and Protected Mode (bit 0 = PE) in CR0 */
    mov eax, cr0
    or  eax, ((1 << 31) | (1 << 0))
    mov cr0, eax

    /* 8. Load 64-bit Global Descriptor Table (GDT) */
    lgdt [gdt64_pointer]

    /* 9. Far jump to 64-bit Kernel Code Segment (selector 0x08) to enter 64-bit mode */
    jmp 0x08:long_mode_start


/* =========================================================================
 * 64-Bit Long Mode Entry Point (.code64)
 * ========================================================================= */
.code64
long_mode_start:
    /* 10. Reload all data segment registers with Kernel Data Segment selector (0x10) */
    mov ax, 0x10
    mov ds, ax
    mov es, ax
    mov fs, ax
    mov gs, ax
    mov ss, ax

    /* 11. Set up 64-bit stack pointer (16-byte aligned) */
    mov rsp, offset stack_top
    xor rbp, rbp

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
