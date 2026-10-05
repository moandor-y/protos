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
    /* Disable interrupts and clear direction flag */
    cli
    cld

    /* Preserve Multiboot magic (eax) in ebp; ebx holds Multiboot info physical address */
    mov ebp, eax

    /* Zero the entire .bss section (page tables, IDT, stack, and C++ globals) */
    .extern g_bss_start
    .extern g_bss_end
    xor eax, eax
    mov edi, offset g_bss_start
    mov ecx, offset g_bss_end
    sub ecx, edi
    shr ecx, 2
    rep stosd

    /* Mask all legacy 8259 PIC interrupts (master 0x21, slave 0xA1) */
    mov al, 0xFF
    out 0x21, al
    out 0xA1, al

    /* Set up bootstrap identity paging using 2 MiB huge pages for first 64 MiB
     * PML4[0] -> PDPT
     * PDPT[0] -> PD
     * PD[0..31] -> 0x00000000 .. 0x03FFFFFF (2 MiB pages, Present + RW + PS)
     * Remaining physical RAM is identity-mapped dynamically once PMM initializes.
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
    cmp ecx, 32
    jne .Lmap_pd_loop

    /* Load CR3 with physical base address of PML4 */
    mov eax, offset pml4_table
    mov cr3, eax

    /* Enable PAE (bit 5), OSFXSR (bit 9), and OSXMMEXCPT (bit 10) in CR4 */
    mov eax, cr4
    or  eax, ((1 << 5) | (1 << 9) | (1 << 10))
    mov cr4, eax

    /* Enable Long Mode in IA32_EFER MSR (0xC0000080, bit 8 = LME) */
    mov ecx, 0xC0000080
    rdmsr
    or  eax, (1 << 8)
    wrmsr

    /* Enable Paging (bit 31 = PG), Monitor Coprocessor (bit 1 = MP),
     * and Protected Mode (bit 0 = PE) while clearing Emulation (bit 2 = EM) in CR0 */
    mov eax, cr0
    and eax, ~(1 << 2)
    or  eax, ((1 << 31) | (1 << 1) | (1 << 0))
    mov cr0, eax

    /* Load 64-bit Global Descriptor Table (GDT) */
    lgdt [gdt64_pointer]

    /* Far jump to 64-bit Kernel Code Segment (selector 0x08) to enter 64-bit mode */
    jmp 0x08:long_mode_start


/* =========================================================================
 * 64-Bit Long Mode Entry Point (.code64)
 * ========================================================================= */
.code64
long_mode_start:
    /* Reload all data segment registers with Kernel Data Segment selector (0x10) */
    mov ax, 0x10
    mov ds, ax
    mov es, ax
    mov fs, ax
    mov gs, ax
    mov ss, ax

    /* Set up 64-bit stack pointer (16-byte aligned) */
    mov rsp, offset stack_top

    /* Populate 32-entry 64-bit IDT (vectors 0..31) so bare-metal NMIs do not
     * cause a double/triple fault reset. */
    lea rdi, [idt64]
    mov ecx, 32
    lea rdx, [exception_halt_stub]
.Lidt_init_loop:
    mov eax, edx
    and eax, 0xFFFF
    or  eax, (0x08 << 16)       /* Selector 0x08 (64-bit Kernel Code) */
    mov dword ptr [rdi], eax
    mov eax, edx
    and eax, 0xFFFF0000
    or  eax, 0x8E00             /* Present=1, DPL=0, 64-bit Interrupt Gate */
    mov dword ptr [rdi + 4], eax
    mov qword ptr [rdi + 8], 0
    add rdi, 16
    dec ecx
    jnz .Lidt_init_loop

    /* Override Vector 2 (NMI) to return cleanly via iretq */
    lea rdx, [nmi_isr_stub]
    lea rdi, [idt64 + 2 * 16]
    mov eax, edx
    and eax, 0xFFFF
    or  eax, (0x08 << 16)
    mov dword ptr [rdi], eax
    mov eax, edx
    and eax, 0xFFFF0000
    or  eax, 0x8E00
    mov dword ptr [rdi + 4], eax

    lidt [idt64_pointer]

    /* Pass (uint32_t magic, uint64_t mb_info_phys) in (edi, rsi) per System V AMD64 ABI */
    mov edi, ebp
    mov esi, ebx
    xor rbp, rbp

    /* Call freestanding C++ kernel entry point */
    .extern kernel_main
    call kernel_main

    /* Clean CPU halt loop if kernel_main returns */
.Lhalt_loop:
    cli
    hlt
    jmp .Lhalt_loop

.align 16
nmi_isr_stub:
    iretq

.align 16
exception_halt_stub:
    cli
    hlt
    jmp exception_halt_stub


/* =========================================================================
 * 64-Bit Global Descriptor Table & IDT Pointer (Read-Only Data)
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

.align 16
idt64_pointer:
    .word 32 * 16 - 1           /* 16-bit Limit (32 gate descriptors) */
    .quad idt64                 /* 64-bit Base Address */


/* =========================================================================
 * BSS Section: Page Tables (4 KiB aligned), IDT, and Kernel Stack
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
idt64:
    .skip 32 * 16

.align 16
stack_bottom:
    .skip 16384                 /* 16 KiB Kernel Stack */
stack_top:

.section .note.GNU-stack,"",@progbits
