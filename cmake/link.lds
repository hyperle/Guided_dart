/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020/12/12     bernard      The first version
 */

OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    /* 64-bit userspace base */
    . = 0x200000000;

    /* ================= TEXT ================= */
    .text :
    {
        __text_start__ = .;

        *(.start)
        *(.text .text.*)

        KEEP(*(.init))
        KEEP(*(.fini))

        /* Constructors */
        *crtbegin.o(.ctors)
        *crtbegin?.o(.ctors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
        *(SORT(.ctors.*))
        *(.ctors)

        /* Destructors */
        *crtbegin.o(.dtors)
        *crtbegin?.o(.dtors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
        *(SORT(.dtors.*))
        *(.dtors)

        /* Read-only data */
        *(.rodata .rodata.*)

        /* ===== C++ exception support (CRITICAL) ===== */
        KEEP(*(.eh_frame*))
        KEEP(*(.eh_frame_hdr))
        *(.gcc_except_table .gcc_except_table.*)

        . = ALIGN(16);
        __text_end__ = .;
        _etext = .;
    }

    /* Page align */
    . = ALIGN(0x1000);

    /* ================= PLT / GOT ================= */
    .plt : { *(.plt*) }

    .got :
    {
        __got_start__ = .;
        *(.got .got.*)
        __got_end__ = .;
    }

    /* ================= DATA ================= */
    .data :
    {
        __data_start__ = .;

        *(vtable)
        *(.data .data.*)
        *(.sdata .sdata.*)

        . = ALIGN(16);

        /* preinit */
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP(*(.preinit_array))
        PROVIDE_HIDDEN (__preinit_array_end = .);

        /* init */
        . = ALIGN(16);
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN (__init_array_end = .);

        /* fini */
        . = ALIGN(16);
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array))
        PROVIDE_HIDDEN (__fini_array_end = .);

        . = ALIGN(16);
        __data_end__ = .;
    }

    /* ================= TLS (for OpenMP / musl) ================= */
    . = ALIGN(64);

    .tdata :
    {
        __tdata_start__ = .;
        *(.tdata .tdata.*)
        __tdata_end__ = .;
    }

    .tbss (NOLOAD) :
    {
        __tbss_start__ = .;
        *(.tbss .tbss.* .tcommon)
        __tbss_end__ = .;
    }

    . = ALIGN(64);

    /* Keep the TLS image contiguous so the symbol-based size matches PT_TLS. */
    PROVIDE(__tls_start__ = __tdata_start__);
    PROVIDE(__tls_end__ = __tbss_end__);
    PROVIDE(__tls_size__ = __tls_end__ - __tls_start__);

    /* ================= BSS ================= */
    .bss (NOLOAD) :
    {
        __bss_start__ = .;
        *(.sbss .sbss.*)
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(16);
        __bss_end__ = .;
    }

    /* Optional stack section (if used) */
    .stack (NOLOAD) :
    {
        *(.stack)
    }

    /* ================= END ================= */
    _end = .;
}
