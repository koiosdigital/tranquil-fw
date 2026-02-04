/*
 * TEE Handlers Linker Script
 *
 * Links handlers to execute at 0x600FE300 (RTC FAST).
 * Used to create a standalone binary that's embedded in the bootloader.
 */

ENTRY(tee_dispatch)

MEMORY
{
    rtc_fast (RWX) : ORIGIN = 0x600FE300, LENGTH = 0x500
}

SECTIONS
{
    .text 0x600FE300 :
    {
        _tee_handlers_start = .;
        *(.text.tee_dispatch)    /* tee_dispatch MUST be first */
        *(.text.tee_handle_*)    /* Then other handlers */
        *(.text)
        *(.text.*)
        *(.literal)
        *(.literal.*)
        . = ALIGN(4);
        _tee_handlers_end = .;
    } > rtc_fast

    .rodata :
    {
        *(.rodata)
        *(.rodata.*)
        . = ALIGN(4);
    } > rtc_fast

    /DISCARD/ :
    {
        *(.comment)
        *(.note.*)
        *(.eh_frame*)
        *(.debug*)
    }
}
