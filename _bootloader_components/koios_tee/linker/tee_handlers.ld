/*
 * TEE Handlers Linker Script
 *
 * Links handlers to execute at 0x600FE300 (RTC FAST).
 * Used to create a standalone binary that's embedded in the bootloader.
 *
 * WCL monitors 0x600FE300 for W1->W0 transitions, so tee_entry MUST be first.
 */

ENTRY(tee_entry)

MEMORY
{
    rtc_fast (RWX) : ORIGIN = 0x600FE300, LENGTH = 0x500
}

SECTIONS
{
    /*
     * tee_entry (assembly) is placed first at 0x600FE300.
     * It calls tee_handler_main (C) which handles dispatch + WCL setup.
     */
    .text 0x600FE300 :
    {
        _tee_handlers_start = .;

        /* Literals for entry (must be before text that uses them) */
        *(.literal.tee_entry)

        /* Assembly entry point */
        *(.text.tee_entry)

        /* C handler main (called from entry) */
        *(.literal.tee_handler_main)
        *(.text.tee_handler_main)

        /* Service handlers */
        *(.text.tee_handle_*)
        *(.literal.tee_handle_*)

        /* Any remaining text */
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
