;// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
;//
;//     BootLdEntry.asm
;//
;//   This provides an entry point from the Bootloader back into the main program
;//		via a vector at a fixed location.
;//
;// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    .ref _c_int00
	.global  _vector_from_bootloader
	.global _BL_key
;***********************************************************************
;* Function: bootentry section
;*
;* Description: Branch to code starting point
;***********************************************************************
    .sect "bootentry"

_vector_from_bootloader:
        LB _c_int00         ;//Branch to start of main program boot.asm in RTS library

_BL_key  .word 0x2001		;//When bootloader sees these values in Flash, rather than 0xFFFF's
         .word 0x1776       ;//It knows that the Main program is installed, and that
                            ;//it can jump to _vector_from_bootloader:

;end bootentry section

