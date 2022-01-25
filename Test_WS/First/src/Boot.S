// -------------------------------------------------------------------------------------------
// (c) Bernhard Lang, Hochschule Osnabrueck
// -------------------------------------------------------------------------------------------
#include "riscv_csr_encoding.h"

/* --------------------------------------------------------------
 * Initial jump and vector table entries.
 ---------------------------------------------------------------- */

.section .vector_table,"ax",%progbits

// entry point at startup with jump to startup code
.global entrypoint
.type entrypoint,@function
entrypoint:
  j startup_code

  .org entrypoint+16
// vector table
.global vtable
.type vtable, %object
vtable:
  j exception_handler              // vtable+0x00000000
  j supervisor_software_irhandler  // vtable+0x00000004
  j default_handler                // vtable+0x00000008
  j machine_software_irhandler     // vtable+0x0000000c
  j user_timer_irhandler           // vtable+0x00000010
  j supervisor_timer_irhandler     // vtable+0x00000014
  j default_handler                // vtable+0x00000018
  j machine_timer_irhandler        // vtable+0x0000001c
  j user_external_irhandler        // vtable+0x00000020
  j supervisor_external_irhandler  // vtable+0x00000024
  j default_handler                // vtable+0x00000028
  j machine_external_irhandler     // vtable+0x0000002c
  j default_handler                // vtable+0x00000030
  j default_handler                // vtable+0x00000034
  j default_handler                // vtable+0x00000038
  j default_handler                // vtable+0x0000003c
  j LocalInt0_handler              // vtable+0x00000040
  j LocalInt1_handler              // vtable+0x00000044
  j LocalInt2_handler              // vtable+0x00000048
  j LocalInt3_handler              // vtable+0x0000004C
  j LocalInt4_handler              // vtable+0x00000050
  j LocalInt5_handler              // vtable+0x00000054
  j LocalInt6_handler              // vtable+0x00000058
  j LocalInt7_handler              // vtable+0x0000005C
  j LocalInt8_handler              // vtable+0x00000060
  j LocalInt9_handler              // vtable+0x00000064
  j LocalIntA_handler              // vtable+0x00000068
  j LocalIntB_handler              // vtable+0x0000006C
  j LocalIntC_handler              // vtable+0x00000070
  j LocalIntD_handler              // vtable+0x00000074
  j LocalIntE_handler              // vtable+0x00000078
  j LocalIntF_handler              // vtable+0x0000007C


.section .text,"ax",%progbits

/* --------------------------------------------------------------
 * Assembly 'reset handler' function to initialize core CPU registers.
 ---------------------------------------------------------------- */
startup_code:

    // Disable interrupts until they are needed.
    csrc mstatus, (MSTATUS_MIE | MSTATUS_SIE | MSTATUS_UIE)
    // Activate vector table for interrupts.
//  csrs mtvec, 1
    // Load the initial stack pointer value.
    la   sp, _sp
    // Call 'premain()'
    call premain
    // All other settings are done there
    // .data/.bss sections initialized there
    // At the end main is called

    // endless loop after unexpected premain return
the_end:
    j the_end

