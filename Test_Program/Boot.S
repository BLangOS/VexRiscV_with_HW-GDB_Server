#include "riscv_csr_encoding.h"

/* --------------------------------------------------------------
 * Initial jump and vector table entries.
 ---------------------------------------------------------------- */

.section .vector_table,"ax",%progbits

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
    j user_external_irhandler        // vtable+0x00000010
    j supervisor_external_irhandler  // vtable+0x00000014 
    j default_handler                // vtable+0x00000018
    j machine_external_irhandler     // vtable+0x0000001c
    j default_handler                // vtable+0x00000020
    j default_handler                // vtable+0x00000024
    j default_handler                // vtable+0x00000028
    j default_handler                // vtable+0x0000002c
    j LocalInt0_handler              // vtable+0x00000030
    j LocalInt1_handler              // vtable+0x00000034
    j LocalInt2_handler              // vtable+0x00000038
    j LocalInt3_handler              // vtable+0x0000003C
    j LocalInt4_handler              // vtable+0x00000040
    j LocalInt5_handler              // vtable+0x00000044
    j LocalInt6_handler              // vtable+0x00000048
    j LocalInt7_handler              // vtable+0x0000004C
    j LocalInt8_handler              // vtable+0x00000050
    j LocalInt9_handler              // vtable+0x00000054
    j LocalIntA_handler              // vtable+0x00000058
    j LocalIntB_handler              // vtable+0x0000005C
    j LocalIntC_handler              // vtable+0x00000060
    j LocalIntD_handler              // vtable+0x00000064
    j LocalIntE_handler              // vtable+0x00000068
    j LocalIntF_handler              // vtable+0x0000006C

/* --------------------------------------------------------------
 * Weak aliases to point each exception handler to the
 * 'default_interrupt_handler', unless the application defines
 * a function with the same name to override the reference.
 ---------------------------------------------------------------- */
    .global exception_handler
    .weak   exception_handler
    .set    exception_handler,default_handler
  
    .global supervisor_software_irhandler
    .weak   supervisor_software_irhandler
    .set    supervisor_software_irhandler,default_handler
    .global machine_software_irhandler
    .weak   machine_software_irhandler
    .set    machine_software_irhandler,default_handler
  
    .global user_timer_irhandler
    .weak   user_timer_irhandler
    .set    user_timer_irhandler,default_handler
    .global supervisor_timer_irhandler
    .weak   supervisor_timer_irhandler
    .set    supervisor_timer_irhandler,default_handler
    .global machine_timer_irhandler
    .weak   machine_timer_irhandler
    .set    machine_timer_irhandler,default_handler
   
    .global user_external_irhandler
    .weak   user_external_irhandler
    .set    user_external_irhandler,default_handler
    .global supervisor_external_irhandler
    .weak   supervisor_external_irhandler
    .set    supervisor_external_irhandler,default_handler
    .global machine_external_irhandler
    .weak   machine_external_irhandler
    .set    machine_external_irhandler,default_handler
    
    .global LocalInt0_handler
    .weak   LocalInt0_handler
    .set    LocalInt0_handler,default_handler
    .global LocalInt1_handler
    .weak   LocalInt1_handler
    .set    LocalInt1_handler,default_handler
    .global LocalInt2_handler
    .weak   LocalInt2_handler
    .set    LocalInt2_handler,default_handler
    .global LocalInt3_handler
    .weak   LocalInt3_handler
    .set    LocalInt3_handler,default_handler
    .global LocalInt4_handler
    .weak   LocalInt4_handler
    .set    LocalInt4_handler,default_handler
    .global LocalInt5_handler
    .weak   LocalInt5_handler
    .set    LocalInt5_handler,default_handler
    .global LocalInt6_handler
    .weak   LocalInt6_handler
    .set    LocalInt6_handler,default_handler
    .global LocalInt7_handler
    .weak   LocalInt7_handler
    .set    LocalInt7_handler,default_handler
    .global LocalInt8_handler
    .weak   LocalInt8_handler
    .set    LocalInt8_handler,default_handler
    .global LocalInt9_handler
    .weak   LocalInt9_handler
    .set    LocalInt9_handler,default_handler
    .global LocalIntA_handler
    .weak   LocalIntA_handler
    .set    LocalIntA_handler,default_handler
    .global LocalIntB_handler
    .weak   LocalIntB_handler
    .set    LocalIntB_handler,default_handler
    .global LocalIntC_handler
    .weak   LocalIntC_handler
    .set    LocalIntC_handler,default_handler
    .global LocalIntD_handler
    .weak   LocalIntD_handler
    .set    LocalIntD_handler,default_handler
    .global LocalIntE_handler
    .weak   LocalIntE_handler
    .set    LocalIntE_handler,default_handler
    .global LocalIntF_handler
    .weak   LocalIntF_handler
    .set    LocalIntF_handler,default_handler

.section .text,"ax",%progbits

/* --------------------------------------------------------------
 * Assembly 'reset handler' function to initialize core CPU registers.
 ---------------------------------------------------------------- */
startup_code:

    // Disable interrupts until they are needed.
    csrc mstatus, (MSTATUS_MIE | MSTATUS_SIE | MSTATUS_UIE)

    // Load the initial stack pointer value.
    la   sp, _sp
  
// ----------------------------------------------  
test_begin:
    addi x3, x0, 0    //  0
    addi x3, x3, 1    //  1
.option rvc
    c.addi x3, 2      //  3
.option norvc
    addi x3, x3, 3    //  6
    addi x3, x3, 4    // 10
    addi x3, x3, 5    // 15
.option rvc
    addi x3, x3, 6    // 21
    addi x3, x3, 7    // 28
.option norvc
    addi x3, x3, 8    // 36
    addi x3, x3, 9    // 45
    addi x3, x3, 10   // 55
    j test_begin
// ----------------------------------------------  
  
// default handler
default_handler:
  j default_handler
  