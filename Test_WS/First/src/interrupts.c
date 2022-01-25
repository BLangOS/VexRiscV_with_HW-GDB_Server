// -------------------------------------------------------------------------------------------
// (c) Bernhard Lang, Hochschule Osnabrueck
// -------------------------------------------------------------------------------------------

#include <stdint.h>

#include "interrupts.h"
#include "riscv_csr_encoding.h"

asm("\t.weak exception_handler\n            \t.set exception_handler             ,default_handler\n");
asm("\t.weak supervisor_software_irhandler\n\t.set supervisor_software_irhandler ,default_handler\n");
asm("\t.weak machine_software_irhandler\n   \t.set machine_software_irhandler    ,default_handler\n");
asm("\t.weak user_timer_irhandler\n         \t.set user_timer_irhandler          ,default_handler\n");
asm("\t.weak supervisor_timer_irhandler\n   \t.set supervisor_timer_irhandler    ,default_handler\n");
asm("\t.weak machine_timer_irhandler\n      \t.set machine_timer_irhandler       ,default_handler\n");
asm("\t.weak user_external_irhandler\n      \t.set user_external_irhandler       ,default_handler\n");
asm("\t.weak supervisor_external_irhandler\n\t.set supervisor_external_irhandler ,default_handler\n");
asm("\t.weak machine_external_irhandler\n   \t.set machine_external_irhandler    ,default_handler\n");
asm("\t.weak LocalInt0_handler\n            \t.set LocalInt0_handler             ,default_handler\n");
asm("\t.weak LocalInt1_handler\n            \t.set LocalInt1_handler             ,default_handler\n");
asm("\t.weak LocalInt2_handler\n            \t.set LocalInt2_handler             ,default_handler\n");
asm("\t.weak LocalInt3_handler\n            \t.set LocalInt3_handler             ,default_handler\n");
asm("\t.weak LocalInt4_handler\n            \t.set LocalInt4_handler             ,default_handler\n");
asm("\t.weak LocalInt5_handler\n            \t.set LocalInt5_handler             ,default_handler\n");
asm("\t.weak LocalInt6_handler\n            \t.set LocalInt6_handler             ,default_handler\n");
asm("\t.weak LocalInt7_handler\n            \t.set LocalInt7_handler             ,default_handler\n");
asm("\t.weak LocalInt8_handler\n            \t.set LocalInt8_handler             ,default_handler\n");
asm("\t.weak LocalInt9_handler\n            \t.set LocalInt9_handler             ,default_handler\n");
asm("\t.weak LocalIntA_handler\n            \t.set LocalIntA_handler             ,default_handler\n");
asm("\t.weak LocalIntB_handler\n            \t.set LocalIntB_handler             ,default_handler\n");
asm("\t.weak LocalIntC_handler\n            \t.set LocalIntC_handler             ,default_handler\n");
asm("\t.weak LocalIntD_handler\n            \t.set LocalIntD_handler             ,default_handler\n");
asm("\t.weak LocalIntE_handler\n            \t.set LocalIntE_handler             ,default_handler\n");
asm("\t.weak LocalIntF_handler\n            \t.set LocalIntF_handler             ,default_handler\n");

void __attribute__ ( (interrupt) ) default_handler (void) {
  uint32_t the_cause;
  the_cause = read_csr(mcause);
  while (1) {}
}

void __attribute__ ( (interrupt) ) exception_handler (void) {
  // handler code
  uint32_t the_cause;
  the_cause = read_csr(mcause);
  switch(the_cause) {
    case 0x0:        break;  // Instruction address misaligned
    case 0x1:        break;  // Instruction access fault
    case 0x2:        break;  // Illegal instruction
    case 0x3:        break;  // Breakpoint
    case 0x4:        break;  // Load address misaligned
    case 0x5:        break;  // Load access fault
    case 0x6:        break;  // Store/AMO address misaligned
    case 0x7:        break;  // Store/AMO access fault
    case 0x8:        break;  // Environment call from U-mode
    case 0x9:        break;  // Environment call from S-mode
    case 0xb:        break;  // Environment call from M-mode
    case 0xc:        break;  // Instruction page fault
    case 0xd:        break;  // Load page fault
    case 0xe:        break;  // Reserved for future standard use
    case 0xf:        break;  // Store/AMO page fault
    default:         break;
  }
}
