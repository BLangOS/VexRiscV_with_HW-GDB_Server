// -------------------------------------------------------------------------------------------
// (c) Bernhard Lang, Hochschule Osnabrueck
// -------------------------------------------------------------------------------------------
#include <stdint.h>
#include <string.h>

#include "riscv_csr_encoding.h"

int main();

// Pre-defined memory locations for program initialization.
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;

void premain() {
  uint8_t *src, *dst;
  size_t len;
  
  // Copy initialized data from .sidata (Flash) to .data (RAM)
  src = (uint8_t *)&_sidata;
  dst = (uint8_t *)&_sdata;
  while (dst < (uint8_t *)&_edata)
      *dst++ = *src++;

  // Clear the .bss RAM section.
  dst = (uint8_t *)&_sbss;
  while (dst < (uint8_t *)&_ebss)
      *dst++ = 0;

  main();
  
  while(1) {}
}
