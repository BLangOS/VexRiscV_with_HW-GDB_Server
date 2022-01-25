// -------------------------------------------------------------------------------------------
// (c) Bernhard Lang, Hochschule Osnabrueck
// -------------------------------------------------------------------------------------------
#include <stdint.h>
#include <string.h>

#include "riscv_csr_encoding.h"
#include "interrupts.h"


#define Read_Word(Address) (*(volatile uint32_t*)(Address))
#define Write_Word(Address,Value) {*((volatile uint32_t*)(Address))=Value;}

#define GPIO0_BASE 0x8000
#define GPIO1_BASE 0x8100
#define GPIO_Dir         0x00
#define GPIO_Dir_Set     0x04
#define GPIO_Dir_Delete  0x08
#define GPIO_Dir_Toggle  0x0c
#define GPIO_Data        0x10
#define GPIO_Data_Set    0x14
#define GPIO_Data_Delete 0x18
#define GPIO_Data_Toggle 0x1c
#define GPIO_PIN         0x20

#define TIMER0_BASE 0x8200
#define TIMER1_BASE 0x8300
#define TIMER_VALUE  0x00
#define TIMER_START  0x04
#define TIMER_STATUS 0x08

volatile uint32_t timer_irs=0;
volatile uint32_t timer_status;
volatile uint32_t timer_value;

void __attribute__ ( (interrupt) ) machine_timer_irhandler (void) {
  timer_status = Read_Word(TIMER0_BASE+TIMER_STATUS); // clear interrupt by reading timer status
  timer_irs++;
}

volatile uint32_t segments[4];
volatile uint32_t segment_counter;

void __attribute__ ( (interrupt) ) LocalInt0_handler (void) {
  timer_status = Read_Word(TIMER1_BASE+TIMER_STATUS); // clear interrupt by reading timer status
  if (++segment_counter>3) { segment_counter=0; }
  Write_Word(GPIO1_BASE+GPIO_Data,segments[segment_counter]);
}

void set_digit(uint32_t value, uint32_t dp, uint32_t segment) {
  static uint32_t decode_tab[16] = {0x40,0x79,0x24,0x30,0x19,0x12,0x02,0x78,0x00,0x10,0x08,0x03,0x27,0x21,0x06,0x0e};
  uint32_t sseg;
  if (value<16) {	sseg = decode_tab[value]; }
  else          { sseg = 0x7f;              }
  if (segment<4) { segments[segment] = ((~(1<<segment)&0xf)<<8) | (((dp^1)&1)<<7) | sseg; }
}

void set_display(uint32_t value, uint32_t dps) {
  for (int i=0; i<4; i++) {
	set_digit(value&0xf,dps&1,i);
	value = value>>4;
	dps   = dps>>1;
  }
}

void write_LEDs(uint32_t value) {
  Write_Word(GPIO0_BASE+GPIO_Data,value);
}

uint32_t read_SWs() {
  return (Read_Word(GPIO0_BASE+GPIO_PIN)>>16) & 0xffff;
}


int main( void ) {
  volatile uint32_t counter =0; // Variable for endlessly increment

  // enable all machine interrupts in mstatus-register
  set_csr(mstatus,MSTATUS_MIE); // MSTATUS_SIE  MSTATUS_UIE

  // Initialize GPIO0: lower 16 Bits are the LED outputs, upper 16 Bits are the switch inputs
  Write_Word(GPIO0_BASE+GPIO_Dir,0xffff);

  // Initialize GPIO1: lower 7 Bits are the Segment outputs, bit 7 is the dp output and Bits 8 to 11 are the anode outputs
  Write_Word(GPIO1_BASE+GPIO_Dir,0xfff);

  // Initialize Timer0 and enable timer interrupt
  Write_Word(TIMER0_BASE+TIMER_START,50000000-1); // At 50MHz generate interrupt each second
  set_csr(mie,(1 << IRQ_M_TIMER));

  // Initialize Timer1 and enable timer interrupt
  Write_Word(TIMER1_BASE+TIMER_START,50000-1); // At 50MHz generate interrupt each millisecond
  set_csr(mie,(1 << IRQ_LOCAL_0));
  //set_csr(mie,(1 << 16));

  while ( 1 ) {
    timer_value  = Read_Word(TIMER0_BASE+TIMER_VALUE); // only for debugging, value not used
    timer_value  = Read_Word(TIMER1_BASE+TIMER_VALUE); // only for debugging, value not used
    counter++;                                         // only for debugging, value not used

    set_display(timer_irs,(1<<0)); // set to SevenSeg display
    write_LEDs(read_SWs());        // read switches and write them to the LEDs
  }
  return 0;
}
