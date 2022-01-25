#ifndef interrupts_h
#define interrupts_h

void __attribute__ ( (interrupt) ) default_handler               (void);
void __attribute__ ( (interrupt) ) exception_handler             (void);
void __attribute__ ( (interrupt) ) supervisor_software_irhandler (void);
void __attribute__ ( (interrupt) ) machine_software_irhandler    (void);
void __attribute__ ( (interrupt) ) user_timer_irhandler          (void);
void __attribute__ ( (interrupt) ) supervisor_timer_irhandler    (void);
void __attribute__ ( (interrupt) ) machine_timer_irhandler       (void);
void __attribute__ ( (interrupt) ) user_external_irhandler       (void);
void __attribute__ ( (interrupt) ) supervisor_external_irhandler (void);
void __attribute__ ( (interrupt) ) machine_external_irhandler    (void);

void __attribute__ ( (interrupt) ) LocalInt0_handler (void);
void __attribute__ ( (interrupt) ) LocalInt1_handler (void);
void __attribute__ ( (interrupt) ) LocalInt2_handler (void);
void __attribute__ ( (interrupt) ) LocalInt3_handler (void);
void __attribute__ ( (interrupt) ) LocalInt4_handler (void);
void __attribute__ ( (interrupt) ) LocalInt5_handler (void);
void __attribute__ ( (interrupt) ) LocalInt6_handler (void);
void __attribute__ ( (interrupt) ) LocalInt7_handler (void);
void __attribute__ ( (interrupt) ) LocalInt8_handler (void);
void __attribute__ ( (interrupt) ) LocalInt9_handler (void);
void __attribute__ ( (interrupt) ) LocalIntA_handler (void);
void __attribute__ ( (interrupt) ) LocalIntB_handler (void);
void __attribute__ ( (interrupt) ) LocalIntC_handler (void);
void __attribute__ ( (interrupt) ) LocalIntD_handler (void);
void __attribute__ ( (interrupt) ) LocalIntE_handler (void);
void __attribute__ ( (interrupt) ) LocalIntF_handler (void);

#define IRQ_LOCAL_0 0x10
#define IRQ_LOCAL_1 0x11
#define IRQ_LOCAL_2 0x12
#define IRQ_LOCAL_3 0x13
#define IRQ_LOCAL_4 0x14
#define IRQ_LOCAL_5 0x15
#define IRQ_LOCAL_6 0x16
#define IRQ_LOCAL_7 0x17
#define IRQ_LOCAL_8 0x18
#define IRQ_LOCAL_9 0x19
#define IRQ_LOCAL_A 0x1A
#define IRQ_LOCAL_B 0x1B
#define IRQ_LOCAL_C 0x1C
#define IRQ_LOCAL_D 0x1D
#define IRQ_LOCAL_E 0x1E
#define IRQ_LOCAL_F 0x1F

#endif
