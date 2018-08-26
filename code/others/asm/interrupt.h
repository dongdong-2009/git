#ifndef _INTERRUPT_H_
#define _INTERRUPT_H_

//首先定义所有可用的寄存器
#define SRCPND              (volatile unsigned int *)0x4A000000
#define INTMOD              (volatile unsigned int *)0x4A000004
#define INTMSK              (volatile unsigned int *)0x4A000008
#define PRIORITY            (volatile unsigned int *)0x4A00000c
#define INTPND              (volatile unsigned int *)0x4A000010
#define INTOFFSET           (volatile unsigned int *)0x4A000014
#define SUBSRCPND           (volatile unsigned int *)0x4A000018
#define INTSUBMSK           (volatile unsigned int *)0x4A00001c

#define EINTMASK            (volatile unsigned int *)0x560000a4
#define EINTPEND            (volatile unsigned int *)0x560000a8


void init_irq(void);
void handle_init(void);

#endif
