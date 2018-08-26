#ifndef _GPIO_H_
#define _GPIO_H_
#include "myincludes.h"

/*********************************led为对应GPB5、6、7、8 **********************/
//定义寄存器  虚拟地址  使能mmu后的地址 LED
#ifdef MMU_ON
#define GPBCON (volatile unsigned int *)0xAB000010
#define GPBDAT (volatile unsigned int *)0xAB000014
#else
//定义寄存器  物理地址
#define GPBCON (volatile unsigned int *)0x56000010
#define GPBDAT (volatile unsigned int *)0x56000014
#endif

/*******************************按键对应GPF1、4、2、0****************************/
//定义按键的相关寄存器
#define GPFCON (volatile unsigned int *)0x56000050
#define GPFDAT (volatile unsigned int *)0x56000054

#define GPF0_MSK (3<<(0*2))
#define GPF1_MSK (3<<(1*2))
#define GPF2_MSK (3<<(2*2))
#define GPF4_MSK (3<<(4*2))

#define GPF0_INIT (2<<(0*2))
#define GPF1_INIT (2<<(1*2))
#define GPF2_INIT (2<<(2*2))
#define GPF4_INIT (2<<(4*2))



void led_init(void);
void led_on(void);
void led_off(void);

void button_init(void);

#endif

