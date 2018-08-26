#include "myincludes.h"

//中断初始化：打开四个按键中断的MASK，使能中断
void init_irq(void)
{	
	//对应4、5、6、7四个中断，只相应4号外部中断，该寄存器的配置必须在INTMSK之前
	*(EINTMASK) &= ~(1<<4);
	
	//将INTMSK的相应位设置为0，以打开相应的中断
	*(INTMSK) &= ~((1<<0) | (1<<1) | (1<<2) | (1<<4));  

	//因为在start.S里面把所有的中断都关闭了，这里需要打开中断，需要多CPSR寄存器进行操作
	asm(
		"mrs r0, cpsr;"
		"bic r0, r0, #0x80;"
		"msr cpsr_c, r0;"
		:
		:
	);
}

//该函数主要完成两个工作，1：判断中断源 2：调用中断处理函数
#define INTOFFSET           (volatile unsigned int *)0x4A000014

void handle_init(void)
{
	unsigned int int_num = *(INTOFFSET);		//读取寄存器的值
												//判断是那一个中断产生了
	//跳转到相应的终端服务程序
	switch(int_num)
	{
		case 1:  //KEY1
			led_on();
			break;
			
		case 4: //KEY2
			led_off();
			break;
			
		case 2: //KEY3
			led_on();
			break;
			
		case 0: //KEY4
			led_off();
			break;
			
		default:
			break;
	}
	
	//中断清除
	*(SRCPND) = 1<<int_num;    //相应位写1就会清零
	*(INTPND) = 1<<int_num;
	
	//当是4号按键被按下时，因为4、5、6、7是复用外部中断的，还需要将GPIO中的EINTPEND寄存器也清零
	if(4 == int_num) *(EINTPEND) = 1<<4;
}

