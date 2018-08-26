#include "myincludes.h"

void led_init(void)
{
	*GPBCON = 0x15400;   //设置为输出，其实这一步可以省略，因为在boot中已经做了
}

void led_on(void)
{
	*GPBDAT = ~0x20;
}

void led_off(void)
{
	*GPBDAT = ~0x00;   //led为对应GPB5、6、7、8 为1的时候关闭
}

void button_init(void)
{
	*(GPFCON) &= ~(GPF0_MSK | GPF1_MSK | GPF2_MSK | GPF4_MSK);       //先将相应位清零
	*(GPFCON) |= (GPF0_INIT | GPF1_INIT | GPF2_INIT | GPF4_INIT);   //按键对应GPF1、4、2、0
}





