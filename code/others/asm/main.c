#include "myincludes.h"

int my_main()
{
	light_led();         //调用汇编，这里还是用物理地址访问的
	
#ifdef MMU_ON
	mmu_init();          //只有在用户程序中才使能mmu
#endif	

	led_init();
	led_on();

	button_init();

	init_irq();    //中断初始化

	while(1);

    return 0;
}
