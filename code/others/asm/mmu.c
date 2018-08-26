#include "myincludes.h"
//实际在boot阶段一般部使用MMU，只有在用户程序中使用mmu

void creat_page_table(void)
{
	unsigned int *ttb = (unsigned int *)0x30000000;     //将表存放在SDRAM内存的起始地址处
	unsigned int vaddr, paddr;                         //定义虚拟地址和物理地址
	
	vaddr = 0xAB000000;                                 //要映射的虚拟地址
	paddr = 0x56000000;                                 //要映射到的物理地址
	*(ttb + (vaddr>>20)) = (paddr & 0xFFF00000) | MMU_SECDESC;
	
	vaddr = 0x30000000;
	paddr = 0x30000000;                                 //内存地址也需要映射，物理地址和虚拟地址重合
	while(vaddr < 0x34000000)                           //映射全部的SDRAM的内存空间，共64M
	{
		*(ttb + (vaddr>>20)) = (paddr & 0xFFF00000) | MMU_SECDESC_WB;
		vaddr += 0x100000;                              //段映射的大小为1M，映射完后需要将地址加1M
		paddr += 0x100000;
	}
}

void mmu_enable(void)
{
	__asm__(
	//写入TTB
	"ldr    r0, =0x30000000\n"                  
    "mcr    p15, 0, r0, c2, c0, 0\n"  
	
	//不进行权限检查
    "mvn    r0, #0\n"                   
    "mcr    p15, 0, r0, c3, c0, 0\n"   
    
   //使能MMU
    "mrc    p15, 0, r0, c1, c0, 0\n"    
    "orr    r0, r0, #0x0001\n"          
    "mcr    p15, 0, r0, c1, c0, 0\n"    
    : 
    : 
	:"r0"   //如果没有毁坏区的话，这里的冒号是不能要的，否则不能编译
	);
}

void mmu_init(void)
{
	creat_page_table();
	mmu_enable();
}
