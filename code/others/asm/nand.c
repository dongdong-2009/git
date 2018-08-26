#include "myincludes.h"

//片选nand芯片
void chip_sel(void)
{
	NFCONT &= ~(1<<1);
}

//取消选择nand
void chip_desel(void)
{
	NFCONT |= (1<<1);
}

//清除R/B信号
void clear_RnB(void)
{
	NFSTAT |= (1<<2);
}

//发送命令
void send_cmd(unsigned int cmd)
{
	NFCMD = cmd;
}

//写入地址，每次只能写入地址的八位
void send_addr(unsigned int addr)
{
	NFADDR = addr;
}

//等待RnB信号
void wait_RnB(void)
{
	while(!(NFSTAT & (1<<2)));
}

//复位外部nand flash
void nand_reset(void)
{
	//选择nand芯片
	chip_sel();
	
	//清除RnB
	clear_RnB();
	
	//发送命令0xFF
	send_cmd(0xFF);
	
	//等待RnB信号从低变高
	wait_RnB();
	
	//取消选择NAND
	chip_desel();
}

//nand初始化
void nand_init(void)
{
	//设置NFCONF寄存器，设置相关时序
	NFCONF = (TACLS<<12 | TWRPH0<<8 | TWRPH1<<4);
	
	//设置NFCONT寄存器 先不片选，使能NAND控制器
	NFCONT = ((1<<0) | (1<<1));
	
	//复位外部nand flash
	nand_reset();
}

//页读取   这里的addr为页地址
void nand_page_read(unsigned int addr, unsigned char* buff)
{
	unsigned int i = 0;
	//片选芯片，强制使外部nFCE引脚为低
	chip_sel();
	
	//清除R/B信号
	clear_RnB();
	
	//发送命令0x00
	send_cmd(0x00);
	
	//发送列地址
	send_addr(0x00);     //页读时列地址为0
	send_addr(0x00);
	
	//发送行地址
	send_addr(addr&0xff);
	send_addr((addr>>8)&0xff);
	send_addr((addr>>16)&0xff);
	
	//发送命令0x30
	send_cmd(0x30);
	
	//等待R/B信号，知道该信号为高电平
	wait_RnB();
	
	//读取数据 一个页的大小为(2K+64)字节，故这里需要2048个字节
	for(i=0; i<2048; i++)
	{
		buff[i] = NFDATA;    //感觉这里有问题
	}
	
	//取消片选
	chip_desel();	
}

//将nand里面的前4K的数据搬运到内存里面
//size表示复制多少个字节数，注意，这里的size一定要用int型，不能用unsigned int型，否则不能正常复制程序
void nand_to_ram(unsigned int start_addr, unsigned int sdram_addr, int size)
{
	unsigned int addr = 0;
	
	//地址右移11位得到页的起始地址
	for(addr=(start_addr >> 11); size > 0;)
	{
		nand_page_read(addr, (unsigned char*)sdram_addr);    //每读出一页，就读出了2048个字节
		size -= 2048;
		sdram_addr += 2048;
		addr++;                                                 //注意，这里是页号加1，而不是加2048
	}
}
