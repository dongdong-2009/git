#ifndef _NAND_H_
#define _NAND_H_

//寄存器定义
#define NFCONF (*(volatile unsigned int*)0x4E000000)
#define NFCONT (*(volatile unsigned int*)0x4E000004)
#define NFCMD  (*(volatile unsigned char*)0x4E000008)
#define NFADDR (*(volatile unsigned char*)0x4E00000C)
#define NFDATA (*(volatile unsigned char*)0x4E000010)
#define NFSTAT (*(volatile unsigned char*)0x4E000020)

//NAND控制器对应特定的芯片，必须满足下面的三个时序
#define TACLS  1
#define TWRPH0 2
#define TWRPH1 1


void nand_init(void);
void nand_reset(void);
void nand_page_read(unsigned int addr, unsigned char* buff);
void chip_sel(void);
void chip_desel(void);
void clear_RnB(void);
void send_cmd(unsigned int cmd);
void send_addr(unsigned int addr);
void wait_RnB(void);
void nand_to_ram(unsigned int start_addr, unsigned int sdram_addr, int size);

#endif
