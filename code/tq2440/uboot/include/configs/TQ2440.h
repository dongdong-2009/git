/*
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 * Gary Jennejohn <garyj@denx.de>
 * David Mueller <d.mueller@elsoft.ch>
 *
 * Configuation settings for the SAMSUNG SMDK2410 board.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/*
 * High Level Configuration Options
 * (easy to change)
 */

#define CONFIG_ARM920T		/* This is an ARM920T Core */
#define CONFIG_S3C24X0		/* in a SAMSUNG S3C24x0-type SoC */

#define CONFIG_S3C2440
#define CONFIG_TQ2440

/*
#define CONFIG_BOOT_FROM_NAND
#ifndef CONFIG_BOOT_FROM_NAND
#define CONFIG_SYS_TEXT_BASE 0x0
#else
#define CONFIG_SYS_TEXT_BASE 0x31000000
#define CONFIG_SYS_NO_FLASH
#endif
*/
#define CONFIG_SYS_TEXT_BASE 0x31000000
#define CONFIG_BOOT_FROM_NAND
//#define CONFIG_SYS_NO_FLASH

#define CONFIG_SYS_ARM_CACHE_WRITETHROUGH

/* input clock of PLL (the SMDK2410 has 12MHz input clock) */
#define CONFIG_SYS_CLK_FREQ	12000000

#undef CONFIG_USE_IRQ		/* we don't need IRQ/FIQ stuff */

#define CONFIG_CMDLINE_TAG	/* enable passing of ATAGs */
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG

/*
 * Hardware drivers
 */

#define CONFIG_DRIVER_DM9000
#define CONFIG_DM9000_BASE		   0x20000000
#define DM9000_IO			       CONFIG_DM9000_BASE
#define DM9000_DATA			       (CONFIG_DM9000_BASE + 4)
#define CONFIG_DM9000_USE_16BIT
#define CONFIG_DM9000_NO_SROM


/*
 * select serial console configuration
 */
#define CONFIG_S3C24X0_SERIAL
#define CONFIG_SERIAL1		0	/* we use SERIAL 0 on TQ2440 */
#define CONFIG_BAUDRATE		115200

/*
 * Command line configuration.
 */
#include <config_cmd_default.h>

#define CONFIG_CMD_BSP
#define CONFIG_CMD_CACHE
#define CONFIG_CMD_ELF
#define CONFIG_CMD_NAND
#define CONFIG_CMD_PING
#define CONFIG_CMD_REGINFO
#define CONFIG_CMD_MTDPARTS
#define CONFIG_MTD_DEVICE
#define MTDIDS_DEFAULT    "nand0=TQ2440_Nand0"
#define MTDPARTS_DEFAULT  "mtdparts=TQ2440_Nand0:256k(uboot),"    \
	                      "128k(params),"                         \
	                      "128k(Remedy for BB),"                  \
	                      "4m(kernel),"                           \
	                      "128k(Remedy for BB),"                  \
	                      "-(rootfs)"

#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_CMDLINE_EDITING

/* autoboot */
#define CONFIG_BOOTDELAY	3
#define CONFIG_BOOT_RETRY_TIME	-1
#define CONFIG_RESET_TO_RETRY
#define CONFIG_ZERO_BOOTDELAY_CHECK

#define CONFIG_NETMASK		255.255.255.0
#define CONFIG_GATEWAYIP    192.168.1.1
#define CONFIG_SERVERIP		192.168.1.1
#define CONFIG_IPADDR		192.168.1.110
#define CONFIG_ETHADDR      10:BF:48:B7:0E:FB

#if defined(CONFIG_CMD_KGDB)
#define CONFIG_KGDB_BAUDRATE	115200	/* speed to run kgdb serial port */
/* what's this ? it's not used anywhere */
#define CONFIG_KGDB_SER_INDEX 0
#endif

/*
 * Miscellaneous configurable options
 */
#define CONFIG_SYS_LONGHELP		/* undef to save memory */
//BYHP
//#define CONFIG_SYS_PROMPT	"TQ2440@Ll:> "
#define CONFIG_SYS_PROMPT	"uboot@hp:> "
#define CONFIG_SYS_CBSIZE	256
/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE	(CONFIG_SYS_CBSIZE + \
				sizeof(CONFIG_SYS_PROMPT)+16)
#define CONFIG_SYS_MAXARGS	16
#define CONFIG_SYS_BARGSIZE	CONFIG_SYS_CBSIZE

#define CONFIG_DISPLAY_CPUINFO				/* Display cpu info */

#define CONFIG_SYS_MEMTEST_START	0x30000000	/* memtest works on */
#define CONFIG_SYS_MEMTEST_END		0x33F00000	/* 63 MB in DRAM */

#define CONFIG_SYS_LOAD_ADDR		0x30800000

#define CONFIG_SYS_HZ			1000

/* support additional compression methods */
#define CONFIG_BZIP2
#define CONFIG_LZO
#define CONFIG_LZMA
#define CONFIG_CMD_NAND_YAFFS
#define CONFIG_BOOTARGS    "console=ttySAC0 root=/dev/mtdblock3"
#define CONFIG_BOOTCOMMAND "nand read 0x30000000 kernel;bootm 0x30000000"


/*-----------------------------------------------------------------------
 * Stack sizes
 *
 * The stack sizes are set up in start.S using the settings below
 */
#define CONFIG_STACKSIZE	(128*1024)	/* regular stack */
#ifdef CONFIG_USE_IRQ
#define CONFIG_STACKSIZE_IRQ	(4*1024)	/* IRQ stack */
#define CONFIG_STACKSIZE_FIQ	(4*1024)	/* FIQ stack */
#endif

/*-----------------------------------------------------------------------
 * Physical Memory Map
 */
#define CONFIG_NR_DRAM_BANKS	1      /* we have 1 bank of DRAM */
#define PHYS_SDRAM_1		0x30000000 /* SDRAM Bank #1 */
#define PHYS_SDRAM_1_SIZE	0x04000000 /* 64 MB */

/*-----------------------------------------------------------------------
 * FLASH and environment organization
 */
#define PHYS_FLASH_1		    0x00000000 /* Flash Bank #0 */
#define CONFIG_SYS_FLASH_BASE	PHYS_FLASH_1

#ifndef CONFIG_SYS_NO_FLASH
#define CONFIG_SYS_FLASH_CFI
#define CONFIG_FLASH_CFI_DRIVER
#define CONFIG_FLASH_CFI_LEGACY
#define CONFIG_SYS_FLASH_LEGACY_1Mx16
#define CONFIG_FLASH_SHOW_PROGRESS	45
#define CONFIG_SYS_MAX_FLASH_BANKS	1
#define CONFIG_SYS_FLASH_BANKS_LIST { CONFIG_SYS_FLASH_BASE }
#define CONFIG_SYS_MAX_FLASH_SECT	(35)
#endif

#define CONFIG_ENV_IS_IN_NAND
#define CONFIG_ENV_OFFSET           0x40000
#define CONFIG_ENV_SIZE             0x20000
#define CONFIG_ENV_RANGE            CONFIG_ENV_SIZE

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE

/*
 * Size of malloc() pool
 * BZIP2 / LZO / LZMA need a lot of RAM
 */
#define CONFIG_SYS_MALLOC_LEN	(4 * 1024 * 1024)

#define CONFIG_SYS_MONITOR_LEN	(448 * 1024)
#define CONFIG_SYS_MONITOR_BASE	CONFIG_SYS_FLASH_BASE

/*
 * NAND configuration
 */
#ifdef CONFIG_CMD_NAND
#define CONFIG_NAND_S3C2440

#define CONFIG_SYS_NAND_MAX_CHIPS   1
#define CONFIG_SYS_NAND_PAGE_SIZE   2048
#define CONFIG_MTD_NAND_VERIFY_WRITE

#define CONFIG_S3C24XX_CUSTOM_NAND_TIMING
#ifdef CONFIG_S3C24XX_CUSTOM_NAND_TIMING
#define CONFIG_S3C24XX_TACLS  (3+1)
#define CONFIG_S3C24XX_TWRPH0 (7+1)
#define CONFIG_S3C24XX_TWRPH1 (7+1)
#endif

//开启硬件校验，BYHP
#define CONFIG_S3C2440_NAND_HWECC
#ifdef CONFIG_S3C2440_NAND_HWECC
#define CONFIG_SYS_NAND_ECCSIZE     CONFIG_SYS_NAND_PAGE_SIZE
#define CONFIG_SYS_NAND_ECCBYTES    4
#endif

#define CONFIG_SYS_MAX_NAND_DEVICE	1
#define CONFIG_SYS_NAND_BASE		0x4E000000
#endif

/* additions for new relocation code, must be added to all boards */
#define CONFIG_SYS_SDRAM_BASE	PHYS_SDRAM_1
#define CONFIG_SYS_INIT_SP_ADDR	(CONFIG_SYS_SDRAM_BASE + 0x1000 - \
				GENERATED_GBL_DATA_SIZE)

#define CONFIG_BOARD_EARLY_INIT_F

#define BOOT_FLAG *(unsigned long *)PHYS_SDRAM_1

#endif /* __CONFIG_H */
