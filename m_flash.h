/********************************************************************************************************
**                            (c) Copyright  苏州市江海通讯发展实业有限公司
**                                      All Rights Reserved                                        
**
**---------------------------- 文件信息 -----------------------------------------------------------------
** 文   件   名:  usart_pdc.c
** 创   建   人:  林依刚
** 版        本:  V0.1
** 最后修改日期:  2010-12-17
** 描        述:  AT91SAM7X(C)256 通过SPI接口读写AT45DB021B驱动程序头文件
** 操作系统版本： μC/OS-II V2.76
********************************************************************************************************/

#ifndef	FLASH_H
#define	FLASH_H


#include "m_spi.h"


/*
*********************************************************************************************************
*                                            常量及宏定义
*********************************************************************************************************
*/
#define FLASH_EXT extern

#define	FLASH_FALSE                         0
#define	FLASH_TRUE                          1 

// DataFlash外部占用的片选号、片选引脚、SPI控制器寄存器基地址、读写波特率定义
#define	FLASH_NPCS_ID	                    0
#define	FLASH_NPCS	    AT91C_PA12_SPI0_NPCS0
#define	FLASH_SPI_BASE		  AT91C_BASE_SPI0
#define FLASH_BAUD_RATE	             10000000	  // 通过SPI口读写FLASH使用的波特率(10M)

// 指定DataFlash固有参数
#define	FLASH_DEV_ID			         0x0C     // FLASH设备ID,通过读取状态寄存器相关位域可得    
#define	FLASH_PAGE_NUM			         1024     // FLASH页面数量
#define	FLASH_PAGE_SIZE			          264     // FLASH页面大小
#define	FLASH_BUF_SIZE	      FLASH_PAGE_SIZE     // FLASH读写缓冲区大小

// DataFlash状态寄存器位域掩码
#define	FLASH_STATUS_RDY	             0x80     // RDY/BUSY#
#define	FLASH_STATUS_COMP		         0x40	  // 比较位(0：main memory page matches the data in the buffer.)
#define	FLASH_STATUS_ID_MASK		     0x3C	  // 0x00111100(正好是5:2位域，该位域保存FLASH设备ID)

// 实现高位数据到低位数据的交换，比如0x12345678经过转换后成为0x78563412
#define	HighToLowSwitch(u32Value)    ((u32Value & 0xFF000000) >> 24 | (u32Value & 0x00FF0000) >> 8 \
                                    | (u32Value & 0x0000FF00) << 8  | (u32Value & 0x000000FF) << 24)


/*
*********************************************************************************************************
*                                            ERROR CODES
*********************************************************************************************************
*/
#define	FLASH_NO_ERR                        0
#define	FLASH_ERR_HARD_FAULT               1u
#define	FLASH_ERR_NULL_PTR                 2u
#define	FLASH_ERR_DATA_OVF                 3u
#define	FLASH_ERR_INVALID_PAGE_ADDR        4u
#define	FLASH_ERR_INVALID_BYTE_ADDR        5u
#define	FLASH_ERR_INVALID_OPT              6u


/*
*********************************************************************************************************
*                                            数据结构定义
*********************************************************************************************************
*/
typedef union {
	INT32U	u32Opcode;
	struct {
		INT32U	ByteAddr	:9;		              // 页面内的偏移地址
		INT32U	PageAddr	:10;                  // 页面地址，实际上就是页面序号
		INT32U	Reserved	:5;                   // 保留
		INT32U	Opcode		:8;		              // 操作码
	} BitsOpcode;		
}FLASH_OPCODE_TYPE;


/*
*********************************************************************************************************
*                             AT45DBxxxB Serial DataFlash Command Definition
*********************************************************************************************************
*/
/* READ COMMANDS */
#define DB_CONTINUOUS_ARRAY_READ      0xE8        /* Continuous array read */
#define DB_BURST_ARRAY_READ           0xE8        /* Burst array read */
#define DB_PAGE_READ                  0xD2        /* Main memory page read */
#define DB_BUF1_READ                  0xD4        /* Buffer 1 read */
#define DB_BUF2_READ                  0xD6        /* Buffer 2 read */
#define DB_STATUS                     0xD7        /* Status Register */

/* PROGRAM and ERASE COMMANDS */
#define DB_BUF1_WRITE                 0x84        /* Buffer 1 write */
#define DB_BUF2_WRITE                 0x87        /* Buffer 2 write */
#define DB_BUF1_PAGE_ERASE_PGM        0x83        /* Buffer 1 to main memory page program with built-In erase */
#define DB_BUF1_PAGE_ERASE_FASTPGM    0x93        /* Buffer 1 to main memory page program with built-In erase, Fast program */
#define DB_BUF2_PAGE_ERASE_PGM        0x86        /* Buffer 2 to main memory page program with built-In erase */
#define DB_BUF2_PAGE_ERASE_FASTPGM    0x96        /* Buffer 1 to main memory page program with built-In erase, Fast program */
#define DB_BUF1_PAGE_PGM              0x88        /* Buffer 1 to main memory page program without built-In erase */
#define DB_BUF1_PAGE_FASTPGM          0x98        /* Buffer 1 to main memory page program without built-In erase, Fast program*/
#define DB_BUF2_PAGE_PGM              0x89        /* Buffer 2 to main memory page program without built-In erase */
#define DB_BUF2_PAGE_FASTPGM          0x99        /* Buffer 1 to main memory page program without built-In erase, Fast program*/
#define DB_PAGE_ERASE                 0x81        /* Page Erase  */
#define DB_BLOCK_ERASE                0x50        /* Block Erase */
#define DB_PAGE_PGM_BUF1              0x82        /* Main memory page through buffer 1 */
#define DB_PAGE_FASTPGM_BUF1          0x92        /* Main memory page through buffer 1, Fast program */
#define DB_PAGE_PGM_BUF2              0x85        /* Main memory page through buffer 2 */
#define DB_PAGE_FastPGM_BUF2          0x95        /* Main memory page through buffer 2, Fast program */

/* ADDITIONAL COMMANDS */
#define DB_PAGE_2_BUF1_TRF            0x53        /* Main memory page to buffer 1 transfert */
#define DB_PAGE_2_BUF2_TRF            0x55        /* Main memory page to buffer 2 transfert */
#define DB_PAGE_2_BUF1_CMP            0x60        /* Main memory page to buffer 1 compare */
#define DB_PAGE_2_BUF2_CMP            0x61        /* Main memory page to buffer 2 compare */
#define DB_AUTO_PAGE_PGM_BUF1         0x58        /* Auto page rewrite throught buffer 1 */
#define DB_AUTO_PAGE_PGM_BUF2         0x59        /* Auto page rewrite throught buffer 2 */


/*
*********************************************************************************************************
*                                             全局函数原型声明
*********************************************************************************************************
*/
FLASH_EXT void DF_CtrlCfg(void);
FLASH_EXT void CheckFlashID(void);
FLASH_EXT BOOLEAN DF_PageWrite(INT16U page_addr, INT16U byte_addr, INT8U *pdata, INT16U data_len);
INT8U     DF_ContinuousRead(INT16U page_addr, INT16U byte_addr, INT8U *pdata, INT32U data_len);
INT8U     DF_PageErase(INT16U page_addr);
FLASH_EXT INT8U DF_GetStatus(void);


#endif
