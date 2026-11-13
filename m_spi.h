/********************************************************************************************************
**                            (c) Copyright  苏州市江海通讯发展实业有限公司
**                                      All Rights Reserved                                        
**
**---------------------------- 文件信息 -----------------------------------------------------------------
** 文   件   名:  spi.h
** 创   建   人:  林依刚
** 版        本:  V0.1
** 最后修改日期:  2010-12-17
** 描        述:  spi设备相关的常量、宏、函数原型定义
** 操作系统版本： μC/OS-II V2.76
********************************************************************************************************/

#ifndef	SPI_H
#define	SPI_H
#include "bsp.h"


/*
*********************************************************************************************************
*                                             常量、变量及宏定义
*********************************************************************************************************
*/
#define SPI_EXTERN       extern

#define	SPI_TRUE              1
#define	SPI_FALSE             0

// 定义SPI端口号
#define	SPI0	              0
#define	SPI1	              1

// SPI0相关引脚定义
#define	SPI0_PINS (AT91C_PA16_SPI0_MISO | AT91C_PA17_SPI0_MOSI | AT91C_PA18_SPI0_SPCK)

// SPI1相关引脚定义
#define	SPI1_PINS (AT91C_PA24_SPI1_MISO | AT91C_PA23_SPI1_MOSI | AT91C_PA22_SPI1_SPCK)

// SPI中断优先级
#define	SPI_IRQ_PRIO	      7

//SPI0访问阻塞句柄
SPI_EXTERN OS_EVENT     *hSPI0;


/*
*********************************************************************************************************
*                                             数据结构定义
*********************************************************************************************************
*/
// SPI设备接口描述数据结构
typedef void(*PFUN_SPI_ISR)(void);
typedef struct {
	AT91PS_SPI 		pSPI;				// 指向SPI控制器寄存器基址的指针
	INT32U     		Pins;				// SPI相关引脚（片选引脚除外）
	INT8U	   		SpiID;    			// SPI在AT91SAM7X256内部分配的ID号（SPI0:4 SPI1:5）
	INT32U			NeedPullUpPin;		// 需要使能内部上拉电阻的引脚
	INT8U			IrqPrio;			// 中断优先级
	PFUN_SPI_ISR	pFunSpiIsr;			// SPI中断服务子函数入口地址
} SPI_DEV_DESC_TYPE;

// SPI设备读写操作数据结构
typedef int(*PFUN_CALLBACK)(void *);
typedef struct {
	INT8U 			Port;				// SPI控制器的端口号（0~1）                    
	INT8U			CsID;				// 片选ID号（NPCS0~NPCS3） 
	INT8U		   *pCmd;				// 待发送的命令行指针
	INT8U			CmdLen;				// 命令长度
	INT8U		   *pData;				// 读写数据指针
	INT32U			DataLen;			// 读写数据长度
	PFUN_CALLBACK	pFunCallBack;       // 回调函数，在读写前后调用
	void		   *pArg;				// 传递给回调函数的参数
}SPI_DEV_RW_TYPE;


/*
*********************************************************************************************************
*                                             全局函数原型声明
*********************************************************************************************************
*/
SPI_EXTERN void SPI_Init(INT8U u8SPIPort);
SPI_EXTERN void SPI_WaitRxEnd(void);
SPI_EXTERN void SPI_ExecuteRW(SPI_DEV_RW_TYPE *rw_data);


#endif
