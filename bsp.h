/********************************************************************************************************
**                            (c) Copyright  苏州市江海通讯发展实业有限公司
**                                      All Rights Reserved                                        
**
**---------------------------- 文件信息 -----------------------------------------------------------------
** 文   件   名:  bsp.h
** 创   建   人:  林依刚
** 版        本:  V1.0
** 最后修改日期:  2007-12-10
** 描        述:  板级支持包头文件，包含微控制器资源定义、库函数、各种外设驱动程序头文件。
** 操作系统版本： μC/OS-II V2.76 
********************************************************************************************************/

#ifndef BSP_H
#define BSP_H

/********************************************************************************************************
*                                       INCLUDE FILES                                         
********************************************************************************************************/
#include  "includes.h"                            /* Include the μC/OS-II source code.                */
#include  "AT91SAM7X256.h"                        /* AT91SAM7X256's header files.                      */
#include  "lib_AT91SAM7X256.h"                    /* AT91SAM7X256's library function file.             */
#include  "usart_pdc.h"                           /* usart driver's header file.                       */               
#include  "DS2788.h"                              /* DS2788 driver's header file.                      */ 
#include  "m_flash.h"                             /* dataflash driver's header file.                   */
#include  "board.h"                               /* the target board define file                      */

#ifndef false
#define false	                      0u
#endif

#ifndef true
#define true	                      1u
#endif

#define BSP_NO_ERR                    0u
#define BSP_ERR_NULL_PTR              1u


//日期与时间数据结构
typedef struct bsp_date_time_type{ 
	INT16U          year;                         // 年
	INT8U           month;                        // 月（1~12）
	INT8U           day;                          // 日（1~31）
    INT8U           hour;                         // 时（0~23）
    INT8U           minute;                       // 钟（0~59）
    INT8U           sec;                          // 秒（0~59）
    INT8U           centisecond;                  // 百分之一秒,厘秒 = 0.01秒（0~99）
} BSP_DATE_TIME_TYPE;

extern  volatile  BSP_DATE_TIME_TYPE  BspTime;
/********************************************************************************************************
*                                       PROTOTYPES                                         
********************************************************************************************************/
void   BSP_Init(void);
void   DBGU_Open(INT32U baud_rate, INT8U parity_type);
void   DBGU_Close(void);
void   DBGU_Printk(char *psrc); 
INT8U  DBGU_Write(INT8U *psrc, INT32U len);

#endif
