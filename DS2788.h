/********************************************************************************************************
**                            (c) Copyright  苏州市江海通讯发展实业有限公司
**                                      All Rights Reserved                                        
**
**---------------------------- 文件信息 -----------------------------------------------------------------
** 文   件   名:  DS2788.h
** 创   建   人:  林依刚
** 版        本:  V2.0
** 最后修改日期:  2009-7-22
** 描        述:  声明通过1-Wire总线对DS288进行读写以及测量电池剩余电量的各种函数。
** 操作系统版本： 无RTOS
**             
**--------------------------- 历史版本信息 --------------------------------------------------------------
** 创   建   人:  林依刚
** 版        本:  V1.0
** 日　      期:  2007-12-15
** 描　      述:  原始版本
**
********************************************************************************************************/
#ifndef DS2788_H
#define DS2788_H

#include "bsp.h"
#include "OWDriver.h"

#define DS2788_INIT_FLAG      0xAA                // DS2788 初始化标志


typedef struct battery_info_type{	              //--电池信息
    unsigned char             m_InitFlag;         // 初始化标志(0xAA:已经初始化;其它：未初始化)
    unsigned char             m_BatID[2];         // 电池编号
    unsigned char             m_InitDate[4];      // 初始化时间：年（2字节）、月、日             
    unsigned char             m_InitCap[2];       // 初始化容量             
} BATTERY_INFO_TYPE;


// 外部全局变量声明
extern BATTERY_INFO_TYPE  g_BatInfo;


/* 读取1-Wire总线数据接口函数
*  参数：port     : 对应微控制器的I/O口号
*/
int inp(unsigned port);


/* 写1-Wire总线数据接口函数
*  参数：port     : 对应微控制器的I/O口号
*        databyte : 待写数据
*/
int outp(unsigned port, int databyte);


/* 定时器2初始化函数 */
void TC2_Init(void);


/* 时钟节拍延时函数
*  参数：tick     : 需延时的时钟节拍个数(每个节拍:1/4us)
*/
void tickDelay(int tick);


/* DS2788初始化函数 */
unsigned char DS2788_Init(void);


/* 配置DS2788内部寄存器及工作参数 */ 
unsigned char DS2788_CfgBatInfo(void);


/* 配置电池组编号及启用日期 */ 
unsigned char DS2788_CfgBatIDAndDate(void); 

  
/* 电池剩余电量测量 
*  参数:  pdest : 电池剩余电量测量结果保存位置的首地址
*/
void DS2788_BatMeas(unsigned char *pdest);


/* 读取DS2788内部RAM的内容
*  参数:  pdest : 读取缓冲区的首地址
*         addr  : 读取内部RAM的起始地址
*         size  : 读取数据的长度
*/
unsigned char DS2788_ReadData(unsigned char *pdest,unsigned char addr,unsigned char size);


/* 写数据至DS2788内部RAM
*  参数:  psrc  : 待写数据的首地址
*         addr  : 待写内部RAM的起始地址
*         size  : 待写数据的长度
*/
unsigned char DS2788_WriteData(unsigned char *psrc,unsigned char addr,unsigned char size);


/* 将DS2788用户EEPROM中的数据恢复至内部RAM中
*  参数:  addr  : 待恢复内部RAM的起始地址
*         size  : 待恢复数据的长度
*/
unsigned char DS2788_RecallData(unsigned char addr,unsigned char size);


/* 将DS2788内部RAM数据拷贝至用户EEPROM中
*  参数:  addr  : 待拷贝内部RAM的起始地址
*         size  : 待拷贝数据的长度
*/
unsigned char DS2788_CopyData(unsigned char addr,unsigned char size);

#endif