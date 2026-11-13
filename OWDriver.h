/********************************************************************************************************
**                            (c) Copyright  苏州市江海通讯发展实业有限公司
**                                      All Rights Reserved                                        
**
**---------------------------- 文件信息 -----------------------------------------------------------------
** 文   件   名:  OWDriver.h
** 创   建   人:  林依刚
** 版        本:  V2.0
** 最后修改日期:  2009-7-22
** 描        述:  声明1-Wire总线进行位、字节和块操作时所必需的函数。使用本软件包需要结合不同的硬件平台，
**                实现三个外部接口函数:outp(...), inp(...), tickDelay(...)
** 操作系统版本： 无RTOS
**             
**--------------------------- 历史版本信息 --------------------------------------------------------------
** 创   建   人:  林依刚
** 版        本:  V1.0
** 日　      期:  2007-12-15
** 描　      述:  原始版本
**
********************************************************************************************************/
#ifndef OWDRIVER_H
#define OWDRIVER_H

#define PORTADDRESS           ((unsigned int) 1 << 2)

//--1-Wire总线网络地址命令
#define CMD_READ_NEST_ADDR    0x33      /* 读取DS2788的1-Wire网络地址                                  */
#define CMD_MATCH_NEST_ADDR   0x55      /* 访问1-Wire总线上某个特定的DS2788                            */
#define CMD_SKIP_NEST_ADDR    0xCC      /* 仅有一个DS2788时,允许总线主机在不指定从设备地址的情况下操作 */
#define CMD_SRCH_NEST_ADDR    0xF0      /* 允许总线主机采用排除法来识别总线上所有从机的1-Wire网络地址  */
#define CMD_RESUME_ADDR       0xA5      /* 与Skip Net Address 命令类似，使用于多个1-Wire网络地址       */


//--1-Wire总线功能命令
#define CMD_READ_DATA         0x69      /* 从存储器地址XX开始读取DS2788的数据                          */
#define CMD_WRITE_DATA        0x6C      /* 从存储器地址XX开始将数据写入DS2788                          */
#define CMD_COPY_DATA         0x48      /* 将EEPROM映射RAM的内容复制到EEPROM模块中地址为XX的EEPROM单元 */
#define CMD_RECALL_DATA       0xB8      /* 将EEPROM模块中位置为XX的EEPROM单元存储内容恢复到映射存储器中*/
#define CMD_LOCK_DATA         0x6A      /* 锁定(写保护)包含存储器地址XX的EEPROM存储器模块              */


/********************************************************************************************************
** 依赖于两个通用的‘C’函数outp和inp，从IO端口读写字节数据。他们通常位于标准库中。
** 在进行移植时，需要结合具体硬件平台来实现以下三个外部函数: outp(...), inp(...), tickDelay(...)
********************************************************************************************************/
// 通过端口设置1-Wire总线状态(根据databyte的逻辑值，拉高或拉低总线)
extern int outp(unsigned int port, int databyte);

// 通过端口读入1-Wire总线状态(总线逻辑电平值)
extern int inp(unsigned int port); 

// 精确延时tick个时钟节拍(1/4us)，需要结合具体硬件平台来实现该函数
extern void tickDelay(int tick); 

int  OWReadBit(void);
int  OWReadByte(void);
int  OWTouchByte(int data_val);
int  OWTouchReset(void);
int  OWOverdriveSkip(unsigned char *pdata, int data_len);
void OWWriteBit(int bit_level);
void OWWriteByte(int data_val);
void OWBlock(unsigned char *pdata, int data_len); 
void SetSpeed(int standard);

#endif