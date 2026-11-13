/********************************************************************************************************
**                            (c) Copyright  苏州市江海通讯发展实业有限公司
**                                      All Rights Reserved                                        
**
**---------------------------- 文件信息 -----------------------------------------------------------------
** 文   件   名:  board.h
** 创   建   人:  林依刚
** 版        本:  V1.0
** 最后修改日期:  2010-4-19
** 描        述:  目标板硬件定义头文件。
** 操作系统版本： 无
********************************************************************************************************/
#ifndef BOARD_H
#define BOARD_H

#include  "AT91SAM7X256.h"                        /* AT91SAM7X256's header files.                      */
#include  "lib_AT91SAM7X256.h"                    /* AT91SAM7X256's library function file.             */



/*************************************** 系统时钟 ******************************************************/
#define EXT_OSC                 18432000          /* Exetrnal oscillator MAINCK                        */ 
#define MCK                     47923200          /* MCK (PLLRC div by 2)                              */    


/*************************************** 微控制器引脚的分配与定义 **************************************/
// VHF模块控制信号
#define PLL_DATA                AT91C_PIO_PB0     // Serial data input using binary code.
#define PLL_LE                  AT91C_PIO_PB1     // Load enable signal input
#define PLL_CLK                 AT91C_PIO_PB2     // Clock input for the 19-bit shift register.
#define PLL_LD                  AT91C_PIO_PB4     // Lock detect signal output(LD)
#define VHF_PTT                 AT91C_PIO_PB6     // VHF模块收/发控制      (1:接收; 0:发射)
#define VHF_PWR_CHK_IN          AT91C_PIO_PB5     // VHF模块发射功率检测输入
#define TONE_PWR_SW            AT91C_PIO_PB19     // VHF模块电源控制       (1:关闭; 0:启动)
#define VHF_MUTE                AT91C_PIO_PB8     // VHF模块静音控制       (1:静音; 0:正常)
#define VHF_DSC_SW              AT91C_PIO_PB3     // DSC发送切换开关       (1:发送DSC信号; 0:发送话音)
#define VHF_LOCK               AT91C_PIO_PB25     // VHF模块接收信号锁定指示  (1:锁定; 0:未锁定)

// FSK调制、解调芯片MSM7512B控制信号
#define FSK_TEST               AT91C_PIO_PB10     // Chip test input.TEST should be open or digital "1".         
#define FSK_RS                 AT91C_PIO_PB11     // FSK signal and answer tone transmit enable.active low            
#define FSK_MOD1               AT91C_PIO_PB12     // Operation mode select.           
#define FSK_MOD2               AT91C_PIO_PB13     // Operation mode select.            
#define FSK_XD                 AT91C_PIO_PB23     // Modem transmit serial data input.            
#define FSK_RD                 AT91C_PIO_PB25     // Modem receive serial data output.          

// VHF 121.5MHz航空模块控制信号
#define PLL2_CE                AT91C_PIO_PA29     // PLL2 chip enable
#define PLL2_CLK               AT91C_PIO_PB24     // PLL2 clock
#define PLL2_DATA              AT91C_PIO_PB26     // PLL2 data
#define AM_SEL                 AT91C_PIO_PB9      // 用于121.5Mhz和VHF的功能切换，也用于121.5Mhz模块的开启和关闭。
                                                     //1：天线导向AM模块并开启AM模块，软件送出AM模块频合数据；
                                                     //0：天线导向VHF模块并关闭AM模块。
#define AM_PTT                 AT91C_PIO_PB15     // AM模块收/发控制       (1:接收;   0:发射)
#define AM_FREQ                AT91C_PIO_PA19     // AM模块PLL锁定指示     (1:锁定;   0:未锁定)
#define AM_PA                  AT91C_PIO_PA20     // AM模块功放检测指示    (1:异常;   0:正常)
#define AM_LOCK                AT91C_PIO_PA21     // AM模块接收锁定指示    (1:未锁定; 0:锁定)

// 浮标内通信设备话筒PPT键检测信号
#define MIKE_PTT               AT91C_PIO_PB7      // 话筒PTT键检测信号     (1:接收; 0:发送)     

// 9G(NineGHz)模块控制信号
#define NG_STOP                AT91C_PIO_PB30     // 9G模块应答控制        (1:禁止应答; 0:使能应答)         
#define NG_RESP_IN             AT91C_PIO_PB29     // 9G模块应答信号输入         
#define NG_CHK                 AT91C_PIO_PB27     // 9G模块检测信号输出         
#define NG_CHK_IN              AT91C_PIO_PB28     // 9G模块检测信号输入       

// 浮标内备用电池切换开关、当前电源指示、断缆检测信号、工作模式选择信号（1、2）
#define BAT_SW                 AT91C_PIO_PB18     // 浮标备用电池控制开关  (1:启动; 0:关闭)          
#define SYS_PWR_CUR            AT91C_PIO_PB20     // 系统当前工作电源指示  (1:艇电; 0:电池)     
#define SYS_CABLE_CHK          AT91C_PIO_PB22     // 浮标信号电缆测试线    (1:未断; 0:断缆) 针对定型电路
#define SYS_WORK_MODE1         AT91C_PIO_PA9      // 选择运行35#或37#程序  (1:37#;  0:35#)
#define SYS_WORK_MODE2         AT91C_PIO_PB21     // 保留 

// 闪光灯电源控制信号
#define LED_FLASH_LAMP         AT91C_PIO_PB14     // 闪光灯电源控制开关    (1:关闭; 0:开启)      

// MAX13487EESA控制信号
//#define MAX13487_RE            AT91C_PIO_PA26     // 接收使能控制信号      (1:自动方向; 0:接收使能)     
//#define MAX13487_SHDN          AT91C_PIO_PA30     // 芯片关断控制信号      (1:正常工作; 0:关断芯片)       
#define AM_MC_SW               AT91C_PIO_PA26     // AM模块静噪控制信号      (1:关闭; 0:开启)     
#define OUT_5V                 AT91C_PIO_PA30     // 保留
//-------------------------------------------------------------------------------------------------------

/*************************************** 全局函数宏定义 ************************************************/
#define IS_TONE_PWR_ON()      (((*AT91C_PIOB_PDSR) & TONE_PWR_SW) ? false : true)
#define IS_AM_SEL()           (((*AT91C_PIOB_PDSR) & AM_SEL) ? true : false)
#define IS_AM_MC_EN()         (((*AT91C_PIOA_ODSR) & AM_MC_SW) ? false : true)
#define IS_9G_WORKING()       (((*AT91C_PIOB_ODSR) & NG_STOP) ? false : true)

#define VHF_MUTE_EN()         (*AT91C_PIOB_SODR = VHF_MUTE)      /* vhf module mute enable.            */
#define VHF_MUTE_DIS()        (*AT91C_PIOB_CODR = VHF_MUTE)      /* mute disable,normal state          */
 
#define VHF_PTT_EN()          (*AT91C_PIOB_CODR = VHF_PTT)       /* Switch the VHF module for TX       */
#define VHF_PTT_DIS()         (*AT91C_PIOB_SODR = VHF_PTT)       /* Switch the VHF module for RX       */

#define FSK_RS_EN()           (*AT91C_PIOB_CODR = FSK_RS)        /* Enable msm7512b transmit.          */
#define FSK_RS_DIS()          (*AT91C_PIOB_SODR = FSK_RS)        /* Disable msm7512b transmit.         */
#define FSK_TEST_EN()         (*AT91C_PIOB_CODR = FSK_TEST)      /* Set the msm7512b in test mode.     */
#define FSK_TEST_DIS()        (*AT91C_PIOB_SODR = FSK_TEST)      /* Cancle the msm7512b in test mode.  */

#define AM_SEL_EN()           (*AT91C_PIOB_SODR = AM_SEL)        /* Select AM module.                  */
#define AM_SEL_DIS()          (*AT91C_PIOB_CODR = AM_SEL)        /* Select VHF module.                 */

#define AM_PTT_EN()           (*AT91C_PIOB_CODR = AM_PTT)        /* Switch the AM module for TX        */
#define AM_PTT_DIS()          (*AT91C_PIOB_SODR = AM_PTT)        /* Switch the AM module for RX        */

#define AM_MC_EN()            (*AT91C_PIOA_CODR = AM_MC_SW)      /* Enable the AM module mute control  */
#define AM_MC_DIS()           (*AT91C_PIOA_SODR = AM_MC_SW)      /* Disable the AM module mute control */

#define BAT_ON()              (*AT91C_PIOB_SODR = BAT_SW)        /* Turn on the battery of buoyage     */
#define BAT_OFF()             (*AT91C_PIOB_CODR = BAT_SW)        /* Turn off the battery of buoyage    */

//#define RS485_EN()            (*AT91C_PIOA_SODR = MAX13487_SHDN) /* Enable the MAX13487EESA.           */
//#define RS485_DIS()           (*AT91C_PIOA_CODR = MAX13487_SHDN) /* Shutdown the MAX13487EESA.         */
//#define RS485_AUTO_EN()       (*AT91C_PIOA_SODR = MAX13487_RE)   /* RS485 bus autodirection control    */	
//#define RS485_AUTO_DIS()      (*AT91C_PIOA_CODR = MAX13487_RE)   /* RS485 bus receive output enable    */

#define LED_ON(led)           (*AT91C_PIOB_CODR = led)           /* The power supply of LED is enable  */
#define LED_OFF(led)          (*AT91C_PIOB_SODR = led)           /* The power supply of LED is disable */

#endif
