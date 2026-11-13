/********************************************************************************************************
**                            (c) Copyright  苏州市江海通讯发展实业有限公司
**                                      All Rights Reserved                                        
**
**---------------------------- 文件信息 -----------------------------------------------------------------
** 文   件   名:  DS2788.c
** 创   建   人:  林依刚 
** 版        本:  V2.0
** 最后修改日期:  2009-7-22
** 描        述:  实现通过1-Wire 总线对多节锂离子电池电量管理芯片DS2788的读、写操作，对备用电池的电量进行
**                管理并测试电池的剩余电量。                                        
** 操作系统版本： None RTOS. 
** 
**--------------------------- 修订版本信息 --------------------------------------------------------------
** 修   订   人:  林依刚
** 版        本:  V1.1
** 日　      期:  2009-5-16
** 描　      述:  将V1.0版本(51平台版本),升级到基于ARM7TDMI内核的AT91SAM7X256芯片适用版本,完善程序注释。
**            
**--------------------------- 原始版本信息 --------------------------------------------------------------
** 创   建   人:  林依刚
** 版        本:  V1.0
** 日　      期:  2008-12-5
** 描　      述:  原始版本
**
********************************************************************************************************/
#include"DS2788.h"
                                                    
//--Here define the data to be sent to the parameter EEPROM
#define CONTROL              0     /* Control Register                                                 */
#define AB                   0     /* 电流累计偏移，将任意偏移量引入电流累计过程                       */
#define AC_H                 0     /* 老化容量(AC)，存放额定电池容量用来估算正常情况下电池             */
#define AC_L                 0     /* 容量减少的情况,单位与ACR相同(6.5uV/h)                            */
#define VCHG                 0     /* 充电电压(VCHG)存储充电电压门限值，用来检测完全充满状态           */
#define IMIN                 0     /* 最小充电电流(IMIN)存储充电电流门限值，用来检测完全充满状态       */
#define VAE                  0     /* 有效空电压(VAE),检测有效工作时空电池电量点的电压门限             */
#define IAE                  0     /* 有效空电流(IAE),检测有效工作时空电池电量点的放电电流门限         */
#define AE50                 0     /* Active Empty 50                                                  */
#define RSNSP               50     /* 检流电阻初值,用于计算绝对电量,以1字节电导形式保存,单位西门子     */
#define F50_H             0x70     /* Full 50 MSB,from here to SE1, the data need to set               */
#define F50_L             0x80     /* Full 50 LSB                                                      */
#define FS4                  0     /* Full Segment 4 Slope,地址68h-77h存放电池包参数,可具体修改这些值  */
#define FS3                  0     /* Full Segment 3 Slope                                             */
#define FS2                  0     /* Full Segment 2 Slope                                             */
#define FS1                  0     /* Full Segment 1 Slope                                             */
#define AE4                  0     /* Active Empty(AE) Segment 4 Slope                                 */
#define AE3                  0     /* AE Segment 3 Slope                                               */
#define AE2                  0     /* AE Segment 2 Slope                                               */
#define AE1                  0     /* AE Segment 1 Slope                                               */
#define SE4                  0     /* Standby Empty(SE) Segment 4 Slope                                */
#define SE3                  0     /* SE Segment 3 Slope                                               */
#define SE2                  0     /* SE Segment 2 Slope                                               */
#define SE1                  0     /* SE Segment 1 Slope                                               */
#define RSG_H             0xF4     /* RSGAIN:Sense Register Gain MSB                                   */
#define RSG_L             0x1C     /* RSGAIN:Sense Register Gain LSB                                   */
#define RSTC                 0     /* Sense Register Temp Coefficient 检流电阻温度补偿                 */
#define COB                  0     /* Current Offset Bias                                              */
#define TBP23                0     /* EEPROM address: 0x7c                                             */
#define TBP12              -20     /* EEPROM address: 0x7d                                             */
#define VG_H              0x33     /* VGAIN:Voltage Gain MSB                                           */
#define VG_L              0xF0     /* VGAIN:Voltage Gain LSB                                           */
 
#define AS                0x79     /* Age Scalar 老化系数(AS) 推荐为95%,95/0.78%不需要主机写操作       */

#define RAAC_H_ADDR       0x02     /* the address of remaining active absolute capacity MSB            */
#define RAAC_L_ADDR       0x03     /* the address of remaining active absolute capacity LSB            */
#define RSAC_H_ADDR       0x04     /* the address of remaining standby absolute capacity MSB           */
#define RSAC_L_ADDR       0x05     /* the address of remaining standby absolute capacity LSB           */
#define RARC_ADDR         0x06     /* the address of remaining active relative capacity                */
#define RSRC_ADDR         0x07     /* the address of remaining standby relative capacity               */

#define VOLT_H_ADDR       0x0C     /* the address of voltage register MSB                              */
#define VOLT_L_ADDR       0x0D     /* the address of voltage register LSB                              */
#define ACR_H_ADDR        0x10     /* the address of accumulated current register MSB                  */
#define ACR_L_ADDR        0x11     /* the address of accumulated current register LSB                  */

#define USER_EEPROM_START_ADDR     0x20     /* User EEPROM(0x20-0x2F) start address                    */
#define USER_EEPROM_MAX_SIZE         16     /* User EEPROM(0x20-0x2F) start address                    */


//--设置LSH20电池初始满电量值
// 该电池的标称容量:13.0Ah,考虑到温度、放电速率等因素,设定满电量为9Ah,初始化值为满电量的99.5%即8.955Ah.
// 当Rsns = 20mOhm,ACR的单位为0.3125mAh
#define ACR_H_INIT_VAL    0x6F     /* Initialize value of ACR MSB.                                     */ 
#define ACR_L_INIT_VAL    0xF0     /* Initialize value of ACR LSB.                                     */ 


// 全局变量定义   
BATTERY_INFO_TYPE         g_BatInfo = {DS2788_INIT_FLAG, 0,0, 0,0,0,0, ACR_H_INIT_VAL,ACR_L_INIT_VAL};


/*********************************************************************************************************
** 函数名称： inp
** 函数功能： Get the port value  
** 入口参数： port :  the offset of pin in the bank.
** 出口参数： the current status of the pin.
********************************************************************************************************/
int inp(unsigned int port)
{   
    int result; 
    result = (*AT91C_PIOA_PDSR) & port; 
    result >>= 2;
    return(result);	
}

/********************************************************************************************************
** 函数名称： outp
** 函数功能： set the states of the port.
** 入口参数： port:     the offset of pin in the bank;
**            datatype: the value of port nead setting.
** 出口参数： None
********************************************************************************************************/
int outp(unsigned int port, int databyte)  
{  
    if(databyte)
        *AT91C_PIOA_SODR = port;
    else
        *AT91C_PIOA_CODR = port;
  	
    return(1);
}

/********************************************************************************************************
** 函数名称： TC2_Init
** 函数功能： Used to create 0.25us time tick.
** 入口参数： None
** 出口参数： None
********************************************************************************************************/
void TC2_Init(void)  
{   
    *AT91C_PMC_PCER = 1 << AT91C_ID_TC2;                    /* Enable the clock of the TC2.            */
  
    *AT91C_TCB_BMR &= (~(0x00000003<<4));                   /* Clear the bit4,5 of Block Mode Register.*/
    *AT91C_TCB_BMR |= AT91C_TCB_TC2XC2S_NONE;               /* BMR's external clock single 2 select is none.*/

    *AT91C_TC2_CMR = AT91C_TC_CLKS_TIMER_DIV1_CLOCK |       /* clk = MCK/2 = 48MHz/2                   */
                     AT91C_TC_EEVT_XC1 |                    /* Signal as ext event: XC0 TIOB: output   */
                     AT91C_TC_WAVESEL_UP_AUTO |             /* UP mode with auto trigger on RC Compare */
                     AT91C_TC_WAVE |                        /* Waveform mode                           */
                     AT91C_TC_ACPA_SET |
                     AT91C_TC_BCPB_SET |
                     AT91C_TC_ACPC_CLEAR |  
                     AT91C_TC_BCPC_CLEAR; 
                                    
    *AT91C_TC2_RA = 0x03;                                   /* Hign level is 0.125us                   */
    *AT91C_TC2_RB = 0x03;                                   /* Low level is 0.125us                    */
    *AT91C_TC2_RC = 0x06;                                   /* Period is: 1/4us                        */
}

/*********************************************************************************************************
** 函数名称： tickDelay
** 函数功能： Used to delay tick*0.25 us
** 入口参数： tick:  the number of tick
** 出口参数： None
********************************************************************************************************/
void tickDelay(int tick)
{
    *AT91C_TC2_CCR = AT91C_TC_SWTRG;                        /* Soft trigger the TC reset.              */
    *AT91C_TC2_RC  = 0x06 * tick;                           /* Period is tick*1/4 us.                  */
    *AT91C_TC2_CCR = AT91C_TC_SWTRG|AT91C_TC_CLKEN;         /* Soft trigger the TC reset and start TC. */
    while(!(*AT91C_TC2_SR & 0x10));  	                    /* Wait until RC compare.                  */
}

/********************************************************************************************************
** 函数名称： DS2788_Init
** 函数功能： Initialize the port(DQ),checking the test point if the ic have been written,return directly, 
**            otherwise,write the battery parameter into the parameter eeprom.
** 入口参数： None
** 出口参数： if operate succeess return(0),otherwise return(1)
********************************************************************************************************/
unsigned char DS2788_Init(void)
{	
    unsigned char  temp = 0;
    
    SetSpeed(1);                                            /* Set the 1-Wire timing to 'standard'.    */	
    TC2_Init();                                             /* Used to creat a 1/4uS tick.             */
	                         
    // Assign and configure the pin for 1-wire communication
    *AT91C_PIOA_PER   = PORTADDRESS;
    *AT91C_PIOA_OER   = PORTADDRESS;
    *AT91C_PIOA_OWER  = PORTADDRESS;
    *AT91C_PIOA_IFER  = PORTADDRESS;
    *AT91C_PIOA_PPUER = PORTADDRESS;
    *AT91C_PIOA_MDER  = PORTADDRESS;   
     
    if (OWTouchReset()) 	                                /* Reset the 1-Wire bus                    */ 
        return(1);                                          /* Return if no devices found              */ 
       
    DS2788_RecallData(USER_EEPROM_START_ADDR, sizeof(BATTERY_INFO_TYPE));
    DS2788_ReadData((unsigned char *)&temp, USER_EEPROM_START_ADDR, 1); 	
    if (temp == DS2788_INIT_FLAG)
        return(0);
    else
        return(DS2788_CfgBatInfo());
    
}

/********************************************************************************************************
** 函数名称： DS2788_CfgBatInfo
** 函数功能： Configure the internal register and work parameter of the DS2788
** 入口参数： None
** 出口参数： if operate succeess return(0),otherwise return(1)
********************************************************************************************************/    
unsigned char DS2788_CfgBatInfo(void) 
{   	    
    if (OWTouchReset()) 	                                /* Reset the 1-Wire bus                    */ 
        return(1);                                          /* Return if no devices found              */ 
        
    DS2788_WriteData((unsigned char *)&g_BatInfo, USER_EEPROM_START_ADDR, sizeof(BATTERY_INFO_TYPE));
    DS2788_CopyData(USER_EEPROM_START_ADDR, sizeof(BATTERY_INFO_TYPE));	
    DS2788_WriteData(g_BatInfo.m_InitCap, ACR_H_ADDR, 2);   /* Initilize ACR                           */
    DS2788_CopyData(ACR_H_ADDR, 2);
    
    // Write the parameter of the battery cell
    OWWriteByte(CMD_SKIP_NEST_ADDR);
    OWWriteByte(CMD_WRITE_DATA);
    OWWriteByte(0x61);
    OWWriteByte(AB);
    OWWriteByte(AC_H);
    OWWriteByte(AC_L);
    OWWriteByte(VCHG);
    OWWriteByte(IMIN);
    OWWriteByte(VAE);
    OWWriteByte(IAE);    
    OWWriteByte(AE50);
    OWWriteByte(RSNSP);
    OWWriteByte(F50_H);
    OWWriteByte(F50_L);
    OWWriteByte(FS4);
    OWWriteByte(FS3);
    OWWriteByte(FS2);
    OWWriteByte(FS1);    
    OWWriteByte(AE4);
    OWWriteByte(AE3);
    OWWriteByte(AE2); 
    OWWriteByte(AE1);
    OWWriteByte(SE4);
    OWWriteByte(SE3);
    OWWriteByte(SE2); 
    OWWriteByte(SE1);
    DS2788_CopyData(0x61, 23);
    
    OWWriteByte(CMD_SKIP_NEST_ADDR);
    OWWriteByte(CMD_WRITE_DATA);
    OWWriteByte(0x7A);
    OWWriteByte(RSTC);  
    OWWriteByte(COB);
    OWWriteByte(TBP12);   
    OWWriteByte(TBP23);
    DS2788_CopyData(0x7A, 4);
    	
    return(OWTouchReset());                                 // do a 1-Wire reset and return presence result 
}

/********************************************************************************************************
** 函数名称： CfgBatIDAndDate
** 函数功能： Configure the battery id and date
** 入口参数： None
** 出口参数： if operate succeess return(0),otherwise return(1)
********************************************************************************************************/    
unsigned char DS2788_CfgBatIDAndDate(void)
{   	    
    if (OWTouchReset()) 	                                /* Reset the 1-Wire bus                    */ 
        return(1);                                          /* Return if no devices found              */ 
        
    DS2788_WriteData((unsigned char *)g_BatInfo.m_BatID, USER_EEPROM_START_ADDR+1, 6);
    DS2788_CopyData(USER_EEPROM_START_ADDR+1, 6);	
    return(OWTouchReset());                                 // do a 1-Wire reset and return presence result 
}

/********************************************************************************************************
** 函数名称： DS2788_ReadData
** 函数功能： Read the needed information from DS2788
** 入口参数： pdest :  the buffer pointer used to save the read data;
**            addr  :  the start address nead reading;
**            size  :  the length nead reading.
** 出口参数： if operate succeed return 0, otherwise return 1
********************************************************************************************************/
unsigned char DS2788_ReadData(unsigned char *pdest,unsigned char addr,unsigned char size)
{
    unsigned char loop;
  
    if (OWTouchReset()) 	                                /* reset the 1-Wire device.                */        
        return(1); 	                                        /* return if none device is existent.      */	  
    OWWriteByte(CMD_SKIP_NEST_ADDR);                        /* send net address command.               */                       
    OWWriteByte(CMD_READ_DATA);                             /* send read data command.                 */                       
    OWWriteByte(addr);                                      /* send the first address nead to read.    */
    for (loop = 0; loop < size; loop++)
        *(pdest+loop) = OWReadByte();                       /* Continous read RAM of DS2788.           */	  		  	
    return(OWTouchReset());                                 // do a 1-Wire reset and return presence result  	
}
/********************************************************************************************************
** 函数名称： DS2788_WriteData
** 函数功能： Write data to the ram of DS2788.
** 入口参数： psrc :  the first pointer of data neading to be written;
**            addr :  the start address of ram;
**            size :  the length nead to be Written.
** 出口参数： if operate succeed return 0, otherwise return 1
********************************************************************************************************/
unsigned char DS2788_WriteData(unsigned char *psrc,unsigned char addr,unsigned char size)
{
    unsigned char loop;
  
    if (OWTouchReset()) 	                                    
        return(1);
    OWWriteByte(CMD_SKIP_NEST_ADDR);                          
    OWWriteByte(CMD_WRITE_DATA);                               
    OWWriteByte(addr);                                      /* Write the first address nead reading.   */
    for (loop = 0; loop < size; loop++)
        OWWriteByte(*(psrc + loop));                        /* Continous Write data to the RAM.        */        	  		  	
    return(OWTouchReset());                                 // do a 1-Wire reset and return presence result      
}
/********************************************************************************************************
** 函数名称： DS2788_RecallData
** 函数功能： Copy the data from eeprom to ram.
** 入口参数： addr :  the start address for operate;
**            size :  the length nead to recalled.
** 出口参数： if operate succeed return 0, otherwise return 1
********************************************************************************************************/
unsigned char DS2788_RecallData(unsigned char addr,unsigned char size)
{
    unsigned char loop;
  
    if (OWTouchReset()) 	                                    
        return(1);
    OWWriteByte(CMD_SKIP_NEST_ADDR);                          
    OWWriteByte(CMD_RECALL_DATA);                              
    for (loop = 0; loop < size; loop++)
        OWWriteByte(addr + loop);                           /* Recall data from the eeprom to RAM .    */        	  		  	
    return(OWTouchReset());                                 // do a 1-Wire reset and return presence result      
}
/********************************************************************************************************
** 函数名称： DS2788_CopyData
** 函数功能： Copy the data from RAM to eeprom.
** 入口参数： addr :  the start address nead copying;
**            size :  the length nead to be copyed.
** 出口参数： if operate succeed return 0, otherwise return 1
********************************************************************************************************/
unsigned char DS2788_CopyData(unsigned char addr,unsigned char size)
{
    unsigned char loop;
  
    if (OWTouchReset())
        return(1);
    OWWriteByte(CMD_SKIP_NEST_ADDR);                          
    OWWriteByte(CMD_COPY_DATA);                              
    for (loop = 0; loop < size; loop++)
        OWWriteByte(addr + loop);                           /* copy data from the RAM to eeprom.       */
    return(OWTouchReset());                                 // do a 1-Wire reset and return presence result
}

/********************************************************************************************************
** 函数名称： DS2788_BatMeas
** 函数功能： pdest  电池剩余电量测量结果保存位置的首地址
** 入口参数： None
** 出口参数： None
********************************************************************************************************/
void DS2788_BatMeas(unsigned char *pdest)
{
    unsigned char result = 0;
	
	
	if(pdest == NULL)
		return;
		
    // 读取电池组满电量数据初始化时间（年、月、日）   
    DS2788_RecallData(USER_EEPROM_START_ADDR, sizeof(BATTERY_INFO_TYPE));
    DS2788_ReadData((unsigned char *)&g_BatInfo, USER_EEPROM_START_ADDR, sizeof(BATTERY_INFO_TYPE)); 
    memcpy((unsigned char *)pdest, (unsigned char *)g_BatInfo.m_InitDate, 4);	
    	
	// 读取ACR寄存器高8位的值（低8位绝对数据值小，在此忽略不计）
    DS2788_ReadData(&result, ACR_H_ADDR, 1);
    
    // 复制ACR寄存器的值至EEPROM，为提高测量精度，高、低8位均保存
    DS2788_CopyData(ACR_H_ADDR, 2);
    
    // 计算相对于满电量时，剩余有效电量的百分比	
    result = (result * 100) /(int)F50_H;
    if (result > 100)
        result = 100;
    
    *(pdest+4) = result;          
}