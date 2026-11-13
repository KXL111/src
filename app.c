/********************************************************************************************************
**                            (c) Copyright  苏州市江海通讯发展实业有限公司
**                                      All Rights Reserved
**
**---------------------------- 文件信息 -----------------------------------------------------------------
** 修   定   人:  林依刚
** 版        本:  V2.1
** 最后修改日期:  2010-11-8
** 修改内容    :  根据海上试验实际情况，修改卫星报警流程。启动新一轮报警后，设备需重新定位获取位置信息。
**                适用"一体化指挥平台网遇险报通信协议V1.2"（V2.0适用"一体化指挥平台网遇险报通信协议V1.1"）。
**                改进卫星模块功率测试部分。复位后首次功率测试时，发送用户指令使卫星模块进入测试模式。复位
**                前，所有后续测试将不再用户指令。
**---------------------------- 历史版本 -----------------------------------------------------------------
** 修   定   人:  林依刚
** 版        本:  V2.0
** 最后修改日期:  2010-5-7
** 修改内容    :  根据要求调整V1.3版本电池电量检测部分;软件定型测试完成,统一将软件版本升级至V2.0。
                                    
**---------------------------- 历史版本 -----------------------------------------------------------------
** 修   定   人:  林依刚
** 版        本:  V1.3
** 最后修改日期:  2010-3-22
** 修改内容    :  增加北斗卫星模块及SIM卡自毁功能; 适用"一体化指挥平台网遇险报通信协议-V1.1"。
                                    
**---------------------------- 历史版本 -----------------------------------------------------------------
** 修   定   人:  林依刚
** 版        本:  V1.2
** 最后修改日期:  2009-12-28
** 修改内容    :  增加卫星授时功能；增加软件自身擦除功能；
                  改进DSC报警处理程序结构，在DSC报警期间除能系统所有中断，报警信息发送完成后再使能相关中断。

**---------------------------- 历史版本 -----------------------------------------------------------------
** 修   定   人:  林依刚
** 版        本:  V1.1
** 最后修改日期:  2009-9-26
** 修改内容    :  增加电池电量检测功能； 针对闪光等硬件设计的改动，更改闪光灯控制方式；                                                     
                  将软件升级至兼容"北斗一号用户机数据接口要求"V2.1、3.0、4.0版本(需要修改app_cfg.h文件中的
                  "北斗一号用户机数据接口要求"版本配置宏(BD_PROTOCOL_VERSION，并重新编译、下载程序)。
                  适用与正样机艇载设备内部通信协议。          
                              
**---------------------------- 历史版本 -----------------------------------------------------------------
** 文   件   名:  app.c
** 创   建   人:  林依刚
** 版        本:  V1.0
** 最后修改日期:  2008-5-11
** 描        述:  艇载设备浮标内控制单元软件。
                  实现对“北斗一号”卫星用户机模块、VHF话音通信模块、9GHz雷达应答模块的控制，以及闪光灯控制、
                  工厂测试 (卫星模块误码测试、卫星发送功率测试、DSC信号检测)、系统自检等功能。                                    
                  北斗卫星:  适用于北斗一号3.0通信协议，实现功率检测、通信、定位、报警等功能。                              
                  VHF模块 ： 实现VHF模块自检、DSC信号检测、DSC报警、话音通话时工作频率与收/发控制；
                             控制话音通话与DSC报警的切换开关。                              
                  9GHz模块： 具有自检和自动应答功能
                             接收到终端发出的自检命令后，向模块发送自检信号，判断其是否处于正常工作状态下,
                             并向终端报告自检情况;
                             检测设备是否被雷达扫描到，并发送相关信息给终端设备，指示9G模块的应答状态.
                              
** 操作系统版本:  μC/OS-II V2.76
********************************************************************************************************/

#include "app.h"
 

/********************************************************************************************************
*                                  EXTERN GLOBAL VARIABLES DEFINE
********************************************************************************************************/
volatile  SYS_FLAG_TYPE   g_Flg;                  // 软件运行相关标志位
                 
/********************************************************************************************************
*                                  STATIC GLOBAL VARIABLES DEFINE
********************************************************************************************************/
static  SYS_INIT_TYPE	  g_SysInit;              // 系统初始化信息

#if BD1_PROTOCOL_VERSION >= 4.0
static  BD1S_XTZJ      g_XTZJ        = {'$','X','T','Z','J',0,BD1_XTZJ_TOTAL_LEN,0,0,0,0,0,0};
#else  
static  BD1S_GLJC      g_GLJC        = {'$','G','L','J','C',0,BD1_GLJC_TOTAL_LEN,0,0,0,0,0};
#endif

static  BD1S_TXSQ      g_TXSQ        = {'$','T','X','S','Q',};  
static  BD1S_DWXX      g_DWXX        = {'$','D','W','X','X',}; 
static  OS_EVENT      *g_pMboxFkxx;               // 北斗反馈消息邮箱
 
 // 人工输入位置信息，初始值为默认无时间、位置信息：时间相关字节全为88，象限为0，经纬度相关字节全为99。
static  ALM_MSG_POS_TYPE   g_ManPos      = {{88,88,88,88}, 0, {99,99,99,99}, {99,99,99,99}};              

static  INT32U            g_ErrBits     = 0;      // 误码测试，错误比特数
static  INT32U            g_TotBits     = 0;      // 误码测试，测试总比特数

//SPI0访问阻塞句柄
OS_EVENT  *hSPI0;

OS_EVENT     *MIYAO_TASK ; 
OS_EVENT     *StarControl_TASK ; 
                                          
/********************************************************************************************************
*                                  STATIC FUNCTION PROTOTYPES
********************************************************************************************************/
static  INT8U   GetInitData(void);
static  INT8U   GetChecksum(const INT8U *psrc, INT32U len);
static  INT32U  GetStringLength(INT8U *pstr);
static  void    NGModuleInit(void);
static  INT8U   NGModuleCheck(void);
static  void    PIOB_IntHandler(void);

static  INT8U   StarDWSQ(INT8U type);
static  INT8U   StarSJSC(INT16U out_freq);
static  INT8U   StarICJC(INT8U frame_nbr);
static  INT8U   StarZHSQ(void);
static INT8U  StarKJBG(void);
static INT8U  StarGJBG(void);
static INT8U  StarJMSQ(void);
static INT8U  StarJMJC(void);
static  INT8U   StarJSZL(void);
static  INT8U   StarYHZL(void);
static  INT8U	StarSXCX(void);
static INT8U StarZHSQBD(void);
static  INT8U   StarLoopTest(BD1S_TXSQ *ptxsq);
static void    StarBitErrTest(INT8U iq, INT8U wave_nbr, INT8U min);

static  void    VhfModuleInit(void);
static  INT8U   VhfModuleCheck(void);
static  void    TimeDly(INT16U cnt);
static  void    TC1_Init(void);
static  INT8U   VhfChngCh(INT8U ch_nbr);
static  void    FskInit(void);
static  void    FskSetMode(INT8U mode);
static  INT16U  DscHorCheck(INT8U ch);
static  INT16U  DscVertCheck(const INT16U *psrc, INT16U len);
static  void    DscListCreate(INT16U *pdest);
static  void    DscTest(void);
static  void    TC0_Init(void);
static  INT8U   FlashEraseAll(void);


static  void   AMInit(void);
static  INT8U  AMCheck(void);
static  INT8U  AMSetPLL(INT8U ch_nbr);

#define BOARD_MCK             ((18432000*73/14)/2)     /* Output PLL Clock 48MHz                       */ 

unsigned char IsMiYaoWoShou = 0;				  // 初始化密钥握手标志位为0 若为1则表示已经密钥握手成功
unsigned char IsMiYaoZuanFaStart = 0;		      // 初始化密钥转发标志位为0 若为1则表示已经密钥转发开始
unsigned char NG_Work_Status = 0;			      //9GHz模块工作状态标志位，默认0为打开，1为关闭
//OS_EVENT			*Star_Err_Bit_Test;	   // 卫星误码率测试信号量
/********************************************************************************************************
* Function Name : CommInit
*
* Description   : Initialize all of the communication modules,including pin,flags,drivers and state etc.
*
* Arguments     : none
*                   
* Returns       : none
********************************************************************************************************/
void  CommInit(void)
{	
    INT8U      err;
	OS_CPU_SR  cpu_sr;
		        
    BSP_Init(); 
    
    // Set the μC/OS-II system time.
    OSTimeSet(0);
    
    g_Flg.m_WorkMode    = (*AT91C_PIOA_PDSR & SYS_WORK_MODE1) ? SYS_WORK_MODE_37 : SYS_WORK_MODE_35;
  	g_Flg.m_LampMode    = 0x01;                             /* 设定默认闪光灯工作模式(10S闪烁两次)     */
  	g_Flg.m_SigDetectEn = 1;                                /* 使能周期性卫星信号检测功能              */
    
    g_pMboxFkxx   = OSMboxCreate(NULL);                     /* 创建消息邮箱，接收卫通反馈信息的指针    */
    InitCycBuf();
    
    MIYAO_TASK = OSSemCreate(0); 
    StarControl_TASK = OSSemCreate(0); 
    
    NGModuleInit();
    VhfModuleInit();
    AMInit();
         
    OS_ENTER_CRITICAL();
    DS2788_Init();
    OS_EXIT_CRITICAL();
        
    hSPI0 = OSMutexCreate(13, &err);                        // 为SPI接口操作创建互斥型信号量
    //Star_Err_Bit_Test = OSSemCreate(0);
    SPI_Init(0);                                            // 初始化SPI接口0
    DF_CtrlCfg();                                           // DataFlash芯片控制初始化
    
    // 从DATAFLASH读取初始化数据，保存至全局变量g_SysInit
    GetInitData();

    // Configure and enable PIOB interrupt.
    AT91F_AIC_ConfigureIt( AT91C_BASE_AIC, AT91C_ID_PIOB, PIOB_INT_PRIO, 
                           AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL, PIOB_IntHandler);                            
    *AT91C_AIC_ICCR = 1 << AT91C_ID_PIOB;                   /* Clear PIOB interrupt flag after reset.  */
    AT91F_AIC_EnableIt (AT91C_BASE_AIC, AT91C_ID_PIOB);     /* Enable the PIOB interrupt.              */
           
#if BD1_PROTOCOL_VERSION >= 4.0
    OSTimeDlyHMSM(0, 0, 2, 0);                              /* 等待卫星模块工作稳定，否则ICXX内容不全  */ 
    StarICJC(0);	                                        /* 获取卫星模块ICXX,用户ID及通信时间频度等 */
    OSTimeDlyHMSM(0, 0, 0, DLY_MS_MATCH_STAR); 
    g_XTZJ.checksum = GetChecksum((INT8U *)&g_XTZJ, BD1_XTZJ_TOTAL_LEN - 1);
#else              
    memcpy(g_GLJC.user_id, g_SysInit.user_id, 3);           /* 更新功率检测全局变量(g_GLJC)            */
    g_GLJC.checksum = GetChecksum((INT8U *)&g_GLJC, BD1_GLJC_TOTAL_LEN - 1);
#endif
}

/********************************************************************************************************
* Function Name : GetInitData
*
* Description   : Get the initialize data from dataflash.
*
* Arguments     : none
*                   
* Returns       : 0: no error; 1: do forthly read; 2: resume default value.                    
********************************************************************************************************/
static  INT8U  GetInitData(void)
{
    INT8U  i;
    INT8U  len = sizeof(g_SysInit);
    INT8U  buf1[sizeof(g_SysInit)];
    INT8U  buf2[sizeof(g_SysInit)];
    INT8U  buf3[sizeof(g_SysInit)];
    INT8U *ptr = (INT8U *)&g_SysInit;
    INT8U  reti = 0;
    
    // 从DATAFLASH读取初始化数据及通信记录索引
    DF_ContinuousRead(MEM_PA_INIT_DATA, MEM_BA_INIT_DATA, buf1, sizeof(g_SysInit));
    DF_ContinuousRead(MEM_PA_INIT_DATA, MEM_BA_INIT_DATA, buf2, sizeof(g_SysInit));
    DF_ContinuousRead(MEM_PA_INIT_DATA, MEM_BA_INIT_DATA, buf3, sizeof(g_SysInit));
    
    for (i=0; i< len; i++)
    {
    	if (buf1[i] == buf2[i])
    		*(ptr+i) = buf1[i];
    	else if (buf1[i] == buf3[i])
    		*(ptr+i) = buf1[i];
    	else if (buf2[i] == buf3[i])
    		*(ptr+i) = buf2[i];
    	else
        {
        	DF_ContinuousRead(MEM_PA_INIT_DATA, MEM_BA_INIT_DATA, (INT8U *)&g_SysInit, sizeof(g_SysInit));
        	reti = 1;
        	break;
        }
    }
    
    if ((g_SysInit.bank_id[0] == 0xff) && (g_SysInit.bank_id[1] == 0xff) && (g_SysInit.bank_id[2] == 0xff))
    {
    	g_SysInit.ship_id[0] = 0x29;
    	g_SysInit.ship_id[1] = 0x14;
    	g_SysInit.ship_id[2] = 0x0B;
    	g_SysInit.ship_id[3] = 0x0B;
    	g_SysInit.ship_id[4] = 0x0A;
    	
    	g_SysInit.distress_kind = 107;
    	
    	g_SysInit.user_id[0] = 0x00;
    	g_SysInit.user_id[1] = 0x00;
    	g_SysInit.user_id[2] = 0x00;
    	
    	g_SysInit.bank_id[0] = 0x03;
    	g_SysInit.bank_id[1] = 0xD0;
    	g_SysInit.bank_id[2] = 0xEE;
    	
    	g_SysInit.interval[0] = 0;
    	g_SysInit.interval[1] = 60;
   	    
   	    g_SysInit.term_time[0] = BspTime.year >> 8;
   	    g_SysInit.term_time[1] = BspTime.year & 0xff;
        memcpy((INT8U *)&g_SysInit.term_time[2], (INT8U *)&BspTime.month, 6);

    	DF_PageWrite(MEM_PA_INIT_DATA, MEM_BA_INIT_DATA, (INT8U *)&g_SysInit, sizeof(g_SysInit));
        reti = 2;
    }
    return (reti);
}

/********************************************************************************************************
* Function Name : GetChecksum
*
* Description   : checksummat for a string.
*
* Arguments     : psrc      the first pointer neading to check.
*                 len       the length neading to check. 
*      
* Returns       : result    the result of checksum.
********************************************************************************************************/
static  INT8U  GetChecksum(const INT8U *psrc, INT32U len)
{
    INT8U   result = 0;
    INT32U  offset;
    
    if ((psrc != NULL) && (len > 0))
    { 
        for (offset = 0; offset < len; offset++)
            result ^= *(psrc + offset);
    }
    return (result);
}

/********************************************************************************************************
** 函数名称： GetStringLength
** 函数功能： 计算字符串的长度，即从首地址至结束符'\0'之间的字符的个数。
** 入口参数： pstr  字符串的首地址
** 出口参数： len   字符串长度
********************************************************************************************************/
static INT32U GetStringLength(INT8U *pstr)
{
    INT32U  len = 0;
    
    if (pstr == NULL)
        return (len);
    
    while(*(pstr + len) != '\0')
    {    
        len++;
        if (len == 0xffffffff)
            break;     
    }
        
    return (len);
}

/********************************************************************************************************
* Function Name : TermCommRece
*
* Description   : Receive and decode the information from terminal,control the modules.
*
* Arguments     : none
*                   
* Returns       : 0   the terminal data is ok
*                 1   the terminal data is error
********************************************************************************************************/
INT8U  TermCommRece(void)
{	
    INT8U           i;
    INT8U           result       = 0;	          /* saved the temporary compute result                */
    OS_CPU_SR       cpu_sr;
    TERM_COMM_TYPE  term_rx;                      /* the received buffer structure                     */                  
    INT8U          *ptr;
    INT8U           u8_temp      = 0;	          /* the temporary variable                            */
    
   	INT16U          year;
    INT8U           month;
    INT8U           date;
    	
    // Maxium days per month for leap year and normal year 
    INT8U const     MDPMForLeapYear[12] = {31,29,31,30,31,30,31,31,30,31,30,31};
    INT8U const     MDPMForNormYear[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
        
 	memset((INT8U *)&term_rx, 0x00, sizeof(TERM_COMM_TYPE));     /* 清零数据接收缓冲区.                */  

	//终端
    UsartRead(0, (INT8U *)&term_rx, sizeof(TERM_COMM_TYPE), 0);  /* receive one frame from terminal.   */   
      
    //新增密钥控制程序--------宋玉祥--------//
	if( (IsMiYaoWoShou == 1) && (IsMiYaoZuanFaStart == 1))
	{
		//如果已经开启了密钥注入，本程序只负责转发数据，即将从终端接收到的数据原封不动地转发给卫星
		UsartWrite(1, (INT8U *)&term_rx, sizeof(TERM_COMM_TYPE), 0);
	}else{
	//新增密钥控制程序--------宋玉祥--------//
    if ((term_rx.header != TERM_FR_HEADER) || (term_rx.len < 4) || (term_rx.len > sizeof(TERM_COMM_TYPE))) 
    {	
        TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
        return (1);
    }
    result = GetChecksum((INT8U *)&term_rx, term_rx.len - 1);    /* compute the checksum.              */
    if (result != *((INT8U *)&term_rx + term_rx.len - 1)) 
    {	
        TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
        return (1);
    }	   			
  
    switch(term_rx.type)                                    /* distinguish the received frame type.    */
    {    
		//新增密钥控制程序--------宋玉祥--------//
    	case 0x99:
			IsMiYaoWoShou = 1;								// 开启密钥握手
			TermCommSend(MIYAOWSHF,(INT8U *)0, 0);			// 密钥握手回复
			if( (IsMiYaoWoShou == 1) && (IsMiYaoZuanFaStart == 1))
			{
				OSTimeDlyHMSM(0, 0, 0, 200);	//延时200MS
				
				OSSemPost(MIYAO_TASK);		//释放密钥注入任务信号量
				
			}
			break;

		case 0x9b:
			IsMiYaoZuanFaStart = 1;							// 开始密钥转发
			TermCommSend(MIYAOZFSTART,(INT8U *)0, 0);	    // 密钥转发回复
			if( (IsMiYaoWoShou == 1) && (IsMiYaoZuanFaStart == 1))
			{
				
				OSTimeDlyHMSM(0, 0, 0, 200);	//延时200MS
				
				OSSemPost(MIYAO_TASK);		//释放密钥注入任务信号量
				
			}
			break;
	
  	    case T2B_CMD_WSZL:                                  /* handshake code.                         */
  	    	if( term_rx.len != 0x04 )
    		{
    			TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
    		}else{
	  	    	OSTimeDlyHMSM(0, 0, 0, DLY_MS_MATCH_TERM);
	  	    	TermCommSend(B2T_CMD_WSHF, (INT8U *)0, 0);
	  	    }
  	        break;
                        
  	    case T2B_CMD_CSXX:                                  /* initilize data                          */
    		if( term_rx.len != 0x1a )
			{
				TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
			}else{
	   	        BspTime.year = (g_SysInit.term_time[0]<<8) + g_SysInit.term_time[1];
	   	        memcpy((INT8U *)&BspTime.month, (INT8U *)&g_SysInit.term_time[2], 6);  	                    
	  	        
	  	        TermCommSend(B2T_CMD_ACK, (INT8U *)0, 0);
				
	  	        ptr=(INT8U *)&g_SysInit;
	  	        u8_temp = sizeof(SYS_INIT_TYPE);
	  	        u8_temp = u8_temp - 8;
	            for (i=0; i<u8_temp; i++)
	            {
	                if (*(ptr+i) != term_rx.msg[i])
	                {
	                    memcpy((INT8U *)&g_SysInit, (INT8U *)term_rx.msg, sizeof(SYS_INIT_TYPE));
	                    DF_PageWrite(MEM_PA_INIT_DATA, MEM_BA_INIT_DATA, (INT8U *)&g_SysInit, sizeof(SYS_INIT_TYPE));
	                    break; 
	                }
	            }
				
	#if BD1_PROTOCOL_VERSION >= 4.0
	            OSTimeDlyHMSM(0, 0, 0, DLY_MS_MATCH_TERM);
	            StarICJC(0);
	            OSTimeDlyHMSM(0, 0, 1, 0); 
	            StarICJC(0);
	            OSTimeDlyHMSM(0, 0, 1, DLY_MS_MATCH_TERM);          
	            memcpy(g_XTZJ.user_id, g_SysInit.user_id, 3);   /*renovate the $XTZJ                       */
	            g_XTZJ.checksum = GetChecksum((INT8U *)&g_XTZJ, BD1_XTZJ_TOTAL_LEN - 1);
	#else              
	            memcpy(g_GLJC.user_id, g_SysInit.user_id, 3);   /*renovate the $GLJC                       */
	            g_GLJC.checksum = GetChecksum((INT8U *)&g_GLJC, BD1_GLJC_TOTAL_LEN - 1);
	#endif         	    	          	                 
			}
            break; 
             
  	    case T2B_CMD_ZJZL:                                  /* Start all of the modules self_check.    */
    		if( term_rx.len != 0x04 )
			{
				TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
			}else{
	  	        TermCommSend(B2T_CMD_ACK,(INT8U *)0, 0); 		                                                       
	            StarSigDetect();
	            OSTimeDlyHMSM(0, 0, 0, DLY_MS_MATCH_STAR);
	            result = (g_Flg.m_WaveValid ? 0x00 : 0x01);  
	            if (!VhfModuleCheck())
	      	        result |= 0x01 << 1;   
	      	        
	            if (!NGModuleCheck())
	            {
	      	        result |= 0x01 << 2;
	      	    }
	      	    if (!AMCheck())
	      	    	result |= 0x01 << 3;	      	
	            TermCommSend(B2T_CMD_ZJXX, &result, 1);
	        }
            break;                                                                              
  		  	   	    	
  	    case T2B_CMD_DCJC:                                  /* Coulometry for the battery.             */
    		if( term_rx.len != 0x04 )
			{
				TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
			}else{
	            TermCommSend(B2T_CMD_ACK,(INT8U *)0, 0);
	            OS_ENTER_CRITICAL();
	            DS2788_BatMeas((INT8U *)&term_rx);
	            OS_EXIT_CRITICAL();
	            OSTimeDlyHMSM(0, 0, 1, 0);
	            TermCommSend(B2T_CMD_DCXX, (INT8U *)&term_rx, 5);
	        }
            break;
	    	        
  	    case T2B_CMD_WORK_MODE:                             /* work model selecting.                   */
    		if( term_rx.len != 0x05 )
			{
				TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
			}else{
	            if (term_rx.msg[0] == 0x00)                     /* Only star module is working.            */
	            {     
	                VhfPwrDis();
	                NGRespDis();
	                NG_Work_Status = 1;
	            }
	  		    	else if (term_rx.msg[0] == 0x01)                /* All of the modules are working.         */
	            {       
	                VhfPwrEn();
	                AM_SEL_DIS();
	                VHF_PTT_DIS();
	                VhfChngCh(16);
	                
	                NGRespEn();
	                NG_Work_Status = 0;
	                
	                g_Flg.m_LampMode  = 0x01;
	                OSTimeDlyResume(LAMP_TASK_PRIO);
	                OSTaskResume(LAMP_TASK_PRIO);
	            }
		  		    else
		  		    {                                               /* Other byte return error.                */
	                TermCommSend(B2T_CMD_NAK,(INT8U *)0, 0); 
	  			    		break;
	  					}
	            TermCommSend(B2T_CMD_ACK,(INT8U *)0, 0);
	        }
            break;
  		  	    	
  	    case T2B_CMD_BD1_SW:                                /* control the state of star module.       */
  	    	if( term_rx.len != 0x05 )
			{
				TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
			}else{
	  		    if (term_rx.msg[0] == 0x00) 
	  			    OSTaskResume(STAR_RECE_TASK_PRIO);
	  		    else if (term_rx.msg[0] == 0x01) 
	  			    OSTaskSuspend(STAR_RECE_TASK_PRIO);
	  		    else
	  		    { 
	  			    TermCommSend(B2T_CMD_NAK,(INT8U *)0, 0);
	  			    break;
	  			}
	  			TermCommSend(B2T_CMD_ACK,(INT8U *)0, 0);
	  		}
  			break;
  			  		 	    
  	    case T2B_CMD_VHF_SW:                                /* control the state of VHF module.        */
  	    	if( term_rx.len != 0x05 )
			{
				TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
			}else{
	  		    if (term_rx.msg[0] == 0x00) 
	  			{
	                VhfPwrEn();
	                AM_SEL_DIS();
	                if (g_Flg.m_NatAlm || g_Flg.m_PassAlm)
	                {
	                    g_Flg.m_DscAlmCtr = 1;                  /* DSC报警次数置1，表示开始新一轮报警      */
	                    OSTimeDlyResume (DSC_ALARM_TASK_PRIO);  /* 取消DSC报警任务延时                     */
	                    OSTaskResume(DSC_ALARM_TASK_PRIO);
	                }   
	  			}
	  		    else if (term_rx.msg[0] == 0x01) 
	  			{    
	                VhfPwrDis();
	                if (g_Flg.m_NatAlm || g_Flg.m_PassAlm)
	                {
	                    OSTimeDlyResume (DSC_ALARM_TASK_PRIO);  /* 取消DSC报警任务延时                     */
	                    OSTaskSuspend(DSC_ALARM_TASK_PRIO);
	                }
	  			}
	  		    else
	  		    { 
	  		    	TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
	  			    break;
	  			}
	  			TermCommSend(B2T_CMD_ACK,(INT8U *)0, 0);
	  		}
  			break;
  		  	    	  		
  	    case T2B_CMD_9G_SW:                                 /* control the 9G module respond or not.   */
  	    	if( term_rx.len != 0x05 )
			{
				TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
			}else{
	  		    if (term_rx.msg[0] == 0x00)
	  		    	{
	  			    NGRespEn();
	  			    NG_Work_Status = 0;
	  			  }
	  		    else if (term_rx.msg[0] == 0x01)
	  		    	{
	  			    NGRespDis();
	  			    NG_Work_Status = 1;
	  			  }
	  		    else 
	  		    {
	  			    TermCommSend(B2T_CMD_NAK,(INT8U *)0, 0);
	  			    break;
	  			}
	   			TermCommSend(B2T_CMD_ACK,(INT8U *)0, 0);
	   		}
  			break;
  		   	    	
  	    case T2B_CMD_VHF_PTT:                               /* Tx or Rx for VHF module.                */
  	    	if( term_rx.len != 0x05 )
			{
				TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
			}else{
	  		    if ((IS_TONE_PWR_ON() == false) || (IS_AM_SEL() == true))
	  		    {
	  			    TermCommSend(B2T_CMD_NAK,(INT8U *)0, 0);
	  			    break;
	  			}
	  		    else if (term_rx.msg[0] == 0x00)
	  		    {
	  		    	NGRespDis();
	  			    VHF_PTT_EN();
	  			  }
	  		    else if (term_rx.msg[0] == 0x01) 
	  		    {
	  		    	if( NG_Work_Status == 0 )				//如果当前9GHz模块是打开着的
	  		    	{
	  		    		NGRespEn();						//恢复到原先的状态--打开
	  		    	}else{
	  		    		NGRespDis();				  //恢复到原先的状态--关闭
	  		    	}
	  			    VHF_PTT_DIS();
	  			  }
	  		    else 
	  		    {
	  			    TermCommSend(B2T_CMD_NAK,(INT8U *)0, 0); 
	  			    break;
	  			}
	  	  	    VhfChngCh(g_Flg.m_ChCur);
	  			TermCommSend(B2T_CMD_ACK, (INT8U *)0, 0);
	  		}
  			break;
  			    	  		
  	    case T2B_CMD_VHF_CH:                                /* Change the channal of the VHF module.   */
  	    	if( term_rx.len != 0x05 )
			{
				TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
			}else{
	  		    if (IS_TONE_PWR_ON() == false)
	  		    {
	  			    TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
	  			    break;
	  			}
	  			else if  (term_rx.msg[0] == 0)                  /* 选择航空模块工作           */ 
	  		    {
	  		    	AM_SEL_EN();
	  		    	OSTimeDlyHMSM(0, 0, 0, DLY_MS_AM_PWR_ON);
	  		    	AM_PTT_DIS();
	  		    	AMSetPLL(0);
	  		    	TermCommSend(B2T_CMD_ACK, (INT8U *)0, 0);
	  			    break;
	  		    }
	  		    else if (term_rx.msg[0] == 70)                  /* 舱内话音通话不能设置在70CH上            */                   
	  		    	TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
	  		    else if (VhfChngCh(term_rx.msg[0])) 
	  		    {    
	  		    	TermCommSend(B2T_CMD_ACK, (INT8U *)0, 0);
	  		    	if (IS_AM_SEL() == true)
	  		    		AM_SEL_DIS();
	  		    }
	  		    else 
	  			    TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0); 
	  		}
  		    break;
  		      	        	       		
  	    case T2B_CMD_SJXX:                                  /* Get terminal system time.               */
  	    	if( term_rx.len != 0x0C )
			{
				TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
			}else{
	            memcpy(g_SysInit.term_time, term_rx.msg, 0x08);
	            g_Flg.m_SysTime = 1;
	            TermCommSend(B2T_CMD_ACK, (INT8U *)0, 0);
	        }
            break;
						  		
  	    case T2B_CMD_WZXX:                                  /* Get terminal maual location.            */	
  	    	if( term_rx.len != 0x11 )
			{
				TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
			}else{	
				memcpy((INT8U *)&g_ManPos, term_rx.msg, sizeof(ALM_MSG_POS_TYPE));						
				g_Flg.m_ManPos = 1;	
				TermCommSend(B2T_CMD_ACK,(INT8U *)0, 0);
			}								
            break;
      	            		
  	    case T2B_CMD_ZHZL:                                  /* Loop test between buoyage and bank.     */
  	    	if( term_rx.len != 0x04 )
			{
				TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
			}else{
	  		    TermCommSend(B2T_CMD_ACK,(INT8U *)0, 0);
		        StarLoopTest(&g_TXSQ);
		    }
	        break;
	            	  	        		
  	    case T2B_CMD_BWZL:                                  /* Send $TXSQ                              */            	          	        
  	        if ((term_rx.msg[0]==0x44 || term_rx.msg[0]==0x46) && (term_rx.len >= 8))
  	        {   
  	        	INT8U  msg_bytes = (term_rx.len-8 >= BD1_MAX_MSG_LEN-2) ? BD1_MAX_MSG_LEN : (term_rx.len-8+2);
  	        	INT16U txsq_len  = BD1_TXSQ_FIXED_LEN + msg_bytes;
                
                TermCommSend(B2T_CMD_ACK,(INT8U *)0, 0);   /* terminal data is ok                     */
  	            memcpy(g_TXSQ.user_id, g_SysInit.user_id, 3);     	              		        
  		        g_TXSQ.len[0]     = txsq_len >> 8;
  		        g_TXSQ.len[1]     = txsq_len;
  		        memcpy(&g_TXSQ.type, term_rx.msg, 4);
  		        g_TXSQ.msg_len[0] = (msg_bytes << 3) >> 8;
  		        g_TXSQ.msg_len[1] = msg_bytes << 3;  		         
 	            g_TXSQ.msg[0]     = ALM_MSG_HEADER0;
                g_TXSQ.msg[1]     = (g_TXSQ.type == 0x44) ? ALM_MSG_HEADER1_CHR : ALM_MSG_HEADER1_CODE;
                memcpy(&g_TXSQ.msg[2], &term_rx.msg[4], msg_bytes - 2);
  	            *(g_TXSQ.msg + msg_bytes) = GetChecksum((INT8U *)&g_TXSQ, txsq_len-1);
		        WrCycBuf((INT8U *)&g_TXSQ, txsq_len);
		    }
		    else		    
		    	TermCommSend(B2T_CMD_NAK,(INT8U *)0, 0);   /* terminal data is error(type faild)      */
            break; 
  		  	    	
  	    case T2B_CMD_DWZL:                                  /* Send orient command($DWSQ)              */
  	    	if( term_rx.len != 0x04 )
			{
				TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
			}else{
	  		    TermCommSend(B2T_CMD_ACK, (INT8U *)0, 0);
	  		    //StarDWSQ(0x04);
	  		    StarDWSQ(0x00);
	  		}
	        break;
	  	    
  	    case T2B_CMD_LAMP_MODE:                             /* Set the working model of lamp           */
  	    	if( term_rx.len != 0x05 )
			{
				TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
			}else{
//	  		    TermCommSend(B2T_CMD_ACK,(INT8U *)0, 0);
	  		    g_Flg.m_LampMode = term_rx.msg[0]; 
	  			OSTimeDlyResume(LAMP_TASK_PRIO);
	  			OSTaskResume(LAMP_TASK_PRIO);
	  		}
	        break;
	    	     
  	    case T2B_CMD_BJZL:                                  /* Startup alarming                        */
  	    	if( term_rx.len != 0x0c )
			{
				TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
			}else{
	  		    TermCommSend(B2T_CMD_ACK,(INT8U *)0, 0);	    		        
		        g_Flg.m_NatAlm   = 1;                           /* Set the native alarm flag               */
		        g_Flg.m_PassAlm  = 0;                           /* Clear the passive alarm flag            */
		        g_Flg.m_BdAlmCtr = 1;                           /* Mean starting a new alarm operation     */
		        g_Flg.m_ExistPos = 0;	                        /* 报警时，重新定位                        */
		        
		        // 更新内部时钟
		        BspTime.year = (term_rx.msg[0]<<8) + term_rx.msg[1];
	   	        memcpy((INT8U *)&BspTime.month, (INT8U *)&term_rx.msg[2], 6);
		                  	        
		        OSTaskResume(STAR_ALARM_TASK_PRIO);	                    
	            if (IS_TONE_PWR_ON())
	            {	
	            	g_Flg.m_DscAlmCtr = 1;                      /* DSC报警次数置1，表示开始新一轮报警      */ 
	            	OSTimeDlyResume (DSC_ALARM_TASK_PRIO);
	                OSTaskResume(DSC_ALARM_TASK_PRIO);
	            }
	        }
            break;
                                    
        case T2B_CMD_YXXZ:                                  /* Set the distress kind.                  */
        	if( term_rx.len != 0x05 )
			{
				TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
			}else{
	            if ((term_rx.msg[0]<100) || (term_rx.msg[0]>108))
	                TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
	            else
	            {     
	                TermCommSend(B2T_CMD_ACK, (INT8U *)0, 0); 
	                if (g_SysInit.distress_kind != term_rx.msg[0])
	                {
	                    g_SysInit.distress_kind = term_rx.msg[0];
	                    DF_PageWrite(MEM_PA_INIT_DATA, MEM_BA_INIT_DATA, (INT8U *)&g_SysInit, sizeof(SYS_INIT_TYPE));
	                }
	            }
	        }
            break;
            
        case T2B_CMD_SSZL: 	                                /* Send($SJSC)                             */
        	if( term_rx.len != 0x04 )
			{
				TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
			}else{
	        	TermCommSend(B2T_CMD_ACK,(INT8U *)0, 0);
	        	StarSJSC(0);
	        }
        	break;
        	
        case T2B_CMD_AM_PTT:                                // 航空模块-PTT控制指令
        	if( term_rx.len != 0x05 )
			{
				TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
			}else{
	  		    if ((IS_TONE_PWR_ON() == false) || (IS_AM_SEL() == false))
	  		    {
	  			    TermCommSend(B2T_CMD_NAK,(INT8U *)0, 0);
	  			    break;
	  			}
	  		    else if (term_rx.msg[0] == 0x00) 
	  			{
	  				NGRespDis();
	  				AM_PTT_EN();
	  	//			OSTimeDlyHMSM(0, 0, 0, DLY_MS_AM_PWR_ON);
	  				AMSetPLL(0);
	  			}    
	  		    else if (term_rx.msg[0] == 0x01) 
	  			{
	  				if( NG_Work_Status == 0 )				//如果当前9GHz模块是打开着的
  		    	{
  		    		NGRespEn();						//恢复到原先的状态--打开
  		    	}else{
  		    		NGRespDis();				  //恢复到原先的状态--关闭
  		    	}
	  				AM_PTT_DIS();
	  	//			OSTimeDlyHMSM(0, 0, 0, DLY_MS_AM_PWR_ON);
	    			AMSetPLL(0);
	  			}
	  		    else 
	  		    {
	  			    TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0); 
	  			    break;
	  			}
	  			TermCommSend(B2T_CMD_ACK,(INT8U *)0, 0);
	  		}
  			break;
  			
        case T2B_CMD_TONE_SWEEP:                            // 扫频控制
        	if( term_rx.len != 0x05 )
			{
				TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
			}else{
	  		    if (IS_TONE_PWR_ON() == false)
	  		    {
	  			    TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
	  			    break;
	  			}
	  		    else if (term_rx.msg[0] == 0x00) 
	  			{
	  				g_Flg.m_Sweep = 1;  				
	  				AM_PTT_DIS();  				
	  				VHF_PTT_DIS();
	  				OSTaskResume(TONE_SWEEP_TASK_PRIO);
	  			}    
	  		    else if (term_rx.msg[0] == 0x01) 
	  			{
	    	        g_Flg.m_Sweep = 0;
	    	        AM_SEL_DIS();
	    	        AM_PTT_DIS();
	    	        VHF_PTT_DIS();
	    	        VhfChngCh(16);
	  			}
	  		    else
	  		    {
	  			    TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0); 
	  			    break;
	  			}
	  			TermCommSend(B2T_CMD_ACK, (INT8U *)0, 0);
	  		}
     	    break;
     	   
     	case T2B_CMD_WLYB:                                  // 无缆预备控制
     		if( term_rx.len != 0x05 )
			{
				TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
			}else{
	  		    if (term_rx.msg[0] == 0x00) 
	  			{
	  				g_Flg.m_PreWlMode = 1;
	  			}    
	  		    else if (term_rx.msg[0] == 0x01) 
	  			{
	  				g_Flg.m_PreWlMode = 0;
	  			}
	  		    else
	  		    {
	  			    TermCommSend(B2T_CMD_NAK,(INT8U *)0, 0); 
	  			    break;
	  			}
	  			TermCommSend(B2T_CMD_ACK,(INT8U *)0, 0);
	  		}
     	    break;
     	
     	case T2B_CMD_AM_JZ:                                 // AM模块静噪控制
     		if( term_rx.len != 0x05 )
			{
				TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
			}else{
	  		    if (term_rx.msg[0] == 0x00) 
	  			{
	  				AM_MC_EN();
	  			}    
	  		    else if (term_rx.msg[0] == 0x01) 
	  			{
	  				AM_MC_DIS();
	  			}
	  		    else
	  		    {
	  			    TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0); 
	  			    break;
	  			}
	  			TermCommSend(B2T_CMD_ACK,(INT8U *)0, 0);
	  		}
     	    break;
     	    
        case T2B_CMD_BD1_GLCS:                              /* Check transmit power of star module     */
        	if( term_rx.len != 0x04 )
			{
				TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
			}else{
	  		    TermCommSend(B2T_CMD_ACK, (INT8U *)0, 0);
	  		    StarYHZL();					//使北斗卫通模块进入测试模式，仅国芯模块有效！
  		    	OSTimeDlyHMSM(0, 0, 0, 200);	//等待模块进入测试模式！
		   	    StarLoopTest(&g_TXSQ);
		   	}
	   	    break;
	   		     	
  	    case T2B_CMD_BD1_WMCS:                              /* Star module error code test.            */
  	    	if( term_rx.len != 0x05 )
			{
				TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
			}else{
	  		    TermCommSend(B2T_CMD_ACK,(INT8U *)0, 0);	
		        StarBitErrTest(0, 1, 1);
		    }
	   	    break;
	   	  	    	
  	    case T2B_CMD_DSC_TEST:                              /* DSC code test.                          */ 
  	    	if( term_rx.len != 0x04 )
			{
				TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
			}else{
	  		    TermCommSend(B2T_CMD_ACK,(INT8U *)0, 0);  
		   	    DscTest();
		   	}
	   	    break;
	   	    
  	    case T2B_CMD_BAT_DATE:                              /* 电池启用日期更改            */
  		case T2B_CMD_BAT_DATE_CAP: 
  			if(( (term_rx.len != 0x09) && (term_rx.type == T2B_CMD_BAT_DATE_CAP) ) || ( (term_rx.len != 0x08) && (term_rx.type == T2B_CMD_BAT_DATE) ))
			{
				TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
			}else{   
	    	    year  = (term_rx.msg[0] << 8) + term_rx.msg[1];
	    	    month = term_rx.msg[2];
	    	    date  = term_rx.msg[3];
	    	    u8_temp = 0;
		        	  
	    		if ((year > 2099) || (year < 2000))
	    			u8_temp = 1;
	    		else if ((month > 12) || (month == 0))
	    			u8_temp = 2;
	    		else if (((year%4 == 0) && (year%100!=0)) || (year%400 == 0))
	    		{
	    			if ((date > MDPMForLeapYear[month-1]) || (date == 0))
	    			    u8_temp = 3;
	    		} 
	    		else
	    		{	
	    			if ((date > MDPMForNormYear[month-1]) || (date == 0))
	    			    u8_temp = 3; 			     			    
	    		}

	        	if (u8_temp == 0)
	        	{    
	        	    g_BatInfo.m_InitFlag = DS2788_INIT_FLAG;
	        	    g_BatInfo.m_BatID[0] = 0;
	        	    g_BatInfo.m_BatID[1] = 0; 
	        	    memcpy((INT8U *)g_BatInfo.m_InitDate, (INT8U *)term_rx.msg, 4);
	        	    if (term_rx.type == T2B_CMD_BAT_DATE)
	        	    {
	        	        if (DS2788_CfgBatIDAndDate())
	        	            u8_temp = 5;
	        	    }
	        	    else if (term_rx.msg[4] > 100)
	    			    u8_temp = 4; 
	        	    else
	        	    {
	        	        g_BatInfo.m_InitCap[0] = (288 * term_rx.msg[4]) >> 8;
	        	        g_BatInfo.m_InitCap[1] = (288 * term_rx.msg[4]);
	        	        if (g_BatInfo.m_InitCap[1] < 0x80)
	        	        	g_BatInfo.m_InitCap[1] = 0x80;
	        	        else
	        	        	g_BatInfo.m_InitCap[1] = 0xff;
	        	        	 
	                    if (DS2788_CfgBatInfo())
	                        u8_temp = 6;     
	        	    }  
	        	}	
	    
	            if (u8_temp)
	                TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
	            else
	                TermCommSend(B2T_CMD_ACK, (INT8U *)0, 0);
            }
            break;
            
            
	    #if BD1_PROTOCOL_VERSION >= 4.0
			
  		case T2B_CMD_KJBG:
			if(term_rx.len!=0x04)
			{
				TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
			}
			else
			{
				TermCommSend(B2T_CMD_ACK,(INT8U *)0, 0); 
				StarKJBG();
			}
			break;
		case T2B_CMD_GJBG:
			if(term_rx.len!=0x04)
			{
				TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
			}
			else
			{
				TermCommSend(B2T_CMD_ACK,(INT8U *)0, 0); 
				StarGJBG();
			}
			break;
		case T2B_CMD_JMSQ:
			if(term_rx.len!=0x04)
			{
				TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
			}
			else
			{
				TermCommSend(B2T_CMD_ACK,(INT8U *)0, 0); 
				StarJMSQ();
			}
			break;	
		case T2B_CMD_JMJC:
			if(term_rx.len!=0x04)
			{
				TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
			}
			else
			{
				TermCommSend(B2T_CMD_ACK,(INT8U *)0, 0); 
				StarJMJC();
			}
			break;	
		case T2B_CMD_SXCX:
			if(term_rx.len!=0x04)
			{
				TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);
			}
			else
			{
				
				TermCommSend(B2T_CMD_ACK,(INT8U *)0, 0); 
				StarSXCX();	
			}

		case T2B_CMD_ZHSQ:
			if(term_rx.len!=0x04)
			{
				TermCommSend(B2T_CMD_NAK,(INT8U*)0,0);
			}
			else
			{
				TermCommSend(B2T_CMD_ACK,(INT8U*)0,0);
				StarZHSQBD();
				
			}
		#endif	
			
		
  	    default:                                            /* Undefined command.                      */
  		    TermCommSend(B2T_CMD_NAK, (INT8U *)0, 0);          
  		    return (1);
  	}
		}
    return (0);
}

/********************************************************************************************************
* Function Name : TermCommSend
*
* Description   : Send information to the terminal.
*
* Arguments     : type     :  the information type neading to send to terminal
*                 pmsg     :  the pointer of message
*                 msg_len  :  the length of the message
* Returns       : 0        :  send the information ok
                  1        :  a null pointer to the message                    
********************************************************************************************************/
INT8U  TermCommSend(INT8U type, INT8U *pmsg, INT8U msg_len)
{
    static  TERM_COMM_TYPE  term_tx = {TERM_FR_HEADER};
    	
	if (msg_len) 
	{
	    if (pmsg == NULL)
			return (1);
	    else if (msg_len > (sizeof(TERM_COMM_TYPE)-4))
            term_tx.len = sizeof(TERM_COMM_TYPE);
        else
        	term_tx.len = msg_len + 4;

        memcpy(term_tx.msg, pmsg, term_tx.len-4);
	}
	else
		term_tx.len = 4;
		
	term_tx.type = type;		
	*((INT8U *)&term_tx + term_tx.len-1) = GetChecksum((INT8U *)&term_tx, term_tx.len-1);
	
	UsartWrite(0, (INT8U *)&term_tx, term_tx.len, 0);
	return (0);
}

/********************************************************************************************************
* Function Name : NGModuleInit
*
* Description   : Initialize the nine GHz module.
*
* Arguments     : none
*     
* Returns       : none                  
********************************************************************************************************/
static  void  NGModuleInit(void)
{	
    *AT91C_PIOB_PER   = NG_STOP | NG_CHK | NG_RESP_IN | NG_CHK_IN; // Enable the PIO to control the pin.
    *AT91C_PIOB_OER   = NG_STOP | NG_CHK;	                       // Enable the PIO output enable
	
    *AT91C_PIOB_ODR   = NG_RESP_IN | NG_CHK_IN;	                   // Enable the PIO input enable
    *AT91C_PIOB_IFER  = NG_RESP_IN | NG_CHK_IN;	                   // Enables the input glitch filter  
    *AT91C_PIOB_PPUDR = NG_RESP_IN | NG_CHK_IN;                    // Disable the pull_up resister        
    NGRespEn();  	                                               // Enable NG_RESP_IN interrupt
}

/********************************************************************************************************
* Function Name : NGModuleCheck
*
* Description   : 9GHz模块自检时，自检控制输出引脚NG_CHK置为高电平。
*                 若模块应答引脚有脉冲输出（10KHz）,且自检输入引脚NG_CHK_IN==1，则自检正常，否则，自检错误。
*
* Arguments     : none
*     
* Returns       : true      success.
*                 false     faild.                  
********************************************************************************************************/
static  INT8U  NGModuleCheck(void)
{	    
    INT8U  result = false;
	    
    if (IS_9G_WORKING() == false)                 // Returned,if 9GHz module's power is off
  	    return (result);
   
    g_Flg.m_9GTest = 1;
    g_Flg.m_9GAck  = 0;
    NGRespEn();  	    
    *AT91C_PIOB_SODR = NG_CHK;
    OSTimeDlyHMSM(0, 0, 0, 500);                  // Wait for the 9GHz module steady.

    if ((g_Flg.m_9GAck) && (*AT91C_PIOB_PDSR & NG_CHK_IN))
        result = true;
    
    *AT91C_PIOB_CODR = NG_CHK;
    g_Flg.m_9GAck  = 0;
    g_Flg.m_9GTest = 0; 
    
    return (result);
}

/********************************************************************************************************
* Function Name : PIOB_IntHandler
*
* Description   : the PIOB's interrupt service routine.
*
* Arguments     : none
*     
* Returns       : none                  
********************************************************************************************************/
static  void  PIOB_IntHandler(void)  
{
    INT32U        status     = *AT91C_PIOB_ISR;   // Read status register,avoid enter interrupt again
    static INT32U int_ctr    = 0;                 // Interrupt counter
    static INT32U time_stamp = 0;                 // NG_RESP_IN pin first interrupt time
     
    AT91C_BASE_AIC->AIC_ICCR = 1<<AT91C_ID_PIOB;  // Clear PIOB interrupt
    if (status & NG_RESP_IN)                      // Radar response interrupt
    {     
        if (++int_ctr == 1) 
            time_stamp = OSTimeGet();
        else if (int_ctr >= 3)
        {  
            int_ctr = 0;
            if ((OSTimeGet() - time_stamp) <= 100)
            {    
                g_Flg.m_9GAck   = 1;              // Set the response flag
                *AT91C_PIOB_IDR = NG_RESP_IN;	  // Disable the pin(NG_RESP_IN) interrupt
            }
        }
    }
             
    AT91C_BASE_AIC->AIC_EOICR = 0x00;	          // End of the interrupt service routine
}

/********************************************************************************************************
* Function Name : NGRespEn
*
* Description   : Enable the 9GHz module's responsion function
*
* Arguments     : none
*     
* Returns       : none
********************************************************************************************************/
void  NGRespEn(void)
{
    INT32U  status;

    *AT91C_PIOB_CODR = NG_STOP;                   // Normal response when NG_STOP is low level
    status = *AT91C_PIOB_ISR;                     // Read the PIO_ISR,clear the interrupt flags
    *AT91C_AIC_ICCR  = 1 << AT91C_ID_PIOB;        // Clears PIOB interrupt.
    *AT91C_PIOB_IER  = NG_RESP_IN;	              // Enable the pin(NG_RESP_IN) interrupt
}
/********************************************************************************************************
* Function Name : NGRespDis
*
* Description   : Disable the 9GHz module's responsion function
*
* Arguments     : none
*     
* Returns       : none               
********************************************************************************************************/
void  NGRespDis(void)
{
    INT32U  status;

    *AT91C_PIOB_SODR = NG_STOP;                   // Stop response when NG_STOP is hign level
    status = *AT91C_PIOB_ISR;                     // Read the PIO_ISR,clear the interrupt flags
    *AT91C_AIC_ICCR  = 1 << AT91C_ID_PIOB;        // Clears PIOB interrupt.  
    *AT91C_PIOB_IDR  = NG_RESP_IN;	              // Disable the pin(NG_RESP_IN) interrupt
}

/********************************************************************************************************
* Function Name : AMInit
*
* Description   : Initilize the control line of the vhf module;
*                 Initilize timer used to change channel and send DSC list;
*                 initialize the status of vhf module,the FSK chip control line and status
*
* Arguments     : none
*     
* Returns       : none                  
********************************************************************************************************/
static  void  AMInit(void)
{	
	// Enable the PIO to control the corresponding pin 
	*AT91C_PIOB_PER   = AM_SEL|AM_PTT|PLL2_CLK|PLL2_DATA;
	 
	// Enable the PIO output enable
	*AT91C_PIOB_OER   = AM_SEL|AM_PTT|PLL2_CLK|PLL2_DATA;
  
	*AT91C_PIOA_PER   = PLL2_CE|AM_FREQ|AM_PA|AM_LOCK; // Enables the PIO to control the corresponding pin
	*AT91C_PIOA_OER   = PLL2_CE;		          // Enable the PIO output enable
	*AT91C_PIOA_ODR   = AM_FREQ|AM_PA|AM_LOCK;    // Disable the PIO input enable	   	                         
	*AT91C_PIOA_IFER  = AM_FREQ|AM_PA|AM_LOCK;    // Enables the input glitch filter on the I/O line	 	    
	*AT91C_PIOA_PPUER = AM_FREQ|AM_PA|AM_LOCK;    // Enables the pull_up resister           

	*AT91C_PIOA_SODR  = PLL2_CE;
	*AT91C_PIOB_SODR  = PLL2_CLK|PLL2_DATA;
   
	AM_SEL_DIS();
	AM_PTT_DIS();
    AM_MC_EN();
}

/********************************************************************************************************
* Function Name : AMCheck
*
* Description   : 航空模块自检时必须将模块置于发射状态,待稳定后，引脚AM_FREQ==1，且AM_PA==0
*                 表示自检正常; 其他，表示自检异常.
*                                      
* Arguments     : none
*     
* Returns       : true    自检正常
*                 false   自检错误
********************************************************************************************************/
static  INT8U  AMCheck(void)
{
    INT8U    vhf_st = 0;
    INT32U   status;

    if (IS_TONE_PWR_ON() == false)
    	return (false);                           // Returned,if tone module's power is off
    	
    if (IS_AM_SEL() == false)    
  		vhf_st = 1;
  	AM_SEL_EN();
  	OSTimeDlyHMSM(0, 0, 0, DLY_MS_AM_PWR_ON);
    AM_PTT_EN();                                  // Set the VHF module in transmit status
    AMSetPLL(0);
    OSTimeDlyHMSM(0, 0, 0, 500);                  // Wait for the module steady.
    status = *AT91C_PIOA_PDSR;                    // Read the pio status.
    AM_PTT_DIS();                                 // Set the VHF module in receive status
    if (vhf_st)
    {
    	AM_SEL_DIS();
    	VHF_PTT_DIS();
    	VhfChngCh(g_Flg.m_ChCur);
    }
    else
    	AMSetPLL(1);
    	
    if ((status & AM_FREQ) && !(status & AM_PA))
        return (true);
    else
        return (false);
} 

/********************************************************************************************************
* Function Name : AMSetPLL
*
* Description   : 设置VHF航空模块锁相环芯片工作参数。根据模块当前收发状态，设置不同的工作参数。
*
* Arguments     : mode        0：设置4组24位参数
                              1：仅设置最后1组24位参数
* Returns       : ture        Set Parameter ok;
*                 false       Set Parameter faild.
********************************************************************************************************/
static  INT8U  AMSetPLL(INT8U mode)
{ 
	INT8S      i, j;
	static INT32U rx_para[4] = {0xED8492, 0xED8493, 0xE30900, 0xF4A801};
	static INT32U tx_para[4] = {0xED8492, 0xED8493, 0xE30900, 0xF53C01};
	INT32U    *ptr = rx_para;
	INT32U     tmp;
	OS_CPU_SR  cpu_sr;

    OS_ENTER_CRITICAL();
    if ((*AT91C_PIOB_PDSR & AM_PTT) == 0x00)
        ptr = tx_para;
    j = (mode == 0) ? 0 : 3;
	
	while (j < 4)
	{
		tmp = *(ptr + j);
		*AT91C_PIOA_CODR = PLL2_CE;
		TimeDly(20); 
	    for (i=23; i>=0; i--)
        {
            *AT91C_PIOB_CODR = PLL2_CLK;
            TimeDly(10);
		    if ((tmp>>i) & 0x01)
			    *AT91C_PIOB_SODR = PLL2_DATA;
		    else
			    *AT91C_PIOB_CODR = PLL2_DATA;
			TimeDly(10);    
            *AT91C_PIOB_SODR = PLL2_CLK;
            TimeDly(20); 
	    }
	    *AT91C_PIOA_SODR = PLL2_CE; 
	    TimeDly(20);
	    j++;
	}
			
	*AT91C_PIOA_SODR = PLL2_CE;
	*AT91C_PIOB_SODR = PLL2_CLK;
	*AT91C_PIOB_SODR = PLL2_DATA;
	OS_EXIT_CRITICAL();
	return(true);
}

/********************************************************************************************************
* Function Name : ToneSweep
*
* Description   : Change the VHF module's channel,the control chip is Fujitsu MB15E03
*
* Arguments     : mode        0
*     
* Returns       : ture        Set Parameter ok;
*                 false       Set Parameter faild.
********************************************************************************************************/
/*
void  ToneSweep(void)
{
	INT8U  i;
	INT8U  sweep_ch[] = {0, 6, 13, 16};

    while (g_Flg.m_Sweep)
    {
        AM_SEL_EN();
        OSTimeDlyHMSM(0, 0, 0, DLY_MS_AM_PWR_ON);
        AM_PTT_DIS();
        AMSetPLL(0); 
        OSTimeDlyHMSM(0, 0, 0, SWEEP_TIME_SLOT);
    	if ((*AT91C_PIOA_PDSR & AM_LOCK) == 0)
    	{
    		g_Flg.m_Sweep = 0;
    		TermCommSend(B2T_CMD_TONE_LOCK, &sweep_ch[0], 1);
    		break;
    	}

        AM_SEL_DIS();
        VHF_PTT_DIS();
    	for (i=1; i<4; i++)
        {
    	    VhfChngCh(sweep_ch[i]);
            OSTimeDlyHMSM(0, 0, 0, SWEEP_TIME_SLOT);
    	    if (*AT91C_PIOB_PDSR & VHF_LOCK)
    	    {
    		    g_Flg.m_Sweep = 0;
    		    TermCommSend(B2T_CMD_TONE_LOCK, &sweep_ch[i], 1);
    		    break;
    	    }
        }
    }
    OSTaskSuspend(TONE_SWEEP_TASK_PRIO);
}
*/
void  ToneSweep(void)
{
	INT8U  i, jz_dis = 0;
	INT8U  tmp, tmp1, tmp2, tmp3;
	INT8U  sweep_ch[] = {0, 6, 16, 13};

    if (IS_AM_MC_EN() == false)
    {
    	jz_dis = 1;
    	AM_MC_EN();
    }
    
    while (g_Flg.m_Sweep)
    {
        AM_SEL_EN();
        OSTimeDlyHMSM(0, 0, 0, DLY_MS_AM_PWR_ON);
        AM_PTT_DIS();
        AMSetPLL(0); 
        
        //OSTimeDlyHMSM(0, 0, 0, SWEEP_TIME_SLOT);
        OSTimeDlyHMSM(0, 0, 0, 80);
    	tmp1 = (*AT91C_PIOA_PDSR & AM_LOCK) ? 1 : 0;
        OSTimeDlyHMSM(0, 0, 0, 80);
    	tmp2 = (*AT91C_PIOA_PDSR & AM_LOCK) ? 1 : 0;
        OSTimeDlyHMSM(0, 0, 0, 80);
    	tmp3 = (*AT91C_PIOA_PDSR & AM_LOCK) ? 1 : 0;
    	
    	if ((tmp1 == tmp2) || (tmp1 == tmp3))
    	    tmp = tmp1;
    	else
    	    tmp = tmp2;
    	
    	if (tmp == 0)
    	{
    		g_Flg.m_Sweep = 0;
    		TermCommSend(B2T_CMD_TONE_LOCK, &sweep_ch[0], 1);
    		break;
    	}

        AM_SEL_DIS();
        OSTimeDlyHMSM(0, 0, 0, 50);
        VHF_PTT_DIS();
    	for (i=1; i<4; i++)
        {
    	    VhfChngCh(sweep_ch[i]);
            
            //OSTimeDlyHMSM(0, 0, 0, SWEEP_TIME_SLOT);
            OSTimeDlyHMSM(0, 0, 0, 80);
    	    tmp1 = (*AT91C_PIOB_PDSR & VHF_LOCK) ? 1 : 0;
            OSTimeDlyHMSM(0, 0, 0, 80);
    	    tmp2 = (*AT91C_PIOB_PDSR & VHF_LOCK) ? 1 : 0;
            OSTimeDlyHMSM(0, 0, 0, 80);
    	    tmp3 = (*AT91C_PIOB_PDSR & VHF_LOCK) ? 1 : 0;
    	
    	    if ((tmp1 == tmp2) || (tmp1 == tmp3))
    	        tmp = tmp1;
    	    else
    	        tmp = tmp2;
    	    if (tmp)
    	    {
    		    g_Flg.m_Sweep = 0;
    		    TermCommSend(B2T_CMD_TONE_LOCK, &sweep_ch[i], 1);
    		    break;
    	    }
        }
    }
    
    if (jz_dis == 1)
        AM_MC_DIS();
        
    OSTaskSuspend(TONE_SWEEP_TASK_PRIO);
}
/********************************************************************************************************
* Function Name : VhfModuleInit
*
* Description   : Initilize the control line of the vhf module;
*                 Initilize timer used to change channel and send DSC list;
*                 initialize the status of vhf module,the FSK chip control line and status
*
* Arguments     : none
*     
* Returns       : none                  
********************************************************************************************************/
static  void  VhfModuleInit(void)
{	
	// Enable the PIO to control the corresponding pin 
	*AT91C_PIOB_PER   = VHF_PTT|PLL_DATA|PLL_CLK|PLL_LE|VHF_MUTE|TONE_PWR_SW|VHF_DSC_SW;
	 
	// Enable the PIO output enable                                             
	*AT91C_PIOB_OER   = VHF_PTT|PLL_DATA|PLL_CLK|PLL_LE|VHF_MUTE|TONE_PWR_SW|VHF_DSC_SW;	 	                       		
	*AT91C_PIOB_CODR  = VHF_MUTE;   
	
	*AT91C_PIOB_PER   = PLL_LD|VHF_PWR_CHK_IN|VHF_LOCK;    // Enables the PIO to control the corresponding pin	                                               
	*AT91C_PIOB_ODR   = PLL_LD|VHF_PWR_CHK_IN|VHF_LOCK;    // Disable the PIO input enable	   	                         
	*AT91C_PIOB_IFER  = PLL_LD|VHF_PWR_CHK_IN|VHF_LOCK;    // Enables the input glitch filter on the I/O line	 	    
	*AT91C_PIOB_PPUDR = PLL_LD|VHF_PWR_CHK_IN;    // Disable the pull_up resister            
	
	TC0_Init();                                   // Used to send the DSC list.
    TC1_Init();                                   // Used to send fre control byte.
	VhfPwrEn();                                   // Enable the power of vhf module	 
	VHF_PTT_DIS();                                // Vhf module at receive state after system reset.
    VhfChngCh(16);                                // Set the default channel of vhf is 16.   
    FskInit();                                    // initialize the FSK chip control line and status.
}

/********************************************************************************************************
* Function Name : VhfPwrEn
*
* Description   : Ebable the power of the VHF module and resume the dsc alarm tast.
*
* Arguments     : none
*     
* Returns       : none                   
********************************************************************************************************/
void  VhfPwrEn(void)
{
    *AT91C_PIOB_CODR = TONE_PWR_SW;               // Switch the power of VHF module on
    OSTaskResume(DSC_ALARM_TASK_PRIO);            // Resume the dsc alarm task 	
}

/********************************************************************************************************
* Function Name : VhfPwrDis
*
* Description   : Disable the power of the VHF module and suspend the dsc alarm tast
*
* Arguments     : none
*     
* Returns       : none                   
********************************************************************************************************/ 
void  VhfPwrDis(void)
{
    *AT91C_PIOB_SODR = TONE_PWR_SW;               // Switch the power of VHF module off
    OSTaskSuspend(DSC_ALARM_TASK_PRIO);           // Suspend the dsc alarm task         		
}
 
/********************************************************************************************************
* Function Name : VhfModuleCheck
*
* Description   : VHF话音模块自检时必须将模块置于发射状态,待稳定后，引脚PLL_LD==1，且VHF_PWR_CHK_IN==0 
*                 表示自检正常; 其他，表示自检异常.
*                                      
* Arguments     : none
*     
* Returns       : true    自检正常
*                 false   自检错误
********************************************************************************************************/
static  INT8U  VhfModuleCheck(void)  
{
    INT32U   status;
  
    if (IS_TONE_PWR_ON() == false)                // Returned,if Vhf module's power is off
  	    return (false);                             	     	                      
    
    VHF_PTT_EN();                                 // Set the VHF module in transmit status
    VhfChngCh(g_Flg.m_ChCur);                     // Refresh the frequence for tx
    OSTimeDlyHMSM(0, 0, 0, 500);                  // Wait for the module steady.
    status = *AT91C_PIOB_PDSR;                    // Read the pio status.
    VHF_PTT_DIS();                                // Set the VHF module in receive status
    VhfChngCh(g_Flg.m_ChCur);                     // Refresh the frequence for rx
    
    if ((status & PLL_LD) && !(status & VHF_PWR_CHK_IN))    
        return (true);
    else
        return (false);
} 

/********************************************************************************************************
* Function Name : TimeDly
*
* Description   : software time delay.
*
* Arguments     : cnt  time delay counts
*     
* Returns       : none                  
********************************************************************************************************/
static  void  TimeDly(INT16U cnt)
{
	INT32U i; 
	for (i=10*cnt; i>0; i--)
	{	
	    i = i;
	}	
}  

/********************************************************************************************************
* Function Name : TC1_Init
*
* Description   : Initialize the TC1 , timing for changing the channel of the VHF module.
*
* Arguments     : none
*     
* Returns       : none                  
********************************************************************************************************/
void  TC1_Init(void)
{	
	
	*AT91C_PMC_PCER |= 1 << AT91C_ID_TC1;              // Enable the clock of the TC1.
		  
    *AT91C_TCB_BMR &= (~(0x00000003<<2));              // Clear the bit2,3 of Block Mode Register
    *AT91C_TCB_BMR |= AT91C_TCB_TC1XC1S_NONE;          // BMR's external clock single1 select is none.

    *AT91C_TC1_CMR = AT91C_TC_CLKS_TIMER_DIV4_CLOCK |  // TIMER_CLOCK4 = MCK/128 = 48/128 MHz
                     AT91C_TC_EEVT_XC0 |               // (TC) Signal selected as external event: XC0 TIOB direction: output
                     AT91C_TC_WAVESEL_UP_AUTO |        // UP mode with automatic trigger on RC Compare
                     AT91C_TC_WAVE |                   // Waveform mode
                     AT91C_TC_ACPA_SET |
                     AT91C_TC_BCPB_SET |
                     AT91C_TC_ACPC_CLEAR |  
                     AT91C_TC_BCPC_CLEAR;
                                    
    *AT91C_TC1_RA = 0x15;
    *AT91C_TC1_RB = 0x15;
    *AT91C_TC1_RC = 0x30;                              // fre = 7.8125KHz 
    
}

/********************************************************************************************************
* Function Name : VhfChngCh
*
* Description   : Change the VHF module's channel,the control chip is Fujitsu MB15E03
*
* Arguments     : ch_nbr      the absolute channel number neading to alter.
*     
* Returns       : ture        change channel ok;
*                 false       change channel faild.                 
********************************************************************************************************/
static  INT8U  VhfChngCh(INT8U ch_nbr)
{ 
	
	INT8S      i; 	
	OS_CPU_SR  cpu_sr;                                           
	INT32U     progcnt;                                     // Programmable divider list.                                  	
    // PGM REF CNTR DIV ratio-----CNT=1 | R=512=0x200 | SW=1(M=64)   | FC         | LDS
    static INT32U const refcnt = 0x0001 | (0x200<<1)  | (0x0001<<15) | (0x01<<16) | (0x00<<17);     
	static VHF_CH_PARA_TYPE const  ch_para[10] = {
		                        {4,40,97,4,84},   {6,44,97,8,84},  {8,48,97,12,84},  {13,58,97,22,84},
                                {16,0,98,28,84},  {19,6,98,34,84}, {21,10,98,38,84}, {22,12,98,40,84}, 
                                {70,53,97,53,97}, {80,9,98,37,84}};
              
    OS_ENTER_CRITICAL();
    for (i=0; i<=9; )
    {    
        if (ch_nbr == ch_para[i].m_ChNbr)
        {            	
            if (*AT91C_PIOB_ODSR & VHF_PTT)                 // Receive status.
                progcnt = (ch_para[i].m_RxA << 1) | (ch_para[i].m_RxN << 8); 
	        else                                            // Transmit status.
                progcnt = (ch_para[i].m_TxA << 1) | (ch_para[i].m_TxN << 8);
            break;
        }  	
  	    i++;  	 
  	    if (i > 9)
  	    {
  	    	OS_EXIT_CRITICAL();	
  		    return(false);
  		}
    }                                                      		  			
	*AT91C_PIOB_CODR = PLL_LE;	                            // When LE is low,enable transmitting data to the shift register in the MB15E03.
	*AT91C_PIOB_SODR = PLL_DATA;                            // Pull up the data line.
	*AT91C_TC1_CCR   = AT91C_TC_SWTRG | AT91C_TC_CLKEN;     // software trigger the TC,start the TC.
	 	
	for (i=17; i>=0; i--)                                   // Send the programmable reference counter,start data input with MSB first.					  		 
    {
        *AT91C_PIOB_CODR = PLL_CLK;                         // clock = 0                                		
		while (!(*AT91C_TC1_SR & 0x10));
        *AT91C_TC1_CCR = AT91C_TC_SWTRG;    		  		  	
		if ((refcnt>>i) & 0x01)                             // Serial data input using binary code                                   
			*AT91C_PIOB_SODR = PLL_DATA;     
		else
			*AT91C_PIOB_CODR = PLL_DATA;					
		TimeDly(17 - i);                                    // Adjust the width of pluse for data				                                                       	
        *AT91C_PIOB_SODR = PLL_CLK;                         // clock = 1,data is transmitted to the shift register on the rising edge of clock. 		
        while (!(*AT91C_TC1_SR & 0x10));
		*AT91C_TC1_CCR = AT91C_TC_SWTRG;
	}
	
	// When LE is hign,the data in the shift register is transferred to a latch,according to the control bit in the serial data.
	*AT91C_PIOB_SODR = PLL_LE; 		                          
	*AT91C_TC1_CCR = AT91C_TC_SWTRG | AT91C_TC_CLKDIS;      // Stop the TC	     
	TimeDly(200);	
	*AT91C_PIOB_CODR = PLL_LE;
	*AT91C_TC1_CCR = AT91C_TC_SWTRG | AT91C_TC_CLKEN;       // software trigger the TC,start the TC.
		
	for (i=18; i>=0; i--)                                   // Send programmable counter,start data input with MSB first.				 
	{ 	
	    *AT91C_PIOB_CODR = PLL_CLK;                         // clock = 0                              		
		while (!(*AT91C_TC1_SR & 0x10));
		*AT91C_TC1_CCR = AT91C_TC_SWTRG;		  		  	
		if ((progcnt>>i) & 0x01)                            // Serial data input using binary code                             
			*AT91C_PIOB_SODR = PLL_DATA;     
		else
			*AT91C_PIOB_CODR = PLL_DATA;					
		TimeDly(18 - i);                                    // Adjust the width of pluse for data			
        *AT91C_PIOB_SODR = PLL_CLK;                         // clock = 1      		
        while (!(*AT91C_TC1_SR & 0x10));
		*AT91C_TC1_CCR = AT91C_TC_SWTRG;  				
	}
			
	*AT91C_PIOB_SODR = PLL_LE;			
	*AT91C_TC1_CCR   = AT91C_TC_SWTRG | AT91C_TC_CLKDIS;    // Stop the timer
	g_Flg.m_ChCur    = ch_nbr;                              // Save the new channel number.
	OS_EXIT_CRITICAL();		
	return(true);
}

/********************************************************************************************************
* Function Name : FskInit
*
* Description   : Initilize the FSk chip.
*
* Arguments     : none
*     
* Returns       : none                      
********************************************************************************************************/
static  void  FskInit(void)
{	 
	
	*AT91C_PIOB_PER    = FSK_TEST | FSK_MOD1 | FSK_MOD2 | FSK_RS | FSK_XD | FSK_RD;
	*AT91C_PIOB_OER    = FSK_TEST | FSK_MOD1 | FSK_MOD2 | FSK_RS | FSK_XD;	
	 	                       	                                               
	*AT91C_PIOB_ODR    = FSK_RD;                  // Disable the PIO input enable	  	                         
	*AT91C_PIOB_IFER   = FSK_RD;                  // Enables the input glitch filter on the I/O line 		    
	*AT91C_PIOB_PPUDR  = FSK_RD;                  // Disable the pull_up resister            
	
    FskSetMode(0x03);                             // Set the FSK chip in power down mode
    FSK_TEST_DIS();                               // isn't in test mode
    FSK_RS_DIS();                      	          // transmitting becomes disable
    
}

/********************************************************************************************************
* Function Name : FskSetMode
*
* Description   : Set mode for MSM7512B.
*
* Arguments     : mode  the mode neading to set.
*                       MOD2 MOD1  MODE
*                        0    0    FSK Transmit Mode
*                        0    1    FSK Receive & 75 bps Transmit Mode
*                        1    0    Analog Loop Back Test Mode
*                        1    1    Power Down Mode     
* Returns       : none                      
********************************************************************************************************/
static  void  FskSetMode(INT8U mode)
{ 
	
	(mode & 0x01) ? (*AT91C_PIOB_SODR = FSK_MOD1) : (*AT91C_PIOB_CODR = FSK_MOD1);  	
	(mode & 0x02) ? (*AT91C_PIOB_SODR = FSK_MOD2) : (*AT91C_PIOB_CODR = FSK_MOD2);
	
}

/********************************************************************************************************
* Function Name : DscHorCheck
*
* Description   : DSC list horizontal Check.
*
* Arguments     : ch     the charactor neading to check.
*     
* Returns       : dest   the result of checking.                      
********************************************************************************************************/
static  INT16U  DscHorCheck(INT8U ch)      
{	
    INT8U   i;
    INT8U   zero_nbr = 0;	
    INT16U  dest     = 0;
	
	for (i=0; i<7; i++)
    {    
        if (ch & (0x01<<i))
            dest |= (0x01<<(9-i));
        else
            zero_nbr++;
	}
	dest |= zero_nbr;		
	return(dest);	 
}

/********************************************************************************************************
* Function Name : DscVertCheck
*
* Description   : Dsc vertical check.
*
* Arguments     : psrc   the first pointer neading to check
*                 len    the length neading to check.
*     
* Returns       : dest   the result of checking.                      
********************************************************************************************************/
static  INT16U  DscVertCheck(const INT16U *psrc,INT16U len)
{
    INT8U   x,y;
    INT8U   zero_nbr = 0;
    INT16U  dest     = 0;
    
    if (psrc == NULL)
    	return (0);
    	
    for (x=0; x<len; x++)
        dest ^=*(psrc+x);	   	  
    for (y=3; y<10; y++)                               // calculate zero number
    {     
        if ((dest & (0x01<<y)) == 0)
            zero_nbr++;		
    }  
    dest &= 0xfff8;                                    // clear the last 3bit
    dest |= zero_nbr;                                  // attach the zero number  
    return(dest);
}

/********************************************************************************************************
* Function Name : DscListCreate
*
* Description   : Create the DSC list which transmited to the VHF device.
*
* Arguments     : pdest    is a start pointer to save the checked dsc code
*       
* Returns       : none                      
********************************************************************************************************/
static void DscListCreate(INT16U *pdest)
{  	
    INT8U  i;
    static INT8U const dx_list    = 0x7D;                   /* 定相序列DX位数据                        */
    static INT8U const rx_list[8] = {0x6F,0x6E,0x6D,0x6C,0x6B,0x6A,0x69,0x68};   
    // DSC报警信息       遇险报格式符 | 船名代码、遇险性质 | 位置信息、时间信息   |无线电话、EOS、ECC  */
    DSC_ALM_MSG_TYPE   dsc_msg = {112, 41,20,11,11,10,107,  99,99,99,99,99, 88,88, 126, 127, 0};                 
                             
    //--给DSC序列各组成部分赋值
    for(i=0; i<6; i++)
        *(dsc_msg.ship_id + i) = *(g_SysInit.ship_id + i);  /* 设置船名代码、遇险性质                  */             	       
    if (g_Flg.m_ExistPos)                                   /* 卫星定位信息填入DSC报警电文中           */       
    {     
        // GMT+8 --> UTC    
        dsc_msg.msg_time[0] = (g_DWXX.time.hour >= 8) ? (g_DWXX.time.hour - 8) : (g_DWXX.time.hour + 24 - 8);          
        dsc_msg.msg_time[1] = g_DWXX.time.min; 
               
        dsc_msg.msg_pos[0]  = g_DWXX.lat.deg/10;
        dsc_msg.msg_pos[1]  = (g_DWXX.lat.deg%10)*10 + (g_DWXX.lat.ct/10);
        dsc_msg.msg_pos[2]  = (g_DWXX.lat.ct%10)*10 + (g_DWXX.lon.deg/100);
        dsc_msg.msg_pos[3]  = g_DWXX.lon.deg%100;
        dsc_msg.msg_pos[4]  = g_DWXX.lon.ct;   	
    } 
    else if (g_Flg.m_ManPos)              	                /* 人工时间、位置信息填入DSC报警电文中     */ 	
    {     
        // GMT+8 --> UTC    
        dsc_msg.msg_time[0] = (g_ManPos.time.hour >= 8) ? (g_ManPos.time.hour - 8) : (g_ManPos.time.hour + 24 - 8);           
        dsc_msg.msg_time[1] = g_ManPos.time.min;        
        
        dsc_msg.msg_pos[0]  = g_ManPos.lat.deg/10;          
        dsc_msg.msg_pos[1]  = (g_ManPos.lat.deg%10)*10 + (g_ManPos.lat.ct/10);
        dsc_msg.msg_pos[2]  = (g_ManPos.lat.ct%10)*10 + (g_ManPos.lon.deg/100);
        dsc_msg.msg_pos[3]  = g_ManPos.lon.deg%100;
        dsc_msg.msg_pos[4]  = g_ManPos.lon.ct; 
    }
	
    //--水平校验
    for(i=0; i<16; i++)
        *(&dsc_msg.format + i) = DscHorCheck(*(&dsc_msg.format + i));
    
    //--垂直校验的数据从格式符至系列终止符（格式符和序列终止符分别作为1个字节参与校验）      
    dsc_msg.ecc = DscVertCheck(&dsc_msg.format, 16);
  
    //--组合生成完整的待发送DSC报警通信序列  
    *(pdest+0)  = 0x155;                                    /* 点阵(Dot),不参与水平、垂直校验          */                             
    *(pdest+1)  = 0x155;                                 
    *(pdest+2)  = DscHorCheck(dx_list);                     /* 定相序列不参与垂直校验,但要参与水平校验 */
    for (i=1; i<6; i++)                                
        *(pdest+2+(i<<1)) = *(pdest+2);    
    for (i=0; i<8; i++)
        *(pdest+3+(i<<1)) = DscHorCheck(rx_list[i]);
    
    *(pdest+14) = dsc_msg.format;                           /* 格式符                                  */
    *(pdest+19) = dsc_msg.format;                           /* 对于遇险呼叫，发送两次格式符            */                             
    for (i=0; i<17; i++)                                    /* 按格式规定填充报警信息                  */
    {     
        *(pdest + 16 + (i<<1)) = *(&dsc_msg.format + i);
        *(pdest + 21 + (i<<1)) = *(&dsc_msg.format + i);
    }   
    *(pdest+50) = dsc_msg.eos;                             /* 剩余两个空位用结束符填充                 */
    *(pdest+52) = dsc_msg.eos;         
}
                  
/********************************************************************************************************
* Function Name : TC0_Init
*
* Description   : Initialize the TC0,the timing for DSC list transmittal.
*
* Arguments     : none
*      
* Returns       : none                      
********************************************************************************************************/                
static  void  TC0_Init(void)
{    
    *AT91C_PMC_PCER = 1 << AT91C_ID_TC0;               // Enable the clock of the TC0.    
    AT91F_PIO_CfgPeriph(AT91C_BASE_PIOB,               // Assign and configure the pin for peripheral A.
 		              //((unsigned int)AT91C_PB23_TIOA0) |
 		                ((unsigned int)AT91C_PB24_TIOB0), 	  	
 		                0);
	
    *AT91C_TCB_BMR &= (~0x00000003);                   // Clear the bit0,1 of Block Mode Register
    *AT91C_TCB_BMR |= AT91C_TCB_TC0XC0S_NONE;          // BMR's external clock single1 select is none.

    *AT91C_TC0_CMR = AT91C_TC_CLKS_TIMER_DIV1_CLOCK |  // MCK/2
                     AT91C_TC_EEVT_XC0 |               // (TC) Signal selected as external event: XC0 TIOB direction: output
                     AT91C_TC_WAVESEL_UP_AUTO |        // UP mode with automatic trigger on RC Compare
                     AT91C_TC_WAVE |                   // Waveform mode
                     AT91C_TC_ACPA_SET |
                     AT91C_TC_BCPB_SET |
                     AT91C_TC_ACPC_CLEAR |  
                     AT91C_TC_BCPC_CLEAR;
  
    *AT91C_TC0_RA = 0x270C;  
    *AT91C_TC0_RB = 0x270C;                           
    *AT91C_TC0_RC = 0x4E18;                            // Period is 0.833ms(1200 Hz) 
}

/********************************************************************************************************
* Function Name : DscAlarmProc
*
* Description   : Send DSC list to the VHF module.
*
* Arguments     : none
*      
* Returns       : tx_nbr    the left charactor neading to transmit.
*                           0 means transmitting are finished.
********************************************************************************************************/
INT8U  DscAlarmProc(void)
{   
    INT8S      i;                                          
    INT8U      am_sel = 0; 
    INT8U      vhf_old_ch;                             // save the channel before send dsc list
    INT8U      rpt;                                    // repeat times  
    INT8U      tx_nbr = 0;                             // send number of the dsc list    
    INT16U     dsc_list[54];                           // saved the dsc list which can be sent directly.
    INT32U     dsc_interval;                           // saved the alarm time interval   
    OS_CPU_SR  cpu_sr;
    

    OS_ENTER_CRITICAL();                                           
    *AT91C_PIOB_SODR = VHF_DSC_SW ;                    // Set the DSC swtich at DSC state.  
    FskSetMode(0x00);                                  // FSK Transmit Mode
    FSK_RS_EN();                                       // Transmit enable  
    if (IS_AM_SEL())
    {
    	am_sel = 1;
    	AM_SEL_DIS();
    	AM_PTT_DIS();
    	
    }
    else	
        vhf_old_ch = g_Flg.m_ChCur;                    // Saved the channal number before send DSC list.
    VHF_PTT_EN();                                      // Set VHF module at transmit state.                
    VhfChngCh(70);                                     // set the channel is 70.
    OS_EXIT_CRITICAL();
           
    OSTimeDlyHMSM(0, 0, 0, 500);                       // Wait for the transmitter ready
    
    OS_ENTER_CRITICAL(); 
    DscListCreate(dsc_list);                           // Create the DSC list.
    *AT91C_TC0_CCR = AT91C_TC_SWTRG | AT91C_TC_CLKEN;  // Start timing.
  
    // the first time includes 54 segments,each have 10bits
    while (tx_nbr < 54)                                  
     {     
        for (i=9; i>=0; i--)
        {
            while(!(*AT91C_TC0_SR & 0x10));  
            *AT91C_TC0_CCR = AT91C_TC_SWTRG;           // Soft trigger the TC reset.
            if(dsc_list[tx_nbr] & (0x01<<i))           // Send data to the port.
                *AT91C_PIOB_SODR = FSK_XD;
            else                           
                *AT91C_PIOB_CODR = FSK_XD;
        }
        tx_nbr++;                                      // segments transmitted increase.
    }

    // repeat the DSC list four times,the dot parttern are not included. 
    for (rpt=0; rpt<4; rpt++) 	
    {      
        tx_nbr = 2;
        while (tx_nbr < 54)                              
        {      
            for(i=9; i>=0; i--)
            {     
                while (!(*AT91C_TC0_SR & 0x10));  
                *AT91C_TC0_CCR = AT91C_TC_SWTRG;
                if (dsc_list[tx_nbr] & (0x01<<i))
                    *AT91C_PIOB_SODR = FSK_XD;
                else  
                    *AT91C_PIOB_CODR = FSK_XD;
            } 
            tx_nbr++;
       }  
    }
    while (!(*AT91C_TC0_SR & 0x10));                   // After the last bit,wait for one baud rate interval.
    *AT91C_TC0_CCR   = AT91C_TC_CLKDIS;                // Stopping timing.
    *AT91C_PIOB_SODR = FSK_XD;                         // After send DSC list,set the io at '1'.
    FskSetMode(0x03);                                  // Set FSK chip in power down mode.
    FSK_RS_DIS();                                      // Transmit diable.
    *AT91C_PIOB_CODR = VHF_DSC_SW;                     // Set DSC switch in voice state.
    VHF_PTT_DIS();                                     // Set VHF module in receive state.  
    if (am_sel)
    {
    	AM_SEL_EN();
    	OSTimeDlyHMSM(0, 0, 0, DLY_MS_AM_PWR_ON);    	
    	AM_PTT_DIS();
    	AMSetPLL(0); 	
    }
    else
        VhfChngCh(vhf_old_ch);                         // Resume the channal number before sending DSC list.
    OS_EXIT_CRITICAL();
     
    dsc_interval = DSC_ALM_TIME_BASE + DSC_ALM_TIME_STEP* (g_Flg.m_DscAlmCtr - 1);
    OSTimeDlyHMSM(dsc_interval/3600, (dsc_interval%3600)/60, (dsc_interval%3600)%60, 0);      
    (g_Flg.m_DscAlmCtr >= 255) ? (g_Flg.m_DscAlmCtr = 1) : (g_Flg.m_DscAlmCtr++);                                                
    return(tx_nbr);
}

/********************************************************************************************************
* Function Name : VhfDSCtest
*
* Description   : Creat test signal of DSC transmition.
*
* Arguments     : none
*      
* Returns       : none
********************************************************************************************************/
static  void  DscTest(void) 
{
    INT8U  vhf_old_ch = g_Flg.m_ChCur;            // Saved the channal number before test.
   
    *AT91C_PIOB_SODR = VHF_DSC_SW ;               // Set DSC switch in DSC state.   
    VHF_PTT_EN();                                 // Set VHF module is transfer state first.
    VhfChngCh(70);                                // Set the channel is 70.  
    FskSetMode(0x00);
    FSK_RS_EN(); 
  
    *AT91C_PIOB_CODR = FSK_XD;                    // Set '1', 2100bps
    OSTimeDlyHMSM(0, 1, 0, 0);                    // hold one minute
    *AT91C_PIOB_SODR = FSK_XD;                    // Set '0', 1300bps
    OSTimeDlyHMSM(0, 1, 0, 0);                    // hold one minute
	
    *AT91C_PIOB_CODR = VHF_DSC_SW ;               // Set DSC switch in voice state.
    VHF_PTT_DIS();                                // Set VHF module in receive state.   
    VhfChngCh(vhf_old_ch);                        // Resume the channal number before test.  
    FskSetMode(0x03);
    FSK_RS_DIS();
}
 
/********************************************************************************************************
* Function Name : StarCommSend
*
* Description   : Send the information neading transmitting condition to the star module
*
* Arguments     : none
*
* Returns       : BD1_FR_NO_ERR
*                 false    send failed
********************************************************************************************************/
INT8U  StarCommSend(void)
{   
	INT8U           buf[sizeof(CYC_BUF_TYPE)];         // 缓冲区
	INT8U          *ptx      = buf;                    // 指向待发送数据的起始地址
	INT8U          *pfkxx_msg = NULL;
	INT16U          tx_size  = 0;                      // 待发送数据的长度 	
	INT8U           tx_cond  = 0;                      // 发送条件(需要几个有效卫星波束)
	INT8U           rtn_ok   = 0;                      // 返回给舱内终端的发送成功代码
	INT8U           rtn_fail = 0;                      // 返回给舱内终端的发送失败代码
	INT8U           loop     = 0;                      // 尝试发送的次数
	INT8U           err      = 0;	                   // 接收出错代码
	INT32U          tmp; 
	  

    RdCycBuf(ptx);
    if ((ptx == NULL) || (*ptx != BD1_FR_HEADER))
	    return(BD1_FR_CHK_ERR);	                       // 起始地址为空或起始字节不为'$'，直接返回
	else if (*(ptx+1)=='T' && *(ptx+2)=='X' && *(ptx+3)=='S' && *(ptx+4)=='Q') {
        tx_cond  = 1;                                  // 设置信息发送所需场强条件
		rtn_ok   = B2T_CMD_TXSQ_OK;                    // 设置返回给终端的回执代码
		rtn_fail = B2T_CMD_TXSQ_FAIL;
	}
	else if (*(ptx+1)=='D' && *(ptx+2)=='W' && *(ptx+3)=='S' && *(ptx+4)=='Q') {	
		tx_cond  = 2;                                  // 设置信息发送所需场强条件
		rtn_ok   = B2T_CMD_DWSQ_OK;                    // 设置返回给终端的回执代码
		rtn_fail = B2T_CMD_DWSQ_FAIL;
	}
	else
		return(BD1_FR_CHK_ERR);	                       // 未定义指令，直接返回
	
	tx_size = ((*(ptx + 5)) << 8) + *(ptx + 6);        // 得到待发送数据的长度
  	g_Flg.m_SigDetectEn = 0;                           // 在卫星发送处理期间，暂停周期性场强检测
	do {
		if (g_Flg.m_WaveValid >= tx_cond)              // 满足信息发送的场强要求
		{
            g_Flg.m_WaveValid = 0;
            if (IS_9G_WORKING())                       // 9GHz模块正常工作？
            {
                NGRespDis();                           // 为防止电磁干扰引起误触发，关闭9G模块应答功能
                UsartWrite(1, ptx, tx_size, 0);        // 发送数据至卫星模块
                OSTimeDlyHMSM(0, 0, 0, DLY_MS_MATCH_TERM); // 延时，等待卫星模块数据发送完成 
                //OSTimeDlyHMSM(0, 0, 1, 0);             // 延时，等待卫星模块数据发送完成
                NGRespEn();                            // 恢复9GHz模块应答功能 
            }
            else
            	UsartWrite(1, ptx, tx_size, 0);
            	
            pfkxx_msg = (INT8U *)OSMboxPend(g_pMboxFkxx, 400, &err);
            
            if ((pfkxx_msg != NULL) && (*pfkxx_msg == 0x04)) // 发射频度未到
            {
            	tmp  = *(pfkxx_msg+1) << 24;
            	tmp += *(pfkxx_msg+2) << 16;
            	tmp += *(pfkxx_msg+3) << 8;
            	tmp += *(pfkxx_msg+4);
            	StarSigDetect();
            	g_Flg.m_SigDetectEn = 1;
            	OSTimeDlyHMSM(0, tmp/60, tmp%60, DLY_MS_MATCH_STAR);
            	g_Flg.m_SigDetectEn = 0;
            	continue;
            }
            if ((pfkxx_msg != NULL) && (*pfkxx_msg == 0x01)) // 发射失败
            {
                TermCommSend(rtn_fail, (INT8U *)0, 0); // 发送卫星数据发送失败代码至舱内终端
                g_Flg.m_SigDetectEn = 1;               // 卫星发送处理完成，恢复周期性场强检测功能
                return(BD1_FR_TX_FAIL);
            }
            if (rtn_ok == B2T_CMD_TXSQ_OK)
            {
                TermCommSend(rtn_ok, (INT8U *)0, 0);   // 发送卫星数据发送成功代码至舱内终端
                g_Flg.m_SigDetectEn = 1;               // 卫星发送处理完成，恢复周期性场强检测功能
            	return(BD1_FR_NO_ERR); 	
            }	              	  
            else if (rtn_ok == B2T_CMD_DWSQ_OK)
            {
                if (*(ptx+10)==0x24)
                    g_Flg.m_EmrgPos = 1;
                TermCommSend(rtn_ok, (INT8U *)0, 0);   // 发送卫星数据发送成功代码至舱内终端
                g_Flg.m_SigDetectEn = 1;               // 卫星发送处理完成，恢复周期性场强检测功能
            	return(BD1_FR_NO_ERR);
            }
		}
		StarSigDetect();	                           // 卫星场强检测
        OSTimeDlyHMSM(0, 0, SIG_DETECT_PERIOD, 0);	   // 延时1个检测周期，比常规场强检测周期短
    } while (++loop < (((g_SysInit.interval[0]<<8)+g_SysInit.interval[1])/SIG_DETECT_PERIOD));
			
    TermCommSend(rtn_fail, (INT8U *)0, 0);             // 发送卫星数据发送失败代码至舱内终端
    g_Flg.m_SigDetectEn = 1;                           // 卫星发送处理完成，恢复周期性场强检测功能
    return(BD1_FR_TX_FAIL); 
}

/********************************************************************************************************
* Function Name : StarCommRece
*
* Description   : Receive and decode the star information
*
* Arguments     : none
*
* Returns       : BD1_FR_CHK_ERR     check the star information error.
*                 BD1_FR_NO_ERR      the information check ok.     
*                 BD1_FR_UNDEF       if the information can't be distinguished.
********************************************************************************************************/
INT8U  StarCommRece(void)
{
    INT8U      m,n;                                         /* 临时循环变量                            */ 
    INT8U      result;                                      /* 临时运算结果                            */
    INT16U     fr_len = 0;                                      /* 临时数据帧长度（字节个数）              */
    INT8U      rx_buf[BD1_RX_BUF_SIZE];                      /* 临时接收缓冲区                          */
    volatile   INT8U  sub_fr = 0;

    
    memset(rx_buf, 0x00, BD1_RX_BUF_SIZE);                   /* 清零数据接收缓冲区.                     */ 

	//北斗
    UsartRead(1, rx_buf, BD1_RX_BUF_SIZE, 0);                /* 等待从串口1接收一帧数据                 */
    
    // I路信息($ILXX),用于误码测试，不进行帧头、长度及校验和的检测
    if (rx_buf[1]=='I' && rx_buf[2]=='L' && rx_buf[3]=='X' && rx_buf[4]=='X')     
    {   
        OS_CPU_SR     cpu_sr;
        // 误码测试标准数据,接收数据与此标准数据相比较，以统计误码个数
        static INT8U const lc_std_byte[24] = { 0x00,0x00,0x00,0x00,0x40,0x00,0x02,0x84,0x00,0x00,
                                               0x00,0xC5,0x3D,0xAC,0xD6,0x74,0x22,0xB5,0x81,0x4A,
                                               0x8C,0x15,0x93,0x54};	  		  	
        OS_ENTER_CRITICAL();
        for (m=10; m<34; m++)
        {     
            result = rx_buf[m] ^ lc_std_byte[m-10];      	/* 将接收数据与标准数据进行异或            */ 
            if (result)                                     /* 异或结果不为0,说明此字节包含误码比特    */
            {     
                for (n=0; n<8; n++)                         /* 遍历误码字节，统计错误比特数            */
                {
                    if ((result>>n) & 0x01)                 /* 相关位为 1，说明该位为误码位            */
                        g_ErrBits++;                        /* 错误比特数加 1                          */
                }
            }
        }
        g_TotBits += 192;                                   /* 更新测试总比特数:24*8=192 bits          */
        OS_EXIT_CRITICAL(); 
        return(BD1_FR_NO_ERR); 
    }
SUB_FRAME:   
    if (sub_fr)
    {
    	sub_fr = 0;
    	m = BD1_RX_BUF_SIZE - fr_len; 
    	for(n=0; n<m; n++)
    	    rx_buf[n] = rx_buf[n+fr_len];
    }
    fr_len = (rx_buf[5]<<8) + rx_buf[6];	                /* 计算接收数据帧的总长度                  */
    if ((fr_len < BD1_RX_BUF_SIZE) && (rx_buf[fr_len] == BD1_FR_HEADER))
    	sub_fr = 1;
    if ((rx_buf[0] != BD1_FR_HEADER) || (fr_len < 11) || (fr_len > BD1_RX_BUF_SIZE))
    {   
    	if (sub_fr)
    		goto SUB_FRAME;
    	else
    		return(BD1_FR_CHK_ERR);                            /* 数据帧帧头、长度错误，直接返回          */
    }    
    if (GetChecksum(rx_buf, fr_len-1) != rx_buf[fr_len-1])  /* 计算接收数据帧校验和不等于校验字节      */
    {
    	if (sub_fr)
    		goto SUB_FRAME;
        else
        	return(BD1_FR_CHK_ERR);                            /* 数据帧校验和错误，直接返回              */
    }	
    	
#if BD1_PROTOCOL_VERSION >= 4.0   
    /* 自检信息($ZJXX) */                         	  
    else if (rx_buf[1]=='Z' && rx_buf[2]=='J' && rx_buf[3]=='X' && rx_buf[4]=='X')  
    {   
    	if(( fr_len == 25 ) || ( fr_len == 21 ))
    	{
	        INT8U       wave_peak[10];
	        INT8U       tmp1, tmp2;
	        BD1S_ZJXX  *pzjxx = (BD1S_ZJXX *)rx_buf;
	             
	        for (g_Flg.m_WaveValid=0,m=0; m<10; m++)            /* 统计卫星当前有效波束的个数  */ 
	        {     
	            if (pzjxx->wave_pwr[m])                         /* 当卫星波束功率等级 >= 1,则为有效波束    */
	                g_Flg.m_WaveValid++;
	        }
	        
	        for (m=0; m<5; m++)
	        {
	        	wave_peak[2*m]   = m;
	        	wave_peak[2*m+1] = MAX(pzjxx->wave_pwr[2*m], pzjxx->wave_pwr[2*m+1]);
	        }
	        for (n=0; n<(5-1); n++)
	        {
	        	for(m=0; m<(5-n-1); m++)
	        	{
	        		if (wave_peak[2*m+1] < wave_peak[2*(m+1)+1])
	        		{
	        			tmp1 = wave_peak[2*m];
	        			tmp2 = wave_peak[2*m+1];
	        			wave_peak[2*m]       = wave_peak[2*(m+1)];
	        			wave_peak[2*m+1]     = wave_peak[2*(m+1)+1];
	        			wave_peak[2*(m+1)]   = tmp1;
	        			wave_peak[2*(m+1)+1] = tmp2;
	        		}
	        	}
	        }
	        for(m=0; m<3; m++)
	        {
	        	n = wave_peak[2*m];
	        	wave_peak[2*m]     = pzjxx->wave_pwr[2*n];
	        	wave_peak[2*m + 1] = pzjxx->wave_pwr[2*n+1];
	        }
	        
	        if (!g_Flg.m_WlMode)                                /* 在有缆模式下，将波束信息发送至舱内终端  */
	            TermCommSend(B2T_BD1_GLZK, wave_peak, 6);  
	        
	//        if (!g_Flg.m_WlMode)                                /* 在有缆模式下，将波束信息发送至舱内终端  */
	//            TermCommSend(B2T_BD1_GLZK, pzjxx->wave_pwr, 6);
		}
    }
    
    /* IC信息($ICXX) */
    else if (rx_buf[1]=='I' && rx_buf[2]=='C' && rx_buf[3]=='X' && rx_buf[4]=='X')           
    {   
    	if( fr_len == 22 )
    	{
	    	BD1S_ICXX  *picxx = (BD1S_ICXX *)rx_buf;
	    	        
	        memcpy(g_SysInit.user_id, picxx->user_id, 3);       /* 本机用户地址保存于全局变量g_SysInit中   */        
	        g_SysInit.interval[0] = picxx->serve_freq[0];       /* 通信时间间隔保存于全局变量g_SysInit中   */ 
	        g_SysInit.interval[1] = picxx->serve_freq[1];
	        if (!g_Flg.m_WlMode)
	            TermCommSend(B2T_BD1_ICXX, picxx->user_id, 14);
	     }
    }
#else 
    /* 功率状况($GLZK) */       	             
    else if (rx_buf[1]=='G' && rx_buf[2]=='L' && rx_buf[3]=='Z' && rx_buf[4]=='K')
    {   
    	if( fr_len == 17 )
    	{
	    	BD1S_GLZK  *pglzk = (BD1S_GLZK *)rx_buf;
	    	                     	
	        for (g_Flg.m_WaveValid=0,m=0; m<6; m++)
	        {    
	            if (pglzk->wave_pwr[m])
	                g_Flg.m_WaveValid++;
	        }
	        if (!g_Flg.m_WlMode)
	     }
		TermCommSend(B2T_BD1_GLZK, pglzk->wave_pwr, 6);
    }
#endif

    /* 通信信息($TXXX) */ 
    else if (rx_buf[1]=='T' && rx_buf[2]=='X' && rx_buf[3]=='X' && rx_buf[4]=='X')  
    {   
    	BD1S_TXXX  *ptxxx = (BD1S_TXXX *)rx_buf;
    	         
        if (fr_len > sizeof(BD1S_TXXX))
        	fr_len = sizeof(BD1S_TXXX);
        
        // 代码通信，对控制指令进行解码、处理并将相关信息按艇载设备内部通信协议的格式要求发送至舱内终端
        if (((ptxxx->type & 0xf0) == 0x60) && (fr_len >= 22)) 
        {           	
            if (ptxxx->msg[2]=='B' && ptxxx->msg[3]=='W' && ptxxx->msg[4]=='Q' && ptxxx->msg[5]=='R')
            {   // 汉字、代码报文确认(BWQR)Hex: 42 57 51 52   
            	if (!g_Flg.m_WlMode) 
                    TermCommSend(B2T_CMD_BWOK, &ptxxx->msg[6], 1);
            }                     	                
            else if (ptxxx->msg[2]=='Z' && ptxxx->msg[3]=='D' && ptxxx->msg[4]=='Q' && ptxxx->msg[5]=='R')      
            {   // 报警自动确认(ZDQR)Hex: 5A 44 51 52
            	if (!g_Flg.m_WlMode)
            	{  
            		ptxxx->msg[5] = 0x00;
                    TermCommSend(B2T_CMD_BJOK, &ptxxx->msg[5], 2);       /* 通知舱内终端,收到报警自动确认信息  */
                }
            }                                
            else if (ptxxx->msg[2]=='R' && ptxxx->msg[3]=='G' && ptxxx->msg[4]=='Q' && ptxxx->msg[5]=='R')
            {   // 报警人工确认(RGQR)Hex: 52 47 51 52 
                g_Flg.m_NatAlm  = 0;                             /* 清除本地报警标志                   */
                g_Flg.m_PassAlm = 0;           	                 /* 清除被动报警标志                   */
                if (g_Flg.m_WlMode) 
                    g_Flg.m_LampMode = 0x00;                     /* 无缆模式下收到人工确认,关闭闪光灯  */
                else
                {
                    ptxxx->msg[5] = 0x01;
                    TermCommSend(B2T_CMD_BJOK, &ptxxx->msg[5], 2);       /* 通知舱内终端,收到报警人工确认信息  */
                }
            }          	          	
            else if (ptxxx->msg[2]=='Q' && ptxxx->msg[3]=='D' && ptxxx->msg[4]=='B' && ptxxx->msg[5]=='J')
            {   // 重新启动报警(QDBJ)Hex: 51 44 42 4A  
                if(!g_Flg.m_NatAlm)                              /* 在主动报警期间，无法启动被动报警   */
                {     
                    if (g_Flg.m_WlMode)                          /* 在无缆模式下收到启动报警,开启闪光灯*/   
                    {
                    	g_Flg.m_LampMode = 0x04;                 /* 闪光灯闪烁模式为：20S 闪两次       */
                        OSTimeDlyResume(LAMP_TASK_PRIO);
                        OSTaskResume(LAMP_TASK_PRIO);                      
                    }
                    else
                    {
                        ptxxx->msg[5] = 0x02;
                        TermCommSend(B2T_CMD_BJOK, &ptxxx->msg[5], 2);   /* 通知舱内终端,收到重新启动报警信息  */
                    }
                    g_Flg.m_PassAlm   = 1;             	         /* 被动报警标志置1                    */
                    g_Flg.m_BdAlmCtr  = 1;                       /* 报警次数重新开始计数               */
                    g_Flg.m_ExistPos  = 0;	                     /* 重新定位                           */
                    OSTimeDlyHMSM(0, 0, 2, 0);
                    OSTaskResume(STAR_ALARM_TASK_PRIO);          /* 恢复报警任务                       */
                    if (IS_TONE_PWR_ON())                        /* 在VHF模块工作的情况下，恢复DSC报警 */
                    {                    
                    	g_Flg.m_DscAlmCtr = 1;                   /* DSC报警次数置1，表示开始新一轮报警 */
                        OSTimeDlyResume(DSC_ALARM_TASK_PRIO);    /* 取消DSC报警任务延时                */
                        OSTaskResume(DSC_ALARM_TASK_PRIO);
                    }
                }
            }
            else if (ptxxx->msg[2]=='Z' && ptxxx->msg[3]=='H' && ptxxx->msg[4]=='Q' && ptxxx->msg[5]=='R')
            {   // 自环确认(ZHQR)Hex: 5A 48 51 52 
            	if (!g_Flg.m_WlMode)            		
            	    TermCommSend(B2T_CMD_ZHOK, (INT8U *)0, 0);           /* 通知舱内终端,收到岸基自环确认信息  */
            }
            else if (ptxxx->msg[2]=='C' && ptxxx->msg[3]=='C' && ptxxx->msg[4]=='C' && ptxxx->msg[5]=='X')
            {   // 擦除浮标内设备程序(CCCX)Hex: 43 43 43 58
            	if (g_Flg.m_WlMode)                              /* 在无缆模式下,实现浮标内设备自毁功能*/
            	    StarZHSQ();                                  /* 发送自毁申请，销毁北斗用户机及SIM卡*/
            }
            else if (!g_Flg.m_WlMode)
            {   // 在有缆模式下，转发代码短报文
                ptxxx->msg_len[0] = ptxxx->type & 0xf0;
                ptxxx->msg_len[1] = ptxxx->post_id[0]; 
                ptxxx->msg[0]     = ptxxx->post_id[1];
                ptxxx->msg[1]     = ptxxx->post_id[2];
                TermCommSend(B2T_BD1_TXXX, &ptxxx->msg_len[0], fr_len - 18);   /* fr_len - 20(通信信息固定长度) - 2(2字节报头) + 4(类型、发信方地址) */
            }
        }
        else if (((ptxxx->type & 0xf0) == 0x40) && (!g_Flg.m_WlMode) && (fr_len >= 22)) 
        {   // 在有缆模式下，转发汉字短报文		
            ptxxx->msg_len[0] = ptxxx->type & 0xf0;
            ptxxx->msg_len[1] = ptxxx->post_id[0]; 
            ptxxx->msg[0]     = ptxxx->post_id[1];
            ptxxx->msg[1]     = ptxxx->post_id[2]; 
            TermCommSend(B2T_BD1_TXXX, &ptxxx->msg_len[0], fr_len - 18);
        }  	        	
    }
    
    /* 定位信息($DWXX) */
    else if (rx_buf[1]=='D' && rx_buf[2]=='W' && rx_buf[3]=='X' && rx_buf[4]=='X')        
    {   
    	if( fr_len == 31 )
    	{
	        memcpy((INT8U *)&g_DWXX, rx_buf, sizeof(BD1S_DWXX));  /* 保存定位信息，以便报警时使用       */
	        g_Flg.m_ExistPos = 1;                    	             /* 生成卫星、DSC报警信息时将查询此标志*/
	        OSTimeDlyHMSM(0, 0, 1, DLY_MS_MATCH_TERM);
	        //OSTimeDlyHMSM(0, 0, 0, DLY_MS_MATCH_TERM);
	        if (!g_Flg.m_WlMode)                                     /* 有缆方式下，将定位信息发送至终端   */
	            TermCommSend(B2T_BD1_DWXX, &g_DWXX.time.hour, 16);
	     }
    }
       
    /* 时间信息($SJXX) */
    else if (rx_buf[1]=='S' && rx_buf[2]=='J' && rx_buf[3]=='X' && rx_buf[4]=='X')
    {   
    	if( fr_len == 18 )
    	{
	    	BD1S_SJXX  *psjxx = (BD1S_SJXX *)rx_buf;
	    	
	    	if (!g_Flg.m_WlMode)       
	            TermCommSend(B2T_BD1_SJXX, psjxx->bd1_time, 7);               /* 将时间信息按内部格式发送至舱内终端.*/         
	    }
    } 
    
    /* 自毁确认($ZHQR) */   
    else if (rx_buf[1]=='Z' && rx_buf[2]=='H' && rx_buf[3]=='Q' && rx_buf[4]=='R')     
    {   
    	if( fr_len == 15 )
    	{
	    	BD1S_ZHQR  *pzhqr = (BD1S_ZHQR *)rx_buf;
	    	TermCommSend( B2T_CMD_ZHQR,(INT8U*)0,0);
	        if (g_Flg.m_WlMode && pzhqr->zh_flag[0]==0xAA && pzhqr->zh_flag[1]==0x55 && pzhqr->zh_flag[2]==0x55 && pzhqr->zh_flag[3]==0xAA)
			{
				TermCommSend( B2T_CMD_ZHQR,(INT8U*)0,0);
	            FlashEraseAll();                                     /* 擦除浮标内控制单元程序 */ 

	        }
			
	     } 
    }
    
    /* 反馈信息($FKXX) */
    else if (rx_buf[1]=='F' && rx_buf[2]=='K' && rx_buf[3]=='X' && rx_buf[4]=='X')
    {   
    	if( fr_len == 16 )
    	{
	    	BD1S_FKXX  *pfkxx = (BD1S_FKXX *)rx_buf;
	    	static INT8U   fkxx_msg[5] = {0, 0, 0, 0, 0};
	    	
	        /*
	        0x00:成功，对应指令的四个ASCII字符
	        0x01:失败，对应指令的四个ASCII字符
	        0x04:发射频度未到，需要等待的时间（S）
	        */			

			TermCommSend(B2T_CMD_FKXX, &pfkxx->fk_info[3],1);
			
	        if ((pfkxx->fk_flag == 0x00) || (pfkxx->fk_flag == 0x01) || (pfkxx->fk_flag == 0x04))
	        {
	        	memcpy(fkxx_msg, (INT8U *)&(pfkxx->fk_flag), 5);
	        	OSMboxPost(g_pMboxFkxx, (void *)fkxx_msg);           /* 发送卫通反馈信息至邮箱             */
	        }
	    }
		
    }
    
    /* 工厂设置($GCSZ) */   
    else if (rx_buf[1]=='G' && rx_buf[2]=='C' && rx_buf[3]=='S' && rx_buf[4]=='Z')     
    {   
    	BATTERY_INFO_TYPE  *pbat  = (BATTERY_INFO_TYPE *)(rx_buf+11);
    	INT16U              year  = (pbat->m_InitDate[0] << 8) + pbat->m_InitDate[1];
    	INT8U               month = pbat->m_InitDate[2];
    	INT8U               date  = pbat->m_InitDate[3];
    	INT8U              *ptr   = NULL;
    	
    	// Maxium days per month for leap year and normal year 
    	INT8U const         MDPMForLeapYear[12] = {31,29,31,30,31,30,31,31,30,31,30,31};
    	INT8U const         MDPMForNormYear[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
	    
	    // operation feedback message
	    static INT8U       *pfb_msg[] = {
    	    (INT8U *)"\n\r****Congratulation!operate successful.****\n\r",
    	    (INT8U *)"\n\r****Operate DS2788 failed,please check the device and try again!****\n\r",
    	    (INT8U *)"\n\r****Invalid key,please check and try again!****\n\r",
    	    (INT8U *)"\n\r****Invalid year,please check and try again!****\n\r",
    	    (INT8U *)"\n\r****Invalid month,please check and try again!****\n\r",
    	    (INT8U *)"\n\r****Invalid date,please check and try again!****\n\r",
    	};
    	    		    	          
    	switch (rx_buf[10])
    	{
    		case 0x01:
    		case 0x02:                    
    			if (pbat->m_InitFlag != DS2788_INIT_FLAG)
    			    ptr = *(pfb_msg+2);    	  
    			else if ((year > 2099) || (year < 2000))
    			    ptr = *(pfb_msg+3);        	        
    			else if ((month > 12) || (month == 0))
    			    ptr = *(pfb_msg+4); 
    			else if (((year%4 == 0) && (year%100!=0)) || (year%400 == 0))
    			{
    			    if (date > MDPMForLeapYear[month-1] || (date == 0))
    			        ptr = *(pfb_msg+5);
    			} 
    			else
    			{	
    			    if ((date > MDPMForNormYear[month-1]) || (date == 0))
    			        ptr = *(pfb_msg+5);  			     			    
    			}
    			
    			if ((ptr == NULL) && (rx_buf[10] == 0x01))
        	    {    
        	        memcpy((INT8U *)&g_BatInfo, (INT8U *)pbat, sizeof(BATTERY_INFO_TYPE));
                    ptr = (DS2788_CfgBatInfo()==0) ? *pfb_msg : *(pfb_msg+1);
        	    }
        	    else if ((ptr == NULL) && (rx_buf[10] == 0x02))
        	    {    
        	        memcpy((INT8U *)g_BatInfo.m_BatID, (INT8U *)pbat->m_BatID, 6);
        	        ptr = (DS2788_CfgBatIDAndDate()==0) ? *pfb_msg : *(pfb_msg+1);
        	    }
                
                if (ptr != NULL)         
                    UsartWrite(1, ptr, GetStringLength(ptr), 0);
              
                break;

            case 0x03:
                DS2788_RecallData(0x20, sizeof(BATTERY_INFO_TYPE));
                if (DS2788_ReadData((unsigned char *)rx_buf, 0x20, sizeof(BATTERY_INFO_TYPE)) == 0)
                    UsartWrite(1, rx_buf, sizeof(BATTERY_INFO_TYPE), 0);
                else
                    UsartWrite(1, *(pfb_msg+1), GetStringLength(*(pfb_msg+1)), 0);
                break;			                 
            default:
            	break;
        }
    }
	

	/*增强信号($ZQXH)*/
	else if(rx_buf[1]=='Z' && rx_buf[2]=='Q' && rx_buf[3]=='X' && rx_buf[4]=='H')
	{
	
		//BD1S_ZQXX*pzqxh=(BD1S_ZQXX*)rx_buf;

	    if( fr_len == 8 )
	    {
			TermCommSend(B2T_CMD_ZQXH,(INT8U*)0, 0);
	    }
	}
	/*频度调整($PDTZ)*/
	else if(rx_buf[1]=='P' && rx_buf[2]=='D' && rx_buf[3]=='T' && rx_buf[4]=='Z')
	{
	  if( fr_len == 10 )
	  {
		BD1S_PDTZ*ppdtz=(BD1S_PDTZ*)rx_buf;
		TermCommSend(B2T_CMD_PDTZ,&ppdtz->dep, 2);	
	  }
	}
	/*开机报告($KJBG)*/
	else if(rx_buf[1]=='K' && rx_buf[2]=='J' && rx_buf[3]=='F' && rx_buf[4]=='K')
	{
	   	if( fr_len == 8 )
	   	{
			//BD1S_KJBG*pkjbg=(BD1S_KJBG*)rx_buf;
			INT8U flag=1;
			TermCommSend(B2T_CMD_KGST,&flag, 0);
	   	}
	}
	/*关机报告($GJBG)*/
	else if(rx_buf[1]=='G' && rx_buf[2]=='J' && rx_buf[3]=='F' && rx_buf[4]=='K')
	{
	   if( fr_len == 8 )
	   {
		 //BD1S_GJBG*pgjbg=(BD1S_GJBG*)rx_buf;
		 INT8U flag=0;
		 TermCommSend(B2T_CMD_KGST,&flag, 0);
	   }
			
	}
	/*静默反馈($JMFK)*/
	else if(rx_buf[1]=='J' && rx_buf[2]=='M' && rx_buf[3]=='F' && rx_buf[4]=='K')
	{

	  if( fr_len == 9 )
	  {
		BD1S_JMFK*pjmfk=(BD1S_JMFK*)rx_buf;
		
		TermCommSend(B2T_CMD_JMST,&pjmfk->JMST, 1);
	  }
		
	}

	/*授启成功($SCCG)*/
	else if(rx_buf[1]=='S' && rx_buf[2]=='Q' && rx_buf[3]=='C' && rx_buf[4]=='G')
	{
	   if( fr_len == 9 )
	   {
		//BD1S_SQCG*psqcg=(BD1S_SQCG*)rx_buf;
		TermCommSend(B2T_CMD_JMST,(INT8U*)0, 0);
	   }
		
	}
	/*MY时效*/
	else if(rx_buf[1]=='S'&& rx_buf[2]=='X'&& rx_buf[3]=='X'&& rx_buf[4]=='X')
	{
	    if( fr_len == 20 )
	   {
		BD1S_SXXX*psxxx=(BD1S_SXXX*)rx_buf;
		INT8U MYDATE[12];
		memcpy(MYDATE,&psxxx->date[0],12);
		TermCommSend(B2T_CMD_SXXX,&MYDATE[0],12);
	   }
		
	}
	/*欺骗信号*/
	else if(rx_buf[1]=='Q'&&rx_buf[2]=='P'&&rx_buf[3]=='X'&&rx_buf[4]=='H')
	{
		if( fr_len == 9 )
	   {
		BD1S_QPXH*pqpxh=(BD1S_QPXH*)rx_buf;

		TermCommSend(B2T_CMD_FXTS,&pqpxh->flag,1);
	   }
	
	}
	/*非j码风险提示*/
	else if(rx_buf[1]=='F'&&rx_buf[2]=='X'&&rx_buf[3]=='T'&&rx_buf[4]=='S')
	{
	  if( fr_len == 10 )
	  {
		BD1S_FXTX*pfxts=(BD1S_FXTX*)rx_buf;
		TermCommSend(B2T_CMD_FXTS,&pfxts->alarm,1);
	  }
	}
	
	
    else
    {   
    	if (sub_fr)
    	{	
    		fr_len = (rx_buf[5]<<8) + rx_buf[6];
    		goto SUB_FRAME;
    	}
    	else
    		return(BD1_FR_UNDEF);                                   /* 未定义卫星数据帧                   */
    }
               
    if (sub_fr)
    {	
    	fr_len = (rx_buf[5]<<8) + rx_buf[6];
    	goto SUB_FRAME;
    }
    else
    	{
    	return(BD1_FR_NO_ERR);  
    	}
}

/********************************************************************************************************
* Function Name : StarAlarmProc
*
* Description   : Assemble the star alarm $TXSQ,achieve the task of alarm
*
* Arguments     : none
*
* Returns       : none
********************************************************************************************************/
void  StarAlarmProc(void)
{
	INT8U          loop  = 0;
    INT16U         tick  = (g_SysInit.interval[0]<<8) + g_SysInit.interval[1] + 1; // 卫星通信时间间隔
    BD1S_TXSQ     *ptxsq = (BD1S_TXSQ *)&g_TXSQ;                 // 指针指向通信申请全局变量起始地址
    ALM_MSG_TYPE  *pmsg  = (ALM_MSG_TYPE *)ptxsq->msg;           // 指针指向通信申请全局变量之通信信息起始地址
    static  BSP_DATE_TIME_TYPE  start_alarm_time;                // 保存本组报警的起始时间
           	                              					
    //-------------- Get the time and location information from star or terminal ------------------------
    if (!g_Flg.m_ExistPos && (g_Flg.m_BdAlmCtr != 1)) 
    {        
        while (!g_Flg.m_ExistPos && (loop < 3))                  /* 未收到定位信息时，连续三次普通定位 */
        {
            StarDWSQ(0x04);                                      /* 普通定位                           */
            loop++;		                                         /* 定位次数加 1                       */
            OSTimeDlyHMSM(0, tick/60, tick%60, 0);               /* 延时1个卫星通信时间间隔            */
        }        		                          
        if (!g_Flg.m_ExistPos && !g_Flg.m_WlMode && !g_Flg.m_ManPos) 
            TermCommSend(B2T_CMD_CXWZ, (INT8U *)0, 0);           /* 向舱内终端发送查询人工位置信息指令 */
    } 
    
    //----------------------------- Assemble the alarm message of the $TXSQ -----------------------------	
    ptxsq->len[0] = (BD1_TXSQ_FIXED_LEN + ALM_MSG_TOTAL_LEN) >> 8;
    ptxsq->len[1] = (BD1_TXSQ_FIXED_LEN + ALM_MSG_TOTAL_LEN);
    memcpy(ptxsq->user_id, g_SysInit.user_id, 3);                /* 设置本机用户ID                     */
    memcpy(ptxsq->rece_id, g_SysInit.bank_id, 3);                /* 设置岸基收信方ID                   */
	ptxsq->type   = 0x46;                                        /* 设置通信申请为代码格式             */
    ptxsq->msg_len[0] = (ALM_MSG_TOTAL_LEN << 3) >> 8;           /* 电文内容包含比特数高字节           */
    ptxsq->msg_len[1] = (ALM_MSG_TOTAL_LEN << 3) & 0xFF;         /* 电文内容包含比特数低字节           */
#if BD1_PROTOCOL_VERSION >= 4.0
    ptxsq->is_resp = 0;                                          /* 不应答                             */
#endif
    pmsg->msg_header[0] = ALM_MSG_HEADER0;
    pmsg->msg_header[1] = ALM_MSG_HEADER1_CODE;
    pmsg->msg_instr[0]  = 'B';
    pmsg->msg_instr[1]  = 'J';
    pmsg->msg_instr[2]  = 'X';
    pmsg->msg_instr[3]  = 'X';
    pmsg->msg_ctr  = g_Flg.m_BdAlmCtr;
   	pmsg->msg_flg  = (!g_Flg.m_WlMode) ? (1 << 3) : 0;           /* 设置有缆(bit3=1)无缆(bit3=0)标志   */
    pmsg->msg_flg |= (g_Flg.m_NatAlm) ? (1 << 2) : 0;            /* 设置主动(bit2=1)被动(bit2=0)报警标志*/
   		   		
    memcpy(pmsg->distress_ship_id, g_SysInit.ship_id, 5);	     /* 设置遇险船名代码                   */
	pmsg->distress_kind = g_SysInit.distress_kind;               /* 设置遇险性质                       */
    
    // 保存本组报警第一次报警的时间
    if (g_Flg.m_BdAlmCtr == 1)
    	start_alarm_time = BspTime;

    // 本组报警第一次报警时间填充至报警信息
    pmsg->distress_time[0] = start_alarm_time.year >> 8;
    pmsg->distress_time[1] = start_alarm_time.year & 0xff;
    memcpy(&pmsg->distress_time[2], (INT8U *)&start_alarm_time.month, 6);

    if (g_Flg.m_ExistPos)                                        /* 优先使用卫星定位信息填充报警信息   */
    {
        memcpy((INT8U *)&pmsg->distress_pos.time, (INT8U *)&g_DWXX.time, 4);
        pmsg->distress_pos.quad = 0x00;                          /* 设置位置所属象限，北斗定位为0象限  */
        memcpy((INT8U *)&pmsg->distress_pos.lon, (INT8U *)&g_DWXX.lon, 8);
        pmsg->msg_flg |= 0x01;                                   /* 指示位置信息类型(北斗卫星定位数据) */ 
	}	
    else
    {
        memcpy((INT8U *)&pmsg->distress_pos, (INT8U *)&g_ManPos, sizeof(ALM_MSG_POS_TYPE));
        pmsg->msg_flg |= g_Flg.m_ManPos ? 0x02 : 0x00;           /* 指示位置信息类型(人工输入或默认值) */
    }
    *(ptxsq->msg + ALM_MSG_TOTAL_LEN) = GetChecksum((INT8U *)ptxsq, BD1_TXSQ_FIXED_LEN + ALM_MSG_TOTAL_LEN - 1);
    
    if ((g_Flg.m_NatAlm || g_Flg.m_PassAlm) == 0)
    	return;
    	
    loop = WrCycBuf((INT8U *)ptxsq, BD1_TXSQ_FIXED_LEN + ALM_MSG_TOTAL_LEN);   /* 写入环形缓冲区                     */
    if (loop == CYC_BUF_NO_ERR)
    	(g_Flg.m_BdAlmCtr >= 0xFF) ? (g_Flg.m_BdAlmCtr = 1) : g_Flg.m_BdAlmCtr++;
    
    if (!g_Flg.m_ExistPos && !g_Flg.m_EmrgPos)                   /* 未收到定位信息且未进行过紧急定位？ */
    {
        OSTimeDlyHMSM(0, 0, 2, 0);
        StarDWSQ(0x24);                                          /* 发送紧急定位                       */
        OSTimeDlyHMSM(0, tick/60, tick%60, 0);                   /* 延时1个卫星通信时间间隔            */
    } 
    OSTimeDlyHMSM(0, tick/60, tick%60, 0);                       /* 延时1个卫星通信时间间隔            */
}

/********************************************************************************************************
* Function Name : StarSigDetect
*
* Description   : Enable the star signal output per 1s.
*
* Arguments     : none
*
* Returns       : none
********************************************************************************************************/
void  StarSigDetect(void)
{
#if BD1_PROTOCOL_VERSION >= 4.0
    UsartWrite(1, (INT8U *)&g_XTZJ, BD1_XTZJ_TOTAL_LEN, 0); 
#else
    UsartWrite(1, (INT8U *)&g_GLJC, BD1_GLJC_TOTAL_LEN, 0);
#endif
}

/********************************************************************************************************
* Function Name : StarDWSQ
*
* Description   : Send the orient command to the star module
*
* Arguments     : type   0x24  emergent orient
*                        0x04  normal orient
*                        0x00  test mode type
* Returns       : 0      assemble DWSQ success,post the pointer to mailbox;or send ok(in test mode)
*                 1      assemble DWSQ faild; or send err(in test mode)
********************************************************************************************************/
static  INT8U  StarDWSQ(INT8U type)
{ 		
	static  BD1S_DWSQ  sl_dwsq = {'$','D','W','S','Q',0,sizeof(BD1S_DWSQ)};
	
	memcpy(sl_dwsq.user_id, g_SysInit.user_id, 3);	
	memset(&sl_dwsq.type, 0x00, sizeof(BD1S_DWSQ) - 10);	
	sl_dwsq.type     = (type == 0x24) ? 0x24 : 0x04;
	sl_dwsq.checksum = GetChecksum((INT8U *)&sl_dwsq, sizeof(BD1S_DWSQ)-1);
	    	    
	if ((type==0x04) || (type==0x24)) 
	{	
		WrCycBuf((INT8U *)&sl_dwsq, sizeof(BD1S_DWSQ));
	}
    else if (type == 0x00)
    	UsartWrite(1, (INT8U *)&sl_dwsq, sizeof(BD1S_DWSQ), 0);
	else
		return (1);
	return (0);	
}

/********************************************************************************************************
* Function Name : StarSJSC
*
* Description   : to get "beidou" system time.
*
* Arguments     : out_freq   output frequency.
*
* Returns       : returned value of the fuction "OSMboxPost"
********************************************************************************************************/
static  INT8U  StarSJSC(INT16U out_freq)
{
	static  BD1S_SJSC  sl_sjsc = {'$','S','J','S','C',0,sizeof(BD1S_SJSC),0,0,0,0,0,0};
	
	memcpy(sl_sjsc.user_id, g_SysInit.user_id, 3);
	sl_sjsc.out_freq[0] = out_freq >> 8;
	sl_sjsc.out_freq[1] = out_freq;
	sl_sjsc.checksum    = GetChecksum((INT8U *)&sl_sjsc, sizeof(BD1S_SJSC)-1);
	return(UsartWrite(1, (INT8U *)&sl_sjsc, sizeof(BD1S_SJSC), 0));
} 

/********************************************************************************************************
* Function Name : StarLoopTest
*
* Description   : Send test message to the bank,check the connectivity of the whole system
*
* Arguments     : ptxsq  the pointer of BD1S_TXSQ variable.
*
* Returns       : returned value of the fuction "OSMboxPost"
********************************************************************************************************/
static  INT8U  StarLoopTest(BD1S_TXSQ *ptxsq)
{	    		  
    ptxsq->len[0]     = (BD1_TXSQ_FIXED_LEN + 8) >> 8;
    ptxsq->len[1]     = (BD1_TXSQ_FIXED_LEN + 8);
    ptxsq->type       = 0x46;                               /* Message type is code.                   */
    ptxsq->msg_len[0] = (8 << 3) >> 8;
    ptxsq->msg_len[1] = (8 << 3); 		
    memcpy(ptxsq->user_id, g_SysInit.user_id, 3);
    memcpy(ptxsq->rece_id, g_SysInit.bank_id, 3);
    
    ptxsq->msg[0]     = ALM_MSG_HEADER0;
    ptxsq->msg[1]     = ALM_MSG_HEADER1_CODE;
    ptxsq->msg[2]     = 'Z';
    ptxsq->msg[3]     = 'H';
    ptxsq->msg[4]     = 'C';
    ptxsq->msg[5]     = 'S';
    ptxsq->msg[6]     = 0x01;
    ptxsq->msg[7]     = 0x00;
  	ptxsq->msg[8]     = GetChecksum((INT8U *)ptxsq, BD1_TXSQ_FIXED_LEN + 8 - 1);
    return (WrCycBuf((INT8U *)ptxsq, BD1_TXSQ_FIXED_LEN + 8));
}
 
/********************************************************************************************************
* Function Name : StarICJC
*
* Description   : Query the IC information,including user ID and service frequernce etc.
*
* Arguments     : frame_nbr    0,Query itself user ID.
*
* Returns       : the returned value of UsartWrite();
********************************************************************************************************/
#if BD1_PROTOCOL_VERSION >= 4.0
static  INT8U  StarICJC(INT8U frame_nbr) 
{   
    BD1S_ICJC  l_icjc = {'$', 'I', 'C', 'J', 'C', 0, sizeof(BD1S_ICJC), 0, 0, 0, 0, 0};
    		
    l_icjc.fr_nbr        = frame_nbr;		
    l_icjc.checksum      = GetChecksum((INT8U *)&l_icjc, sizeof(BD1S_ICJC) - 1);
    return (UsartWrite(1, (INT8U *)&l_icjc, sizeof(BD1S_ICJC), 0));
}
#endif

/********************************************************************************************************
* Function Name : StarBitErrTest                            
*
* Description   : star module's error code test
*
* Arguments     : iq         0: test I ramus;  1: test Q ramus; 
*                 wave_nbr   select the wave number(1~6). 
*                 min        test time  unit: minute
*       
* Returns       : none                  
********************************************************************************************************/
void  StarBitErrTest(INT8U iq, INT8U wave_nbr, INT8U min)
{
	INT8U      temp[13] = {'$', 'W', 'M', 'C', 'S', 0, 13, 0, 0, 0, 0, 0, 0};
		  	
	g_ErrBits = 0;		                                    /* clear global error bit counter          */
	g_TotBits = 0;		                                    /* clear global test bit counter           */
	g_Flg.m_SigDetectEn = 0;                                /* 在误码测试期间，暂停周期性卫星信号检测  */
	if ((iq < 2) && (wave_nbr < 7) && (wave_nbr > 0))
	{
		StarYHZL();		                                    /* set the star module in test state       */
		OSTimeDlyHMSM(0, 0, 1, DLY_MS_MATCH_STAR);          /* wait for the star module standy 	       */
	    memcpy(&temp[7], g_SysInit.user_id, 3);	
	    temp[10] = iq;
	    temp[11] = wave_nbr;
        temp[12] = GetChecksum(temp, 12);
        UsartWrite(1, temp, 13, 0);		                    /* send $WMCS command                      */
        OSTimeDlyHMSM(0, min, 0, 0);                        /* set the test duration 	               */
        StarJSZL();	                                        /* cancle the test state                   */
    	//OSTimeDlyHMSM(0, 0, 0, 200);                        /* wait for the star module standy 	       */
    }
    temp[0] = g_ErrBits >> 24;                              /* fotmat the test result                  */
    temp[1] = g_ErrBits >> 16;
    temp[2] = g_ErrBits >> 8;
    temp[3] = g_ErrBits;
    temp[4] = g_TotBits >> 24;
    temp[5] = g_TotBits >> 16;
    temp[6] = g_TotBits >> 8;
    temp[7] = g_TotBits;
    TermCommSend(B2T_CMD_BD1_WMXX, temp, 8);                /* 发送误码测试结果至舱内终端              */
    g_Flg.m_SigDetectEn = 1;                                /* 误码测试结束，恢复周期性卫星信号检测    */
}

/********************************************************************************************************
* Function Name : StarYHZL
*
* Description   : This function is used to set the star module in the test mode.
*                 24 59 48 5A 4C 00 27 00 00 00 4D 53 53 5A 00 00 03 D5 11 03
*                 D5 11 06 00 01 04 01 C4 72 B1 F5 00 0F 05 5A 0F 2F 8F 1C
*
* Arguments     : none
*
* Returns       : the returned value of UsartWrite();
********************************************************************************************************/
static  INT8U  StarYHZL(void)
{
	INT8U  temp[39] = {'$',  'Y',  'H',  'Z',  'L',  0x00, 0x27, 0x00, 0x00, 0x00,
	                   0x4D, 0x53, 0x53, 0x5A, 0x00, 0x00, 0x03, 0xD5, 0x0E, 0x1F,
	                   0xF0, 0x4D, 0x06, 0x00, 0x01, 0x04, 0x01, 0xC4, 0x72, 0xB1,
	                   0xF5, 0x00, 0x0F, 0x05, 0x5A, 0x0F, 0x2F, 0X8F, 0x66};
				
	memcpy(&temp[16], g_SysInit.user_id, 3);
    temp[38] = GetChecksum(temp, 38);
    return (UsartWrite(1, temp, 39, 0));
}

/********************************************************************************************************
* Function Name : StarJSZL
*
* Description   : This function is used to finish the correlative data of star module.
*
* Arguments     : none
*
* Returns       : the returned value of UsartWrite();
********************************************************************************************************/
static  INT8U  StarJSZL(void)
{
	INT8U  temp[14] = {'$', 'J', 'S', 'Z', 'L', 0, 14, 0, 0, 0, 0x55, 0x55, 0x55, 0};
			
	memcpy(&temp[7], g_SysInit.user_id, 3);	
    temp[13] = GetChecksum(temp, 13);    
    return (UsartWrite(1, temp, 14, 0));
}

/********************************************************************************************************
* Function Name : StarZHSQ
*
* Description   : This function is used to destroy the card of star module.
*
* Arguments     : none
*
* Returns       : the returned value of UsartWrite();
********************************************************************************************************/
static  INT8U  StarZHSQ(void)
{
	INT8U  temp[15] = {'$', 'Z', 'H', 'S', 'Q', 0, 15, 0, 0, 0, 0x55, 0xAA, 0xAA, 0x55, 0};

	memcpy(&temp[7], g_SysInit.user_id, 3);
    temp[14] = GetChecksum(temp, 14);
    return (UsartWrite(1, temp, 15, 0));
} 

/*开机报告*/
static INT8U  StarKJBG(void)
{
	INT8U temp[8]={'$', 'K', 'J', 'B', 'G',0,8,28};

	return (UsartWrite(1,temp,8,0));
	
}

/*关机报告*/
static INT8U  StarGJBG(void)
{
	INT8U temp[8]={'$', 'G', 'J', 'B', 'G',0,8,24};

	return (UsartWrite(1,temp,8,0));
}

/*静默申请*/
static INT8U  StarJMSQ(void)
{
	INT8U temp[8]={'$', 'J', 'M', 'S', 'Q',0,8,29};

	return (UsartWrite(1,temp,8,0));
}

/*静默解除*/
static INT8U  StarJMJC(void)
{
	INT8U temp[8]={'$', 'J', 'M', 'j', 'c',0,8,22};

	return (UsartWrite(1,temp,8,0));
}

/*MY时效*/
static INT8U StarSXCX(void)
{
	INT8U temp[8]={'$','S','X','C','X',0,8,0x3C};
	return (UsartWrite(1,temp,8,0));
}

/*自毁申请*/
static INT8U StarZHSQBD(void)
{
	INT8U temp[15]={'$','Z','H','S','Q',0,0x0F,0,0,0,0x55,0xAA,0xAA,0x55,0x3B};
	return(UsartWrite(1,temp,15,0));
}


/********************************************************************************************************
* Function Name : FlashEraseAll
*
* Description   : This function is used to erase all of the internal flash memory.
*
* Arguments     : None
*
* Returns       : true   erase successfull;
*                 false  erase failed.    
********************************************************************************************************/
static  INT8U  FlashEraseAll(void)
{   
	INT32U  status;
									
    while (!(AT91C_BASE_MC->MC_FSR & AT91C_MC_FRDY));       /* Wait for EFC to be ready.               */
	AT91C_BASE_MC->MC_FCR = (0x5A << 24) | AT91C_MC_FCMD_ERASE_ALL; /* Write erase all command.        */
    do
    {
    	status = AT91C_BASE_MC->MC_FSR;
    } while (!(status & AT91C_MC_FRDY));                    /* Wait for EFC to be ready.               */
    if(status & (AT91C_MC_LOCKE | AT91C_MC_PROGE))
    	return (false);
    else
    	return (true);
}

/********************************************************************************************************
* Function Name : FlashEraseAll
*
* Description   : This function is used to erase all of the internal flash memory.
*
* Arguments     : None
*
* Returns       : true   erase successfull;
*                 false  erase failed.    
********************************************************************************************************/
void  MikePttChk(void)
{   
    INT8U  tmp;
    INT8U  ptt_status;		//PTT状态，0为发射，1为接收，发射时应关闭9GHz
    static INT8U ptt_st = 1;
        
    if (g_Flg.m_WorkMode == SYS_WORK_MODE_37)
    {
    	tmp = (*AT91C_PIOB_PDSR & MIKE_PTT) ? 1 : 0;
    	if (ptt_st != tmp)
        {
    	    OSTimeDlyHMSM(0, 0, 0, 50);
    	    tmp = (*AT91C_PIOB_PDSR & MIKE_PTT) ? 1 : 0;
    	    if (ptt_st != tmp)
    	    {
    	    	ptt_st = tmp;
    	        TermCommSend(B2T_CMD_HTZT, &ptt_st, 1);
    	        OSTimeDlyHMSM(0, 0, 0, DLY_MS_MATCH_TERM);
    	    }
        }
    }
    if ( (g_Flg.m_WorkMode == SYS_WORK_MODE_35) || (g_Flg.m_WorkMode == SYS_WORK_MODE_37) )
    {
    			ptt_status = (*AT91C_PIOB_PDSR & MIKE_PTT) ? 1 : 0;	//获取PTT的状态 0为发射，1为接收，发射时应关闭9GHz
    			if( ptt_status == 0 )
    			{
    					NGRespDis();
    			}else
    			{
    					if( NG_Work_Status == 0 )				//如果当前9GHz模块是打开着的
	  		    	{
	  		    		NGRespEn();						//恢复到原先的状态--打开
	  		    	}else{
	  		    		NGRespDis();				  //恢复到原先的状态--关闭
	  		    	}
    			}
    }
}

/******************************************** End Of File **********************************************/
