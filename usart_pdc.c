/********************************************************************************************************
**                            (c) Copyright  苏州市江海通讯发展实业有限公司
**                                      All Rights Reserved                                        
**
**---------------------------- 文件信息 -----------------------------------------------------------------
** 文   件   名:  usart_pdc.c
** 创   建   人:  林依刚
** 版        本:  V1.1
** 最后修改日期:  2009-9-7
** 描        述:  AT91SAM7X(C)256 串口驱动程序
                  在V1.0版本的基础上用DMA方式实现串口数据的接收。
** 操作系统版本： μC/OS-II V2.76
**          
**--------------------------- 历史版本信息 --------------------------------------------------------------
** 创   建   人:  林依刚
** 版        本:  V1.0
** 日　      期:  2007-12-15
** 描　      述:  V1.1(原始版本)
**
********************************************************************************************************/
 
#include "usart_pdc.h" 

#define AT91B_MCK             ((18432000*73/14)/2)     /* Output PLL Clock 48MHz                       */ 
#define US0_INT_PRIO          4                        /* 串口0中断优先级(低优先级)                    */ 
#define US1_INT_PRIO          5                        /* 串口1中断优先级(高优先级)                    */ 
 
#define US0_BAUD_RATE         19200                    /* 串口0波特率                                  */ 
#define US1_BAUD_RATE         115200                   /* 串口1波特率                                  */ 

#define US_RX_BUF_SIZE        512                      /* 定义串口缓冲区容量                           */ 
#define MAX_US_NUM            2                        /* 定义串口数量                                 */ 
#define US_TIMEOUT_VAL        0x08                     /* 定义串口超时时间长度                         */ 

// 标准串口异步传输模式(Standard Asynchronous Mode): 8 bits , 1 stop , no parity
#define US_MODE ( AT91C_US_USMODE_NORMAL + \
                  AT91C_US_NBSTOP_1_BIT + \
                  AT91C_US_PAR_NONE + \
                  AT91C_US_CHRL_8_BITS + \
                  AT91C_US_CLKS_CLOCK )
                
typedef struct usart_data_type{               
    INT8U      m_RxBuf[US_RX_BUF_SIZE];                /* 串口接收缓冲区.                              */ 
    INT16U     m_RxCtr;                                /* 接收数据个数.                                */ 
    INT8U     *m_pTxBuf;                               /* 串口发送缓冲区指针.                          */ 
    INT32U     m_TxLeftCtr;                            /* 剩余待发送数据个数.                          */
    OS_EVENT  *m_SemRx;                                /* 接收信号量，用于标志一帧数据接收完成.        */
} USART_DATA_TYPE;

static USART_DATA_TYPE  g_UsData[MAX_US_NUM];          /* 定义串口数据结构变量.                        */

// 静态函数声明
static void Usart0_IRQ_Handler(void);                  /* 串口0中断处理程序                            */
static void Usart1_IRQ_Handler(void);                  /* 串口1中断处理程序                            */

extern unsigned int us0readsize;
extern unsigned int us1readsize;
/********************************************************************************************************
** 函数名称： Usart0_IRQ_Handler
** 函数功能： US0 interrupt serve routine.
** 入口参数： None
** 出口参数： None
********************************************************************************************************/
static void Usart0_IRQ_Handler(void)
{	
    INT32U        status;
         	    
    AT91F_AIC_DisableIt(AT91C_BASE_AIC, AT91C_ID_US0);      /* 禁止US0中断.                            */
    status = AT91C_BASE_US0->US_CSR & AT91C_BASE_US0->US_IMR; /*读取状态/中断屏蔽寄存器,以清除中断标志 */
    if (status & (AT91C_US_RXBUFF | AT91C_US_TIMEOUT)) {    /* 接收缓冲区满中断或超时中断.             */
        g_UsData[0].m_RxCtr = US_RX_BUF_SIZE - AT91C_BASE_US0->US_RCR;          
		us0readsize = g_UsData[0].m_RxCtr;
        AT91F_PDC_Open(AT91C_BASE_PDC_US0);                 /* 开启PDC_US0.                            */
        
        // 开启PDC_US0接收数据，调用库函数
        AT91F_US_ReceiveFrame(AT91C_BASE_US0, (char *)g_UsData[0].m_RxBuf, US_RX_BUF_SIZE, 0, 0);       
        OSSemPost(g_UsData[0].m_SemRx);                     /* 释放信号量，一帧结束.                   */
    }  			   
    else {
        // 如果是非PDC缓冲区满中断及超时中断，则表明出错，为纠正错误，屏蔽产生此中断的位      
        AT91F_US_DisableIt (AT91C_BASE_US0, status);
    }
    
    AT91C_BASE_US0->US_RTOR = US_TIMEOUT_VAL;               /* 重设超时长度.                           */ 
    AT91C_BASE_US0->US_CR   = AT91C_US_STTTO;               /* Start Time-out.                         */
    AT91C_BASE_US0->US_CR   = AT91C_US_RSTSTA;              /* Resets the status bits in US_CSR.       */        
    AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_US0);       /* 重新开中断.                             */      
    AT91C_BASE_AIC->AIC_EOICR = 0x00;	                    /* End of the interrupt handler.           */ 
}

/********************************************************************************************************
** 函数名称： Usart0_Init
** 函数功能： Initialize the US0
** 入口参数： None
** 出口参数： None
********************************************************************************************************/
void Usart0_Init ( void )
{	    
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_US0;           /* 开US0时钟                               */    
    AT91F_PIO_CfgPeriph( AT91C_BASE_PIOA,                   /* 配置I/O.                                */
 		               ((unsigned int) AT91C_PA0_RXD0) | 
 		               ((unsigned int) AT91C_PA1_TXD0) ,    /* Peripheral A.                           */ 
 		                 0);                                /* Peripheral B.                           */ 
    
    AT91F_US_Configure (AT91C_BASE_US0, AT91B_MCK, US_MODE, US0_BAUD_RATE, 0);/* 配置US0模式、波特率等.*/
    AT91F_US_ResetRx(AT91C_BASE_US0);                       /* Reset Receiver and re-enable it.        */ 
    AT91F_US_ResetTx(AT91C_BASE_US0);                       /* Reset Transmitter and re-enable it.     */    
    AT91C_BASE_US0->US_RTOR = US_TIMEOUT_VAL ;              /* Define Time-Out value.                  */                                
    AT91C_BASE_US0->US_CR   = AT91C_US_STTTO;               /* Start Time-out.                         */     
    AT91F_AIC_ConfigureIt( AT91C_BASE_AIC,                  /* 配置US0中断.                            */
                           AT91C_ID_US0, 
                           US0_INT_PRIO,
                           AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL,
                           Usart0_IRQ_Handler);
                               
    AT91F_PDC_Open(AT91C_BASE_PDC_US0);                     /* 开启PDC_US0.                            */ 
    
    // 开启PDC_US0接收数据，调用库函数
    AT91F_US_ReceiveFrame(AT91C_BASE_US0, (char *)g_UsData[0].m_RxBuf, US_RX_BUF_SIZE, 0, 0);                                   
    AT91F_US_EnableIt(AT91C_BASE_US0, AT91C_US_RXBUFF|AT91C_US_TIMEOUT); /* 允许接收缓冲区满及超时中断.*/                            
    AT91F_AIC_EnableIt (AT91C_BASE_AIC, AT91C_ID_US0);      /* 允许总中断                              */    
    g_UsData[0].m_SemRx = OSSemCreate(0);                   /* 创建串口接收信号量.                     */
}

/********************************************************************************************************
** 函数名称： Usart1_IRQ_Handler
** 函数功能： US1 interrupt serve routine.
** 入口参数： None
** 出口参数： None
********************************************************************************************************/
static void Usart1_IRQ_Handler(void)
{ 	   
    INT32U        status;
         	    
    AT91F_AIC_DisableIt(AT91C_BASE_AIC, AT91C_ID_US1);      /* 禁止US1中断.                            */         
    status = AT91C_BASE_US1->US_CSR & AT91C_BASE_US1->US_IMR; /* 读取状态/中断屏蔽寄存器,以清除中断标志*/            
    if (status & (AT91C_US_RXBUFF|AT91C_US_TIMEOUT)) {      /* 接收缓冲区满中断或超时中断              */ 
        g_UsData[1].m_RxCtr = US_RX_BUF_SIZE - AT91C_BASE_US1->US_RCR;      
		us1readsize = g_UsData[1].m_RxCtr;
        AT91F_PDC_Open(AT91C_BASE_PDC_US1);                 /* 开启PDC_US1.                            */
        
        // 开启PDC_US1接收数据，调用库函数
        AT91F_US_ReceiveFrame(AT91C_BASE_US1, (char *)g_UsData[1].m_RxBuf, US_RX_BUF_SIZE, 0, 0);
       
        OSSemPost(g_UsData[1].m_SemRx);                     /* 释放信号量，一帧结束.                   */
    }  			   
    else {
        // 如果是非PDC缓冲区满中断及超时中断，则表明出错，为纠正错误，屏蔽产生此中断的位      
        AT91F_US_DisableIt (AT91C_BASE_US1, status);
    }
    
    AT91C_BASE_US1->US_RTOR = 50;                           /* 重设超时长度.                           */ 
    AT91C_BASE_US1->US_CR   = AT91C_US_STTTO;               /* Start Time-out.                         */        	   
    AT91C_BASE_US1->US_CR   = AT91C_US_RSTSTA;              /* Resets the status bits in US_CSR.       */         
    AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_US1);       /* 重新开中断.                             */    
    AT91C_BASE_AIC->AIC_EOICR = 0x00;	                    /* End of the interrupt handler.           */ 
}

/********************************************************************************************************
** 函数名称： Usart1_Init
** 函数功能： Initialize the US1
** 入口参数： None
** 出口参数： None
********************************************************************************************************/
void Usart1_Init (void )
{     
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_US1;           /* 开US1时钟                               */
    AT91F_PIO_CfgPeriph ( AT91C_BASE_PIOA,                  /* 配置I/O.                                */
 		                ((unsigned int) AT91C_PA5_RXD1) |      
 		                ((unsigned int) AT91C_PA6_TXD1),    /* Peripheral A.                           */
 		                 0);                                /* Peripheral B.                           */
    
    AT91F_US_Configure (AT91C_BASE_US1, AT91B_MCK, US_MODE, US1_BAUD_RATE, 0); /* 配置US1模式、波特率等*/
    AT91F_US_ResetRx(AT91C_BASE_US1);                       /* Reset Receiver and re-enable it.        */
    AT91F_US_ResetTx(AT91C_BASE_US1);                       /* Reset Transmitter and re-enable it.     */
    AT91C_BASE_US1->US_RTOR = 50;                           /* Define Time-Out value.                  */
    AT91C_BASE_US1->US_CR   = AT91C_US_STTTO;               /* Start Time-out.                         */
    AT91F_AIC_ConfigureIt( AT91C_BASE_AIC,                  /* 配置US1中断.                            */
                           AT91C_ID_US1,
                           US1_INT_PRIO,
                           AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL,
                           Usart1_IRQ_Handler);	
                              
    AT91F_PDC_Open(AT91C_BASE_PDC_US1);                     /* 开启PDC_US1.                            */ 
    
    // 开启PDC_US1接收数据，调用库函数
    AT91F_US_ReceiveFrame(AT91C_BASE_US1, (char *)g_UsData[1].m_RxBuf, US_RX_BUF_SIZE, 0, 0);
    AT91F_US_EnableIt(AT91C_BASE_US1,AT91C_US_RXBUFF|AT91C_US_TIMEOUT);  /* 允许接收缓冲区满及超时中断.*/
    AT91F_AIC_EnableIt (AT91C_BASE_AIC, AT91C_ID_US1);      /* 允许总中断.                             */
    g_UsData[1].m_SemRx = OSSemCreate(1);                   /* 创建串口接收信号量.                     */
}

/********************************************************************************************************
** 函数名称： UsartPutChar
** 函数功能： 通过指定的USART发送一字节数据
** 入口参数： pUSART: USART通道基地址，三个USART分别为：
            　　　　　DBUG--(AT91PS_USART)AT91C_BASE_DBGU
            　　　　　US0---AT91C_BASE_US0
            　　　　　US1---AT91C_BASE_US1
            　ch:     待发送数据
** 出口参数： None
********************************************************************************************************/
void UsartPutChar(AT91PS_USART pUSART, INT8U ch)
{            	   
    while(!(pUSART->US_CSR & 0x02));                        /* Wait for the Transmitter Ready.         */
    pUSART->US_THR = ch;	
}

/********************************************************************************************************
** 函数名称：    UsartSendString
** 函数功能：    通过指定的USART发送字符串
** 入口参数：    pUSART:  USART通道基地址.
** 出口参数：    0--PDC忙，须等待，其它值：见库函数说明（可忽略）
********************************************************************************************************/
unsigned int UsartSendString(AT91PS_USART pUSART, char *psrc)
{
   unsigned char size;
   
   for (size = 0; psrc[size] != '\0'; size++){;}            /* 计算字符串长度.                         */
   return AT91F_US_SendFrame(pUSART, psrc, size, 0, 0);     /* 调用库函数.                             */ 
}

/********************************************************************************************************
** 函数名称： UsartWrite
** 函数功能： 通过指定的USART发送数据
** 入口参数： id      : USART编号，三个USART分别为：         
                        0：US0---AT91C_BASE_US0
                        1：US1---AT91C_BASE_US1    
              psrc    : is a pointer to the 'source' memory block      
              size    : is the number of bytes to write.
              timeout : is an optional timeout period (in clock ticks). 

** 出口参数： US_NO_ERR            The call was successful and the data was sent.
            　US_ERR_NULL_PTR      If 'psrc' is a NULL pointer.
            　US_ERR_INVALID_PORT  An invalid usart port was specified
********************************************************************************************************/
INT8U UsartWrite(INT8U id, INT8U *psrc, INT32U size, INT16U timeout)    
{   
    AT91PS_USART  pUSART;
        
    if (id >= MAX_US_NUM) 
        return (US_ERR_INVALID_PORT);
  	if (psrc == NULL)
  		return (US_ERR_NULL_PTR);
  	if (size == 0x00)
    	return (US_NO_ERR);
    	
    pUSART = id ? AT91C_BASE_US1 : AT91C_BASE_US0;	   
                       
    //AT91F_US_SendFrame(pUSART,(char *)psrc, size, 0, 0);  /* 调用库函数, 使用PDC方式发送数据.        */
    
    g_UsData[id].m_pTxBuf    = (INT8U *)psrc;               /* 得到待发送数据区的首地址.               */
    g_UsData[id].m_TxLeftCtr = size;                        /* 得到待发送数据区的长度.                 */
    while (g_UsData[id].m_TxLeftCtr--) {                    /* 采用轮询方式发送数据,调整剩余发送个数   */
        while(!(pUSART->US_CSR & AT91C_US_TXRDY));          /* 等待串口发送保持寄存器空闲.             */
        pUSART->US_THR = *g_UsData[id].m_pTxBuf++;          /* 写发送保持寄存器,指针指向下一待发送数据 */
    }             
    return (US_NO_ERR);           
}

/********************************************************************************************************
** 函数名称： UsartRead
** 函数功能： read data from the USART 
** 入口参数： id      : is the usart's ID (0..1)       
              pdest   : is a pointer to the 'destination' memory block
              size    : is the number of bytes to read.
              timeout : is an optional timeout period (in clock ticks).
              
** 出口参数： US_NO_ERR            The call was successful and the data was sent.
              US_ERR_NULL_PTR      If 'psrc' is a NULL pointer.
              US_ERR_INVALID_PORT  An invalid usart port was specified
              US_ERR_READ_TIMEOUT  If no data was read during timeout period.
********************************************************************************************************/
INT16U UsartRead(INT8U id, INT8U *pdest, INT32U size, INT16U timeout)
{   
    INT8U         err;                                      /* 接收μC/OS-II出错代码.                   */
    INT16U        loop;                                     /* 临时循环变量.                           */
    INT16U        rxlen;                                    /* 实际读取数据长度.	                   */  
    OS_CPU_SR     cpu_sr;                                   /* Storage for CPU status register.        */
    	
    if (id >= MAX_US_NUM)
        return (US_ERR_INVALID_PORT);
    if (pdest == NULL)
    	return (US_ERR_NULL_PTR);
    if (size == 0x00)
    	return (US_NO_ERR);
    		                
    OSSemPend(g_UsData[id].m_SemRx,timeout,&err);           /* 等待接收信号量.                         */    
    if (err == OS_TIMEOUT)                                  /* 等待信号量超时,返回.                    */  
        return (US_ERR_READ_TIMEOUT);
        
    OS_ENTER_CRITICAL();
   
    // 计算读取数据的长度，参数读取个数小于实际接收个数则读取参数要求个数，否则读取实际接收个数.
    rxlen = (size<g_UsData[id].m_RxCtr) ? size : g_UsData[id].m_RxCtr;               
    memcpy(pdest, g_UsData[id].m_RxBuf, rxlen);             /* 读取串口接收数据.                       */ 
   
    // 将剩余未读取的接收数据移至串口缓冲区的头部，释放信号量模拟一帧结束，以便下一次读取.
    if (rxlen < g_UsData[id].m_RxCtr) {                  
        for (loop=0; loop<g_UsData[id].m_RxCtr-rxlen; loop++) {
            g_UsData[id].m_RxBuf[loop]       = g_UsData[id].m_RxBuf[loop+rxlen];
            g_UsData[id].m_RxBuf[loop+rxlen] = 0x00;
        }               
        OSSemPost(g_UsData[id].m_SemRx);                 
    }          
    g_UsData[id].m_RxCtr -= rxlen;                          /* 更新剩余数据个数.                       */                         
    OS_EXIT_CRITICAL();
    return (US_NO_ERR);
}