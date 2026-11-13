/********************************************************************************************************
**                            (c) Copyright  苏州市江海通讯发展实业有限公司
**                                      All Rights Reserved                                        
**
**---------------------------- 文件信息 -----------------------------------------------------------------
** 文   件   名:  bsp.c
** 创   建   人:  林依刚
** 版        本:  V1.0(原始版本)
** 最后修改日期:  2007-12-10
** 描        述:  板级支持包C文件，实现中断的映射处理，初始化周期间隔定时器作为实时操作系统内部脉动时钟。
** 操作系统版本： μC/OS-II V2.76 
**             
********************************************************************************************************/
#include "bsp.h"


/********************************************************************************************************
*                                            DATA TYPES
********************************************************************************************************/
typedef  void  (*BSP_PFNCT)(void);                /* BSP_PFNCT is the pointer type to the function.    */


/********************************************************************************************************
*                                  EXTERN GLOBAL VARIABLES DEFINE
********************************************************************************************************/
// BSP system time, can be written and read in the application program
// it is updated in the Tmr_TickISR_Handler.  
volatile  BSP_DATE_TIME_TYPE  BspTime = {0, 0, 0, 0, 0, 0, 0};

 
/********************************************************************************************************
*                                            PROTOTYPES
********************************************************************************************************/
static  void  BSP_DummyISR_Handler(void);
static  void  BSP_IntCtrlInit(void);
static  void  Tmr_TickInit(void);
static  void  Tmr_TickISR_Handler(void);
void          BSP_UpdateTime(void);

/********************************************************************************************************
* Function name:    HwInit
*
* Description  :    Initialize the target board hardware.
*               
* Arguments    :    none
********************************************************************************************************/
void  HwInit(void)
{
	// Initialize and start the watchdog timer. 
	// The mode register can be wirtten once after reset. 
    // watchdog timeout: 16S.
    *AT91C_WDTC_WDMR = 0xFFE | AT91C_WDTC_WDRSTEN | 0xFFF<<16; 
    	   	 	          	       	        
    // Enable clock of the PIOA and PIOB.
    *AT91C_PMC_PCER   = (1 << AT91C_ID_PIOA) | (1 << AT91C_ID_PIOB); 
            
    // configure I/O.
    /* Enables PIO to control the corresponding pin.*/ 
    *AT91C_PIOA_PER   = AM_MC_SW | OUT_5V | SYS_WORK_MODE1; 
    *AT91C_PIOA_OER   = AM_MC_SW | OUT_5V;             /* Enable the PIO output enable.	               */
    *AT91C_PIOA_ODR   = SYS_WORK_MODE1;                /* Disable the PIO output,pure input.           */
    *AT91C_PIOA_IFER  = SYS_WORK_MODE1;                /* Enables the input glitch filter on the line. */
    *AT91C_PIOA_PPUER = SYS_WORK_MODE1;                /* Enable the pull_up resister.                 */

    *AT91C_PIOB_PER   = SYS_PWR_CUR | SYS_CABLE_CHK | BAT_SW | LED_FLASH_LAMP | MIKE_PTT;
    *AT91C_PIOB_OER   = BAT_SW      | LED_FLASH_LAMP;
    *AT91C_PIOB_ODR   = SYS_PWR_CUR | SYS_CABLE_CHK | MIKE_PTT;
    *AT91C_PIOB_IFER  = SYS_PWR_CUR | SYS_CABLE_CHK | MIKE_PTT;
    *AT91C_PIOB_PPUDR = SYS_PWR_CUR;
    *AT91C_PIOB_PPUER = MIKE_PTT;
  	
  	BAT_ON();                                          /* 加载浮标内备用电池                           */
  	//RS485_EN();
    //RS485_AUTO_EN();
    AM_MC_EN();
    Usart0_Init();
    Usart1_Init();
}
/********************************************************************************************************
* Function name:    BSP_DummyISR_Handler
*
* Description  :    This function is called to handle invalid IRQs
*               
* Arguments    :    none
********************************************************************************************************/
static  void  BSP_DummyISR_Handler (void)
{	
	AT91C_BASE_AIC->AIC_IVR = 0;                /* Debug variant of vector read (protect mode is used).*/      
}

/********************************************************************************************************
* Function name:    BSP_IntCtrlInit
*
* Description  :    Initialize all the IRQ handdlers to dummy function
*               
* Arguments    :    none
********************************************************************************************************/
static  void  BSP_IntCtrlInit (void)
{
	INT16U  i;
   
    AT91C_BASE_AIC->AIC_IDCR  = 0xFFFFFFFF;                 /* Disable all interrupts.                 */
	AT91C_BASE_AIC->AIC_EOICR = 0x00000000;                 /* End-of-interrupt command.               */
	for (i = 0; i < 32; i++) {                              /* Disable all ISRs.                       */
        AT91C_BASE_AIC->AIC_SVR[i] = (INT32U)BSP_DummyISR_Handler; 
		AT91C_BASE_AIC->AIC_SMR[i] = 0;
	}
}

/********************************************************************************************************
* Function name:    BSP_Init
*
* Description  :    This function should be called by your application code before 
*                   you make use of any of the functions found in this module.
*               
* Arguments    :     none
********************************************************************************************************/
void  BSP_Init (void)
{
	BSP_IntCtrlInit();                                      /* Initialize the Interrupt Controller.    */
	Tmr_TickInit(); 	                                    /* Initialize the PIT for time tick.       */
	HwInit();   
	DBGU_Open(19200, 0x04);
}

/********************************************************************************************************
* Function Name : BSP_UpdateTime
*
* Description   : Update the global bsp time variable.
*
* Arguments     : None
*                 
* Used module   : BSP_TIME_TYPE  BspTime                  
* Returns       : None    
********************************************************************************************************/
void  BSP_UpdateTime(void)
{   
	if (BspTime.sec >= 59)
	{	
	    if (BspTime.minute >= 59)
	    {
		    if (BspTime.hour >= 23)
	        {
		        switch (BspTime.month)
		        {
			        case 1:
		            case 3:
		            case 5:
		            case 7:
		            case 8:
		            case 10:
		            case 12:
		    	        if (BspTime.day >= 31)
		    		        (BspTime.month == 12) ? (BspTime.year++,BspTime.month = 1,BspTime.day = 1) : (BspTime.month++,BspTime.day = 1);
                        else
                	        BspTime.day++;
                        break;	

                    case 2:	
                	    if (((BspTime.year%4 == 0) && (BspTime.year%100!=0)) || (BspTime.year%400 == 0))
		    	            (BspTime.day >= 29) ? (BspTime.month++,BspTime.day = 1) : BspTime.day++;
                        else
                    	    (BspTime.day >= 28) ? (BspTime.month++,BspTime.day = 1) : BspTime.day++; 
                        break;
                    	                			
		            case 4:
		            case 6:
		            case 9:
		            case 11:
		    	        (BspTime.day >= 30) ? (BspTime.month++,BspTime.day = 1) : BspTime.day++;
                        break;
		            default:
		    	        break;		
	            }
	            BspTime.hour = 0;
            }
            else 
    	        BspTime.hour++;
    	    BspTime.minute = 0;
        }
        else
    	    BspTime.minute++;
    	BspTime.sec = 0;
    }
    else
    	BspTime.sec++;      	  	
    //DBGU_Write((INT8U *)&BspTime, 8);	
}

/********************************************************************************************************
* Function name:    OS_CPU_IRQ_ISR_Handler
*
* Description  :    IRQ ISR HANDLER        
*                   This function is called by OS_CPU_IRQ_ISR() to determine the 
*                   source of the interrupt and process it accordingly. 
*
* Arguments    :    None                         
* Returns      :    None
********************************************************************************************************/
void  OS_CPU_IRQ_ISR_Handler (void)
{
	BSP_PFNCT  pfnct;
	
#if 1  
	
    pfnct = (BSP_PFNCT)AT91C_BASE_AIC->AIC_IVR;             /* Read the interrupt vector from the VIC. */
    if (pfnct != (BSP_PFNCT)0) {                            /* Make sure we don't have a NULL pointer. */
        (*pfnct)();                                         /* Execute ISR for the interrupting device.*/
    }
#else
    pfnct = (BSP_PFNCT)AT91C_BASE_AIC->AIC_IVR;             /* Read the interrupt vector from the VIC. */
    while (pfnct != (BSP_PFNCT)0) {                         /* Ma ke sure we don't have a NULL pointer.*/
        (*pfnct)();                                         /* Execute ISR for the interrupting device.*/
         pfnct = (BSP_PFNCT)AT91C_BASE_AIC->AIC_IVR;        /* Read the interrupt vector from the VIC. */
    }
#endif      
}

/********************************************************************************************************
* Function name:    OS_CPU_FIQ_ISR_Handler
*
* Description  :    FIQ ISR HANDLER
*                   This function is called by OS_CPU_FIQ_ISR() to determine the source of the interrupt
*                   and process it accordingly.  
*
* Arguments    :    None                         
* Returns      :    None
********************************************************************************************************/
void  OS_CPU_FIQ_ISR_Handler (void)
{
	BSP_PFNCT  pfnct;
	
#if 1   
	
	pfnct = (BSP_PFNCT)AT91C_BASE_AIC->AIC_FVR;             /* Read the interrupt vector from the VIC. */
	if (pfnct != (BSP_PFNCT)0) {                            /* Make sure we don't have a NULL pointer. */
        (*pfnct)();                                         /* Execute ISR for the interrupting device.*/
	}
#else    
	pfnct = (BSP_PFNCT)AT91C_BASE_AIC->AIC_FVR;             /* Read the interrupt vector from the VIC. */
	while (pfnct != (BSP_PFNCT)0) {                         /* Make sure we don't have a NULL pointer. */
        (*pfnct)();                                         /* Execute ISR for the interrupting device.*/
        pfnct = (BSP_PFNCT)AT91C_BASE_AIC->AIC_FVR;         /* Read the interrupt vector from the VIC. */
	}
#endif    
}

/********************************************************************************************************
* Function name:    Tmr_TickInit
*
* Description  :    TICKER INITIALIZATION
*                   This function is called to initialize uC/OS-II's tick source which uses the PIT,
*                   typically a timer generating interrupts every 1 to 100 mS.
* Note         :    1) PIT Interrupt frequency:
*
*                                  MCLK        1
*                        Freq =    ---- * -----------
*                                   16     (PIV + 1)
*
*                                  MCLK      1
*                        PIV  =  ( ---- * ------ ) - 1
*                                   16     Freq
*                        Where:
*                             MCLK = 48 MHz (i.e 48,000,000)
*                             Freq = Desired frequency (i.e. OS_TICKS_PER_SEC)   OS_TICKS_PER_SEC = 100
*
* Arguments    :    None                         
* Returns      :    None
********************************************************************************************************/
static void  Tmr_TickInit (void)
{
	INT32U  counts;

	// Set the vector address for PIT
	AT91C_BASE_AIC->AIC_SVR[AT91C_ID_SYS] = (INT32U)Tmr_TickISR_Handler;
	AT91C_BASE_AIC->AIC_SMR[AT91C_ID_SYS] = AT91C_AIC_SRCTYPE_INT_POSITIVE_EDGE
                                          | AT91C_AIC_PRIOR_LOWEST; 
	AT91C_BASE_AIC->AIC_ICCR              = 1 << AT91C_ID_SYS;
	AT91C_BASE_AIC->AIC_IECR              = 1 << AT91C_ID_SYS;
					
	counts                                = (48000000 / 16 / OS_TICKS_PER_SEC) - 1;
	AT91C_BASE_PITC->PITC_PIMR            = AT91C_PITC_PITEN | AT91C_PITC_PITIEN | counts;
}

/********************************************************************************************************
* Function name:  Tmr_TickISR_Handler
*
* Description  :  This function handles the PIT interrupt that is used to generate TICKs for uC/OS-II.
*                 DBGU uart interrupt.  
* Arguments    :  None
*                         
* Returns      :  None
********************************************************************************************************/
static  void  Tmr_TickISR_Handler (void) 
{
	volatile static INT32U  status;
  
    
	AT91C_BASE_AIC->AIC_IVR   = 0;            /* Debug variant of vector read (protect mode is used).  */
	AT91C_BASE_AIC->AIC_ICCR  = AT91C_ID_SYS; /* Clear system controller interrupt.                    */
	AT91C_BASE_AIC->AIC_EOICR = 0;            /* Signal end of interrupt.                              */
       	     
	// PIT interrupt handler 
	if((AT91PS_PITC)AT91C_BASE_PITC->PITC_PISR) {
		status = AT91C_BASE_PITC->PITC_PIVR;
        OSTimeTick();                        /* Tell uC/OS-II about clock tick.                        */
        	
	    if (BspTime.centisecond >= 99)
	    {
		    BspTime.centisecond = 0;
		    BSP_UpdateTime();	
	    }
	    else
		    BspTime.centisecond++;
	}	  
}

/********************************************************************************************************
* Function Name : DBGU_Open
*
* Description   : Initialize all of the communication modules,including pin,flags,drivers and state etc.
*
* Arguments     : baud_rate  CD = 0,Disable; CD = 1,MCK; CD = 2~65535, MCK / (CD x 16). 
*                  --------------------------------------
*                 |   PAR   |   Parity Type              |
*                 |---------|----------------------------|
*                 | 0  0  0 | Even parity                |
*                 | 0  0  1 | Odd parity                 |
*                 | 0  1  0 | Space: parity forced to 0  |
*                 | 0  1  1 | Mark: parity forced to 1   |
*                 | 1  x  x | No parity                  |
*                  --------------------------------------
* Returns       : none                    
********************************************************************************************************/
void DBGU_Open(INT32U baud_rate, INT8U parity_type)
{   	
	AT91F_DBGU_CfgPIO();                                    /* Configure PIO controllers to periph mode*/ 
	AT91F_DBGU_CfgPMC();                                    /* Enable Peripheral clock in PMC for DBGU */
	
	AT91C_BASE_DBGU->DBGU_MR = AT91C_US_CHMODE_NORMAL | ((parity_type & 0x07) << 9);
	AT91C_BASE_DBGU->DBGU_BRGR = (MCK >> 4) / baud_rate;
	AT91C_BASE_DBGU->DBGU_CR = AT91C_US_RSTRX | AT91C_US_RSTTX;
	AT91C_BASE_DBGU->DBGU_CR = AT91C_US_TXEN;               // AT91C_US_RXEN 
	//AT91C_BASE_DBGU->DBGU_IER = AT91C_US_RXRDY;
}

/********************************************************************************************************
* Function Name : DBGU_Close
*
* Description   : Disable the DBGU(disabled the receiver and transmitter)
*
* Arguments     : none
*                   
* Returns       : none                   
********************************************************************************************************/
void DBGU_Close(void)
{
	AT91C_BASE_DBGU->DBGU_CR = AT91C_US_RXDIS | AT91C_US_TXDIS;
}

/********************************************************************************************************
* Function Name : DBGU_Printk
*
* Description   : This function is used to send a string through the DBGU port(Very low level debugging).
*                 usually,used with sprintf (included in stdio.h)etc ;
* Arguments     : psrc        pointer to a string ending by '\0'
*                   
* Returns       : none                
********************************************************************************************************/
void DBGU_Printk(char *psrc) 
{   	
	char *tmp_ptr = psrc;
	
	if (tmp_ptr != NULL) { 
		while(*tmp_ptr != '\0') {
		    while (!(AT91C_BASE_DBGU->DBGU_CSR & AT91C_US_TXRDY)) ;
		    AT91C_BASE_DBGU->DBGU_THR = *tmp_ptr++;
		}
	}
}

/********************************************************************************************************
* Function Name : DBGU_Write
*
* Description   : Transmit data through the DBGU.
*
* Arguments     : psrc  the pointer of data;
*                 len   the size of data neading to send.
*                   
* Returns       : BSP_ERR_NULL_PTR      the parameter psrc is a null pointer.
*                 BSP_NO_ERR            transmit data ok.        
********************************************************************************************************/
INT8U DBGU_Write(INT8U *psrc, INT32U len)
{	
	INT8U *tmp_ptr = psrc;
	INT32U tmp_len = len;
	
	if (tmp_ptr == NULL)
		return (BSP_ERR_NULL_PTR);
				
	while (tmp_len > 0) {
		while (!(AT91C_BASE_DBGU->DBGU_CSR & AT91C_US_TXRDY)) ;  /* Wait for the DBGU tx ready.        */		
		AT91C_BASE_DBGU->DBGU_THR = *tmp_ptr++;                  /* Write register,move to next byte.  */ 
		tmp_len--;		                                         /* Left counter decrese.              */
	}	
	return (BSP_NO_ERR);	
}
