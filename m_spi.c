/********************************************************************************************************
**                            (c) Copyright  苏州市江海通讯发展实业有限公司
**                                      All Rights Reserved                                        
**
**---------------------------- 文件信息 -----------------------------------------------------------------
** 文   件   名:  spi.c
** 创   建   人:  林依刚
** 版        本:  V0.1
** 最后修改日期:  2010-12-17
** 描        述:  SPI接口驱动
** 操作系统版本： μC/OS-II V2.76
********************************************************************************************************/

#include "m_spi.h"


/*
*********************************************************************************************************
*                                             函数原型声明区 
*********************************************************************************************************
*/
__inline void SPI_PinsConfig(INT32U pins);
__inline void SPI_RegConfig(INT8U spi_id, AT91PS_SPI pspi);
static   void SPI_IrqHandler(void);


/*
*********************************************************************************************************
*                                             常量、变量及宏定义
*********************************************************************************************************
*/
static const SPI_DEV_DESC_TYPE SPI[2] = {
    {AT91C_BASE_SPI0, SPI0_PINS, AT91C_ID_SPI0, AT91C_PA16_SPI0_MISO, SPI_IRQ_PRIO, SPI_IrqHandler},
    {AT91C_BASE_SPI1, SPI1_PINS, AT91C_ID_SPI1, AT91C_PA24_SPI1_MISO, SPI_IRQ_PRIO, SPI_IrqHandler}
};

static volatile INT8U   g_SpiRxEnd;     // SPI接收结束标志（SPI_TRUE：接收结束；SPI_FALSE：接收未结束）
static SPI_DEV_RW_TYPE *g_DevRWCur;     // SPI读写数据


/*
*********************************************************************************************************
*                                             函数区
*********************************************************************************************************
*/
/********************************************************************************************************
** 函数名称： SPI_PinsConfig
** 函数功能： SPI相关引脚配置函数
** 入口参数： pins  相关外设引脚
** 出口参数： 无
********************************************************************************************************/
__inline void SPI_PinsConfig(INT32U pins)
{
	// 首先禁止相关引脚中断
	AT91C_BASE_PIOA->PIO_IDR   = pins;
	
	
	AT91C_BASE_PIOA->PIO_PPUDR = pins;
	AT91C_BASE_PIOA->PIO_ASR   = pins;
	AT91C_BASE_PIOA->PIO_PDR   = pins;
}


/********************************************************************************************************
** 函数名称： SPI_RegConfig
** 函数功能： SPI相关控制寄存器配置
** 入口参数： spi_id  SPI接口占用的外设ID号
**            pspi    SPI相关控制寄存器的基址
** 出口参数： 无
********************************************************************************************************/
__inline void SPI_RegConfig(INT8U spi_id, AT91PS_SPI pspi)
{
	// 使能SPI输入时钟
	AT91C_BASE_PMC->PMC_PCER = 1 << spi_id;
	
	// 软件复位SPI
	pspi->SPI_CR = AT91C_SPI_SWRST;
	
	// SPI may need two software reset.
	pspi->SPI_CR = AT91C_SPI_SWRST;	
	
	// 主机模式，禁止模式故障检测，禁止片选
	pspi->SPI_MR = AT91C_SPI_MSTR | AT91C_SPI_MODFDIS | AT91C_SPI_PCS;
	
	// 禁止PDC收发
	pspi->SPI_PTCR = AT91C_PDC_RXTDIS | AT91C_PDC_TXTDIS;
	
	// 使能SPI
	pspi->SPI_CR = AT91C_SPI_SPIEN;
	
	// 禁止SPI时钟
	AT91C_BASE_PMC->PMC_PCDR = 1 << spi_id;
}


/********************************************************************************************************
** 函数名称： SPI_Init
** 函数功能： 初始配置SPI0
** 入口参数： port  指定配置的SPI端口号(0代表SPI0，1代表SPI1)
** 出口参数： 无
********************************************************************************************************/
void SPI_Init(INT8U port)
{	
	SPI_PinsConfig(SPI[port].Pins);
	AT91C_BASE_PIOA->PIO_PPUER = SPI[port].NeedPullUpPin;
	SPI_RegConfig(SPI[port].SpiID, SPI[port].pSPI);
	
	// 向AIC注册SPI中断
	AT91F_AIC_ConfigureIt(AT91C_BASE_AIC, 
	                      SPI[port].SpiID, 
	                      SPI[port].IrqPrio, 
	                      AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL, 
	                      SPI[port].pFunSpiIsr);
	AT91C_BASE_AIC->AIC_IECR = 1 << SPI[port].SpiID;
	
	// 禁止所有SPI中断
	SPI[port].pSPI->SPI_IDR = 0xFFFFFFFF;
	
	g_SpiRxEnd = SPI_FALSE;
}


/********************************************************************************************************
** 函数名称： SPI_IrqHandler
** 函数功能： SPI中断服务子函数
** 入口参数： 无
** 出口参数： 无
********************************************************************************************************/
static void SPI_IrqHandler(void)
{
	INT32U  status;

	status = SPI[g_DevRWCur->Port].pSPI->SPI_SR;
	
	if(status & AT91C_SPI_RXBUFF)
	{
		// 禁止PDC收发器
	    SPI[g_DevRWCur->Port].pSPI->SPI_PTCR = AT91C_PDC_RXTDIS | AT91C_PDC_TXTDIS;
	    
	    // 禁止SPI时钟输入
	    AT91C_BASE_PMC->PMC_PCDR = 1<< SPI[g_DevRWCur->Port].SpiID;
	    
	    // 禁止SPI接收结束中断
	    SPI[g_DevRWCur->Port].pSPI->SPI_IDR = AT91C_SPI_RXBUFF;
	    
	    // 调用SPI设备用户注册的回调函数
	    if(g_DevRWCur->pFunCallBack != NULL)
		    g_DevRWCur->pFunCallBack(g_DevRWCur->pArg);
	}
	
	g_SpiRxEnd = SPI_TRUE;
	
	// End of the interrupt handler
	AT91C_BASE_AIC->AIC_EOICR = 0x00;
}


/********************************************************************************************************
** 函数名称： SPI_ExecuteRW
** 函数功能： 通过SPI口对相关设备执行读写操作
** 入口参数： dev_rw  指向具体读写操作数据结构(SPI_DEV_RW_TYPE结构体) 的指针
** 出口参数： 无
********************************************************************************************************/
void SPI_ExecuteRW(SPI_DEV_RW_TYPE *dev_rw)
{
	INT32U	mode_reg_val;               // 临时保存SPI模式寄存器的值

    // 使能SPI时钟输入
	AT91C_BASE_PMC->PMC_PCER = 1 << SPI[dev_rw->Port].SpiID;
	
	// 首先禁止SPI PDC收发器
	SPI[dev_rw->Port].pSPI->SPI_PTCR = AT91C_PDC_RXTDIS | AT91C_PDC_TXTDIS;
	
	// 修改SPI模式寄存器以使片选信号指向待操作的SPI设备
	mode_reg_val = SPI[dev_rw->Port].pSPI->SPI_MR;
	mode_reg_val |= AT91C_SPI_PCS;
	mode_reg_val &= ~(1 << (dev_rw->CsID + 16));
	SPI[dev_rw->Port].pSPI->SPI_MR = mode_reg_val;
	
	// 初始化SPI PDC收发器
	SPI[dev_rw->Port].pSPI->SPI_RPR  = (INT32U)dev_rw->pCmd;
	SPI[dev_rw->Port].pSPI->SPI_RCR  = dev_rw->CmdLen;
	SPI[dev_rw->Port].pSPI->SPI_TPR  = (INT32U)dev_rw->pCmd;
	SPI[dev_rw->Port].pSPI->SPI_TCR  = dev_rw->CmdLen;
	
	SPI[dev_rw->Port].pSPI->SPI_RNPR = (INT32U)dev_rw->pData;
	SPI[dev_rw->Port].pSPI->SPI_RNCR = dev_rw->DataLen;
	SPI[dev_rw->Port].pSPI->SPI_TNPR = (INT32U)dev_rw->pData;
	SPI[dev_rw->Port].pSPI->SPI_TNCR = dev_rw->DataLen;
	
	// 使能PDC收、发
	SPI[dev_rw->Port].pSPI->SPI_PTCR = AT91C_PDC_RXTEN | AT91C_PDC_TXTEN;
	
	// 当前设备读写数据结构保存至静态全局变量中，供中断服务程序使用	
	g_DevRWCur = dev_rw;
	
	// 使能SPI接收结束中断（SPI_RCR与SPI_RNCR的值同为0时，触发中断）
	SPI[dev_rw->Port].pSPI->SPI_IER = AT91C_SPI_RXBUFF;
	
	// 等待SPI设备接收结束
	SPI_WaitRxEnd();
}


/********************************************************************************************************
** 函数名称： SPI_WaitRxEnd
** 函数功能： 等待SPI设备接收结束
** 入口参数： 无
** 出口参数： 无
********************************************************************************************************/
void SPI_WaitRxEnd(void)		
{
	while(g_SpiRxEnd==SPI_FALSE);
	g_SpiRxEnd = SPI_FALSE;
}
