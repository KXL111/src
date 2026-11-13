/********************************************************************************************************
** 文   件   名:  low_level_init.c
** 描        述:  AT91SAM7X(C)系列微控制器低级初始化程序供汇编启动代码调用                
********************************************************************************************************/
#include  "AT91SAM7X256.h"                        /* AT91SAM7X256's header files.                      */
#include "app.h"


// 以下函数必须在ARM模式下实现，并被异常向量直接调用
extern void AT91F_SpuriousHandler(void);
extern void AT91F_DefaultIRQHandler(void);
extern void AT91F_DefaultFIQHandler(void);
typedef unsigned int uint32_t;

//以下定义操作时钟初始化服务
#define CLOCK_TIMEOUT    0xFFFFFFFF		//定义超时极限大小
#define PMC_MCKR_CSS_Pos 0
#define PMC_MCKR_CSS_Msk (0x3u << PMC_MCKR_CSS_Pos) /**< \brief (PMC_MCKR) Master Clock Source Selection */
#define   PMC_MCKR_CSS_MAIN_CLK (0x1u << 0) /**< \brief (PMC_MCKR) Main Clock is selected */
#define PMC_SR_MCKRDY (0x1u << 3) /**< \brief (PMC_SR) Master Clock Status */
/********************************************************************************************************
** 函数名称： AT91F_LowLevelInit
** 函数功能： AT91SAM7X(C)系列微控制器低级初始化程序供汇编启动代码调用，主要实现：
              设置片内flash读、写等待周期；
              设置片内硬件看门狗定时器(在不使用看门狗时将其禁止，本软件在应用程序中初始化并使用)；
              设置主机时钟分频、倍频参数并等待主机时钟稳定；
              为每个终端设置哑元中断服务程序；
              在调试阶段，设置存储器重映射，将片内SRAM的起始地址映射到地址0。
** 入口参数： 无
** 出口参数： 无
********************************************************************************************************/
void AT91F_LowLevelInit(void) 
{
	  unsigned int i = 0;
    unsigned int timeout = 0;
    unsigned int temp;

    AT91PS_PMC  pPMC = AT91C_BASE_PMC;
    
    //读取复位控制器的模式寄存器的值
    temp = AT91C_BASE_RSTC->RSTC_RMR;
    //进行处理器复位
    AT91C_BASE_RSTC->RSTC_RMR = 0xA5000001;
		//等待复位引脚变成高电平，即等待处理器复位完成
    while(!(AT91C_BASE_RSTC->RSTC_RSR & (0x1u << 16)));
    //恢复模式寄存器的值
    AT91C_BASE_RSTC->RSTC_RMR = 0xA5000000 | temp;
    
    // Set Flash Wait state
    // Single Cycle Access at Up to 30MHz, or 40MHz
    // if MCK = 47923200Hz,have 50 Cycle for 1us
    AT91C_BASE_MC->MC_FMR = ((AT91C_MC_FMCN)&(50 <<16)) | AT91C_MC_FWS_1FWS ;
    
    //使能主振荡器，并等待6*8个慢速时钟周期，时间：48/32768 = 1.46484375 ms
    pPMC->PMC_MOR = AT91C_CKGR_MOSCEN | (AT91C_CKGR_OSCOUNT & (0x06 << 8)); 
    timeout = 0;
    //等待主振荡器使能成功
    while(!(pPMC->PMC_SR & AT91C_PMC_MOSCS) && (timeout++ < CLOCK_TIMEOUT));
    //设置校验主振荡器
    pPMC->PMC_MCKR = (pPMC->PMC_MCKR & ~(uint32_t)PMC_MCKR_CSS_Msk) | PMC_MCKR_CSS_MAIN_CLK;
    //等待校验主振荡器成功
    for ( timeout = 0; !(pPMC->PMC_SR & PMC_SR_MCKRDY) && (timeout++ < CLOCK_TIMEOUT) ; );
    
    //设置PLL，ARM7的主频最高为48MHz，而输入的晶振为18.432MHz。
    //这里对该晶振进行24分频作为PLL的输入时钟源，即18.432/24=0.768MHz，
    //再乘以倍频系数124+1=125，即0.768*125=96MHz。
    pPMC->PMC_PLLR = ((AT91C_CKGR_DIV & 24) |(AT91C_CKGR_PLLCOUNT & (28<<8)) |
                      (AT91C_CKGR_MUL & (124<<16)));
    timeout = 0;
    //等待设置PLL成功
    while(!(pPMC->PMC_SR & AT91C_PMC_LOCK) && (timeout++ < CLOCK_TIMEOUT));
    
    //设置主机时钟，将PLL输出进行2分频作为主机时钟，即96/2=48MHz。
    pPMC->PMC_MCKR = AT91C_PMC_CSS_PLL_CLK | AT91C_PMC_PRES_CLK_2 ;
    //等待主机时钟设置成功
    for ( timeout = 0; !(pPMC->PMC_SR & PMC_SR_MCKRDY) && (timeout++ < CLOCK_TIMEOUT) ; );
    
    //初始化中断服务程序代码段
    AT91C_BASE_AIC->AIC_SVR[0]     = (int)AT91F_DefaultFIQHandler ;
    for (i=1; i<31; i++) 
        AT91C_BASE_AIC->AIC_SVR[i] = (int)AT91F_DefaultIRQHandler ;
    AT91C_BASE_AIC->AIC_SPU        = (int)AT91F_SpuriousHandler;
}
 