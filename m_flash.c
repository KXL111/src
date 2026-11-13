/********************************************************************************************************
**                            (c) Copyright  苏州市江海通讯发展实业有限公司
**                                      All Rights Reserved                                        
**
**---------------------------- 文件信息 -----------------------------------------------------------------
** 文   件   名:  usart_pdc.c
** 创   建   人:  林依刚
** 版        本:  V0.1
** 最后修改日期:  2010-12-17
** 描        述:  AT91SAM7X(C)256 通过SPI接口读写AT45DB021B驱动程序
** 操作系统版本： μC/OS-II V2.76
********************************************************************************************************/
#include "ucos_ii.h"
#include "m_flash.h"


//*------------------------------------------------------------------------------------------------
//* 函数名称 : DF_CtrlCfg
//* 功能描述 : 外部FLASH初始设置，实际完成与FLASH相连的SPI接口的初始设置工作
//* 入口参数 : 无
//* 出口参数 : 无
//*------------------------------------------------------------------------------------------------
void DF_CtrlCfg(void)
{
	INT32U  temp;
	
	
	// 保存模式寄存器的值
	temp = AT91C_BASE_RSTC->RSTC_RMR;
	
	// 设置NRST外部复位的持续时间并执行复位（拉低NRST引脚）持续2^(ERSTL+1)个慢速时钟周期
	AT91C_BASE_RSTC->RSTC_RMR = 0xA5000000 | (0x09 << 8);
	AT91C_BASE_RSTC->RSTC_RCR = 0xA5000000 | AT91C_RSTC_EXTRST;

	// 等待NRST引脚恢复为高电平
	while(!(AT91C_BASE_RSTC->RSTC_RSR & AT91C_RSTC_NRSTL));
	
	// 恢复模式寄存器的值
    AT91C_BASE_RSTC->RSTC_RMR = 0xA5000000 | temp; 
    
	// 使能FLASH设备的片选线
	AT91C_BASE_PIOA->PIO_IDR   = FLASH_NPCS;
	AT91C_BASE_PIOA->PIO_PPUDR = FLASH_NPCS;
	AT91C_BASE_PIOA->PIO_ASR   = FLASH_NPCS;
	AT91C_BASE_PIOA->PIO_PDR   = FLASH_NPCS;

	// 设定主机SPI接口为模式0（CPOL：0，NCPHA：1），波特率为10MHz
	// 片选有效之后延时12/MCK时间之后输出SPCK，两个连续传输之间等待(32*1)/MCK时间
    FLASH_SPI_BASE->SPI_CSR[FLASH_NPCS_ID] = AT91C_SPI_NCPHA | ((MCK/FLASH_BAUD_RATE) << 8) | 
                                             ((unsigned int)12 << 16) | ((unsigned int)1 << 24);
}
//*------------------------------------------------------------------------------------------------
//* 函数名称 : DF_SendOpcode
//* 功能描述 : 向FLASH发送操作码
//* 入口参数 : <pCmd>[in]         操作指令的指针
//*          : <CmdLen>[in]       操作指令的长度
//*          : <pdata>[in]        操作数据的指针
//*          : <data_len>[in]     操作数据的长度
//*          : <pFunCallBack>[in] 回调函数指针
//*          : <pArg>[in]         传递给回调函数的参数
//* 出口参数 : 无
//*------------------------------------------------------------------------------------------------
static void DF_SendOpcode(INT8U *pcmd,  INT8U cmd_len, 
                          INT8U *pdata, INT32U data_len, 
                          PFUN_CALLBACK pFunCallBack, void *pArg)
{
	SPI_DEV_RW_TYPE    at45db;
	
	
	memset(&at45db, 0, sizeof(SPI_DEV_RW_TYPE));
	at45db.Port    = SPI0;
	at45db.CsID    = FLASH_NPCS_ID;
	at45db.pCmd    = pcmd;
	at45db.CmdLen  = cmd_len;
	at45db.pData   = pdata;
	at45db.DataLen = data_len;
	at45db.pFunCallBack = pFunCallBack;
	at45db.pArg    = pArg;
	
	SPI_ExecuteRW(&at45db);
}
//*------------------------------------------------------------------------------------------------
//* 函数名称 : DF_GetStatus
//* 功能描述 : 获得外部Flash状态
//* 入口参数 : 无
//* 出口参数 : 无
//*------------------------------------------------------------------------------------------------
INT8U DF_GetStatus(void)
{
	INT8U cmd_line = DB_STATUS;
	INT8U status = 0xFF;
	
	DF_SendOpcode(&cmd_line, 1, &status, 1, NULL, NULL);
		
	return status;
}

//*------------------------------------------------------------------------------------------------
//* 函数名称 : DF_ContinuousRead
//* 功能描述 : 从指定页面的指定位置读取数据
//* 入口参数 : <page_addr>[in] 指定要读取的页面
//*          : <byte_addr>[in] 指定数据在页面中的读取地址
//*          : <pdata>[in]     指向接收缓冲区的指针
//*          : data_len[in]    要读取数据的长度
//* 出口参数 : 无
//*------------------------------------------------------------------------------------------------
INT8U DF_ContinuousRead(INT16U page_addr, INT16U byte_addr, INT8U *pdata, INT32U data_len)
{
	__align(4) INT8U cmd_line[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	FLASH_OPCODE_TYPE opcode;
	INT8U  err;


	if ((page_addr >= FLASH_PAGE_NUM))
		return FLASH_ERR_INVALID_PAGE_ADDR;	
	
	if (byte_addr > FLASH_BUF_SIZE)
		return FLASH_ERR_INVALID_BYTE_ADDR;
	
	if (pdata == NULL)
		return FLASH_ERR_NULL_PTR;		
		
    memset(cmd_line, 0, sizeof(cmd_line));
    
    //* 按Dataflash读写操作格式生成命令行
    opcode.BitsOpcode.ByteAddr = byte_addr;
    opcode.BitsOpcode.PageAddr = page_addr;
    opcode.BitsOpcode.Reserved = 0;
    opcode.BitsOpcode.Opcode   = DB_CONTINUOUS_ARRAY_READ;
    *((unsigned int*)&cmd_line[0]) = HighToLowSwitch(opcode.u32Opcode);
    
    OSMutexPend(hSPI0, 0, &err);
    DF_SendOpcode(cmd_line, 8, pdata, data_len, NULL, NULL);
    OSMutexPost(hSPI0);
    
    return FLASH_NO_ERR;
}
//*------------------------------------------------------------------------------------------------
//* 函数名称 : DF_PageIsErased
//* 功能描述 : 判断指定页面是否已经被擦除
//* 入口参数 : <page_addr>[in] 指定要检查的页面号
//* 出口参数 : FLASH_TRUE  - 已擦除
//*          : FLASH_FALSE - 未擦除或未擦除干净
//*------------------------------------------------------------------------------------------------
static BOOLEAN DF_PageIsErased(INT16U page_addr)
{
	INT16U  i;
	INT8U   temp_buf[FLASH_BUF_SIZE];

    if (page_addr >= FLASH_PAGE_NUM)
        return FLASH_FALSE;	
        				  
    DF_ContinuousRead(page_addr, 0, (INT8U *)temp_buf, FLASH_BUF_SIZE);

    for (i=0; i<FLASH_PAGE_SIZE; i++)
    {
    	if (temp_buf[i] != 0xFF)
    		return FLASH_FALSE; 
    }
    
    return FLASH_TRUE; 
}
//*------------------------------------------------------------------------------------------------
//* 函数名称 : DF_PageErase
//* 功能描述 : 擦除指定页面
//* 入口参数 : <page_addr>[in] 指定要擦除的页面
//* 出口参数 : FLASH_FALSE    操作失败
//*            FLASH_TRUE     操作成功	
//*------------------------------------------------------------------------------------------------
INT8U DF_PageErase(INT16U page_addr)
{
	INT8U              i = 0;
	INT8U              err;
	INT8U              temp;
	__align(4) INT8U   cmd_line[4] = {0, 0, 0, 0};	
	FLASH_OPCODE_TYPE  opcode;
    
    
    if (page_addr >= FLASH_PAGE_NUM)
        return FLASH_ERR_INVALID_PAGE_ADDR;	
		
    while(i++ < 3)
    {
    	if(DF_PageIsErased(page_addr) == FLASH_TRUE)
    		return FLASH_NO_ERR;
    	else
    	{
            //* 按Dataflash读写操作格式生成命令行
		    opcode.BitsOpcode.ByteAddr = 0;
		    opcode.BitsOpcode.PageAddr = page_addr;
		    opcode.BitsOpcode.Reserved = 0;
		    opcode.BitsOpcode.Opcode   = DB_PAGE_ERASE;
		    *((unsigned int*)&cmd_line[0]) = HighToLowSwitch(opcode.u32Opcode);
		    
	        OSMutexPend(hSPI0, 0, &err);
    		DF_SendOpcode(cmd_line, 4, &temp, 0, NULL, NULL);
            while(!(DF_GetStatus() & FLASH_STATUS_RDY));
	        OSMutexPost(hSPI0);
    	}
    }
    return FLASH_ERR_HARD_FAULT;
}
//*------------------------------------------------------------------------------------------------
//* 函数名称 : DF_PageWrite
//* 功能描述 : 向指定页面的指定位置写入数据
//* 入口参数 : <page_addr>[in] 指定要擦除的页面
//*          : <byte_addr>[in] 指定数据在页面中的写入地址
//*          : <pdata>[in]     写入数据的指针
//*          : <data_len>[in]  写入数据的长度
//* 出口参数 : 操作执行结果
//*------------------------------------------------------------------------------------------------
INT8U DF_PageWrite(INT16U page_addr, INT16U byte_addr, INT8U *pdata, INT16U data_len)
{
	__align(4) INT8U   cmd_line[4] = {0, 0, 0, 0};	
	INT8U              temp_buf[FLASH_BUF_SIZE];
	FLASH_OPCODE_TYPE  opcode;
	INT16U             i;
	INT8U              err;
	
	if (page_addr >= FLASH_PAGE_NUM)
		return FLASH_ERR_INVALID_PAGE_ADDR;	
	
	if (byte_addr > FLASH_BUF_SIZE)
		return FLASH_ERR_INVALID_BYTE_ADDR;
	
	if (pdata == NULL)
		return FLASH_ERR_NULL_PTR;
		
	if (data_len > FLASH_BUF_SIZE)
		return FLASH_ERR_DATA_OVF;
			
	
 	//* 将源数据复制至临时缓冲区
	memcpy(temp_buf, pdata, data_len);
	   		
    //* 按Dataflash读写操作格式生成命令行
	opcode.BitsOpcode.ByteAddr = byte_addr;
	opcode.BitsOpcode.PageAddr = page_addr;
    opcode.BitsOpcode.Reserved = 0;
    opcode.BitsOpcode.Opcode   = DB_PAGE_PGM_BUF1;
    *((unsigned int*)&cmd_line[0]) = HighToLowSwitch(opcode.u32Opcode);
    
	OSMutexPend(hSPI0, 0, &err);
	DF_SendOpcode(cmd_line, 4, temp_buf, data_len, NULL, NULL);
	while(!(DF_GetStatus() & FLASH_STATUS_RDY));
	OSMutexPost(hSPI0);
    
    DF_ContinuousRead(page_addr, byte_addr, temp_buf, data_len);
    
    for(i=0; i<data_len; i++)
    {
    	if(temp_buf[i] != pdata[i])
    		return FLASH_ERR_HARD_FAULT;
    }		
	return FLASH_NO_ERR;
}
