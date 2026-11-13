/********************************************************************************************************
**                            (c) Copyright  苏州市江海通讯发展实业有限公司
**                                      All Rights Reserved                                        
**
**---------------------------- 文件信息 -----------------------------------------------------------------
** 文   件   名:  OWDrivers.c
** 创   建   人:  lyg
** 版        本:  V2.0
** 最后修改日期:  2009-7-22
** 描        述:  1-Wire 驱动程序，实现对1-Wire总线进行位、字节和块操作时所必需的函数。                   
** 操作系统版本： 无RTOS 
**             
**--------------------------- 历史版本信息 --------------------------------------------------------------
** 创   建   人:  林依刚
** 版        本:  V1.0
** 日　      期:  2007-12-15
** 描　      述:  原始版本
**
********************************************************************************************************/
#include"OWDriver.h"


// 定义1-WIRE通信时隙参数
int Tick_A,Tick_B,Tick_C,Tick_D,Tick_E,Tick_F,Tick_G,Tick_H,Tick_I,Tick_J;                


/********************************************************************************************************
** 函数名称： SetSpeed
** 函数功能： Set tick values for 1-Wire operate.Adjust tick values depending on speed type.
** 入口参数： stardard： speed type(1:"Standard Speed";  0:"Overdrive Speed")
** 出口参数： None
** 说    明： Set the 1-Wire timing to 'standard' (standard==1) or 'overdrive' (standard==0).
********************************************************************************************************/
void SetSpeed(int standard)
{ 
	if (standard)
	{ 	
		Tick_A = 6 * 4;
		Tick_B = 64 * 4;
		Tick_C = 60 * 4;
		Tick_D = 10 * 4;
		Tick_E = 9 * 4;
		Tick_F = 55 * 4;
		Tick_G = 0;
		Tick_H = 480 * 4;
		Tick_I = 70 * 4;
		Tick_J = 410 * 4;
	}
	else
	{  
		Tick_A = 1.5 * 4;
		Tick_B = 7.5 * 4;
		Tick_C = 7.5 * 4;
		Tick_D = 2.5 * 4;
		Tick_E = 0.75 * 4;
		Tick_F = 7 * 4;
		Tick_G = 2.5 * 4;
		Tick_H = 70 * 4;
		Tick_I = 8.5 * 4;
		Tick_J = 40 * 4;
	}
}

/********************************************************************************************************
** 函数名称： OWTouchReset
** 函数功能： Generate a 1-Wire reset
** 入口参数： None
** 出口参数： return 1 if no presence detect was found,return 0 otherwise.
** 说    明： Does not handle alarm presence from DS2404/DS1994.
********************************************************************************************************/
int OWTouchReset(void)
{
	int result;

	tickDelay(Tick_G);
	outp(PORTADDRESS,0x00);             // Drives DQ low 
	tickDelay(Tick_H);
	outp(PORTADDRESS,0x01);             // Releases the bus
	tickDelay(Tick_I);
	result = inp(PORTADDRESS) & 0x01;   // Sample for presence pulse from slave
	tickDelay(Tick_J);                  // Complete the reset sequence recovery
	return(result);                     // Return sample presence pulse result
}

/********************************************************************************************************
** 函数名称： OWWriteBit
** 函数功能： Send a 1-Wire write bit. Provide 10us recovery time.
** 入口参数： bit_level:   the level nead to set on the bus.
** 出口参数： None
********************************************************************************************************/
void OWWriteBit(int bit_level)
{
	if (bit_level){                     // Write '1' bit		
		outp(PORTADDRESS,0x00);         // Drives DQ low
		tickDelay(Tick_A);
		outp(PORTADDRESS,0x01);         // Releases the bus
		tickDelay(Tick_B);              // Complete the time slot and 10us recovery
	}
	else{                               // Write '0' bit		
		outp(PORTADDRESS,0x00);         // Drives DQ low
		tickDelay(Tick_C);
		outp(PORTADDRESS,0x01);         // Releases the bus
		tickDelay(Tick_D);
	}
}

/********************************************************************************************************
** 函数名称： OWReadBit
** 函数功能： Read a bit from the 1-Wire bus and return it. Provide 10us recovery time.
** 入口参数： None
** 出口参数： result： the value of read bit.
********************************************************************************************************/
int OWReadBit(void)
{
	int result;

	outp(PORTADDRESS,0x00);             // Drives DQ low
	tickDelay(Tick_A);
	outp(PORTADDRESS,0x01);             // Releases the bus
	tickDelay(Tick_E);
	result = inp(PORTADDRESS) & 0x01;   // Sample the bit value from the slave
	tickDelay(Tick_F);                  // Complete the time slot and 10us recovery

	return(result);
}

/********************************************************************************************************
** 函数名称： OWWriteByte
** 函数功能： Write 1-Wire data byte
** 入口参数： data_val:  the data nead to write
** 出口参数： None
********************************************************************************************************/
void OWWriteByte(int data_val)
{
	int loop;

	// Loop to write each bit in the byte, LS-bit first
	for (loop = 0; loop < 8; loop++)
	{
		OWWriteBit(data_val & 0x01);		
		data_val >>= 1;                      // shift the data byte for the next bit
	}
}

/********************************************************************************************************
** 函数名称： OWReadByte
** 函数功能： Read 1-Wire data byte and return it
** 入口参数： None
** 出口参数： result： the read byte
********************************************************************************************************/
int OWReadByte(void)
{
	int loop;
	int result = 0;

	for (loop = 0; loop < 8; loop++) {		
		result >>= 1;                        // shift the result to get it ready for the next bit		
		if (OWReadBit())                     // if result is one, then set MS-bit
			result |= 0x80;
	}
	return(result);
}

/********************************************************************************************************
** 函数名称： OWTouchByte
** 函数功能： Write a 1-Wire data byte and return the sampled result.
** 入口参数： data_val:  待写的字节数据
** 出口参数： 返回read-wrire-read 之结果
** 说    明： OWTouchByte函数可以同时完成读写1-Wire总线数据，通过该函数可以实现数据块的读写。
**            在一些平台上执行效率更高， Dallas Semiconductor提供的API就采用了这种函数。
**            通过OWTouchByte函数，OWBlock函数简化了1-Wire总线的数据块发送和接收。
**            注意：OWTouchByte(0xFF)与OWReadByte()等效，OWTouchByte(data)与OWWriteByte(data)等效。
********************************************************************************************************/
int OWTouchByte(int data_val)
{
	int loop;
	int result = 0;

	for (loop = 0; loop < 8; loop++) {		
		result >>= 1;                        // shift the result to get it ready for the next bit		
		if (data_val & 0x01) {               // If sending a '1' then read a bit else write a '0'
			if (OWReadBit())
				result |= 0x80;
		}
		else
			OWWriteBit(0);		
		data_val >>= 1;                      // shift the data byte for the next bit
	}
	return(result);
}

/********************************************************************************************************
** 函数名称： OWBlock
** 函数功能： Write a block 1-Wire data bytes and return the sampled result in the same buffer.
** 入口参数： pdata   :  待写数据块之指针
**            data_len:  待写数据之长度
** 出口参数： 无
********************************************************************************************************/
void OWBlock(unsigned char *pdata, int data_len)
{
	int loop;

	for (loop = 0; loop < data_len; loop++) {
		pdata[loop] = OWTouchByte(pdata[loop]);
	}
}

/********************************************************************************************************
** 函数名称： OWOverdriveSkip
** 函数功能： Set all devices on 1-Wire to overdrive speed.
** 入口参数： pdata   :  待写数据块之指针
**            data_len:  待写数据之长度
** 出口参数： Return '1' if at least one overdrive capable device is detected.
********************************************************************************************************/
int OWOverdriveSkip(unsigned char *pdata, int data_len)
{	
	SetSpeed(1);                        // set the speed to 'standard'		
	if (OWTouchReset())                 // reset all devices                  
		return 0;                           
	
	OWWriteByte(0x3C);                  // overdrive skip command	
	SetSpeed(0);                        // set the speed to 'overdrive'	
	return(OWTouchReset());             // do a 1-Wire reset in 'overdrive' and return presence result
}