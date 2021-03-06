/**
 *********************************************************************************************************
 * @file
 * @Author       chen
 * @Version      V1.0
 * @Date         2021-xx-xx
 * @Libsupports  STM32F4xx_DFP(x.x.x)
 * @Brief
 *********************************************************************************************************
**/
/* Global Defines ---------------------------------------------------------------------------*/
/* Includes Files ---------------------------------------------------------------------------*/
/**头文件**/
#include "MyAon.h"

/* Defines ----------------------------------------------------------------------------------*/
/**全局变量定义**/
_ano MyAno = {0};			//发送的数据
_ano Ano_refresh = {0};     //波形同步刷新结构体
/* Local Defines ---------------------------------------------------------------------------*/
/**局部变量声明**/

/* Local Functions Prototypes ---------------------------------------------------------------*/
/**内部函数声明**/

/* Global Functions -------------------------------------------------------------------------*/
/**全局函数定义**/
 /*
 *******************************************************************************
 * @brief   
 * @param   
 * @retval  
 *******************************************************************************
 */
/** 	
 *	函数名:	参数初始化
 *	形	参:	无
 *	返回值:	无
**/
void Ano_Init(void) 
{
	MyAno.Head = 0xAA;
	MyAno.Addr = 0xFF;
	MyAno.Lenth = 8;
}

/**	
 *	函数名:	发送数据和校验&附加校验计算
 *	形	参:	ano结构体
 *	返回值:	1->校验成功 0->校验失败
**/
static uint8_t Send_CheckData(_ano *ano)	
{
	uint8_t i = 0;
	uint8_t sumcheck = 0,addcheck = 0;
	for(i = 0;i < ano->Lenth + 4;i++)
	{
		sumcheck += ano->SendBuff[i];
		addcheck += sumcheck;
	}	
	memcpy(ano->SendBuff + 4 + ano->Lenth,(uint8_t*)&sumcheck,sizeof(sumcheck));
	memcpy(ano->SendBuff + 5 + ano->Lenth,(uint8_t*)&addcheck,sizeof(addcheck));
	/* 其中 ano->SendBuff[3] 表示数据长度 */
	if(sumcheck == ano->SendBuff[ano->SendBuff[3] + 4] && addcheck == ano->SendBuff[ano->SendBuff[3] + 5])
		return 1;
	else
		return 0;
}

/**	
 *	函数名:	发送数据函数
 * 	形	参:	id->功能码（0xF1-0xFA） *Data->发送的数据 lenth->数据长度（sizeof）
 *	返回值:	无
**/
void Ano_Send_Data(uint8_t id,int16_t Data1,int16_t Data2,int16_t Data3,int16_t Data4,int16_t Data5,
	                          int16_t Data6,int16_t Data7,int16_t Data8,int16_t Data9,int16_t Data10,uint8_t lenth)	//发送函数
{
	static uint8_t check;
	MyAno.ID = id;
	MyAno.Lenth = lenth;
	
//	uint16_t data = Data;
//	if(*data > 10000)
//	{
//		*data=0;
//	}

	memcpy(MyAno.SendBuff,(uint8_t*)&MyAno,4);
	memcpy(MyAno.SendBuff + 4 ,&Data1,2);
	memcpy(MyAno.SendBuff + 6 ,&Data2,2);
	memcpy(MyAno.SendBuff + 8 ,&Data3,2);
	memcpy(MyAno.SendBuff + 10,&Data4,2);
	memcpy(MyAno.SendBuff + 12,&Data5,2);
	memcpy(MyAno.SendBuff + 14,&Data6,2);
	memcpy(MyAno.SendBuff + 16,&Data7,2);
	memcpy(MyAno.SendBuff + 18,&Data8,2);
	memcpy(MyAno.SendBuff + 20,&Data9,2);
	memcpy(MyAno.SendBuff + 22,&Data10,2);
	
	check = Send_CheckData(&MyAno);
	if(check)	//如果校验成功则发送数据，校验失败就丢弃此包
	{
		#if USER_DMA_USART	//使用DMA形式
			HAL_UART_Transmit_DMA(&usart,MyAno.SendBuff,MyAno.Lenth + 6);	//使用DMA形式
		#else	//未使用DMA形式
			HAL_UART_Transmit(&usart,MyAno.SendBuff,MyAno.Lenth + 6,0xFFFF);	//未使用DMA形式USART
		#endif
	}
}
	
void Ano_Refresh_Synchronous(void){
 static uint8_t check;
	MyAno.Head  = 0xAA;
	MyAno.Addr  = 0xFF;
	MyAno.ID    = 0xFB;
	MyAno.Lenth = 0x00;
	
    memcpy(MyAno.SendBuff,(uint8_t*)&MyAno,4);
	
	check = Send_CheckData(&MyAno);
	if(check)	//如果校验成功则发送数据，校验失败就丢弃此包
	{
		#if USER_DMA_USART	//使用DMA形式
			HAL_UART_Transmit_DMA(&usart,MyAno.SendBuff,MyAno.Lenth + 6);	//使用DMA形式
		#else	//未使用DMA形式
			HAL_UART_Transmit(&usart,MyAno.SendBuff, 6,0xFFFF);	//未使用DMA形式USART
		#endif
	}
}

/* Local Functions --------------------------------------------------------------------------*/
/**内部函数定义**/


/* File Of End ------------------------------------------------------------------------------*/

