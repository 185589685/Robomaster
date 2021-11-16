/**
 *********************************************************************************************************
 * @file                
 * @Author          chen
 * @Version         V1.0
 * @Date             2021-xx-xx
 * @Libsupports   STM32F4xx_DFP(x.x.x)
 * @Brief              
 *********************************************************************************************************
**/
#ifndef __MYANO_H
#define __MYANO_H
/* Includes Files ---------------------------------------------------------------------------*/
#include "can.h"
#include "pid.h"
#include "stm32f4xx_hal_uart.h"
#include "stdio.h"
#include "math.h"
#include <string.h>
#include "usart.h"

typedef struct
{
 uint8_t Head;
 uint8_t Addr;
 uint8_t ID;
 uint8_t Lenth;
 uint8_t SendBuff[1024]; //发送缓存数组
 uint8_t ReceiveBuf[10]; //接收缓存数组
}_ano;

/* Global Nacros ----------------------------------------------------------------------------*/
/**宏定义**/
#define USER_DMA_USART	0		//使用DMA发送接收数据（0为不使用DMA）
#define usart huart6			//使用串口一

/* Global Defines ---------------------------------------------------------------------------*/
/**全局变量声明**/

/* Global Functions prototypes --------------------------------------------------------------*/
/**全局函数声明**/
/****** 用户不可调用函数 ******/
static uint8_t Send_CheckData(_ano *ano);			//发送数据和校验&附加校验计算

/****** 用户可调用函数 ******/
void Ano_Init(void); 												//参数初始化
void Ano_Send_Data(uint8_t id, void *Data, uint8_t lenth);			//发送数据函数
void Ano_Refresh_Synchronous(void);
#endif
/* File Of End ------------------------------------------------------------------------------*/















