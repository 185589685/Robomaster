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
#include "Peripherals_can.h" 
#include "can.h"
#include "pid.h"
#include "stm32f4xx_hal_uart.h"
#include "stdio.h"
#include "math.h"
#include <string.h>
#include "usart.h"

extern uint8_t _cnt,ac,sc;
void ANODT_SendF1(int16_t _a,int16_t _b,int32_t _c);
enum AnoID
{
 Stop = 0x00,  //停止运行
 Operation,   //开始运行
 Store,    //储存

 HWTYPE = 0x05, //硬件种类

 PID_1_P = 11,
 PID_1_I,
 PID_1_D,  //第一组PID

 PID_2_P,
 PID_2_I,
 PID_2_D,  //第二组PID

 PID_3_P,
 PID_3_I,
 PID_3_D   //第三组PID
};

typedef struct
{
 uint8_t Head;
 uint8_t Addr;
 uint8_t ID;
 uint8_t Lenth;
 uint8_t SendBuff[1024]; //发送缓存数组
 uint8_t ReceiveBuf[10]; //接收缓存数组
}_ano;

typedef struct 
{
 uint8_t OrderState;  //命令接收状态

 uint16_t PID_Par1_P;
 uint16_t PID_Par1_I;
 uint16_t PID_Par1_D; //第一组PID

 uint16_t PID_Par2_P;
 uint16_t PID_Par2_I;
 uint16_t PID_Par2_D; //第二组PID

 uint16_t PID_Par3_P;
 uint16_t PID_Par3_I;
 uint16_t PID_Par3_D; //第三组PID
 /* data */
}_Para;

/* Global Nacros ----------------------------------------------------------------------------*/
/**宏定义**/
#define USER_DMA_USART	0		//使用DMA发送接收数据（0为不使用DMA）

#define usart huart6			//使用串口一

#define Color_Black  	0		
#define Color_Red  		1
#define Color_Green  	2	//Log打印颜色

/* Global Defines ---------------------------------------------------------------------------*/
/**全局变量声明**/
extern _ano MyAno;		//声明外部变量
extern _Para MPara;		//参数

/* Global Functions prototypes --------------------------------------------------------------*/
/**全局函数声明**/
/****** 用户不可调用函数 ******/
static void SendParaData(uint8_t Id,int32_t para);		//发送ID对应的参数
static void SendCheckAnalysis(uint8_t id,uint8_t *sumcheck,uint8_t *addcheck);		//发送数据校验帧0x00
static uint8_t Receive_CheckData(uint8_t *_da);		//接收数据校验检测
static uint8_t Send_CheckData(_ano *ano);			//发送数据和校验&附加校验计算

/****** 用户可调用函数 ******/
void Ano_Init(void); 												//参数初始化
void Ano_Send_Data(uint8_t id, void *Data, uint8_t lenth);			//发送数据函数
void Ano_Set_Mdata(uint8_t id,void *data,uint8_t len,uint8_t num);	//多数据配置
void Ano_SendMdata(void);											//多数据发送
void Ano_DataAnalysis(uint8_t *_da);         						//接收数据解析(放串口回调函数里面)
void Ano_SendString(const char *str,uint8_t color);					//发送字符串
void Ano_SendStringVal(const char *str,int32_t Val);				//发送字符串+数据值

void Show_Test(void);	//测试显示函数

/****** 用户可调用&重写函数 ******/
__weak void ControlOrder(uint8_t _Ord);					//控制指令
__weak void ParaOfReturn_Set(uint16_t _id);				//参数回传设置
__weak void ParaRead_Set(uint16_t _id,int32_t _val);	//参数数据读取设置


#endif
/* File Of End ------------------------------------------------------------------------------*/















