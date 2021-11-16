#ifndef  __BASIS_CAN_H
#define __BASIS_CAN_H
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "stm32f427xx.h"
#include "stdint.h"
#include "can.h"

extern CAN_FilterTypeDef CAN_FilterConfigStructure;

void CAN_Filter_Init(CAN_FilterTypeDef  CAN_FilterConfigStructure);//初始化can滤波器




/*****CAN总线接收GM6020的ID*****/
typedef enum
{
	CAN_GM6020_Base_ID = 0x204,
	CAN_GM6020_Moto1_ID = 0x205,
	CAN_GM6020_Moto2_ID = 0x206,
	CAN_GM6020_Moto3_ID = 0x207,
	CAN_GM6020_Moto4_ID = 0x208,
	CAN_GM6020_Moto5_ID = 0x209,
	CAN_GM6020_Moto6_ID = 0x20A,
	CAN_GM6020_Moto7_ID = 0x20B,

}CAN_Message_GM6020_ID;
/*******************************/

/*****CAN总线接收M3508的ID*****/
typedef enum
{
	CAN_M3508_Base_ID = 0x200,
	CAN_M3508_Moto1_ID = 0x201,
	CAN_M3508_Moto2_ID = 0x202,
	CAN_M3508_Moto3_ID = 0x203,
	CAN_M3508_Moto4_ID = 0x204,
	CAN_M3508_Moto5_ID = 0x205,
	CAN_M3508_Moto6_ID = 0x206,
	CAN_M3508_Moto7_ID = 0x207,
	CAN_M3508_Moto8_ID = 0x208,

}CAN_Message_M3508_ID;
/**********************************/


#endif


