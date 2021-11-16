/*
*****************************************************************************
*@file    Basis_DR16.c
*@author  Dr.Wu
*@version V1.0
*@date
*@brief
*
*****************************************************************************
*@attention
*
*****************************************************************************
*/
#include "Basis_CAN.h"


CAN_FilterTypeDef CAN_FilterConfigStructure;//定义过滤器配置结构体

/*初始化can滤波器*/
void CAN_Filter_Init(CAN_FilterTypeDef CAN_FilterConfigStructure) {
	//uint32_t  StdId = 0x201;
	//uint32_t  ExtId = 0x0000;  

	CAN_FilterConfigStructure.FilterIdHigh = 0x0000;// StdId<<5 STID[10:3]STID[2:0]EXID[17:13] 这里全是0的话表示基本ID和拓展ID的13-17位为0
	CAN_FilterConfigStructure.FilterIdLow = 0x0000;//ID的：EXID[12:5] EXID[4:0] IDE RTR(判断是数据帧还是遥控帧，若此位为0，表示为数据帧)
	CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;//0xfdf8;//这里为掩码的高16位，全是0的话表示基本ID和拓展ID的13-17位为0，表示全不关心，这里我选择的是全为1，就是在这16位中每一位都要对的上
	CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;//掩码低16位	          
	CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
	CAN_FilterConfigStructure.FilterBank = 14;
	CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK; //掩码模式，另一种是列表模式CAN_FILTERMODE_IDLIST
	CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_FilterConfigStructure.FilterActivation = ENABLE;
	CAN_FilterConfigStructure.SlaveStartFilterBank = 0;

	HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfigStructure);
}


