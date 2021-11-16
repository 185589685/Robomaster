/*
*****************************************************************************
*@file    BSP_DR16.c
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
#include "APP_CAN.h"
CAN_Struct Send_CAN_DATA = { 0 };//发送数据的结构体
/*选择邮箱，设置标识符（ID）、数据长度、发送数据*/
void CAN_SendData(CAN_Struct* CAN_Data, uint32_t Standard, int i1, int i2, int i3, int i4) { //赋值电机的电流

	uint32_t TxBox = CAN_TX_MAILBOX0;//CAN发送box0

	CAN_Data->can_send.StdId = Standard;// 0x200;//0x1FF;//标识符ID 
	CAN_Data->can_send.IDE = CAN_ID_STD;//标准帧
	CAN_Data->can_send.RTR = CAN_RTR_DATA;//数据帧
	CAN_Data->can_send.DLC = 0x08;//数据长度
	CAN_Data->can_send.TransmitGlobalTime = DISABLE;

	CAN_Data->can_send_data[0] = i1 >> 8;
	CAN_Data->can_send_data[1] = i1;
	CAN_Data->can_send_data[2] = i2 >> 8;
	CAN_Data->can_send_data[3] = i2;
	CAN_Data->can_send_data[4] = i3 >> 8;
	CAN_Data->can_send_data[5] = i3;
	CAN_Data->can_send_data[6] = i4 >> 8;
	CAN_Data->can_send_data[7] = i4;
	//HAL库发送信息基本函数
	HAL_CAN_AddTxMessage(&hcan1, &CAN_Data->can_send, CAN_Data->can_send_data, (uint32_t*)&TxBox);
}









