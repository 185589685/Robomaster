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
CAN_Struct Send_CAN_DATA = { 0 };//�������ݵĽṹ��
/*ѡ�����䣬���ñ�ʶ����ID�������ݳ��ȡ���������*/
void CAN_SendData(CAN_Struct* CAN_Data, uint32_t Standard, int i1, int i2, int i3, int i4) { //��ֵ����ĵ���

	uint32_t TxBox = CAN_TX_MAILBOX0;//CAN����box0

	CAN_Data->can_send.StdId = Standard;// 0x200;//0x1FF;//��ʶ��ID 
	CAN_Data->can_send.IDE = CAN_ID_STD;//��׼֡
	CAN_Data->can_send.RTR = CAN_RTR_DATA;//����֡
	CAN_Data->can_send.DLC = 0x08;//���ݳ���
	CAN_Data->can_send.TransmitGlobalTime = DISABLE;

	CAN_Data->can_send_data[0] = i1 >> 8;
	CAN_Data->can_send_data[1] = i1;
	CAN_Data->can_send_data[2] = i2 >> 8;
	CAN_Data->can_send_data[3] = i2;
	CAN_Data->can_send_data[4] = i3 >> 8;
	CAN_Data->can_send_data[5] = i3;
	CAN_Data->can_send_data[6] = i4 >> 8;
	CAN_Data->can_send_data[7] = i4;
	//HAL�ⷢ����Ϣ��������
	HAL_CAN_AddTxMessage(&hcan1, &CAN_Data->can_send, CAN_Data->can_send_data, (uint32_t*)&TxBox);
}









