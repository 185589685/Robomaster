#ifndef __APP_CAN_H
#define __APP_CAN_H
#include "Basis_CAN.h"
/********����can�շ����ݵĽṹ��*******/
typedef struct {
	CAN_HandleTypeDef CAN;
	CAN_TxHeaderTypeDef can_send;   //can���ͽṹ��Ķ���
	CAN_RxHeaderTypeDef can_recevice;//can���սṹ��Ķ���
	uint8_t can_receive_data[8];       //���崢���յ����ݵ�����
	uint8_t can_send_data[8];          //���崢�淢�����ݵ�����
}CAN_Struct;
/*************************************/
extern CAN_Struct Send_CAN_DATA;
void CAN_SendData(CAN_Struct* CAN_Data, uint32_t Standard, int i1, int i2, int i3, int i4);

#endif

