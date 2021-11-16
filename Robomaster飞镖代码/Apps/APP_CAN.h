#ifndef __APP_CAN_H
#define __APP_CAN_H
#include "Basis_CAN.h"
/********定义can收发数据的结构体*******/
typedef struct {
	CAN_HandleTypeDef CAN;
	CAN_TxHeaderTypeDef can_send;   //can发送结构体的定义
	CAN_RxHeaderTypeDef can_recevice;//can接收结构体的定义
	uint8_t can_receive_data[8];       //定义储存收到数据的数组
	uint8_t can_send_data[8];          //定义储存发送数据的数组
}CAN_Struct;
/*************************************/
extern CAN_Struct Send_CAN_DATA;
void CAN_SendData(CAN_Struct* CAN_Data, uint32_t Standard, int i1, int i2, int i3, int i4);

#endif

