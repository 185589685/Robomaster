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


CAN_FilterTypeDef CAN_FilterConfigStructure;//������������ýṹ��

/*��ʼ��can�˲���*/
void CAN_Filter_Init(CAN_FilterTypeDef CAN_FilterConfigStructure) {
	//uint32_t  StdId = 0x201;
	//uint32_t  ExtId = 0x0000;  

	CAN_FilterConfigStructure.FilterIdHigh = 0x0000;// StdId<<5 STID[10:3]STID[2:0]EXID[17:13] ����ȫ��0�Ļ���ʾ����ID����չID��13-17λΪ0
	CAN_FilterConfigStructure.FilterIdLow = 0x0000;//ID�ģ�EXID[12:5] EXID[4:0] IDE RTR(�ж�������֡����ң��֡������λΪ0����ʾΪ����֡)
	CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;//0xfdf8;//����Ϊ����ĸ�16λ��ȫ��0�Ļ���ʾ����ID����չID��13-17λΪ0����ʾȫ�����ģ�������ѡ�����ȫΪ1����������16λ��ÿһλ��Ҫ�Ե���
	CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;//�����16λ	          
	CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
	CAN_FilterConfigStructure.FilterBank = 14;
	CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK; //����ģʽ����һ�����б�ģʽCAN_FILTERMODE_IDLIST
	CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_FilterConfigStructure.FilterActivation = ENABLE;
	CAN_FilterConfigStructure.SlaveStartFilterBank = 0;

	HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfigStructure);
}


