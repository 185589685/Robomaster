#include "Peripherals_can.h" 
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "stm32f427xx.h"
#include "can.h"
//extern 	short int can_receive_data[8];
//CAN_Struct get_can_data;//�������can���ݵĽṹ
//CAN_Struct CAN_DATA;//��ŵ��ǵõ��Ĳ������ݣ���ӵ���λ���ϵ�����Ҳ�Ǵ���������ѡ��
 
moto_measure_t moto_chassis[7] = {0};//4 chassis moto

CAN_FilterTypeDef CAN_FilterConfigStructure;//������������ýṹ��

                               //�����ݴ��뵽���ݽṹ����   ��ʼ��can�Ľṹ�彫������ϵ����
void get_moto_metrical_infromation(moto_measure_t* ptr,  CAN_Struct *GET_CAN_Data)//�õ����������Ϣ
{
	
	
	ptr->angle = (unsigned short int)(GET_CAN_Data->can_receive_data[0]<<8|GET_CAN_Data->can_receive_data[1]); //����ȡ��������ĵ�0λ�͵�1λƴ��������������ĽǶ���Ϣ������intǿ��ת����ʮ���Ƶ���
	ptr->speed_rpm = (signed short int)(GET_CAN_Data->can_receive_data[2]<<8|GET_CAN_Data->can_receive_data[3]); //����ȡ��������ĵ�2λ�͵�3λƴ���������������ת����Ϣ������intǿ��ת����ʮ���Ƶ���
	ptr->real_current = (int)(GET_CAN_Data->can_receive_data[4]<<8|GET_CAN_Data->can_receive_data[5])*5.f/16384.f*50;//ԭ����ȡ����ת�أ�ͨ�����㽫��ȡ����ת��ת�����˵���
    ptr->moto_temperature =(unsigned short int)(GET_CAN_Data->can_receive_data[6]);

}

/*ѡ��һ�������䣬���ñ�ʶ����ID�������ݳ��ȡ���������*/
void assignment_moto_M2006_current(CAN_Struct* Assignment_CAN_Data,int i1,int i2,int i3,int i4){ //��ֵ����ĵ���
	
   
    uint32_t TxBox = CAN_TX_MAILBOX0;//CAN����box0
//    uint8_t FreeTxMailBoxNum;//���������������

	Assignment_CAN_Data->my_can_send.StdId = 0x200;//0x1FF;//��ʶ��ID 
	Assignment_CAN_Data->my_can_send.IDE = CAN_ID_STD;//��׼֡
	Assignment_CAN_Data->my_can_send.RTR = CAN_RTR_DATA;//����֡
	Assignment_CAN_Data->my_can_send.DLC = 0x08;//���ݳ���
	Assignment_CAN_Data->my_can_send.TransmitGlobalTime = DISABLE;
    //���������������
	//if(HAL_CAN_IsTxMessagePending(&hcan1,CAN_TX_MAILBOX0)){}
	
	
//    FreeTxMailBoxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);//�鿴������������
//  	 while(0 == FreeTxMailBoxNum)
//   { }
 
	Assignment_CAN_Data->can_send_data[0] = i1>>8;
	Assignment_CAN_Data->can_send_data[1] = i1;
	Assignment_CAN_Data->can_send_data[2] = i2>>8;
	Assignment_CAN_Data->can_send_data[3] = i2;
	Assignment_CAN_Data->can_send_data[4] = i3>>8;
	Assignment_CAN_Data->can_send_data[5] = i3;
	Assignment_CAN_Data->can_send_data[6] = i4>>8;
	Assignment_CAN_Data->can_send_data[7] = i4;
   //������Ϣ
    HAL_CAN_AddTxMessage(&hcan1,&Assignment_CAN_Data->my_can_send,Assignment_CAN_Data->can_send_data, (uint32_t *)&TxBox);
	//   //������Ͳ��ɹ�����-����ʧ�ܴ���
//	if(HAL_CAN_AddTxMessage(&hcan1,&Assignment_CAN_Data->my_can_send,Assignment_CAN_Data->can_send_data, (uint32_t *)&TxBox) != HAL_OK)
//	{
//	Error_Handler();
//	};
	
}

  
/*��ʼ��can�˲���*/
void can_filter_init(CAN_FilterTypeDef sCAN_FilterConfigStructure){
//	uint32_t  StdId = 0x201;
	//uint32_t  ExtId =0x1800f001;  
	
    //ȫ��0��ʲô��˼�����ֻ����ID201��������ô��
	//
	sCAN_FilterConfigStructure.FilterIdHigh =0x0000;// StdId<<5;//STID[10:3]STID[2:0]EXID[17:13] ����ȫ��0�Ļ���ʾ����ID����չID��13-17λΪ0
	sCAN_FilterConfigStructure.FilterIdLow = 0x0000;//ID�ģ�EXID[12:5] EXID[4:0] IDE RTR(����֡Ϊ0����ң��֡)
	sCAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;//0xfdf8;//����Ϊ����ĸ�16λ��ȫ��0�Ļ���ʾ����ID����չID��13-17λΪ0����ʾȫ�����ģ�������ѡ�����ȫΪ1����������16λ��ÿһλ��Ҫ�Ե���
	sCAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;//�����16λ	          
	sCAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
	sCAN_FilterConfigStructure.FilterBank = 14;
	sCAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK; //����ģʽ����һ�����б�ģʽCAN_FILTERMODE_IDLIST
	sCAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
	sCAN_FilterConfigStructure.FilterActivation = ENABLE;
	sCAN_FilterConfigStructure.SlaveStartFilterBank = 0;
 
	HAL_CAN_ConfigFilter(&hcan1, &sCAN_FilterConfigStructure);
	
//	if( != HAL_OK)
//	{
		//err_deadloop(); //show error!
//	}

}

uint32_t FlashTimer;
/*******************************************************************************************
  * @Func			void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
  * @Brief     HAL���б�׼��CAN������ɻص���������Ҫ�ڴ˴���ͨ��CAN���߽��յ�������
  *                                        �޸İ�
  * @Param		
  * @Retval		None 
  * @Date     
 *******************************************************************************************/
void HAL_CAN_RxCpltCallback(CAN_Struct *_HCAN)
{
//ʹ�ư���һ��Ƶ����˸��֤��������������
//	if(HAL_GetTick() - FlashTimer>500){
//			
//		HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
//		FlashTimer = HAL_GetTick();
//		
//	}

  //�൱��һ���򵥵Ĺ��ˣ������ʶ���������¼������еĻ�����Դ�������
	switch(_HCAN->my_can_recevice.StdId){
		case CAN_M3508_Moto1_ID:
		case CAN_M3508_Moto2_ID:
		case CAN_M3508_Moto3_ID:
		case CAN_M3508_Moto4_ID:
	    case CAN_M3508_Moto5_ID:
	    case CAN_M3508_Moto6_ID:
		case CAN_M3508_Moto7_ID:
			{
			    char i;
				i = _HCAN->my_can_recevice.StdId  - CAN_M3508_Base_ID;//�ж����ڵĵ��IDΪ�ڼ�λ
				
				get_moto_metrical_infromation(&moto_chassis[i], _HCAN);//�õ���Ϣ�����뵽IDΪ��i������
			}
			break;
	}		
		//_HCAN->my_can_recevice = get_can_data.my_can_recevice;
	/*#### add enable can it again to solve can receive only one ID problem!!!####**/
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_FULL);//ʹ��can1��rx0�����ж�
}

//hal�������һЩ����
/*
 ���HAL_CAN_GetState(����˵can1) ==ok ��
�ٽ�����һ��
HAL_CAN_GetRxFifoFillLevel();�õ���������ȼ�
HAL_CAN_IsTxMessagePending();׼��������Ϣ
HAL_CAN_AbortTxRequest();can�ж�����
HAL_CAN_AddTxMessage();���can������Ϣ
HAL_CAN_Start();can��ʼ
HAL_CAN_ConfigFilter();can���˲�������
HAL_CAN_MspInit();can���ų�ʼ������
*/

