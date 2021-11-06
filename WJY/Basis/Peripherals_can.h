#ifndef __PERIPHERALS_CAN_H
#define __PERIPHERALS_CAN_H
#include "stm32f4xx_hal.h"


extern CAN_FilterTypeDef CAN_FilterConfigStructure;

#define FILTER_BUF_LEN 5   //�궨���˲������ݳ���
/*������յ������Ϣ�Ľṹ��*/
typedef struct{
signed short int speed_rpm;    //���ÿ����ת��
float real_current;    //��ʵ����ֵ
int given_current;//��������ֵ
unsigned short int moto_temperature;//����¶�
unsigned short int angle;        //���β����Ƕ�
//unsigned short int out_angle;    //����ĽǶ�
//unsigned short int offset_angle; //ƫ�ƽǶ�
//unsigned short int angle_buf[FILTER_BUF_LEN];//�Ƕ���Ϣ �������˲������ݳ���???
}moto_measure_t;

/*����can�շ����ݵĽṹ��*/
typedef struct{
	CAN_HandleTypeDef CAN;
	CAN_TxHeaderTypeDef my_can_send;   //can���ͽṹ��Ķ���
	CAN_RxHeaderTypeDef my_can_recevice;//can���սṹ��Ķ���
	uint8_t can_receive_data[8];       //���崢���յ����ݵ�����
	uint8_t can_send_data[8];          //���崢�淢�����ݵ�����

}CAN_Struct;

/*CAN���ͻ��ǽ��յ�ID*/
typedef enum
{
	CAN_GM6020_Base_ID  = 0x204,
	CAN_GM6020_Moto1_ID = 0x205,
	CAN_GM6020_Moto2_ID = 0x206,
	CAN_GM6020_Moto3_ID = 0x207,
	CAN_GM6020_Moto4_ID = 0x208,
	CAN_GM6020_Moto5_ID = 0x209,
	CAN_GM6020_Moto6_ID = 0x20A,
	CAN_GM6020_Moto7_ID = 0x20B,
	
}CAN_Message_GM6020_ID;
 
typedef enum
{
	CAN_M3508_Base_ID  = 0x200,
	CAN_M3508_Moto1_ID = 0x201,
	CAN_M3508_Moto2_ID = 0x202,
	CAN_M3508_Moto3_ID = 0x203,
	CAN_M3508_Moto4_ID = 0x204,
	CAN_M3508_Moto5_ID = 0x205,
	CAN_M3508_Moto6_ID = 0x206,
	CAN_M3508_Moto7_ID = 0x207,
	
}CAN_Message_M3508_ID;
extern CAN_Struct get_can_data;//�������can���ݵĽṹ
//moto_measure_t moto_wheel[4];
void can_filter_init(CAN_FilterTypeDef sCAN_FilterConfigStructure);//��ʼ��can�˲���
void assignment_moto_M2006_current(CAN_Struct* Assignment_CAN_Data,int i1,int i2,int i3,int i4);//���õ���ֵ
void get_moto_metrical_infromation(moto_measure_t* ptr,  CAN_Struct *GET_CAN_Data);//�õ�����Ĳ�������
void HAL_CAN_RxCpltCallback(CAN_Struct *_HCAN);
extern moto_measure_t moto_chassis[7];//4 chassis moto




#endif



