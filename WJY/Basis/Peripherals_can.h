#ifndef __PERIPHERALS_CAN_H
#define __PERIPHERALS_CAN_H
#include "stm32f4xx_hal.h"


extern CAN_FilterTypeDef CAN_FilterConfigStructure;

#define FILTER_BUF_LEN 5   //宏定义滤波器数据长度
/*定义接收到电机信息的结构体*/
typedef struct{
signed short int speed_rpm;    //电机每分钟转速
float real_current;    //真实电流值
int given_current;//给定电流值
unsigned short int moto_temperature;//电机温度
unsigned short int angle;        //本次测量角度
//unsigned short int out_angle;    //输出的角度
//unsigned short int offset_angle; //偏移角度
//unsigned short int angle_buf[FILTER_BUF_LEN];//角度信息 参数：滤波器数据长度???
}moto_measure_t;

/*定义can收发数据的结构体*/
typedef struct{
	CAN_HandleTypeDef CAN;
	CAN_TxHeaderTypeDef my_can_send;   //can发送结构体的定义
	CAN_RxHeaderTypeDef my_can_recevice;//can接收结构体的定义
	uint8_t can_receive_data[8];       //定义储存收到数据的数组
	uint8_t can_send_data[8];          //定义储存发送数据的数组

}CAN_Struct;

/*CAN发送或是接收的ID*/
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
extern CAN_Struct get_can_data;//定义接收can数据的结构
//moto_measure_t moto_wheel[4];
void can_filter_init(CAN_FilterTypeDef sCAN_FilterConfigStructure);//初始化can滤波器
void assignment_moto_M2006_current(CAN_Struct* Assignment_CAN_Data,int i1,int i2,int i3,int i4);//设置电流值
void get_moto_metrical_infromation(moto_measure_t* ptr,  CAN_Struct *GET_CAN_Data);//得到电机的测量数据
void HAL_CAN_RxCpltCallback(CAN_Struct *_HCAN);
extern moto_measure_t moto_chassis[7];//4 chassis moto




#endif



