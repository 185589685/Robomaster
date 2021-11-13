#ifndef __PID_H
#define __PID_H
#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "Peripherals_can.h" 
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "stm32f427xx.h"
#include "can.h"
#include "math.h"
#include "stdio.h"
#include "typedef.h"
typedef struct  
{
 float kp;
 float ki;
 float kd;
 float k_gain; //����Ŵ�ϵ�� �����1
 float k_restrain;//����ϵ��
 float High_error_margin;//�ϸߵ�������
 float Low_error_margin;//�ϵ͵�������
 float kp_out;
 float ki_out;
 float kd_out;
	
 float error;//�˴����ֵ
 float last_error;//�ϴε����ֵ
 float last_last_error;//�ϴε����ֵ
 float allowed_minimum_error; 	
 int16_t Target;//Ŀ��ֵ
 int16_t Measure; //��ʵֵ
 int16_t last_true_value;//��һ����ʵֵ
 int16_t output_value;//���ֵ
 } PID_Typedef ;

typedef struct{
	uint16_t realAngle;			        //�������Ļ�е�Ƕ�
	int16_t  realSpeed;			        //���������ٶ�
	int16_t  realCurrent;		        //��������ʵ�ʵ���
	uint8_t  temperture;            //�������ĵ���¶�
	
	int16_t  targetCurrent;			    //Ŀ�����
	int16_t  targetSpeed;			      //Ŀ���ٶ�
	int32_t  targetAngle;			      //Ŀ��Ƕ�
	int16_t  outCurrent;				    //�������

	PID_Typedef incremental_pid;
	PID_Typedef position_pid;
}Motor_t;

extern Motor_t M3508[8];

extern PID_Typedef incremental_pid_angle,incremental_pid_speed,incremental_pid_current,position_pid_angle,position_pid_speed,angle_pid;
extern float  Incremental_PID(Motor_t  *ptr,int32_t measured,int32_t target,float kp,float ki,float kd);
extern float  Position_PID(Motor_t *ptr,int32_t measured,int32_t target,float kp,float ki,float kd);
extern float  Expert_PID(PID_Typedef *ptr,float Measure,float Target,float KP, float KI, float KD, float k_gain, float k_restrain, float High_error_margin,float Low_error_margin, float allowed_minimum_error);

extern uint8_t message;
extern uint8_t Len;
 
extern int32_t target_angle[8],target_speed[8],target_current[8];
extern int16_t circle_cnt;
extern int32_t Encoder_values,Last_Encoder_values;
//extern float k_gain,k_restrain,High_error_margin,Low_error_margin,allowed_minimum_error;
extern float Zero_Treated(float ture_value);

void Motor_para_Init(void);
#endif


 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 





















