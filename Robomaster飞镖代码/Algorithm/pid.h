#ifndef __PID_H
#define __PID_H
#include "stm32f4xx_hal.h"
#include "stm32f427xx.h"
#include "stm32f4xx_hal_can.h"
#include "can.h"
#include "math.h"
#include "stdio.h"
#include "stdint.h"
extern int16_t target;
typedef struct  
{
/****************pid����*****************/	
 float kp;
 float ki;
 float kd;
 float kp_out;
 float ki_out;
 float kd_out;	
 float error;//�˴����ֵ
 float last_error;//�ϴε����ֵ
 float last_last_error;//�ϴε����ֵ
/****************pid����*****************/		
 int16_t Target;//Ŀ��ֵ
 int16_t Measure; //��ʵֵ
 int16_t output_value;//���ֵ	
	
/*******���²���Ϊר��pid�㷨����********/	
 float k_gain; //����Ŵ�ϵ�� �����1
 float k_restrain;//����ϵ��
 float High_error_margin;//�ϸߵ�������
 float Low_error_margin;//�ϵ͵�������
 float allowed_minimum_error;//�������С���	
/****************************************/
 } PID_Typedef ;
/**************��ͬpid�㷨�¼���ֵ���Ͳ�ͬ***********/
typedef struct{
	PID_Typedef incremental_pid;    //����ʽpid
	PID_Typedef position_pid;       //λ��ʽpid
	PID_Typedef Expert_PID;         //ר��pid
}PID_t;
/****************************************************/
/*����CAN�����Ͻ��յ������Ϣ�Ľṹ��*/
typedef struct{
 int16_t speed_rpm;    //���ÿ����ת��
 int16_t current;    //��ʵ����ֵ
 int16_t temperature;//����¶�
 int16_t angle;        //���β����Ƕ�
}moto_measure_t;
/************************************/

/**************PID��ʽ����Ľṹ�嶨��****************/
typedef struct{
	uint16_t realAngle;			    //��ȡ�Ļ�е�Ƕ�
	int16_t  realSpeed;			    //��ȡ���ٶ�
	int16_t  realCurrent;		    //��ȡ��ʵ�ʵ���
	uint8_t  realtemperture;        //��ȡ�ĵ���¶�
	uint16_t last_realAngle;		//��ȡ�Ļ�е�Ƕ�
	int16_t  circle_cnt;            //���Ȧ��
	int16_t  targetCurrent;			//Ŀ�����
	int16_t  targetSpeed;			//Ŀ���ٶ�
	int32_t  targetAngle;			//Ŀ��Ƕ�
	
	PID_t Angle;                    //���ڽǶȼ���
	PID_t Speed;                    //�����ٶȼ���
	PID_t Current;                  //���ڵ�������

}Motor_t;
/****************************************************/

/*ȫ�ֶ���*/
extern moto_measure_t moto_chassis[8];/*�洢8�������CAN���߶�ȡ��������ʵֵ*/
extern Motor_t M3508[8];/*���մ�ŵ�����в����Ľṹ��*/
extern float Incremental_PID(PID_Typedef *ptr,int32_t measured,int32_t target,float kp,float ki,float kd,int16_t Max_Output_Limit);/*����ʽpid*/
extern float    Position_PID(PID_Typedef *ptr,int32_t measured,int32_t target,float kp,float ki,float kd,int16_t Max_Output_Limit);/*λ��ʽpid*/
extern float      Expert_PID(PID_Typedef *ptr,float Measure,float Target,float KP, float KI, float KD, float k_gain, float k_restrain, 
	                                      float High_error_margin,float Low_error_margin, float allowed_minimum_error);            /*ר��pid����*/
extern int16_t circle_cnt;/*Ȧ��*/
extern int32_t Encoder_values,Last_Encoder_values;/*������ֵ*/
extern float   Zero_Treated(void);/*���㴦��*/
extern float KP1,KI1,KD1,KP2,KI2,KD2,KP3,KI3,KD3;/*pid��������*/
#endif
