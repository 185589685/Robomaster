/*
*****************************************************************************
*@file    pid.c
*@author  Dr.Wu
*@version V1.0
*@date    2021/11/5
*@brief
*
*****************************************************************************
*@attention
*
*****************************************************************************
*/
#include "pid.h"
/******************������ʼ��*************************/
int32_t Encoder_values,Last_Encoder_values = 0;
int16_t circle_cnt,target= 0;
moto_measure_t moto_chassis[8] = {0};//�洢8�������CAN���߶�ȡ��������ʵֵ
Motor_t M3508[8] = {0};
float KP1,KI1,KD1,KP2,KI2,KD2,KP3,KI3,KD3 = 0 ;
/****************************************************/

/********************���㴦��*************************/
float Zero_Treated(void){
	
	for(int i = 0;i < 8;i++){
	 if(M3508[i].realAngle - M3508[i].last_realAngle > 6000){
	    M3508[i].circle_cnt--;
	 }
	 else if(M3508[i].realAngle - M3508[i].last_realAngle < -6000){
	    M3508[i].circle_cnt++;
	 }
	 switch(M3508[i].circle_cnt){
		 case 32767: M3508[i].circle_cnt = 0;
	 }
	 
	}
	return 0;
//	return M3508[i].circle_cnt;
//   Encoder_values = ture_value;
//   
// //���㴦������ֵ - ��һ������ֵ > 4096 �ж��Ѿ���0
//  if(Encoder_values - Last_Encoder_values > +6000){ 
//	  circle_cnt--;
//  }
//  if(Encoder_values - Last_Encoder_values < -6000 ){
//      circle_cnt++;
//  }
//  Last_Encoder_values = Encoder_values;
//  if(circle_cnt == 32767){
//   circle_cnt = 0;
//  }
//  return circle_cnt;
}
/****************************************************/
/*******************************************************����ʽpid**********************************************************/
float Incremental_PID(PID_Typedef *ptr,int32_t measured,int32_t target,float kp,float ki,float kd,int16_t Max_Output_Limit)
{ 
/*��������*/
  ptr->kp = kp;
  ptr->ki = ki;
  ptr->kd = kd;	
  ptr->Target  = target;
  ptr->Measure = measured;
  ptr-> error = ptr->Target - ptr->Measure;
	
/*��������ֵ�ֵ*/	
  ptr->kp_out = kp*(ptr->error - ptr->last_error);
  ptr->ki_out = ki*ptr->error;
  ptr->kd_out = kd*(ptr->error - 2*ptr->last_error + ptr->last_last_error);		
  ptr->output_value += ptr->kp_out + ptr->ki_out + ptr->kd_out;
/*���ֵ�Ĵ���*/  
  ptr->last_error = ptr->error;
  ptr->last_last_error = ptr->last_error;
	
/*����������*/
  if(ptr->output_value >= Max_Output_Limit)
  {
    ptr->output_value = Max_Output_Limit;
  }
  else if(ptr->output_value < -Max_Output_Limit)
  {
  ptr->output_value = -Max_Output_Limit;
  }
  return  ptr->output_value;
}
/**************************************************************************************************************************/
/*******************************************************λ��ʽpid**********************************************************/
float Position_PID(PID_Typedef *ptr,int32_t measured,int32_t target,float kp,float ki,float kd,int16_t Max_Output_Limit){
//��������
  ptr->kp = kp;
  ptr->ki = ki;
  ptr->kd = kd;	
  ptr->Target  = target;
  ptr->Measure = measured;
  ptr->error = ptr->Target - ptr->Measure;
  
//��ʼ����pid�㷨    
  ptr->kp_out = ptr->kp*ptr->error;	
  ptr->ki_out += ptr->ki*ptr->error;
  ptr->kd_out = ptr->kd*(ptr->error - ptr->last_error);
  ptr->output_value =  ptr->kp_out + ptr->ki_out + ptr->kd_out;
//���˴�����ֵ������һ����� 
  ptr->last_error = ptr->error;  
  
//������������ܳ��������Χ
  if(ptr->output_value >= 6600){ptr->output_value = 6600;}
  else if(ptr->output_value < -6600){ptr->output_value = -6600;}
  
  return  ptr->output_value;
}
/**************************************************************************************************************************/
/****************************************************************************************ר��pid����******************************************************************************************************/
float  Expert_PID(PID_Typedef *ptr,float Measure,float Target,float KP, float KI, float KD, float k_gain, float k_restrain, float High_error_margin,float Low_error_margin, float allowed_minimum_error) {

	float uk1,uk2 = 0;
	ptr->kp = KP;
	ptr->ki = KI;
	ptr->kd = KD;

	ptr->error = Target - Measure;
	if(ptr->error > ptr->High_error_margin) {
		uk1 = ptr->kp * ptr->error;
	} else {
		if(ptr->error * (ptr->error - ptr->last_error) > 0) {
			if((unsigned)(ptr->error) >= ptr->Low_error_margin ) {
				uk1 = uk2 + ptr->k_gain *(ptr->kp * (ptr->error - ptr->last_error) +
				                          ptr->ki * ptr->error +
				                          ptr->kd * (ptr->error - 2*ptr->last_error +ptr->last_last_error ));
			} else {
				uk1 = uk2 + ptr->kp * (ptr->error - ptr->last_error) +
				      ptr->ki * ptr->error +
				      ptr->kd * (ptr->error - 2 * ptr->last_error +ptr->last_last_error );
			} 
		} else if(ptr->error * (ptr->error - ptr->last_error) < 0) {
			if((ptr->error - ptr->last_error) * (ptr->last_error - ptr->last_last_error ) > 0) {
			} else if((ptr->error - ptr->last_error) * (ptr->last_error - ptr->last_last_error ) < 0) { 
				if((unsigned)(ptr->error) >= ptr->Low_error_margin) {
					uk1 = uk2 + ptr->k_gain * ptr->kp * ptr->last_error ;
				} else {
					uk1 = uk2 + ptr->k_restrain * ptr->kp * ptr->last_error ;
				}
			}
		}
		else if( fabs(ptr->error) < ptr->allowed_minimum_error){
			uk1 += (0.5f * ptr->error);
		}

	}

	uk2 = uk1;
	ptr->last_error = ptr->error;

	ptr->last_last_error = ptr->last_error;
	  if(uk1 >= 30000){uk1 = 30000;}
  else if(uk1 < -30000){uk1 = -30000;}
	
	return uk1;

}
/*********************************************************************************************************************************************************************************************************/








