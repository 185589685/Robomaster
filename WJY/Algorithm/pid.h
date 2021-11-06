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

typedef struct  
{
 float kp;
 float ki;
 float kd;
 float k_gain; //增益放大系数 恒大于1
 float k_restrain;//抑制系数
 float High_error_margin;//较高的误差界限
 float Low_error_margin;//较低的误差界限
 float kp_out;
 float ki_out;
 float kd_out;
	
 float error;//此次误差值
 float last_error;//上次的误差值
 float last_last_error;//上次的误差值
 float allowed_minimum_error; 	
 int32_t Target;//目标值
 int32_t Measure; //真实值
 int32_t last_true_value;//上一次真实值
 float output_value;//输出值
 } PID_Typedef;

 
extern PID_Typedef incremental_pid_angle,incremental_pid_speed,incremental_pid_current,position_pid_angle,position_pid_speed,angle_pid;
extern float Incremental_PID(PID_Typedef  *ptr,int32_t measured,int32_t target,float kp,float ki,float kd);
extern float  Position_PID(PID_Typedef  *ptr,int32_t measured,int32_t target,float kp,float ki,float kd);
extern float  Expert_PID(PID_Typedef *ptr,float Measure,float Target,float KP, float KI, float KD, float k_gain, float k_restrain, float High_error_margin,float Low_error_margin, float allowed_minimum_error);

extern uint8_t message;
extern uint8_t Len;
 
extern int32_t target_angle[8],target_speed[8],target_current[8];
extern int16_t circle_cnt;
extern int32_t Encoder_values,Last_Encoder_values;
extern float k_gain,k_restrain,High_error_margin,Low_error_margin,allowed_minimum_error;

#endif


 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 





















