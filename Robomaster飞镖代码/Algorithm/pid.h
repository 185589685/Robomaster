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
/****************pid参数*****************/	
 float kp;
 float ki;
 float kd;
 float kp_out;
 float ki_out;
 float kd_out;	
 float error;//此次误差值
 float last_error;//上次的误差值
 float last_last_error;//上次的误差值
/****************pid参数*****************/		
 int16_t Target;//目标值
 int16_t Measure; //真实值
 int16_t output_value;//输出值	
	
/*******以下参数为专家pid算法独有********/	
 float k_gain; //增益放大系数 恒大于1
 float k_restrain;//抑制系数
 float High_error_margin;//较高的误差界限
 float Low_error_margin;//较低的误差界限
 float allowed_minimum_error;//允许的最小误差	
/****************************************/
 } PID_Typedef ;
/**************不同pid算法下计算值类型不同***********/
typedef struct{
	PID_Typedef incremental_pid;    //增量式pid
	PID_Typedef position_pid;       //位置式pid
	PID_Typedef Expert_PID;         //专家pid
}PID_t;
/****************************************************/
/*定义CAN总线上接收到电机信息的结构体*/
typedef struct{
 int16_t speed_rpm;    //电机每分钟转速
 int16_t current;    //真实电流值
 int16_t temperature;//电机温度
 int16_t angle;        //本次测量角度
}moto_measure_t;
/************************************/

/**************PID公式计算的结构体定义****************/
typedef struct{
	uint16_t realAngle;			    //读取的机械角度
	int16_t  realSpeed;			    //读取的速度
	int16_t  realCurrent;		    //读取的实际电流
	uint8_t  realtemperture;        //读取的电机温度
	uint16_t last_realAngle;		//读取的机械角度
	int16_t  circle_cnt;            //电机圈数
	int16_t  targetCurrent;			//目标电流
	int16_t  targetSpeed;			//目标速度
	int32_t  targetAngle;			//目标角度
	
	PID_t Angle;                    //用于角度计算
	PID_t Speed;                    //用于速度计算
	PID_t Current;                  //用于电流计算

}Motor_t;
/****************************************************/

/*全局定义*/
extern moto_measure_t moto_chassis[8];/*存储8个电机的CAN总线读取出来的真实值*/
extern Motor_t M3508[8];/*最终存放电机所有参数的结构体*/
extern float Incremental_PID(PID_Typedef *ptr,int32_t measured,int32_t target,float kp,float ki,float kd,int16_t Max_Output_Limit);/*增量式pid*/
extern float    Position_PID(PID_Typedef *ptr,int32_t measured,int32_t target,float kp,float ki,float kd,int16_t Max_Output_Limit);/*位置式pid*/
extern float      Expert_PID(PID_Typedef *ptr,float Measure,float Target,float KP, float KI, float KD, float k_gain, float k_restrain, 
	                                      float High_error_margin,float Low_error_margin, float allowed_minimum_error);            /*专家pid计算*/
extern int16_t circle_cnt;/*圈数*/
extern int32_t Encoder_values,Last_Encoder_values;/*编码器值*/
extern float   Zero_Treated(void);/*过零处理*/
extern float KP1,KI1,KD1,KP2,KI2,KD2,KP3,KI3,KD3;/*pid调试所用*/
#endif
