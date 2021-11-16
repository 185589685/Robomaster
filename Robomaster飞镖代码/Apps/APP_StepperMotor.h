#ifndef  __APP_STEPEERMOTOR_H
#define __APP_STEPEERMOTOR_H
#define stepper_step    1.8/16
#define abs(x)			((x)>0? (x) : (-x))
#include "tim.h"
#include "math.h"
#include "gpio.h"
typedef struct {
	float motor_speed;
	float motor_realangle;
	float motor_targetangle;
	float data;
	float err;
}StepperMotor_t;

void Stepper_setInfo(void);//步进电机参数的置零
void Stepper_setAngle(float* angle);//步进电机的角度函数

#endif 

