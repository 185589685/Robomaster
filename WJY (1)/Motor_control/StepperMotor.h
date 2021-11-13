#ifndef __STEPEERMOTOR_H
#define __STEPEERMOTOR_H

#include "main.h"

#define stepper_step    1.8/16
#define abs(x)			((x)>0? (x) : (-x))
typedef struct{
	float motor_speed;
	float motor_realangle;
	float motor_targetangle;
	float data;
	float err;
}StepperMotor_t;

void Stepper_setInfo(void);
void Stepper_setAngle(float *angle);

#endif 


