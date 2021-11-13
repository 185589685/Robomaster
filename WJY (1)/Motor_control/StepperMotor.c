#include "StepperMotor.h"
#include "tim.h"
#include "math.h"
#include "gpio.h"

StepperMotor_t StepperMotor;
/* speed = (f*60)/(360/T)*x
speed :��������ٶ�
f: ��ʱ��Ƶ��
x: ϸ�ֱ��� =16
T: ����� 1.8

f = 60*speed
T = 1/f
�ı�Ƶ�ʾͿ��Կ����ٶ���
һ������ת��1.8/16��
*/
void Stepper_setInfo(void)
{
	StepperMotor.motor_speed = 0.0f;
	StepperMotor.motor_realangle = 0.0f;
	StepperMotor.data = 0.0f;
	StepperMotor.motor_targetangle = 0.0f;
	StepperMotor.err = 0.0f;
}

void Stepper_setAngle(float *angle)
{
	/* ���࣬ȡ�պõ�ֵ��Ŀ��Ƕ� */
	StepperMotor.data = fmod(*angle,stepper_step);  
	*angle = *angle - StepperMotor.data;
	StepperMotor.motor_targetangle = *angle;
	
	StepperMotor.err = StepperMotor.motor_targetangle - StepperMotor.motor_realangle;
	/* �ж�ת�� */
	if(StepperMotor.err > stepper_step)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	}
	else if(StepperMotor.err < -stepper_step)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	}
	/* ������ʱ��pwm��� */
	if(abs(StepperMotor.err) > stepper_step)
	{
		HAL_TIM_Base_Start_IT(&htim4);
		HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
	}
	
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim4)
	{
		if(abs(StepperMotor.err) <= stepper_step)
		{
			HAL_TIM_PWM_Stop(&htim5,TIM_CHANNEL_1);
			HAL_TIM_Base_Stop_IT(&htim4);
		}
		else 
		{
			if(StepperMotor.err > 0.0f)
			{
				StepperMotor.motor_realangle += stepper_step;
			}
			else StepperMotor.motor_realangle -= stepper_step;
		}
	}
}

