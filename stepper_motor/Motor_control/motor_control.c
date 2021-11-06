#include "motor_control.h"
unsigned short phasecw[4] ={0x0200,0x0100,0x0080,0x0040};// D-C-B-A   反转
unsigned short phaseccw[4]={0x0040,0x0080,0x0100,0x0200};// A-B-C-D   正转
//引脚初始化
//void Moto_Init(void)
//{
//	 GPIO_InitTypeDef GPIO_InitStructure;
//	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
//	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 ;//引脚按着INT1顺序接就行了
//	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	 GPIO_Init(GPIOB,&GPIO_InitStructure);
//	 GPIO_ResetBits(GPIOB,GPIO_Pin_6 | GPIO_Pin_7 |GPIO_Pin_8 |GPIO_Pin_9 );
//}
void MotoRcw(void)  //反转
{  
    int  i;  
  
    for(i=0;i<4;i++)  
    {  
//        HAL_GPIO_WritePin(GPIOB,phasecw[i]);  
        HAL_Delay(3);  
    }  
}

void MotoRccw(void)  //正转
{  
    int i;  
    for(i=0;i<4;i++)  
    {  
//        HAL_GPIO_WritePin(GPIOB,phaseccw[i]);  
        HAL_Delay(3);  
    }  
}

void MotorStop(void) //停止
{  
//    GPIO_Write(GPIOB,0x0000);  
}

//控制电机正转还是反转某个角度
//direction方向，1为正转，0为反转
//angle角度，可为0-360具有实际意义
void Motor_Ctrl_Direction_Angle(int direction, int angle)
{
	uint16_t j;
	if(direction == 1)
	{
		for(j=0;j<64*angle/45;j++) 
		{
			MotoRccw();//正转
		}
		 MotorStop();//停止
  }
	else
	{
		for(j=0;j<64*angle/45;j++) 
		{
			MotoRcw();//反转
		}
		 MotorStop();//停止
	}
}
