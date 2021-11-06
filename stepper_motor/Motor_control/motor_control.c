#include "motor_control.h"
unsigned short phasecw[4] ={0x0200,0x0100,0x0080,0x0040};// D-C-B-A   ��ת
unsigned short phaseccw[4]={0x0040,0x0080,0x0100,0x0200};// A-B-C-D   ��ת
//���ų�ʼ��
//void Moto_Init(void)
//{
//	 GPIO_InitTypeDef GPIO_InitStructure;
//	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
//	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 ;//���Ű���INT1˳��Ӿ�����
//	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	 GPIO_Init(GPIOB,&GPIO_InitStructure);
//	 GPIO_ResetBits(GPIOB,GPIO_Pin_6 | GPIO_Pin_7 |GPIO_Pin_8 |GPIO_Pin_9 );
//}
void MotoRcw(void)  //��ת
{  
    int  i;  
  
    for(i=0;i<4;i++)  
    {  
//        HAL_GPIO_WritePin(GPIOB,phasecw[i]);  
        HAL_Delay(3);  
    }  
}

void MotoRccw(void)  //��ת
{  
    int i;  
    for(i=0;i<4;i++)  
    {  
//        HAL_GPIO_WritePin(GPIOB,phaseccw[i]);  
        HAL_Delay(3);  
    }  
}

void MotorStop(void) //ֹͣ
{  
//    GPIO_Write(GPIOB,0x0000);  
}

//���Ƶ����ת���Ƿ�תĳ���Ƕ�
//direction����1Ϊ��ת��0Ϊ��ת
//angle�Ƕȣ���Ϊ0-360����ʵ������
void Motor_Ctrl_Direction_Angle(int direction, int angle)
{
	uint16_t j;
	if(direction == 1)
	{
		for(j=0;j<64*angle/45;j++) 
		{
			MotoRccw();//��ת
		}
		 MotorStop();//ֹͣ
  }
	else
	{
		for(j=0;j<64*angle/45;j++) 
		{
			MotoRcw();//��ת
		}
		 MotorStop();//ֹͣ
	}
}
