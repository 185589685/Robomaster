/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal_uart.h"
#include "pid.h"
#include "usart.h"
#include "MyAon.h"
#include "DR16_RECIVE.h"
#include "stdio.h"
#include "math.h"
#include "tim.h"
#include "StepperMotor.h"
#include "typedef.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
extern CAN_Struct Send_CAN_DATA;
  
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */
  if(CAN_FLAG_FF0 != HAL_OK){
//	   get_moto_metrical_infromation(moto_chassis,&get_can_data);
	  CAN_Struct get_can_data;
	  HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&get_can_data.my_can_recevice,get_can_data.can_receive_data);
		switch(get_can_data.my_can_recevice.StdId){
		case CAN_M3508_Moto1_ID:
		case CAN_M3508_Moto2_ID:
		case CAN_M3508_Moto3_ID:
		case CAN_M3508_Moto4_ID:
	    case CAN_M3508_Moto5_ID:
	    case CAN_M3508_Moto6_ID:
		case CAN_M3508_Moto7_ID: 
			{
			    char i;
				i = get_can_data.my_can_recevice.StdId  - CAN_M3508_Base_ID - 1;//判断现在的电机ID为第几位
				
				moto_chassis[i].angle = (unsigned short int)(get_can_data.can_receive_data[0]<<8|get_can_data.can_receive_data[1]); //将读取到的数组的第0位和第1位拼凑起来组成完整的角度信息，并用int强制转化成十进制的数
				moto_chassis[i].speed_rpm = (signed short int)(get_can_data.can_receive_data[2]<<8|get_can_data.can_receive_data[3]); //将读取到的数组的第2位和第3位拼凑起来组成完整的转速信息，并用int强制转化成十进制的数
				moto_chassis[i].real_current = (signed short int)(get_can_data.can_receive_data[4]<<8|get_can_data.can_receive_data[5])*5.f/16384.f*50;//原本读取的是转矩，通过计算将读取到的转矩转化成了电流
				moto_chassis[i].moto_temperature =(unsigned short int)(get_can_data.can_receive_data[6]);
				
			}
			break;
	}	
	for(int i = 0;i < 8;i++){
	M3508[i].position_pid.Measure = moto_chassis[i].speed_rpm;	
	M3508[i].incremental_pid.Measure = moto_chassis[i].speed_rpm;	
	}
//	M3508[0].position_pid.Measure = moto_chassis[0].speed_rpm;	
//    M3508[1].position_pid.Measure = moto_chassis[1].speed_rpm;	
//	M3508[0].incremental_pid.Measure = moto_chassis[0].speed_rpm;	
//	M3508[1].incremental_pid.Measure = moto_chassis[0].speed_rpm;	
  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */
  }
//  for(int i = 0;i <8;i++){
////  M3508[i]->position_pid.Measure   = moto_chassis[i].angle;			        
////  M3508[i]->realSpeed   = moto_chassis[i].speed_rpm;			         
////  M3508[i]->realCurrent = moto_chassis[i].real_current;		         
////  M3508[i]->temperture  = moto_chassis[i].moto_temperature;   
//  M3508[i]->position_pid.Measure = moto_chassis[i].speed_rpm;	  
//  }
    
//过零处理
//  for(int i = 0;i<8;i++){
//   Zero_Treated(moto_chassis[i].angle);
//  }
   // M3508[0].position_pid_angle.Measure = 8191*circle_cnt + Encoder_values;
//     M3508[i].position_pid_speed.Measure = moto_chassis[i].speed_rpm;
  

//    position_pid_angle.Measure = 8191*circle_cnt + Encoder_values;
//    position_pid_speed.Measure = moto_chassis[0].speed_rpm;
//    incremental_pid_angle.Measure = 8191*circle_cnt + Encoder_values;
//    incremental_pid_speed.Measure = moto_chassis[0].speed_rpm;
  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */
if(HAL_TIM_Base_GetState(&htim1) == TIM_FLAG_UPDATE){

//    Debug_addData(M3508[0].position_pid.Target,1);//添加数据
//	Debug_addData(M3508[0].position_pid.Measure,2);//添加数据
//	Debug_addData(M3508[1].position_pid.Target,3);//添加数据
//	Debug_addData(M3508[1].position_pid.Measure,4);//添加数据
//	//Debug_addData(position_pid.target_value,4);//添加数据
//    Debug_show(4);//显示数据
//	HAL_Delay(50);
	
//   Ano_Send_Data(0xF1,&M3508[0].position_pid.Target,sizeof(M3508[0].position_pid.Target));	
//   Ano_Send_Data(0xF2,&M3508[0].position_pid.Measure,sizeof(M3508[0].position_pid.Measure));
//	
//   Ano_Send_Data(0xF3,&M3508[1].position_pid.Target,sizeof(M3508[1].position_pid.Target));	
//   Ano_Send_Data(0xF4,&M3508[1].position_pid.Measure,sizeof(M3508[1].position_pid.Measure));
	
   Ano_Send_Data(0xF1,&M3508[0].incremental_pid.Target,sizeof(M3508[0].incremental_pid.Target));	
   Ano_Send_Data(0xF2,&M3508[0].incremental_pid.Measure,sizeof(M3508[0].incremental_pid.Measure));
	
   Ano_Send_Data(0xF3,&M3508[1].incremental_pid.Target,sizeof(M3508[1].incremental_pid.Target));	
   Ano_Send_Data(0xF4,&M3508[1].incremental_pid.Measure,sizeof(M3508[1].incremental_pid.Measure));
	
   Ano_Send_Data(0xF5,&M3508[0].incremental_pid.output_value,sizeof(M3508[0].incremental_pid.output_value));	
   Ano_Send_Data(0xF6,&M3508[1].incremental_pid.output_value,sizeof(M3508[1].incremental_pid.output_value));
  __HAL_TIM_CLEAR_FLAG(&htim1,TIM_FLAG_UPDATE);
}

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
if(HAL_TIM_Base_GetState(&htim2) == TIM_FLAG_UPDATE){
//		  if(Len != 0){
//	for(int i = 0; i < USART_REC_LEN;i++){
//	arr[i] = (int)USART_RX_BUF[i] - (int)'0';
//	}
//		
//	for(int i = Len-2;i >= 0;i--){
//	 
//	for(int j = 1;Len-i > j;j++){ 
//	 arr[i]=arr[i]*10;
//	 }
//	sum += arr[i];
//	}	
//	//target = sum*8191/360;
//	}

//外环控制角度，内环提升响应速度
	           //外环 角度环 KP 8.5 KI 0.1 KD 0.01                                                                                             
//target_speed = Incremental_PID(&incremental_pid_angle,incremental_pid_angle.Measure,target_angle,KP1,KI1,KD1);
	  		  //内环速度环 KP 5 KI 0.5 KD 0.01 
//       	       target_current = Incremental_PID(&incremental_pid_speed,moto_chassis[0].speed_rpm,target_speed,KP2,KI2,KD2);
               
//			   Incremental_PID(&incremental_pid_current,moto_chassis[0].real_current,target_current,KP3,KI3,KD3);

// target_speed[0] = Position_PID(&position_pid_angle,position_pid_angle.Measure,target_angle[0],KP1,KI1,KD1);
                   M3508[0].targetSpeed =  DR16.rc.ch3 *10; //M3508 电机 1号 目标速度等于左拨杆上下移动的值乘以10
                   M3508[1].targetSpeed = -DR16.rc.ch3 *10;//DR16.rc.ch1 *10; //M3508 电机 2号 目标速度等于左拨杆上下移动的值乘以10
				   
				                                                                           //15,0.005,0.005
//				   Position_PID(&M3508[0],M3508[0].position_pid.Measure,M3508[0].targetSpeed,15,0.001,0.001);//kp2
//				   Position_PID(&M3508[1],M3508[1].position_pid.Measure,M3508[1].targetSpeed,15,0.001,0.001);//kp2
				   Incremental_PID(&M3508[0],M3508[0].incremental_pid.Measure,M3508[0].targetSpeed,20,2,5);//kp2
				   Incremental_PID(&M3508[1],M3508[1].incremental_pid.Measure,M3508[1].targetSpeed,KP2,KI2,KD2);//kp2
				   
// position_pid_speed.output_value = Expert_PID(&position_pid_speed,moto_chassis[0].speed_rpm,target_speed,KP,KI,KD,k_gain,k_restrain,High_error_margin,Low_error_margin,allowed_minimum_error);


//在can接收中断中发送信息给电调，电调处理信息赋值给电机
   assignment_moto_M2006_current(&Send_CAN_DATA,M3508[0].incremental_pid.output_value,M3508[1].incremental_pid.output_value,0,0);
// assignment_moto_M2006_current(&Send_CAN_DATA,5000,0,0,0);
	
//	sum = 0;
//	 for(int j = Len; j < USART_REC_LEN;j++){  
//   USART_RX_BUF[j] = '0';
//   }	
	//}
	
__HAL_TIM_CLEAR_FLAG(&htim2,TIM_FLAG_UPDATE);
}
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
//if(HAL_TIM_Base_GetState(&htim4) == TIM_FLAG_UPDATE){
//__HAL_TIM_CLEAR_FLAG(&htim4,TIM_FLAG_UPDATE);
//// __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1,1);


//}
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
   DR_16hander(&huart1);
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream6 global interrupt.
  */
void DMA2_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream6_IRQn 0 */

  /* USER CODE END DMA2_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_tx);
  /* USER CODE BEGIN DMA2_Stream6_IRQn 1 */

  /* USER CODE END DMA2_Stream6_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */

  /* USER CODE END DMA2_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

  /* USER CODE END DMA2_Stream7_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
