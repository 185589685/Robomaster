#include "Peripherals_can.h" 
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "stm32f427xx.h"
#include "can.h"
//extern 	short int can_receive_data[8];
//CAN_Struct get_can_data;//定义接收can数据的结构
//CAN_Struct CAN_DATA;//存放的是得到的测量数据，添加到上位机上的数据也是从这里面挑选的
 
moto_measure_t moto_chassis[7] = {0};//4 chassis moto

CAN_FilterTypeDef CAN_FilterConfigStructure;//定义过滤器配置结构体

                               //将数据传入到数据结构体中   初始化can的结构体将二者联系起来
void get_moto_metrical_infromation(moto_measure_t* ptr,  CAN_Struct *GET_CAN_Data)//得到电机测量信息
{
	
	
	ptr->angle = (unsigned short int)(GET_CAN_Data->can_receive_data[0]<<8|GET_CAN_Data->can_receive_data[1]); //将读取到的数组的第0位和第1位拼凑起来组成完整的角度信息，并用int强制转化成十进制的数
	ptr->speed_rpm = (signed short int)(GET_CAN_Data->can_receive_data[2]<<8|GET_CAN_Data->can_receive_data[3]); //将读取到的数组的第2位和第3位拼凑起来组成完整的转速信息，并用int强制转化成十进制的数
	ptr->real_current = (int)(GET_CAN_Data->can_receive_data[4]<<8|GET_CAN_Data->can_receive_data[5])*5.f/16384.f*50;//原本读取的是转矩，通过计算将读取到的转矩转化成了电流
    ptr->moto_temperature =(unsigned short int)(GET_CAN_Data->can_receive_data[6]);

}

/*选择一个空邮箱，设置标识符（ID）、数据长度、发送数据*/
void assignment_moto_M2006_current(CAN_Struct* Assignment_CAN_Data,int i1,int i2,int i3,int i4){ //赋值电机的电流
	
   
    uint32_t TxBox = CAN_TX_MAILBOX0;//CAN发送box0
//    uint8_t FreeTxMailBoxNum;//定义空闲邮箱数量

	Assignment_CAN_Data->my_can_send.StdId = 0x200;//0x1FF;//标识符ID 
	Assignment_CAN_Data->my_can_send.IDE = CAN_ID_STD;//标准帧
	Assignment_CAN_Data->my_can_send.RTR = CAN_RTR_DATA;//数据帧
	Assignment_CAN_Data->my_can_send.DLC = 0x08;//数据长度
	Assignment_CAN_Data->my_can_send.TransmitGlobalTime = DISABLE;
    //如果产生发送请求
	//if(HAL_CAN_IsTxMessagePending(&hcan1,CAN_TX_MAILBOX0)){}
	
	
//    FreeTxMailBoxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);//查看空闲邮箱数量
//  	 while(0 == FreeTxMailBoxNum)
//   { }
 
	Assignment_CAN_Data->can_send_data[0] = i1>>8;
	Assignment_CAN_Data->can_send_data[1] = i1;
	Assignment_CAN_Data->can_send_data[2] = i2>>8;
	Assignment_CAN_Data->can_send_data[3] = i2;
	Assignment_CAN_Data->can_send_data[4] = i3>>8;
	Assignment_CAN_Data->can_send_data[5] = i3;
	Assignment_CAN_Data->can_send_data[6] = i4>>8;
	Assignment_CAN_Data->can_send_data[7] = i4;
   //发送信息
    HAL_CAN_AddTxMessage(&hcan1,&Assignment_CAN_Data->my_can_send,Assignment_CAN_Data->can_send_data, (uint32_t *)&TxBox);
	//   //如果发送不成功报错-发送失败处理
//	if(HAL_CAN_AddTxMessage(&hcan1,&Assignment_CAN_Data->my_can_send,Assignment_CAN_Data->can_send_data, (uint32_t *)&TxBox) != HAL_OK)
//	{
//	Error_Handler();
//	};
	
}

  
/*初始化can滤波器*/
void can_filter_init(CAN_FilterTypeDef sCAN_FilterConfigStructure){
//	uint32_t  StdId = 0x201;
	//uint32_t  ExtId =0x1800f001;  
	
    //全是0是什么意思，如果只接收ID201的数据怎么办
	//
	sCAN_FilterConfigStructure.FilterIdHigh =0x0000;// StdId<<5;//STID[10:3]STID[2:0]EXID[17:13] 这里全是0的话表示基本ID和拓展ID的13-17位为0
	sCAN_FilterConfigStructure.FilterIdLow = 0x0000;//ID的：EXID[12:5] EXID[4:0] IDE RTR(数据帧为0还是遥控帧)
	sCAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;//0xfdf8;//这里为掩码的高16位，全是0的话表示基本ID和拓展ID的13-17位为0，表示全不关心，这里我选择的是全为1，就是在这16位中每一位都要对的上
	sCAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;//掩码低16位	          
	sCAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
	sCAN_FilterConfigStructure.FilterBank = 14;
	sCAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK; //掩码模式，另一种是列表模式CAN_FILTERMODE_IDLIST
	sCAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
	sCAN_FilterConfigStructure.FilterActivation = ENABLE;
	sCAN_FilterConfigStructure.SlaveStartFilterBank = 0;
 
	HAL_CAN_ConfigFilter(&hcan1, &sCAN_FilterConfigStructure);
	
//	if( != HAL_OK)
//	{
		//err_deadloop(); //show error!
//	}

}

uint32_t FlashTimer;
/*******************************************************************************************
  * @Func			void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
  * @Brief     HAL库中标准的CAN接收完成回调函数，需要在此处理通过CAN总线接收到的数据
  *                                        修改版
  * @Param		
  * @Retval		None 
  * @Date     
 *******************************************************************************************/
void HAL_CAN_RxCpltCallback(CAN_Struct *_HCAN)
{
//使灯按照一定频率闪烁，证明程序正在运行
//	if(HAL_GetTick() - FlashTimer>500){
//			
//		HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
//		FlashTimer = HAL_GetTick();
//		
//	}

  //相当于一个简单的过滤，如果标识符不是以下几个当中的话则忽略传入数据
	switch(_HCAN->my_can_recevice.StdId){
		case CAN_M3508_Moto1_ID:
		case CAN_M3508_Moto2_ID:
		case CAN_M3508_Moto3_ID:
		case CAN_M3508_Moto4_ID:
	    case CAN_M3508_Moto5_ID:
	    case CAN_M3508_Moto6_ID:
		case CAN_M3508_Moto7_ID:
			{
			    char i;
				i = _HCAN->my_can_recevice.StdId  - CAN_M3508_Base_ID;//判断现在的电机ID为第几位
				
				get_moto_metrical_infromation(&moto_chassis[i], _HCAN);//得到信息并传入到ID为：i的数组
			}
			break;
	}		
		//_HCAN->my_can_recevice = get_can_data.my_can_recevice;
	/*#### add enable can it again to solve can receive only one ID problem!!!####**/
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_FULL);//使能can1的rx0接收中断
}

//hal库里面的一些函数
/*
 如果HAL_CAN_GetState(比如说can1) ==ok ！
再进行下一步
HAL_CAN_GetRxFifoFillLevel();得到邮箱的优先级
HAL_CAN_IsTxMessagePending();准备发送信息
HAL_CAN_AbortTxRequest();can中断请求
HAL_CAN_AddTxMessage();添加can发送信息
HAL_CAN_Start();can起始
HAL_CAN_ConfigFilter();can的滤波器配置
HAL_CAN_MspInit();can引脚初始化设置
*/

