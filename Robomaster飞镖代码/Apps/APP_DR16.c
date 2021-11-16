/*
*****************************************************************************
*@file    BSP_DR16.c
*@author  Dr.Wu
*@version V1.0
*@date
*@brief
*
*****************************************************************************
*@attention
*
*****************************************************************************
*/
#include "APP_DR16.h"
//		&huart1
void DR_16hander(UART_HandleTypeDef* huart)

{
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
		__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		__HAL_DMA_DISABLE(huart->hdmarx);

		//if(DR16BufferNumber - DMA_GET_COUNTER(huart->hdmarx->Instance) == DR16BufferTruthNumber)
		if (__HAL_DMA_GET_COUNTER(huart->hdmarx) == DR16BufferLastNumber)
		{
			DR16.DR16_Process(DR16Buffer);
		}

		__HAL_DMA_SET_COUNTER(huart->hdmarx, DR16BufferNumber);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}


void DR16_Process(uint8_t* pData)
{
	if (pData == NULL)
	{
		return;
	}
	DR16.rc.ch0 = (pData[0] | (pData[1] << 8)) & 0x07FF;
	DR16.rc.ch1 = ((pData[1] >> 3) | (pData[2] << 5)) & 0x07FF;
	DR16.rc.ch2 = ((pData[2] >> 6) | (pData[3] << 2) | (pData[4] << 10)) & 0x07FF;
	DR16.rc.ch3 = ((pData[4] >> 1) | (pData[5] << 7)) & 0x07FF;
	DR16.rc.s_left = ((pData[5] >> 4) & 0x000C) >> 2;
	DR16.rc.s_right = ((pData[5] >> 4) & 0x0003);
	DR16.mouse.x = (pData[6]) | (pData[7] << 8);
	DR16.mouse.y = (pData[8]) | (pData[9] << 8);
	DR16.mouse.z = (pData[10]) | (pData[11] << 8);
	DR16.mouse.keyLeft = pData[12];
	DR16.mouse.keyRight = pData[13];
	DR16.keyBoard.key_code = pData[14] | (pData[15] << 8);

	//your control code ….
	DR16.rc.ch4_DW = (pData[16] | (pData[17] << 8)) & 0x07FF;
	DR16.infoUpdateFrame++;

	DR16.rc.ch0 -= 1024;
	DR16.rc.ch1 -= 1024;
	DR16.rc.ch2 -= 1024;
	DR16.rc.ch3 -= 1024;
	DR16.rc.ch4_DW -= 1024;
	/* prevent remote control zero deviation */
	/****************避免误触*****************/
	if (DR16.rc.ch0 <= 20 && DR16.rc.ch0 >= -20)
	   {DR16.rc.ch0 = 0;}
	if (DR16.rc.ch1 <= 20 && DR16.rc.ch1 >= -20)
	   {DR16.rc.ch1 = 0;}
	if (DR16.rc.ch2 <= 20 && DR16.rc.ch2 >= -20)
	   {DR16.rc.ch2 = 0;}
	//	if (DR16.rc.ch3 <= 20 && DR16.rc.ch3 >= -20)
	//		DR16.rc.ch3 = 0;
	if (DR16.rc.ch4_DW <= 20 && DR16.rc.ch4_DW >= -20)
	   {DR16.rc.ch4_DW = 0;}
	   
	   
	switch(DR16.rc.s_left){
		case 1:
		        M3508[0].targetSpeed   =  DR16.rc.ch3 *   10 ; /*目标速度 = 左拨杆值 *   10 */
		        M3508[1].targetSpeed   =  DR16.rc.ch3 * (-10) ; /*目标速度 = 左拨杆值 * (-10)*/ 
		case 3: 
			    M3508[0].targetSpeed   =  DR16.rc.ch3 *   10 ; /*目标速度 = 左拨杆值 *   10 */
		        M3508[1].targetSpeed   =  DR16.rc.ch3 * (-10) ; /*目标速度 = 左拨杆值 * (-10)*/ ;
			    M3508[3].targetSpeed   =  DR16.rc.ch3 *   10 ; /*目标速度 = 左拨杆值 *   10 */
		        M3508[4].targetSpeed   =  DR16.rc.ch3 * (-10) ; /*目标速度 = 左拨杆值 * (-10)*/ ;

		case 2: ;
	
	}
}









